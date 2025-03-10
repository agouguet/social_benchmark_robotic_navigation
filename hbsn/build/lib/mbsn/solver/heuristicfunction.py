import heapq
import time
from matplotlib import pyplot as plt
from shapely import MultiPolygon
import os, sys, math, random, copy, yaml, cv2, warnings, pickle, numpy as np, networkx as nx
from scipy.spatial import distance

from mbsn.mdp.State import State
from concurrent.futures import ThreadPoolExecutor



HEURISTIC_FUNCTIONS = [
    "random_function",
    "closest_to_goal",
    "avoid_humans",
    "social_heuristic",
]

LIMIT_DISTANCE_TO_OTHER_AGENT = 0.6 # TODO CHANGE WITH HUMAN NUMBER

def random_function(mdp, state):
    return random.choice(mdp.get_actions(state))

def astar(start, goal, mdp, limited_action_of_start_state=None):

    def heuristic(cell1, cell2):
        if cell1 in mdp.polygons and cell2 in mdp.polygons:
            a = mdp.polygons[cell1].polygon.centroid
            b = mdp.polygons[cell2].polygon.centroid
            return a.distance(b)
        return math.inf

    open_set = []
    heapq.heappush(open_set, (0, start.robot))
    came_from = {}
    g_score = {start.robot: 0}
    f_score = {start.robot: heuristic(start.robot, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruire le chemin
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start.robot)
            path.reverse()
            return path

        actions = mdp.get_actions(State(current,start.humans))
        if limited_action_of_start_state is not None and current == start.robot:
            actions = limited_action_of_start_state
        
        for neighbor in actions:
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    print("A* ", f_score)
    return None  # Aucun chemin trouvé

def closest_to_goal(mdp, state, goal, limited_action_of_start_state=None):
    # cell_goal = mdp.get_state_from_continuous_position(mdp.goal)
    astar_path = astar(state, goal, mdp, limited_action_of_start_state=limited_action_of_start_state)
    if astar_path is not None:
        return astar_path[1]
    return state.robot

def heuristic_rules_based(mdp, state, prev_actions=None, actions=None):
    if state.robot == mdp.get_state_from_continuous_position(mdp.goal):
        return state.robot
    
    if len(state.humans) == 0:
        return closest_to_goal(mdp, state)

    if actions is None:
        actions = mdp.get_actions(state)
    
    # if prev_actions is not None:
    #     for a in prev_actions:
    #         if a is not None and a != state.robot and a in actions:
    #             actions.remove(a)
                
    for human in state.humans:
        for pos in human.future_predicted_position:
            pos_traj_state = mdp.get_state_from_continuous_position(pos)
            if pos_traj_state in actions:
                actions.remove(pos_traj_state)

    for h in state.humans:
        if h.position.distance(mdp.polygons[state.robot][0].centroid) < 0.7:
            return state.robot

    actions = list(actions)
    if len(actions) == 0:
        return state.robot
    return closest_to_goal(mdp, state, limited_action_of_start_state=actions)    




def heuristic_score_based(mdp, state, goal=None, previous_path=[], w1=1.0, w2=3.0, w3=0.1, w4=0.5, w5=1.0, limit=LIMIT_DISTANCE_TO_OTHER_AGENT, debug=False, prev_actions=None, actions=None, ros_node = None):
    goal = mdp.goal if goal is None else goal
    if state.robot == mdp.get_state_from_continuous_position(goal):
        return state.robot
    
    actions = mdp.get_actions(state)

    if len(state.humans) == 0:
        cell_goal = mdp.get_state_from_continuous_position(goal)
        return closest_to_goal(mdp, state, cell_goal)
    
    def score_of_movement_and_goal_and_path(state, action, previous_path):
        # actions = mdp.get_actions(state)
        robot_pos = mdp.polygons[state.robot].polygon.centroid
        dg_dict = {}
        dm_dict = {}
        dp_dict = {}
        for act in actions:
            dm_dict[act] = robot_pos.distance(mdp.polygons[act].polygon.centroid)
            dg_dict[act] = goal.distance(mdp.polygons[act].polygon.centroid)
            dp_dict[act] = min([point.distance(mdp.polygons[act].polygon.centroid) for point in previous_path], default=1.0)

        min_dm = min(dm_dict.values())
        max_dm = max(dm_dict.values())
        dm = 1 - (dm_dict[action]-min_dm)/(max_dm-min_dm)

        min_dg = min(dg_dict.values())
        max_dg = max(dg_dict.values())
        dg = 1 - (dg_dict[action]-min_dg)/(max_dg-min_dg)


        dp = 1.0
        if len(previous_path) > 1:
            min_dp = min(dp_dict.values())
            max_dp = max(dp_dict.values())
            dp = 1 - (dp_dict[action]-min_dp)/(max_dp-min_dp)

        return dm, dg, dp

    # def min_dist_with_human(state, action):
    #     dist = math.inf
    #     action_pos = mdp.polygons[action].polygon.centroid
    #     for human in state.humans:
    #         dist_robot_human = human.position.distance(action_pos)
    #         if dist_robot_human < dist:
    #             dist = dist_robot_human

    #     for human in state.humans:
    #         for pos in human.future_predicted_position:
    #             dist_robot_future_pos_human = pos.distance(action_pos)
    #             if dist_robot_future_pos_human < dist:
    #                 dist = dist_robot_future_pos_human
    #     return dist

    # def score_from_closest_agent(state, action, weigth_future_pos=0.1, limit=LIMIT_DISTANCE_TO_OTHER_AGENT):
    #     mean_min_dist = 0
    #     for ns, p in mdp.get_transitions(state, action):
    #         mean_min_dist += min_dist_with_human(ns, action) * p
    #     mean_min_dist /= len(mdp.get_transitions(state, action))
    #     dnear = (min_dist_with_human(state, action) + mean_min_dist*weigth_future_pos) / (1 + weigth_future_pos)

    #     return dnear#/limit #LAST VERSION
    
    
    # def min_dist_with_human(state, action):
    #     """ Calcule la distance minimale entre l'action du robot et les humains (actuels et prédictions futures). """
    #     action_pos = mdp.polygons[action].polygon.centroid
    #     min_dist = math.inf  

    #     # Parcours des humains une seule fois
    #     for human in state.humans:
    #         min_dist = min(min_dist, human.position.distance(action_pos))
    #         # ros_node.get_logger().info(str(min_dist) + "   " + str([pos.distance(action_pos) for pos in human.future_predicted_position]))
    #         min_dist = min(min_dist, min([pos.distance(action_pos) for pos in human.future_predicted_position], default=math.inf))

    #     return min_dist

    # def score_from_closest_agent(state, action, weight_future_pos=0.1, limit=LIMIT_DISTANCE_TO_OTHER_AGENT):
    #     """ Calcule un score basé sur la distance à l'agent le plus proche en considérant les prédictions futures. """
    #     transitions = list(mdp.get_transitions(state, action))  # Stocke en cache pour éviter les doubles appels
    #     mean_min_dist = sum(min_dist_with_human(ns, action) * p for ns, p in transitions) / len(transitions)

    #     dnear = (min_dist_with_human(state, action) + mean_min_dist * weight_future_pos) / (1 + weight_future_pos)
        
    #     return dnear  # / limit  # LAST VERSION

    def min_dist_with_human(state, action):
        """ Calcule la distance minimale entre l'action du robot et les humains actuels et futurs en une seule passe. """
        action_pos = mdp.polygons[action].polygon.centroid
        min_heap = []  # Utilisation d'un tas pour suivre les plus petites distances
        action_xy = np.array([action_pos.x, action_pos.y])

        for human in state.humans:
            heapq.heappush(min_heap, human.position.distance(action_pos))  # Distance actuelle
            
            # Vectorisation des distances futures avec numpy si possible
            if hasattr(human, "future_predicted_position") and human.future_predicted_position:
                future_positions = np.array([(pos.x, pos.y) for pos in human.future_predicted_position])
                distances = np.linalg.norm(future_positions - action_xy, axis=1)
                heapq.heappush(min_heap, np.min(distances))  # Plus courte distance future

        return heapq.heappop(min_heap) if min_heap else math.inf

    def score_from_closest_agent(state, action, weight_future_pos=0.1, limit=LIMIT_DISTANCE_TO_OTHER_AGENT):
        """ Calcule un score basé sur la distance à l'agent le plus proche en considérant les prédictions futures. """
        # transitions = list(mdp.get_transitions(state, action))  # Cache les transitions
        # min_dists = [min_dist_with_human(ns, action) * p for ns, p in transitions]
        
        # mean_min_dist = sum(min_dists) / len(min_dists) if min_dists else math.inf
        # dnear = (min_dist_with_human(state, action) + mean_min_dist * weight_future_pos) / (1 + weight_future_pos)
        dnear = min_dist_with_human(state, action)

        return dnear  # / limit


    def score_for_direction_passage_with_humans(state, action):
        if len(state.humans) == 0:
            return 0.0
        
        p1 = mdp.polygons[state.robot].polygon.centroid
        p3 = mdp.polygons[action].polygon.centroid

        closest_human = None
        dist = math.inf
        for human in state.humans:
            dist_robot_human = human.position.distance(p1)
            if dist_robot_human < dist:
                dist = dist_robot_human
                closest_human = human
        p2 = closest_human.position

        # Vecteurs
        v1 = (p2.x - p1.x, p2.y - p1.y)  # Robot → Humain
        v2 = (p3.x - p1.x, p3.y - p1.y)  # Robot → Destination

        # Produit scalaire
        dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        # ros_node.get_logger().info('DOT : ' + str(action) + "      "   + str(dot_product))
        is_human_not_behind = dot_product > 0
        # if dot_product <= 0:
        #     return 0.5  # L'humain est derrière ou orthogonal



        # Produit vectoriel
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x3, y3 = p3.x, p3.y
        cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
        
        if cross_product == 0:
            return 0.0
        elif cross_product > 0:
            return 0.9 * is_human_not_behind
        else:
            return 1.0 * is_human_not_behind

    def standard_deviation(state, action, k=0.5, alpha=0.8):
        if debug:
            ros_node.get_logger().info("\n")

        t = time.time()
        # PERFORMANCE 
        dm, dg, dp = score_of_movement_and_goal_and_path(state, action, previous_path)
        if debug:
            ros_node.get_logger().info("Time DM DG:" + str(time.time()-t))

        # SOCIAL 
        # dnear = 1-math.exp(-alpha*score_from_closest_agent(state, action))
        dist = score_from_closest_agent(state, action)
        if debug:
            ros_node.get_logger().info("Time score_from_closest_agent:" + str(time.time()-t))
        
        dnear = np.where(dist >= limit, 1.0, dist/limit * np.exp(-alpha*(limit-dist)**2))
        if debug:
            ros_node.get_logger().info("Time DNEAR:" + str(time.time()-t))


        do = score_for_direction_passage_with_humans(state, action)
        if debug:
            ros_node.get_logger().info("Time DO:" + str(time.time()-t))

        sum_weighted_score = w1*dg + w2*dnear + w3*dm + w4*do + w5*dp
        mean = sum_weighted_score/(w1+w2+w3+w4+w5)
        weighted_variance = (w1*(dg-mean)**2 + w2*(dnear-mean)**2 + w3*(dm-mean)**2 + w4*(do-mean)**2 + w5*(dp-mean)**2)/(w1+w2+w3+w4+w5)
        weighted_standard_deviation = round(math.sqrt(weighted_variance), 3)
        score = round(mean - k*weighted_standard_deviation, 3)
        if debug:
            ros_node.get_logger().info("Time score :" + str(time.time()-t))

        if debug:
            print("\n", state)
            print("", action, score, "%.2f" % mean, "%.2f" % weighted_standard_deviation)
            print("     ", "%.2f" % dg, "%.2f" % dnear, "%.2f" % dm, "%.2f" % do)
            print("score_from_closest_agent", score_from_closest_agent(state, action), dnear)

            
            ros_node.get_logger().info("" + str(state))
            ros_node.get_logger().info("    " + str(action) + "  " + str(score) + "  %.2f  " % mean  + "  %.2f  " % weighted_standard_deviation)
            ros_node.get_logger().info("    " + "%.2f " % dg + "%.2f " % dnear + "%.2f " % dm + "%.2f " % do + "%.2f" % dp)
            ros_node.get_logger().info("     score_from_closest_agent: " + str(score_from_closest_agent(state, action)) + "  " + str(dnear))

        return score


    t = time.time()
    max_actions = []
    max_value = float("-inf")
    for action in actions:
        value = round(standard_deviation(state, action), 3)
        if value > max_value:
            max_actions = [action]
            max_value = value
        elif value == max_value:
            max_actions += [action]
    result = random.choice(max_actions)
    return result




DEFAULT_HEURISTIC_FUNCTION_STR = "heuristic_rules_based"
DEFAULT_HEURISTIC_FUNCTION = globals()[DEFAULT_HEURISTIC_FUNCTION_STR]
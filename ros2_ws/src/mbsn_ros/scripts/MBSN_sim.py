#!/usr/bin/env python3
from collections import defaultdict
from itertools import combinations
import math
import os
import time

from matplotlib import pyplot as plt
# from human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction

import matplotlib
import mbsn
from mbsn.human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction
from mbsn.mdp.State import State
from mbsn.mdp.MBSN import MBSN
from mbsn.mdp.agent import Agent
from mbsn.polygonal_map.polygonal_map import PolygonalMap
from mbsn.solver.MCTS import MBSNAgentMCTS
from mbsn.solver.heuristicfunction import heuristic_rules_based, heuristic_score_based
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.solver.qtable import QTable
from mbsn.utils.util import astar
import numpy as np
from scipy.spatial import Delaunay, Voronoi

import rclpy # type: ignore
from rclpy.node import Node # type: ignore

import pygeoops
from shapely import LineString, MultiPolygon, Point, Polygon, convex_hull, unary_union, polygonize
from shapely.ops import nearest_points
from geometry_msgs.msg import Pose, Point as RosPoint # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from mbsn_ros.interpreter import Interpreter
from mbsn_ros.util import ros_point_to_shapely_point, ros_quaternion_to_euler
from visualization_msgs.msg import Marker, MarkerArray # type: ignore
from std_msgs.msg import Header # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from geometry_msgs.msg import PoseArray # type: ignore
from nav_msgs.msg import Odometry, OccupancyGrid # type: ignore
from agents_msgs.msg import AgentArray # type: ignore
from graph_msgs.msg import GraphNav # type: ignore
from simulation_msgs.msg import SceneInfo # type: ignore
from agents_msgs.msg import AgentArray, AgentTrajectories, AgentTrajectory # type: ignore
from ament_index_python.packages import get_package_share_directory


id_points = {}

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if position in id_points:
        return id_points[position]

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.polygon.buffer(0.1).contains(position):
                id_points[position] = id
                return id
    return None

LIMIT_DISTANCE_WITH_HUMAN_TO_MOVING = 0.7

class MBSNNode(Interpreter):

    def __init__(self, name_node="mbsn_sim"):
        super().__init__(name_node)

        self.mdp = None
        self.state = None
        self.local_state_timed = time.time()
        self.qfunction = QTable(default=-1e6)
        self.bandit = UpperConfidenceBounds()
        self.start = False
        self.robot_cell = None

        self._map_publisher = self.create_publisher(OccupancyGrid, 'custom_costmap', 10)

    ################
    ### CALLBACK ###
    ################

    def robot_odom_callback(self, msg_odom):
        if self.goal is not None and self.global_path is not None:
            robot_position = msg_odom.pose.pose.position
            robot_position = ros_point_to_shapely_point(robot_position)
            self.robot_position = robot_position
            self.update()
            # if self.mdp is None or self.need_update():
            #     self.init_trajectory()
            #     # self.update()

    def velocity_callback(self, msg_vel):
        linear = msg_vel.linear
        angular = msg_vel.angular
        if linear.x != 0 or linear.y != 0 or linear.z != 0:
            self.start = True
        if angular.x != 0 or angular.y != 0 or angular.z != 0:
            self.start = True

    ##############
    ### UPDATE ###
    ##############

    def need_update(self):        
        if not self.start:
            return True

        # ROBOT STATE
        old_state_robot = self.state.robot
        new_state_robot = self.mdp.get_state_from_continuous_position(self.robot_position)
        if old_state_robot != new_state_robot:
            return True

        state_of_published_local_goal = self.mdp.get_state_from_continuous_position(self.published_local_goal)
        for _, human in self.humans.items():
            if human.position.distance(self.robot_position) <= self._robot_visibility_distance:
                human_state = self.mdp.get_state_from_continuous_position(human.position)
                if state_of_published_local_goal == human_state:
                    return True

        return False

    def update(self):
        ttotal = time.time()
        self.mdp = MBSN(self.polygonal_map, self.robot_position, self.goal, human_trajectory_prediction_function=simple_human_trajectory_prediction, visibility_distance=self._robot_visibility_distance)
        
        # self.get_logger().info('T MBSN : ' + str(time.time() - ttotal))












        # polys = []

        # for poly in self.mdp.polygons.values():
        #     if type(poly.polygon) == Polygon:
        #         polys.append(poly.polygon)

        # mergedPolys = unary_union(polys, grid_size=0.05)
        # convex = convex_hull(mergedPolys)
        # self.exterior_polygon = mergedPolys

        # if type(self.exterior_polygon) != Polygon:
        #     return

        # hull_coords = list(self.exterior_polygon.exterior.coords)

        # valid_distances = []
        # angle_threshold=80

        # for (p1, p2) in combinations(hull_coords, 2):
        #     v_dist = np.array([p2[0] - p1[0], p2[1] - p1[1]])  # Vecteur entre les 2 points

        #     # Vérifier l'angle avec chaque arête proche
        #     for i in range(len(hull_coords) - 1):
        #         a, b = hull_coords[i], hull_coords[i + 1]
        #         v_edge = np.array([b[0] - a[0], b[1] - a[1]])  # Vecteur de l'arête

        #         angle = self.angle_between_vectors(v_dist, v_edge)

        #         # Vérifier si l'angle est proche de 90° (orthogonal)
        #         if 90 - angle_threshold <= angle <= 90 + angle_threshold:
        #             valid_distances.append(Point(p1).distance(Point(p2)))
        #             break  # Un seul angle valide suffit

        # # Calcul des statistiques
        # if valid_distances:
        #     distances = np.array(valid_distances)
        #     stats = {
        #         "min": np.min(distances),
        #         "max": np.max(distances),
        #         "mean": np.mean(distances),
        #         "median": np.median(distances)
        #     }
        # else:
        #     stats = {"min": None, "max": None, "mean": None, "median": None}

        # centerline = pygeoops.centerline(self.exterior_polygon)
        # # self.get_logger().info('DIST : ' + str(self.exterior_polygon.exterior.distance(centerline)))


        # def average_distance(poly1, poly2):
        #     points1 = np.array(poly1.exterior.coords)
        #     points2 = np.array(poly2.exterior.coords)

        #     dist1_to_robot = np.array([self.robot_position.distance(Point(p)) for p in points1])
        #     dist2_to_robot = np.array([self.robot_position.distance(Point(p)) for p in points2])

        #     expo = 0.5
        #     norm_dist1_to_robot = np.power(dist1_to_robot, expo) #1 - ((dist1_to_robot - np.min(dist1_to_robot)) / (np.max(dist1_to_robot) - np.min(dist1_to_robot)))
        #     norm_dist2_to_robot = np.power(dist2_to_robot, expo) #1 - ((dist2_to_robot - np.min(dist2_to_robot)) / (np.max(dist2_to_robot) - np.min(dist2_to_robot)))

        #     # Moyenne des distances de chaque point de poly1 vers poly2
        #     distances1 = [poly2.exterior.distance(Point(points1[i])) * norm_dist1_to_robot[i] for i in range(len(points1))]
            
        #     # Moyenne des distances de chaque point de poly2 vers poly1
        #     distances2 = [poly1.exterior.distance(Point(points2[i])) * norm_dist2_to_robot[i] for i in range(len(points2))]

        #     mean_dist1 = np.sum(distances1) / np.sum(norm_dist1_to_robot)
        #     mean_dist2 = np.sum(distances2) / np.sum(norm_dist2_to_robot)
            
        #     # Moyenne totale
        #     return (mean_dist1 + mean_dist2) / 2


        # centerline_polygon = centerline.buffer(0.01)#polygonize([centerline])
        # # self.get_logger().info('POLY LINE : ' + str(centerline_polygon))
        # self.get_logger().info('AVG DIST : ' + str(average_distance(self.exterior_polygon, centerline_polygon)))

        # # average_distance(self.exterior_polygon, centerline_polygon)
        # self.exterior_polygon = centerline.buffer(0.01)
        








        # ROBOT/GOAL CELLS
        robot_cell  = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, self.robot_position)
        goal_cell = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, self.goal)

        if robot_cell != self.robot_cell:
            self.publish_polygon_map_vizualisation()
            # self.publish_exterior_polygon_map_vizualisation()
        self.robot_cell = robot_cell

        if self.robot_cell  == goal_cell:
            self.publish_local_path([self.robot_position, self.goal])
            # self.publish_local_goal(self.goal)

        # LOCAL GOAL
        mbsn_goal_id = self.robot_cell 
        path = []
        
        for pose in self.global_path:
            # self.get_logger().info('     T G 1: ' + str(time.time() - ttotal))
            pose_cell_id = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, pose)
            # self.get_logger().info('     T G 2: ' + str(time.time() - ttotal))
            if pose_cell_id in self.mdp.polygons:
                mbsn_goal_id = pose_cell_id
                path.append(pose_cell_id)
            else:
                break
        mbsn_goal = self.mdp.polygons[mbsn_goal_id].polygon.centroid
        # self.get_logger().info('T G : ' + str(time.time() - ttotal) + "  NB point in global path: " + str(len(self.global_path)))


        # HUMAN IN SCOPE
        human_in_local_scope = False
        for _, human in self.humans.items():
            if human.position.distance(self.robot_position) <= self._robot_visibility_distance:
                human_in_local_scope = True
        # self.get_logger().info('T H : ' + str(time.time() - ttotal))


        # PATH
        if not human_in_local_scope:
            path  = list(dict.fromkeys(path))
        else:
            path = [self.robot_cell]
            humans_distance = {}
            for id, human in self.humans.items():
                humans_distance[id] = human.position.distance(self.robot_position)
                if humans_distance[id] <= LIMIT_DISTANCE_WITH_HUMAN_TO_MOVING:
                    self.publish_local_path([self.robot_position])
                    return
            humans_distance = {k: v for k, v in sorted(humans_distance.items(), key=lambda item: item[1])}
            closest_humans = np.array([self.humans[id] for id in humans_distance.keys()])[:]

            # closest_humans = np.array(humans_distance.values())
            # self.get_logger().info(str(closest_humans))
            while 1:
                # state = State(path[-1], list(self.humans.values()))
                state = State(path[-1], closest_humans)
                action = heuristic_score_based(self.mdp, state, goal=mbsn_goal, previous_path=self.path, debug=False, 
                                               w1=4.0, 
                                               w2=15.0, 
                                               w3=1.0, 
                                               w4=1.0,
                                               w5=1.0,
                                               limit=1.0,
                                               ros_node=self)
                
                # self.get_logger().info('MBSNAgentMCTS: ' + str(path))
                # solver = MBSNAgentMCTS(self.mdp, self.qfunction, self.bandit, heuristic_function=heuristic_score_based) # MCTS
                # root_node, num_rollouts = solver.mcts(state, timeout=0.1)
                # action, _ = root_node.get_value()

                if len(path) >= 2 and action == path[-2]:
                    break
                path.append(action)
                # self.get_logger().info('P: ' + str(path))
        # self.get_logger().info('T P : ' + str(time.time() - ttotal))

        local_path = []
        for id in range(1, len(path)):
            prev_poly = self.mdp.polygons[path[id-1]].polygon.buffer(0.05)
            next_poly = self.mdp.polygons[path[id]].polygon.buffer(0.05)
            local_path.append(prev_poly.intersection(next_poly).centroid)

        if len(local_path) > 1:
            local_path[0] = self.robot_position
        # self.get_logger().info('T TOTAL : ' + str(time.time() - ttotal))

        if goal_cell in self.mdp.polygons:
            local_path.append(self.goal)

        # self.get_logger().info('Path : ' + str(local_path))
        self.publish_local_path(local_path)
        # self.publish_costmap()
                
    def publish_costmap(self):
        costmap = OccupancyGrid()
        
        # Remplissage des métadonnées
        costmap.header = Header()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = "map"  # Assurez-vous que Rviz utilise ce frame

        costmap.info.resolution = 0.1  # Taille d'une cellule (mètres par pixel)
        costmap.info.width = 20  # Nombre de colonnes
        costmap.info.height = 20  # Nombre de lignes
        
        # Position de l'origine de la costmap
        costmap.info.origin = Pose()
        costmap.info.origin.position.x = -5.0
        costmap.info.origin.position.y = -5.0
        costmap.info.origin.position.z = 0.0

        # Génération des données de la costmap (valeurs entre 0 et 100, -1 pour inconnu)
        data = []
        for y in range(costmap.info.height):
            for x in range(costmap.info.width):
                if 5 <= x < 15 and 5 <= y < 15:  # Zone occupée au centre
                    data.append(100)  # Zone occupée
                else:
                    data.append(0)  # Zone libre
        
        costmap.data = data  # Affectation des valeurs de la costmap

        # Publication du message
        self._map_publisher.publish(costmap)
        self.get_logger().info("Costmap publiée sur /custom_costmap")


def main(args=None):
    rclpy.init(args=args)

    mbsn_node = MBSNNode()

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
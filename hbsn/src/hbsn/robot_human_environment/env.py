
from collections import defaultdict
import random
import time
from matplotlib import pyplot as plt
import matplotlib
import numpy as np
from shapely import MultiPolygon, Point, Polygon
from hbsn.solver.MCTS import HBSNAgentMCTS, HBSNAgentNode
from hbsn.solver.ValueIteration import ValueIteration
from hbsn.solver.heuristicfunction import heuristic_score_based
from hbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from hbsn.solver.qtable import QTable
from shapely.plotting import plot_line, plot_points, plot_polygon
from hbsn.utils.util import astar

colors = [plt.cm.hsv(i) for i in np.linspace(0, 1, 1000)]
random.shuffle(colors)

class Environment():

    def __init__(self, polygonal_map, robot, humans, goal):
        self.polygonal_map = polygonal_map
        self.robot = robot
        self.humans = humans
        self.goal = goal
        
    def move_a_human(self, polygonal_map, human, human_goal_detected_by_robot):
        h_pos = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, human.position)
        g_pos = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, human.goal[0])
        true_path = astar(h_pos, g_pos, self.polygonal_map.grid)
        if len(true_path) > 1:
            human.position = self.polygonal_map.grid[true_path[1]].centroid
        else:
            if len(human.goal) > 1:
                human.goal.pop(0)
                if human.goal != human_goal_detected_by_robot and len(human_goal_detected_by_robot) > 1:
                    human_goal_detected_by_robot.pop(0)

        # if PREDICTION_FUNCTION.__name__ == simple_human_trajectory_prediction.__name__:
        #     human.future_predicted_position = simple_trajectory_prediction_for_one_human(human)
        # else:
        #     h_pos = get_cell_id_in_dict_from_continuous_position(self.map_polygon.grid, human.position)
        #     g_dbr_pos = get_cell_id_in_dict_from_continuous_position(self.map_polygon.grid, human_goal_detected_by_robot[0])
        #     detected_path = astar(h_pos, g_dbr_pos, self.map_polygon.grid)
        #     if len(detected_path) > 1:
        #         human.future_predicted_position = [self.map_polygon.grid[id].centroid for id in detected_path[1:6]]
    
    def step(self, next_robot_position):
        
        self.robot.position = next_robot_position
        visible_human = []

        for h in self.humans:
            if h.position.distance(self.robot.position) <= 3.0:
                visible_human.append(h)

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.polygon.buffer(0.1).contains(position):
                return id
    return None
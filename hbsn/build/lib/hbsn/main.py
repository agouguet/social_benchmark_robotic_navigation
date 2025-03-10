#!/usr/bin/env python3
import os
import random
import sys
import time

import matplotlib
from matplotlib import pyplot as plt

from hbsn.human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction
from hbsn.mdp.State import State
from hbsn.mdp.agent import Agent
from hbsn.robot_human_environment.env import Environment, get_cell_id_in_dict_from_continuous_position
from hbsn.solver.MCTS import HBSNAgentMCTS
from hbsn.solver.heuristicfunction import heuristic_score_based
from hbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from hbsn.solver.qtable import QTable
from hbsn.utils.util import astar
import pygame
from shapely import Point

from hbsn.mdp.HBSN import HBSN
from hbsn.polygonal_map.polygonal_map import PolygonalMap

from hbsn.robot_human_environment.pygame_window import Window    


scenario = "small"
scenario = "./src/hbsn/data/small"
map = PolygonalMap(scenario, type="hexagon", area_minimum=0.1)
robot_position = Point(0, 5)
goal_position = Point(0, -5)

robot = Agent(robot_position)
humans = [Agent(Point(0,0), orientation=90)]

env = Environment(map, robot, humans, goal_position)

window = Window(1280,920,env)

next_step=False



qfunction = QTable(default=-1e6)
bandit = UpperConfidenceBounds()

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.polygon.buffer(0.1).contains(position):
                return id
    return None

while robot.position != goal_position:
    if next_step:
        # ROBOT/GOAL CELLS
        robot_cell = get_cell_id_in_dict_from_continuous_position(map.grid, env.robot.position)
        goal_cell = get_cell_id_in_dict_from_continuous_position(map.grid, env.goal)

        # MDP
        hbsn = HBSN(map, robot.position, goal_position, human_trajectory_prediction_function=simple_human_trajectory_prediction)

        # STATE
        state = State(robot_cell, humans)

        # LOCAL GOAL
        astar_path = astar(robot_cell, goal_cell, map.grid)
        hbsn_goal_id = astar_path[0]
        for cell_id in astar_path:
            if cell_id in hbsn.polygons:
                hbsn_goal_id = cell_id
        hbsn_goal = hbsn.polygons[hbsn_goal_id].polygon.centroid

        # ACTION CHOICE
        # action = random.choice(hbsn.get_actions(state)) # Random
        # action = heuristic_score_based(hbsn, state, goal=hbsn_goal, debug=False, w2=5) # Heuristic

        solver = HBSNAgentMCTS(hbsn, qfunction, bandit, heuristic_function=heuristic_score_based) # MCTS
        root_node, num_rollouts = solver.mcts(state, timeout=1.0)
        action, _ = root_node.get_value()

        # STEP
        env.step(map.grid[action].polygon.centroid)
        next_step = False

    next_step = window.update(fps=10)

pygame.quit()

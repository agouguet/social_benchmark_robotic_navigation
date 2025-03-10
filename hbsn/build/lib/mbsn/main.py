#!/usr/bin/env python3
import os
import random
import sys
import time

import matplotlib
from matplotlib import pyplot as plt

from mbsn.human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction
from mbsn.mdp.State import State
from mbsn.mdp.agent import Agent
from mbsn.robot_human_environment.env import Environment, get_cell_id_in_dict_from_continuous_position
from mbsn.solver.MCTS import MBSNAgentMCTS
from mbsn.solver.heuristicfunction import heuristic_score_based
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.solver.qtable import QTable
from mbsn.utils.util import astar
import pygame
from shapely import Point

from mbsn.mdp.MBSN import MBSN
from mbsn.polygonal_map.polygonal_map import PolygonalMap

from mbsn.robot_human_environment.pygame_window import Window    


scenario = "small"
scenario = "./src/mbsn/data/small"
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
        mbsn = MBSN(map, robot.position, goal_position, human_trajectory_prediction_function=simple_human_trajectory_prediction)

        # STATE
        state = State(robot_cell, humans)

        # LOCAL GOAL
        astar_path = astar(robot_cell, goal_cell, map.grid)
        mbsn_goal_id = astar_path[0]
        for cell_id in astar_path:
            if cell_id in mbsn.polygons:
                mbsn_goal_id = cell_id
        mbsn_goal = mbsn.polygons[mbsn_goal_id].polygon.centroid

        # ACTION CHOICE
        # action = random.choice(mbsn.get_actions(state)) # Random
        # action = heuristic_score_based(mbsn, state, goal=mbsn_goal, debug=False, w2=5) # Heuristic

        solver = MBSNAgentMCTS(mbsn, qfunction, bandit, heuristic_function=heuristic_score_based) # MCTS
        print("Oui")
        root_node, num_rollouts = solver.mcts(state, timeout=1.0)
        print("Oui")
        action, _ = root_node.get_value()

        # STEP
        env.step(map.grid[action].polygon.centroid)
        next_step = False

    next_step = window.update(fps=10)

pygame.quit()

from collections import defaultdict

from shapely import MultiPolygon, Point, coverage_union

from .MDP import MDP
from scipy.spatial import distance
from itertools import combinations 
import networkx as nx
import itertools
import numpy as np
import math
import random

ROBOT_VISIBILITY_DISTANCE = 3.0
PENALITY_DISTANCE_GOAL = 1
PENALITY_COLLISION_HUMAN = 1000
PENALITY_FUTURE_COLLISION_HUMAN = 100
PENALITY_PROXIMITY_HUMAN = 0.1
REWARD_GOAL = 1000
PENALITY_WAIT = 10

class MBSN(MDP):

    def __init__(
        self,
        polygonal_map,
        robot_position,
        goal_position,
        human_trajectory_prediction_function=None,
        number_of_detected_human = 0,
        distance_factor=1.0,
        social_factor=1.0, 
        discount_factor = 0.99,
        visibility_distance = ROBOT_VISIBILITY_DISTANCE
    ):
        self.polygonal_map = polygonal_map
        self.polygons = {}


        self.visibility_polygon = robot_position.buffer(visibility_distance)

        for id, cell in polygonal_map.grid.items():
            if cell.polygon.intersects(self.visibility_polygon):
                self.polygons[cell.id] = cell

        self.goal = goal_position

        self.number_of_detected_human = number_of_detected_human
        self.discount_factor = discount_factor
        self._social_factor = social_factor
        self._distance_factor = distance_factor
        self.human_trajectory_prediction_function = human_trajectory_prediction_function

    def get_states(self):
        pass

    def get_actions(self, state):
        if self.get_state_from_continuous_position(self.goal) == state.robot:
            return ["find_goal"]
        actions = [state.robot]
        polygon = self.polygons[state.robot]
        for n in polygon.neighbors:
            if n.id in self.polygons:
                actions.append(n.id)
        actions = list(np.unique(actions))
        return actions

    def get_next_states(self, state, action):
        pass

    """ Return all non-zero probability transitions for this action
        from this state, as a list of (state, probability) pairs
    """
    def get_transitions(self, state, action):
        return self.human_trajectory_prediction_function(self, state, action)

    """ Return the reward for transitioning from state to
        nextState via action
    """
    def get_reward(self, state, action, next_state):
        reward = 0

        # Penalty Time
        reward -= 5

        # Penalty Stationary
        if state.robot == action:
            reward -= PENALITY_WAIT

        # Penalty Distance Between current state and next state
        reward -= (self.get_position_of_state(state.robot).distance(self.get_position_of_state(next_state.robot)))

        # Reward for Distance from Goal
        reward += self._distance_factor * (self.goal.distance(self.get_position_of_state(state.robot)) - self.goal.distance(self.get_position_of_state(next_state.robot)))

        # Penalty Human
        for human in state.humans:
            human_state = self.get_state_from_continuous_position(human.position)
            if state.robot == human_state or action == human_state:
                reward -= self._social_factor * PENALITY_COLLISION_HUMAN
                break

        # Goal Reward
        # for polygon in self.polygons[state.robot]:
        if self.polygons[state.robot].intersects(self.goal.buffer(0.05)):
            reward += REWARD_GOAL

        return reward

    """ Return true if and only if state is a terminal state of this MDP """
    def is_terminal(self, state):
        if len(self.get_actions(state)) == 0:
            return True

        if self.get_state_from_continuous_position(self.goal) == state.robot:
            return True
        # for polygon in self.polygons[state.robot]:
        if self.polygons[state.robot].intersects(self.goal.buffer(0.1)):
            return True
        return False

    """ Return the discount factor for this MDP """
    def get_discount_factor(self):
        return self.discount_factor
    
    def get_state_from_continuous_position(self, position):
        if not isinstance(position, Point):
            position = Point(position)

        if isinstance(self.polygons, dict):
            for id, polygon in self.polygons.items():
                # print(id, polygon[0].centroid, position)
                if polygon.polygon.buffer(0.1).contains(position) or polygon.polygon.buffer(0.1).intersects(position):
                    return id
        return None
    

    def get_position_of_state(self, state):
        return self.polygons[state].centroid
        return MultiPolygon([p.polygon for p in self.polygons[state]]).centroid
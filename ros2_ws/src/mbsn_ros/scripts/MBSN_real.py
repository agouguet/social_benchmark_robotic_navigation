#!/usr/bin/env python3
from collections import defaultdict
import os
import time

from matplotlib import pyplot as plt
# from human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction

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

from mbsn_ros.interpreter import Interpreter
from mbsn_ros.util import ros_point_to_shapely_point, ros_quaternion_to_euler

import rclpy # type: ignore
from rclpy.node import Node # type: ignore


from shapely import LineString, Point
from shapely.ops import nearest_points
from geometry_msgs.msg import Pose, Point as RosPoint # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore

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

import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

MIN_TIME_OCCUPANCY = 10
TIME_OCCUPANCY = 30


def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.polygon.buffer(0.1).contains(position):
                return id
    return None

class MBSNNode(Interpreter):

    def __init__(self, name_node="mbsn_real"):
        super().__init__(name_node)

        self.mdp = None
        self.state = None
        self.local_state_timed = time.time()
        self.qfunction = QTable(default=-1e6)
        self.bandit = UpperConfidenceBounds()
        self.can_publish_local_goal = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("delay", 0.0)
        self.time_delay = self.get_parameter('delay').value

    def goal_callback(self, msg_goal):
        pos = msg_goal.pose.position
        goal = ros_point_to_shapely_point(pos)
        if self.goal != goal:
            time.sleep(self.time_delay)
            self.goal = goal
            self.get_logger().info('Goal received: ' + str(self.goal))

    def robot_odom_callback(self, msg_odom):

        if self.goal is not None:
            robot_position = msg_odom.pose.pose
            t = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            pose = tf2_geometry_msgs.do_transform_pose(robot_position, t)
            robot_position = pose.position
            self.robot_position = ros_point_to_shapely_point(robot_position)

            if self.mdp is not None:
                humans_state = []
                for _, human in self.humans.items():
                    if human.position.distance(self.robot_position) <= 4.0:
                        human_state = self.mdp.get_state_from_continuous_position(human.position)
                        if human_state is not None and human_state not in humans_state:
                            humans_state.append(human_state)

                        for pos in human.future_predicted_position:
                            future_human_state = self.mdp.get_state_from_continuous_position(pos)
                            if future_human_state is not None and future_human_state not in humans_state:
                                humans_state.append(future_human_state)
                
                state_of_published_local_goal = self.mdp.get_state_from_continuous_position(self.published_local_goal)

                test_if_changement_state_of_published_local_goal = (state_of_published_local_goal in humans_state and state_of_published_local_goal not in self.humans_state) or (state_of_published_local_goal not in humans_state and state_of_published_local_goal in self.humans_state)
                is_not_same_robot_state = self.mdp.get_state_from_continuous_position(self.robot_position) != self.state.robot
                is_robot_position_equal_to_local_goal = self.mdp.get_state_from_continuous_position(self.robot_position) == state_of_published_local_goal
                is_local_goal_equal_to_goal = state_of_published_local_goal == self.mdp.get_state_from_continuous_position(self.goal)

                if (is_not_same_robot_state or test_if_changement_state_of_published_local_goal or is_robot_position_equal_to_local_goal) and self.can_publish_local_goal and not is_local_goal_equal_to_goal:
                    self.humans_state = humans_state
                    self.update()
            else:
                self.update()

    def update(self):
        self.can_publish_local_goal = False
        self.mdp = MBSN(self.polygonal_map, self.robot_position, self.goal, human_trajectory_prediction_function=simple_human_trajectory_prediction)
        self.state = State(self.mdp.get_state_from_continuous_position(self.robot_position), list(self.humans.values()))
        self.publish_polygon_map_vizualisation()

        # ROBOT/GOAL CELLS
        robot_cell = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, self.robot_position)
        goal_cell = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, self.goal)

        # LOCAL GOAL
        astar_path = astar(robot_cell, goal_cell, self.polygonal_map.grid)
        mbsn_goal_id = astar_path[0]
        for cell_id in astar_path:
            if cell_id in self.mdp.polygons:
                mbsn_goal_id = cell_id
        mbsn_goal = self.mdp.polygons[mbsn_goal_id].polygon.centroid

        action = heuristic_score_based(self.mdp, self.state, goal=mbsn_goal, debug=True, w2=5)
        if action != "find_goal":
            local_goal = self.mdp.polygons[action].polygon.centroid
            self.publish_local_goal(local_goal)
        else:
            self.publish_local_goal(self.goal)
        
def main(args=None):
    rclpy.init(args=args)

    mbsn_node = MBSNNode()

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
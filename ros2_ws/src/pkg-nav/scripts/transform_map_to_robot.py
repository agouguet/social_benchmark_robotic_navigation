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
import numpy as np

from mbsn_ros.interpreter import Interpreter
from mbsn_ros.util import ros_point_to_shapely_point, ros_quaternion_to_euler
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from nav_msgs.msg import Odometry


from geometry_msgs.msg import Pose, Point # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore

import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TransformMapToRobot(Node):

    def __init__(self, name_node="transform_map_to_robot"):
        super().__init__(name_node)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', "map"),
                ('robot_frame', "base_link"),
                ('odom_topic', "odom"),
                ('robot_pose_topic', "robot_pose")
            ])

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value

        self.odom_subscription_ = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1)
        self.robot_pose_publisher = self.create_publisher(Pose, self.robot_pose_topic, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            pose = Pose()
            position = Point()
            position.x = translation.x 
            position.y = translation.y
            position.z = translation.z

            pose.position = position
            pose.orientation = rotation
            self.robot_pose_publisher.publish(pose)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform not available yet: {e}")
        
def main(args=None):
    rclpy.init(args=args)

    node = TransformMapToRobot()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
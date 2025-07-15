#!/usr/bin/env python3

import numpy as np
import numpy.matlib
import math, os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from cnn_msgs.msg import CNNdata
from stable_baselines3 import PPO

from custom_cnn_full import *

from ament_index_python.packages import get_package_share_directory


class DrlInference(Node):
    def __init__(self):
        super().__init__('drl_inference_node')

        # Internal data
        self.ped_pos = []
        self.scan = []
        self.goal = []
        self.model = None

        model_directory = get_package_share_directory('ros_gym_env')

        # Declare & retrieve parameter
        self.declare_parameter('model_file', './model/drl_vo')
        model_file = self.get_parameter('model_file').get_parameter_value().string_value

        # Load model
        model_path = os.path.join(model_directory, model_file)
        self.model = PPO.load(model_path)
        self.get_logger().info('DRL-VO model loaded.')

        # ROS 2 Communication
        self.subscription = self.create_subscription(
            CNNdata,
            'cnn_data',
            self.cnn_data_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def cnn_data_callback(self, msg):
        self.ped_pos = msg.ped_pos_map
        self.scan = msg.scan
        self.goal = msg.goal_cart
        cmd_vel = Twist()

        scan = np.array(self.scan[-540:-180])
        scan = scan[scan != 0]
        min_scan_dist = np.amin(scan) if scan.size != 0 else 10.0

        if np.linalg.norm(self.goal) <= 0.9:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        elif min_scan_dist <= 0.4:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.7
        else:
            # Scale ped_pos
            v_min, v_max = -2, 2
            ped_pos = np.array(self.ped_pos, dtype=np.float32)
            ped_pos = 2 * (ped_pos - v_min) / (v_max - v_min) + (-1)

            # Scale scan
            temp = np.array(self.scan, dtype=np.float32)
            scan_avg = np.zeros((20, 80))
            for n in range(10):
                scan_tmp = temp[n * 720:(n + 1) * 720]
                for i in range(80):
                    scan_avg[2 * n, i] = np.min(scan_tmp[i * 9:(i + 1) * 9])
                    scan_avg[2 * n + 1, i] = np.mean(scan_tmp[i * 9:(i + 1) * 9])
            scan_avg = scan_avg.reshape(1600)
            scan_avg_map = np.matlib.repmat(scan_avg, 1, 4)
            scan_scaled = scan_avg_map.reshape(6400)
            s_min, s_max = 0, 30
            scan_scaled = 2 * (scan_scaled - s_min) / (s_max - s_min) + (-1)

            # Scale goal
            g_min, g_max = -2, 2
            goal_original = np.array(self.goal, dtype=np.float32)
            goal_scaled = 2 * (goal_original - g_min) / (g_max - g_min) + (-1)

            # Observation
            observation = np.concatenate((ped_pos, scan_scaled, goal_scaled), axis=None)

            # Inference
            action, _ = self.model.predict(observation)

            # Rescale output
            vx_min, vx_max = 0, 0.5
            vz_min, vz_max = -2, 2
            cmd_vel.linear.x = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
            cmd_vel.angular.z = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min

        # Publish
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z):
            self.publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = DrlInference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

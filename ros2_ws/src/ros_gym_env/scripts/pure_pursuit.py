#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker


import tf2_ros
import numpy as np
import threading


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Paramètres du contrôleur
        self.lookahead = 2.0
        self.rate = self.declare_parameter('rate', 20.0).value
        self.prefix = self.declare_parameter('prefix', "").value
        self.prefix = self.prefix+"/" if self.prefix != "" else self.prefix
        self.goal_margin = 0.9

        self.wheel_base = 0.23
        self.wheel_radius = 0.025
        self.v_max = 0.5
        self.w_max = 5.0

        # Données
        self.robot_pose = PoseStamped()
        self.path = None
        self.lock = threading.Lock()

        # ROS2 publishers/subscribers
        self.robot_pose_subscription_ = self.create_subscription(PoseStamped, 'robot_pose', self.robot_pose_callback, 1)
        self.path_sub = self.create_subscription(Path, 'global_path', self.path_callback, 10)
        self.cnn_goal_pub = self.create_publisher(Point, 'cnn_goal', 1)
        self.final_goal_pub = self.create_publisher(Point, 'final_goal', 1)
        self.marker_pub = self.create_publisher(Marker, 'cnn_goal_marker', 1)


        self.timer = None

    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose

    def path_callback(self, msg):
        self.get_logger().warning('Received path')
        # with self.lock:
        self.path = msg
        if self.timer is None:
            self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def get_current_pose(self):
        x = np.array([self.robot_pose.position.x, self.robot_pose.position.y])
        q = self.robot_pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (x, theta), [q.x, q.y, q.z, q.w]

    def find_closest_point(self, x, seg=-1):
        pt_min = np.array([np.nan, np.nan])
        dist_min = np.inf
        seg_min = -1

        if self.path is None:
            self.get_logger().warning('No path received yet')
            return (pt_min, dist_min, seg_min)

        if seg == -1:
            for i in range(len(self.path.poses) - 1):
                pt, dist, s = self.find_closest_point(x, i)
                if dist < dist_min:
                    pt_min = pt
                    dist_min = dist
                    seg_min = s
        else:
            p_start = np.array([self.path.poses[seg].pose.position.x, self.path.poses[seg].pose.position.y])
            p_end = np.array([self.path.poses[seg+1].pose.position.x, self.path.poses[seg+1].pose.position.y])

            v = p_end - p_start
            length_seg = np.linalg.norm(v)
            v = v / length_seg

            dist_projected = np.dot(x - p_start, v)

            if dist_projected < 0.:
                pt_min = p_start
            elif dist_projected > length_seg:
                pt_min = p_end
            else:
                pt_min = p_start + dist_projected * v

            dist_min = np.linalg.norm(pt_min - x)
            seg_min = seg

        return (pt_min, dist_min, seg_min)

    def find_goal(self, x, pt, dist, seg):
        goal = None
        end_goal_pos = [self.path.poses[-1].pose.position.x, self.path.poses[-1].pose.position.y]
        end_goal_rot = [self.path.poses[-1].pose.orientation.x,
                        self.path.poses[-1].pose.orientation.y,
                        self.path.poses[-1].pose.orientation.z,
                        self.path.poses[-1].pose.orientation.w]

        if dist > self.lookahead:
            goal = pt
        else:
            seg_max = len(self.path.poses) - 2
            p_end = np.array([self.path.poses[seg+1].pose.position.x, self.path.poses[seg+1].pose.position.y])
            dist_end = np.linalg.norm(x - p_end)

            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = np.array([self.path.poses[seg+1].pose.position.x, self.path.poses[seg+1].pose.position.y])
                dist_end = np.linalg.norm(x - p_end)

            if dist_end < self.lookahead:
                pt = np.array([self.path.poses[seg_max+1].pose.position.x, self.path.poses[seg_max+1].pose.position.y])
            else:
                pt, dist, seg = self.find_closest_point(x, seg)
                p_start = np.array([self.path.poses[seg].pose.position.x, self.path.poses[seg].pose.position.y])
                p_end = np.array([self.path.poses[seg+1].pose.position.x, self.path.poses[seg+1].pose.position.y])
                v = p_end - p_start
                length_seg = np.linalg.norm(v)
                v = v / length_seg

                dist_projected_x = np.dot(x - pt, v)
                dist_projected_y = np.linalg.norm(np.cross(x - pt, v))
                pt = pt + (np.sqrt(self.lookahead**2 - dist_projected_y**2) + dist_projected_x) * v

            goal = pt

        return (goal, end_goal_pos, end_goal_rot)

    def timer_callback(self):
        # with self.lock:
        (pose, theta_quat) = self.get_current_pose()
        x, theta = pose
        if np.isnan(x[0]):
            return

        pt, dist, seg = self.find_closest_point(x)
        if np.isnan(pt).any():
            return

        goal, end_goal_pos, end_goal_rot = self.find_goal(x, pt, dist, seg)
        if goal is None or end_goal_pos is None:
            return

        map_T_robot = np.array([[np.cos(theta), -np.sin(theta), x[0]],
                                [np.sin(theta), np.cos(theta), x[1]],
                                [0, 0, 1]])
        goal_h = np.array([[goal[0]], [goal[1]], [1]])
        goal_local = np.linalg.inv(map_T_robot) @ goal_h
        goal_local = goal_local[0:2]

        relative_goal = np.linalg.inv(map_T_robot) @ np.array([[end_goal_pos[0]], [end_goal_pos[1]], [1]])

        orientation_to_target = quaternion_multiply(end_goal_rot,
                                                    quaternion_inverse(theta_quat))
        yaw = euler_from_quaternion(orientation_to_target)[2]

        # Publier cnn_goal
        cnn_goal = Point()
        cnn_goal.x = float(goal_local[0])
        cnn_goal.y = float(goal_local[1])
        cnn_goal.z = 0.0
        if not np.isnan(cnn_goal.x) and not np.isnan(cnn_goal.y):
            self.cnn_goal_pub.publish(cnn_goal)

        # Publier final_goal
        final_goal = Point()
        final_goal.x = float(relative_goal[0])
        final_goal.y = float(relative_goal[1])
        final_goal.z = float(yaw)
        if not np.isnan(final_goal.x) and not np.isnan(final_goal.y):
            self.final_goal_pub.publish(final_goal)

        # Publier un Marker pour visualiser cnn_goal
        marker = Marker()
        marker.header.frame_id = self.prefix + "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cnn_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = cnn_goal.x
        marker.pose.position.y = cnn_goal.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

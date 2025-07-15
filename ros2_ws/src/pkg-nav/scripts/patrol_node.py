#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from sensor_msgs.msg import Joy
import math
import time
from collections import deque
import threading


def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.patrol_points = deque()
        self.robot_pose = None
        self.reach_tolerance = 0.5  # meters
        self.waiting_for_goal = False
        self.active_patrol = False

        self.declare_parameter('goal_topic', '/global_goal')
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value

        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(Pose, '/robot_pose', self.robot_pose_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10) 
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.timer = self.create_timer(1.0, self.update)

        # Thread pour attendre l'entrée utilisateur sans bloquer ROS
        threading.Thread(target=self.wait_for_start_key, daemon=True).start()

    def wait_for_start_key(self):
        print("\n📍 Cliquez les points dans Rviz (/clicked_point)")
        input("▶️  Appuyez sur Entrée pour démarrer la patrouille...\n")
        self.active_patrol = True
        self.get_logger().info("🚶‍♂️Patrouille démarrée")

    def clicked_point_callback(self, msg: PointStamped):
        self.patrol_points.append(msg.point)
        self.get_logger().info(f'🟢 Point ajouté : ({msg.point.x:.2f}, {msg.point.y:.2f})')

    def robot_pose_callback(self, msg: Pose):
        self.robot_pose = msg.position

    def joy_callback(self, msg: Joy):
        # Index du bouton Start sur une manette ROS standard (souvent bouton 7)
        START_BUTTON_INDEX = 7
        if len(msg.buttons) > START_BUTTON_INDEX and msg.buttons[START_BUTTON_INDEX] == 1:
            self.get_logger().info("🎮 Bouton START pressé : lancement de la patrouille")
            self.active_patrol = True

    def update(self):
        if not self.active_patrol:
            return
        if not self.robot_pose or not self.patrol_points:
            return

        current_goal = self.patrol_points[0]

        if not self.waiting_for_goal:
            self.send_goal(current_goal)
            self.waiting_for_goal = True

        if distance(self.robot_pose, current_goal) <= self.reach_tolerance:
            self.get_logger().info(f"✅ Point atteint : ({current_goal.x:.2f}, {current_goal.y:.2f})")
            self.patrol_points.rotate(-1)
            self.waiting_for_goal = False
            time.sleep(1.0)

    def send_goal(self, point):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position = point
        goal_msg.pose.orientation.w = 1.0  # neutre
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"🎯 Envoi objectif : ({point.x:.2f}, {point.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

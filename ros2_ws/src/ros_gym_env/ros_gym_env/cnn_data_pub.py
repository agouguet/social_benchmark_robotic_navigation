#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cnn_msgs.msg import CNNdata
from geometry_msgs.msg import Point, Twist
# from pedsim_msgs.msg import TrackedPersons
from sensor_msgs.msg import LaserScan

import numpy as np

NUM_TP = 10

class CnnDataNode(Node):
    def __init__(self, prefix = ""):
        super().__init__('cnn_data_node_'+prefix)

        # Buffers internes
        self.ped_pos_map = None
        self.ped_pos_map_tmp = np.zeros((2,80,80))
        self.scan_tmp = np.zeros(720, dtype=np.float32)
        self.scan_all_tmp = np.zeros(1080, dtype=np.float32)
        self.scan_buffer = []
        self.goal_cart = np.zeros(2)
        self.vel = np.zeros(2)
        self.ts_cnt = 0

        # Subscriptions
        # self.create_subscription(TrackedPersons, '/track_ped', self.ped_callback, 10)
        self.create_subscription(LaserScan, prefix + '/scan', self.scan_callback, 10)
        self.create_subscription(Point, prefix + '/local_goal', self.goal_callback, 10)
        self.create_subscription(Twist, prefix + '/cmd_vel', self.vel_callback, 10)

        # Publisher
        self.pub = self.create_publisher(CNNdata, prefix + '/cnn_data', 1)

        # Timer à 20 Hz
        timer_period = 1.0 / 20.0
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('CnnDataNode lancé')

    # def ped_callback(self, msg: TrackedPersons):
    #     self.ped_pos_map_tmp = np.zeros((2,80,80))
    #     for ped in msg.tracks:
    #         x = ped.pose.pose.position.x
    #         y = ped.pose.pose.position.y
    #         vx = ped.twist.twist.linear.x
    #         vy = ped.twist.twist.linear.y
    #         if 0 <= x <= 20 and abs(y) <= 10:
    #             c = int(np.floor(-(y - 10) / 0.25))
    #             r = int(np.floor(x / 0.25))
    #             r = min(r,79)
    #             c = min(c,79)
    #             self.ped_pos_map_tmp[0, r, c] = vx
    #             self.ped_pos_map_tmp[1, r, c] = vy

    def scan_callback(self, msg: LaserScan):
        scan = np.array(msg.ranges, dtype=np.float32)
        scan[np.isnan(scan)] = 0.0
        scan[np.isinf(scan)] = 0.0
        # self.scan_tmp = scan[180:900]
        self.scan_tmp = scan[0:720]
        self.scan_all_tmp = scan

    def goal_callback(self, msg: Point):
        self.goal_cart[0] = msg.x
        self.goal_cart[1] = msg.y

    def vel_callback(self, msg: Twist):
        self.vel[0] = msg.linear.x
        self.vel[1] = msg.angular.z

    def timer_callback(self):
        if self.ped_pos_map is None:
            self.ped_pos_map = self.ped_pos_map_tmp.copy()
        else:
            self.ped_pos_map = np.copy(self.ped_pos_map_tmp)
        
        self.scan_buffer.append(self.scan_tmp.copy())
        self.ts_cnt += 1

        if self.ts_cnt >= NUM_TP:
            msg = CNNdata()
            # conversion en listes
            msg.ped_pos_map = [float(v) for v in self.ped_pos_map.flatten()]
            msg.scan = [float(v) for arr in self.scan_buffer for v in arr.tolist()]
            msg.scan_all = self.scan_all_tmp.tolist()
            msg.depth = []
            msg.image_gray = []
            msg.goal_cart = self.goal_cart.tolist()
            msg.goal_final_polar = []
            msg.vel = self.vel.tolist()
            self.pub.publish(msg)

            # maintien de la fenêtre glissante
            self.scan_buffer = self.scan_buffer[1:]
            self.ts_cnt = NUM_TP - 1

def main(args=None):
    rclpy.init(args=args)
    node = CnnDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

def multiple_env():
    rclpy.init()

    num_envs = 8
    nodes = []

    for i in range(num_envs):
        node = CnnDataNode("env_" + str(i))
        nodes.append(node)

    # Lancer tous les nœuds dans un MultiThreadedExecutor
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    executor.spin()

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    multiple_env()
    # main()

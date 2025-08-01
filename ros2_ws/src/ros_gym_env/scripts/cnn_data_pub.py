#!/usr/bin/env python3
import rclpy
from math import cos, sin, atan2, sqrt
from rclpy.node import Node
from cnn_msgs.msg import CNNdata, MyCNNdata, AllCNNdata
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
# from pedsim_msgs.msg import TrackedPersons
from agents_msgs.msg import AgentArray, Agent
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

import numpy as np

NUM_TP = 10

class CnnDataNode(Node):
    def __init__(self):
        super().__init__('cnn_data_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 20.0)
            ]
        )

        self.frequency = self.get_parameter('frequency').value

        # Buffers internes
        # self.agents = []
        self.curr_pose = None
        self.vel = np.zeros(2)
        self.scan_tmp = np.zeros(720, dtype=np.float32)
        self.scan_all_tmp = np.zeros(1080, dtype=np.float32)
        self.scan_buffer = []

        self.final_goal = PoseStamped()
        self.local_goal_from_robot = Point()
        self.local_goal_from_map = Point()
        self.goal_cart = np.zeros(2)
        
        self.agents = AgentArray()
        self.ped_pos_map = None
        self.ped_pos_map_tmp = np.zeros((2,80,80))

        self.ts_cnt = 0

        # Subscriptions
        self.create_subscription(PoseStamped, 'robot_pose', self._robot_pose_callback, 1)
        self.create_subscription(AgentArray, 'agents', self.agents_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Point, 'local_goal', self.goal_callback, 10)
        self.create_subscription(Point, 'local_goal_from_map', self.local_goal_from_map_callback, 1)
        self.create_subscription(Point, 'local_goal_from_robot', self.local_goal_from_robot_callback, 1)
        self.create_subscription(PoseStamped, 'global_goal', self._final_goal_callback, 1)
        self.create_subscription(Twist, 'smooth_cmd_vel', self.vel_callback, 10)

        # Publisher
        self.pub = self.create_publisher(AllCNNdata, 'cnn_data', 1)

        # Timer à 20 Hz
        timer_period = 1.0 / self.frequency
        self.create_timer(timer_period, self.timer_callback)

        # self.get_logger().info('CnnDataNode lancé')

    def _robot_pose_callback(self, robot_pose_msg):
        self.curr_pose = robot_pose_msg

    def agents_callback(self, msg: AgentArray):
        self.agents = msg

        # Position robot pour les calculs relatifs
        if self.curr_pose is None:
            return
        else:
            robot_x = self.curr_pose.pose.position.x
            robot_y = self.curr_pose.pose.position.y
            orientation_q = self.curr_pose.pose.orientation
            _, _, robot_yaw = euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])

            agents = []

            for i in range(len(msg.agents)):
                agent = msg.agents[i]
                
                x = agent.pose.position.x
                y = agent.pose.position.y
                vx = agent.velocity.linear.x
                vy = agent.velocity.linear.y

                dx = x - robot_x
                dy = y - robot_y

                dx_local = cos(-robot_yaw) * dx - sin(-robot_yaw) * dy
                dy_local = sin(-robot_yaw) * dx + cos(-robot_yaw) * dy

                vx_local = cos(-robot_yaw) * vx - sin(-robot_yaw) * vy
                vy_local = sin(-robot_yaw) * vx + cos(-robot_yaw) * vy
                # dist = sqrt(dx**2 + dy**2)

                agent_from_robot = Agent()
                agent_from_robot.header = msg.header
                agent_from_robot.id = i
                agent_from_robot.pose.position.x = dx_local
                agent_from_robot.pose.position.y = dy_local
                agent_from_robot.velocity.linear.x = vx_local
                agent_from_robot.velocity.linear.y = vy_local
                agent_from_robot.visible_by_robot = True
                agents.append(agent_from_robot)
            self.agents = AgentArray()
            self.agents.header = msg.header
            self.agents.agents = agents

    
    def local_goal_from_map_callback(self, msg):
        self.local_goal_from_map = msg
        
    def local_goal_from_robot_callback(self, msg):
        self.local_goal_from_robot = msg

    def _final_goal_callback(self, final_goal_msg):
        self.final_goal = final_goal_msg

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

        if self.curr_pose is None:
            return
        
        self.scan_buffer.append(self.scan_tmp.copy())
        self.ts_cnt += 1

        if self.ts_cnt >= NUM_TP:
            msg = AllCNNdata()
            msg.robot_pose = self.curr_pose
            msg.vel = self.vel.tolist()
            msg.scan = [float(v) for arr in self.scan_buffer for v in arr.tolist()]
            msg.scan_all = self.scan_all_tmp.tolist()

            msg.global_goal = self.final_goal
            msg.local_goal_from_map = self.local_goal_from_map
            msg.local_goal_from_robot = self.local_goal_from_robot
            msg.goal_cart = self.goal_cart.tolist()
            msg.goal_final_polar = []
            
            msg.agents = self.agents 
            msg.ped_pos_map = [float(v) for v in self.ped_pos_map.flatten()]
            
            msg.depth = []
            msg.image_gray = []
            
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

if __name__ == '__main__':
    main()

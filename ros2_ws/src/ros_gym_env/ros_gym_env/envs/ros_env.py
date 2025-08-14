#!/usr/bin/env python3
from abc import ABC, abstractmethod
import subprocess
import time, math, random
from math import cos, sin, atan2, sqrt
import rclpy
from rclpy.node import Node
import gymnasium as gym
import numpy as np
from simulation_msgs.srv import Reset, PausePlay
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from cnn_msgs.msg import CNNdata, MyCNNdata, AllCNNdata
from agents_msgs.msg import AgentArray, Agent
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion


import os
import numpy as np
import matplotlib.pyplot as plt

class ROSEnv(gym.Env, ABC):
    def __init__(self, env_id, config, env_id_display_log=None):
        super().__init__()
        self.config = config
        self.curriculum_level = 0
        
        # env parameters:
        self.env_id = env_id
        self.env_id_display_log = env_id_display_log
        self.prefix = "/env_" + str(self.env_id)
        self.max_iteration = self.config.env.max_iteration
        self.max_time = self.config.env.max_time

        # robot parameters:
        self.robot_radius = self.config.env.robot.robot_radius
        self.min_linear_velocity = self.config.env.robot.min_linear_velocity
        self.max_linear_velocity = self.config.env.robot.max_linear_velocity
        self.min_angular_velocity = self.config.env.robot.min_angular_velocity
        self.max_angular_velocity = self.config.env.robot.max_angular_velocity

        # scan
        self.raw_last_scan = None
        self.scan_history = self.config.env.obs.scan_history
        self.scan_buffer = []

        self.num_iterations = 0
        
        self.info = {}
        # episode done flag:
        self._episode_done = False
        # reset flag:
        self._reset = True
        self.curr_pose = None

        self.ros_debug = self.config.log.ros
        
        # === ROS ===
        self.node = Node("ros_env_" + str(self.env_id))

        self.node.create_subscription(PoseStamped, self.prefix + '/robot_pose', self._robot_pose_callback, 1)
        self.node.create_subscription(AgentArray, self.prefix + '/agents', self.agents_callback, 10)
        self.node.create_subscription(LaserScan, self.prefix + '/scan', self.scan_callback, 10)
        self.node.create_subscription(Path, self.prefix + '/global_path', self.global_path_callback, 10)
        self.node.create_subscription(Point, self.prefix + '/local_goal', self.goal_callback, 10)
        self.node.create_subscription(Point, self.prefix + '/local_goal_from_map', self.local_goal_from_map_callback, 1)
        self.node.create_subscription(Point, self.prefix + '/local_goal_from_robot', self.local_goal_from_robot_callback, 1)
        self.node.create_subscription(PoseStamped, self.prefix + '/global_goal', self._final_goal_callback, 1)
        self.node.create_subscription(Twist, self.prefix + '/smooth_cmd_vel', self.vel_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, self.prefix + self.config.env.ros.cmd_vel, 10)

        if self.ros_debug:
            self.debug_scan_pub = self.node.create_publisher(LaserScan, f"{self.prefix}/debug/scan", 10)

        # === Services ===
        self.reset_client = self.node.create_client(Reset, self.prefix + self.config.env.ros.reset_service)
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Attente du service de reset...')
        
        self.play_client = self.node.create_client(PausePlay, self.prefix + self.config.env.ros.play_service)
        while not self.play_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Attente du service de play...')

        self.set_action_space()
        self.set_observation_space()

        req = PausePlay.Request()
        req.play = True
        future = self.play_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    # === ROS ===

    def _robot_pose_callback(self, robot_pose_msg):
        self.curr_pose = robot_pose_msg.pose

    def agents_callback(self, msg: AgentArray):

        # Position robot pour les calculs relatifs
        if self.curr_pose is None:
            robot_x, robot_y = 0.0, 0.0
            robot_yaw = 0.0
        else:
            robot_x = self.curr_pose.position.x
            robot_y = self.curr_pose.position.y
            orientation_q = self.curr_pose.orientation
            _, _, robot_yaw = euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])

        self.agents = [(0.0, 0.0, 0.0, 0.0, 0.0) for i in range(self.human_number)]

        for i in range(len(msg.agents)):
            if i >= self.human_number:
                break

            agent = msg.agents[i]
            
            x = agent.pose.position.x
            y = agent.pose.position.y
            vx = agent.velocity.linear.x
            vy = agent.velocity.linear.y

            dx = x - robot_x
            dy = y - robot_y
            # Rotation dans le repère local du robot (x: avant/arrière, y: gauche/droite)
            dx_local = cos(-robot_yaw) * dx - sin(-robot_yaw) * dy
            dy_local = sin(-robot_yaw) * dx + cos(-robot_yaw) * dy

            vx_local = cos(-robot_yaw) * vx - sin(-robot_yaw) * vy
            vy_local = sin(-robot_yaw) * vx + cos(-robot_yaw) * vy
            dist = math.sqrt(dx**2 + dy**2)

            self.agents[i] = (dx_local, dy_local, vx, vy, dist)
            # self.agents[i] = (dx_local, dy_local, vx_local, vy_local, dist)

    def global_path_callback(self, msg):
        self.global_path = msg

    def local_goal_from_map_callback(self, msg):
        self.local_goal_from_map = msg
        
    def local_goal_from_robot_callback(self, msg):
        self.local_goal_from_robot = msg

    def _final_goal_callback(self, final_goal_msg):
        self.final_goal = final_goal_msg.pose.position

    def scan_callback(self, msg: LaserScan):
        self.raw_last_scan = msg
        scan = np.array(msg.ranges, dtype=np.float32)
        scan[np.isnan(scan)] = msg.range_max
        scan[np.isinf(scan)] = msg.range_max
        self.scan_buffer.append(scan.copy())
        if len(self.scan_buffer) >= self.scan_history:
            self.scan = [float(v) for arr in self.scan_buffer for v in arr.tolist()]
            self.scan_buffer = self.scan_buffer[1:]
        
        # scan_tmp = scan[0:720]
        # self.scan_buffer.append(scan_tmp.copy())
        # if len(self.scan_buffer) >= self.scan_history:
        #     self.scan = [float(v) for arr in self.scan_buffer for v in arr.tolist()]
        #     self.scan_buffer = self.scan_buffer[1:]

    def goal_callback(self, msg: Point):
        self.goal = msg

    def vel_callback(self, msg: Twist):
        self.curr_vel = msg
    

    # =========


    def set_observation_space(self):
        use_goal = self.config.env.obs.goal
        use_goal_dist = self.config.env.obs.goal_dist
        use_scan = self.config.env.obs.scan
        use_robot_velocity = self.config.env.obs.robot_velocity
        use_human = self.config.env.obs.human

        self.scan_size = self.config.env.obs.scan_dim
        self.nb_slice = self.config.env.obs.scan_slice
        self.scan_history = self.config.env.obs.scan_history
        self.scan_tile = self.config.env.obs.scan_tile
        self.scan_obs_size = int(self.scan_size*self.scan_history*self.scan_tile)
        size = int(self.config.env.obs.scan_avg_pool) + int(self.config.env.obs.scan_min_pool)
        if size > 0:
            self.scan_obs_size = int((self.scan_size / self.nb_slice) * self.scan_history * self.scan_tile * size)


        self.human_number = self.config.env.obs.human_number
        self.human_obs_size = self.human_number * 5

        obs_shape = 0
        if use_goal:
            obs_shape += 1 #2
        if use_goal_dist:
            obs_shape += 1
        if use_robot_velocity:
            obs_shape += 2
        if use_scan:
            obs_shape += self.scan_obs_size
        if use_human:
            obs_shape += self.human_obs_size

        print(obs_shape)
        self.observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(obs_shape,), dtype=np.float32)

    def set_action_space(self):
        self.high_action = np.array([1, 1])
        self.low_action = np.array([-1, -1])
        if self.config.env.action.discrete:
            import itertools

            self.levels_lin = self.config.env.action.discrete_level_linear   # ex: -1.0, 0.0, 1.0
            self.levels_ang = self.config.env.action.discrete_level_angular   # ex: -1.0, 0.0, 1.0
            lin_vals = np.linspace(self.low_action[0], self.high_action[0], self.levels_lin)
            ang_vals = np.linspace(self.low_action[1], self.high_action[1], self.levels_ang)
            self.actions_list = [np.array(a) for a in itertools.product(lin_vals, ang_vals)]
            self.action_space = gym.spaces.Discrete(len(self.actions_list))
        else:
            self.action_space = gym.spaces.Box(low=self.low_action, high=self.high_action, dtype=np.float32)

    def reset(self, *, seed=None, options=None, **kwargs):
        if seed is not None:
            np.random.seed(seed)
        self._set_init()
        obs = self._get_observation()
        info = self._post_information(False)
        return obs, info
    
    def _set_init(self):
        self.node.get_logger().warning("Start initializing robot...", once=True)
        self.cmd_vel_publisher.publish(Twist())

        self.num_iterations = 0
        self.reward_episode = 0
        self._episode_done = False
        self.start_time = self.get_time()

        if(self._reset): 
            self._reset = False
            req = Reset.Request()
            req.level = self.curriculum_level
            future = self.reset_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

        self.agents = None
        self.scan = None
        self.goal = None
        self.mht_peds = None
        self.curr_pose = None
        self.curr_vel = Twist()
        self.info = {}
        self.final_goal = None
        self.local_goal_from_map = None
        self.local_goal_from_robot = None

        while self.curr_pose == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.final_goal == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.local_goal_from_map == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.local_goal_from_robot == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.scan == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)

        self.init_info()

        # Give the system a little time to finish initialization
        self.node.get_logger().fatal("Finish initialize robot.", once=True)

    def send_action(self, action):
        cmd_vel = Twist()
        cmd_vel.linear.x = (float(action[0]) + 1) * (self.max_linear_velocity - self.min_linear_velocity) / 2 + self.min_linear_velocity
        cmd_vel.angular.z = (float(action[1]) + 1) * (self.max_angular_velocity - self.min_angular_velocity) / 2 + self.min_angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel)

    def step(self, action):
        self.num_iterations += 1

        step_time = time.time()
        time_tmp = time.time()
        if self.config.env.action.discrete:
            action = self.actions_list[action]
        self.send_action(action)
        action_time = time.time() - time_tmp

        time_tmp = time.time()
        rclpy.spin_once(self.node, timeout_sec=0.05)
        spin_time = time.time() - time_tmp

        time_tmp = time.time()
        obs = self._get_observation()
        obs_time = time.time() - time_tmp

        time_tmp = time.time()
        reward = self._compute_reward()
        self.reward_episode += reward
        reward_time = time.time() - time_tmp

        done = self._is_done(reward)
        self._episode_done = done
        self._reset = done
        truncated = False

        info = self._post_information(done)

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            if done:
                self.node.get_logger().fatal(f"🪙 Episode Reward : {self.reward_episode}")

        return obs, reward, done, truncated, info

    @abstractmethod
    def init_info(self):
        pass

    @abstractmethod
    def _is_done(self, reward):
        """
        Returns True if self._episode_done
        """
        pass

    @abstractmethod
    def _get_observation(self):
        """
        Returns the observation.
        """
        pass
    
    @abstractmethod
    def _compute_reward(self):
        """
        Calculates the reward to give based on the observations given.
        """
        pass

    @abstractmethod
    def _post_information(self, done):
        pass


    def publish_debug_scan_observation(self, scan):
        scan_msg = LaserScan()
        scan_ranges = scan[-self.scan_size:].tolist()

        # === Entête ===
        if self.raw_last_scan is not None:
            scan_msg.header = self.raw_last_scan.header
        else:
            scan_msg.header.stamp = self.node.get_clock().now().to_msg()
            scan_msg.header.frame_id = "env_" + str(self.env_id)+"/laser_link"

        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(scan_ranges)

        scan_msg.range_min = 0.0
        scan_msg.range_max = 10.0

        # === Données ===
        scan_msg.ranges = scan_ranges

        self.debug_scan_pub.publish(scan_msg)

    def dist_to_goal(self):
        total_distance = 0.0
        poses = self.global_path.poses

        for i in range(len(poses) - 1):
            x1 = poses[i].pose.position.x
            y1 = poses[i].pose.position.y
            z1 = poses[i].pose.position.z

            x2 = poses[i+1].pose.position.x
            y2 = poses[i+1].pose.position.y
            z2 = poses[i+1].pose.position.z

            # Distance Euclidienne entre deux points
            dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            total_distance += dist

        return total_distance

    def on_policy_update_start(self):
        req = PausePlay.Request()
        req.play = True
        future = self.play_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    def on_policy_update_end(self):
        req = PausePlay.Request()
        req.play = False
        future = self.play_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    def close(self):
        self.node.destroy_node()

    def render(self, mode='human'):
        pass

    def get_time(self):
        return self.node.get_clock().now().nanoseconds * 1e-9
    
    def set_curriculum_level(self, level):
        self.curriculum_level = level
        # self.max_iteration = self.config.env.max_iteration * (self.curriculum_level+1)
        self.node.get_logger().info(f"🎯 Changement de niveau de curriculum : {level}")

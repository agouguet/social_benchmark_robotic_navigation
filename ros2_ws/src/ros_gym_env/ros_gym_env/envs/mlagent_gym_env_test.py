#!/usr/bin/env python3
import subprocess
import time, math
import random
import signal
import atexit
import threading
import rclpy
from rclpy.node import Node
import gymnasium as gym
import numpy as np
import numpy.matlib
from simulation_msgs.srv import Reset, Trigger, PausePlay
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from cnn_msgs.msg import CNNdata, MyCNNdata
from threading import Event
from tf_transformations import euler_from_quaternion


class MlAgentGymEnvTest(gym.Env):
    def __init__(self, env_id, config, env_id_display_log=None):
        super().__init__()
        self.config = config
        self.curriculum_level = 0
        
        self.env_id = env_id
        self.env_id_display_log = env_id_display_log
        self.prefix = "/env_" + str(self.env_id)
        self.max_iteration = self.config.env.max_iteration #* (self.curriculum_level+1)
        self.max_time = self.config.env.max_time
        

        # robot parameters:
        self.robot_radius = self.config.env.robot.robot_radius
        self.goal_radius = self.config.env.robot.goal_radius
        self.dist_goal_history_number = self.config.env.robot.dist_goal_history_number
        self.min_linear_velocity = self.config.env.robot.min_linear_velocity
        self.max_linear_velocity = self.config.env.robot.max_linear_velocity

        self.min_angular_velocity = self.config.env.robot.min_angular_velocity
        self.max_angular_velocity = self.config.env.robot.max_angular_velocity

        # bumper:
        self.bump_flag = False
        self.bump_num = 0

        # reward:
        self.dist_to_goal_reg = np.zeros(self.dist_goal_history_number)
        self.init_distance = 0
        self.reset_dist_to_goal_reg = True
        self.num_iterations = 0
        

        # Initialisation Node
        self.node = Node("ros_env_" + str(self.env_id))

        self.start_time = self.get_time()

        # === Spaces ===
        self.scan_size = self.config.env.obs.scan_dim
        self.nb_slice = self.config.env.obs.scan_slice
        self.scan_history = self.config.env.obs.scan_history
        self.scan_tile = self.config.env.obs.scan_tile
        self.scan_obs_size = int((self.scan_size/self.nb_slice)*self.scan_history*2*self.scan_tile)

        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)                                 # Goal
        self.observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(self.scan_obs_size + 5,), dtype=np.float32)                      # Scan

        # observation space
        self.cnn_data = CNNdata()
        self.ped_pos = []
        self.scan = []
        self.goal = []
        self.mht_peds = []

        # info, initial position and goal position
        self.final_goal = Point()
        self.local_goal_from_map = None
        self.local_goal_from_robot = None
        
        self.init_pose = Pose()
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.info = {}
        # episode done flag:
        self._episode_done = False
        # goal reached flag:
        self._goal_reached = False
        # reset flag:
        self._reset = True

        # === ROS topics ===
        self.node.create_subscription(Odometry, self.prefix + self.config.env.ros.robot_odom, self._robot_vel_callback, 1)
        self.node.create_subscription(PoseStamped, self.prefix + self.config.env.ros.robot_pose, self._robot_pose_callback, 1)
        self.node.create_subscription(MyCNNdata, self.prefix + self.config.env.ros.cnn_data, self._cnn_data_callback, 1)
        self.node.create_subscription(PoseStamped, self.prefix + self.config.env.ros.global_goal, self._final_goal_callback, 1)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, self.prefix + self.config.env.ros.cmd_vel, 10)

        self.obs = None
        self.done = False

        # === Services ===
        self.reset_client = self.node.create_client(Trigger, self.prefix + self.config.env.ros.reset_service)
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Attente du service de reset...')


    def reset(self, *, seed=None, options=None, **kwargs):
        """ 
        obs, info = env.reset() 
        """
        if seed is not None:
            np.random.seed(seed)
        self._set_init()
        obs = self._get_observation()
        info = self._post_information()
        return obs, info
    
    def _set_init(self):
        self.node.get_logger().warning("Start initializing robot...", once=True)
        # reset the robot velocity to 0:
        self.cmd_vel_publisher.publish(Twist())

        #self._reset = True
        # reset simulation to orignal:
        t = time.time()
        if(self._reset): 
            self._reset = False
            req = Trigger.Request()
            future = self.reset_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

        # self.node.get_logger().fatal("Time 1 {}".format(time.time() - t))

            

        # initalize info:
        self.cnn_data = None
        self.ped_pos = []
        self.scan = []
        self.goal = []
        self.mht_peds = []

        # info, initial position and goal position
        self.init_pose = Pose()
        self.curr_pose = None
        self.curr_vel = None
        self.info = {}
        self.final_goal = None
        self.local_goal_from_map = None
        self.local_goal_from_robot = None

        # self.init_pose = self.curr_pose # inital_pose.pose.pose
        # self.curr_pose = self.curr_pose # inital_pose.pose.pose
        # self.node.get_logger().warning("Robot was initiated as {}".format(self.init_pose), once=True)

        # reset pose valid flag:
        self.pos_valid_flag = True
        # reset bumper:
        self.bump_flag = False
        self.bump_num = 0
        # reset the number of iterations:
        self.num_iterations = 0
        self.start_time = self.get_time()
        # reset distance to goal register:
        # self.dist_to_goal_reg = np.zeros(self.dist_goal_history_number)
        self.reset_dist_to_goal_reg = True
        # reset episode done flag:
        self._episode_done = False

        while self.cnn_data == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.curr_pose == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.curr_vel == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.final_goal == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.local_goal_from_map == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        while self.local_goal_from_robot == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)

        dist_to_goal = np.linalg.norm(
            np.array([
                self.local_goal_from_robot.x,
                self.local_goal_from_robot.y
            ])
        )
        self.dist_to_goal_reg = np.ones(10)*dist_to_goal
        self.init_distance = dist_to_goal


        # Give the system a little time to finish initialization
        self.node.get_logger().fatal("Finish initialize robot.", once=True)
        
        return self.init_pose, self.goal
        
    def _robot_vel_callback(self, robot_vel_msg):
        self.curr_vel = robot_vel_msg.twist.twist

    def _robot_pose_callback(self, robot_pose_msg):
        self.curr_pose = robot_pose_msg.pose

    def _cnn_data_callback(self, cnn_data_msg):
        self.cnn_data = cnn_data_msg
        self.goal = self.cnn_data.goal_cart
        self.local_goal_from_robot = self.cnn_data.local_goal_from_robot
        if (self.local_goal_from_map is None):
            self.local_goal_from_map = self.cnn_data.local_goal_from_map
            return
        
        dist = np.linalg.norm(
            np.array([
            self.local_goal_from_map.x - self.cnn_data.local_goal_from_map.x,
            self.local_goal_from_map.y - self.cnn_data.local_goal_from_map.y,
            ])
        )
        if (dist < 0.2):
            return
        self.local_goal_from_map = self.cnn_data.local_goal_from_map
        # self.dist_to_goal_reg = np.ones(10)*dist
        
    # def local_goal_from_map_callback(self, msg):
    #     if (msg != self.local_goal_from_map):
    #         self.local_goal_from_map = msg
    #         self.reset_dist_to_goal_reg = True
        

    # def local_goal_from_robot_callback(self, msg):
    #     self.node.get_logger().fatal("local_goal_from_robot_callback")
    #     self.local_goal_from_robot = msg

    def _final_goal_callback(self, final_goal_msg):
        self.final_goal = final_goal_msg.pose.position

    def send_action(self, action):
        cmd_vel = Twist()

        cmd_vel.linear.x = (float(action[0]) + 1) * (self.max_linear_velocity - self.min_linear_velocity) / 2 + self.min_linear_velocity
        cmd_vel.angular.z = (float(action[1]) + 1) * (self.max_angular_velocity - self.min_angular_velocity) / 2 + self.min_angular_velocity

        self.cmd_vel_publisher.publish(cmd_vel)

    def step(self, action):
        self.send_action(action)

        rclpy.spin_once(self.node, timeout_sec=0.05)

        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._is_done(reward)
        truncated = False
        return obs, reward, done, truncated, {}


    def _is_done(self, reward):
        """
        Checks if end of episode is reached. It is reached,
        if the goal is reached,
        if the robot collided with obstacle
        if the reward function returns a high negative value.
        if maximum number of iterations is reached,
        :param current state
        :return: True if self._episode_done
        """
        # self.node.get_logger().warning("Is Done Method")
        # updata the number of iterations:
        self.num_iterations += 1

        # 1) Goal reached?
        # distance to goal:
        dist_to_goal = np.linalg.norm(
            np.array([
            self.curr_pose.position.x - self.final_goal.x,
            self.curr_pose.position.y - self.final_goal.y,
            ])
        )

        if(dist_to_goal <= self.goal_radius):
            # reset the robot velocity to 0:
            self.cmd_vel_publisher.publish(Twist())
            self._episode_done = True
            self._reset = True # reset the simulation world
            return True

        # 2) Obstacle collision?
        scan = self.cnn_data.scan[-self.scan_size:]
        min_scan_dist = np.amin(scan[scan!=0])
        #if(self.bump_flag == True): #or self.pos_valid_flag == False):
        if(min_scan_dist <= self.robot_radius and min_scan_dist >= 0.02):
            self.bump_num += 1

        # stop and reset if more than 5 collisions: 
        # self.node.get_logger().fatal("BUMP NUM  {}  {}  {}".format(min_scan_dist, max_scan_dist, self.bump_num))
        if(self.bump_num >= 3):
            # reset the robot velocity to 0:
            self.cmd_vel_publisher.publish(Twist())
            self.bump_num = 0
            self._episode_done = True
            self._reset = True # reset the simulation world
            return True


        # 4) maximum number of iterations?
        if(self.num_iterations > self.max_iteration):
        # self.node.get_logger().fatal("Time  {}  {}  {}".format(self.start_time, self.get_time(), self.max_time))
        # if (self.start_time < self.get_time() - self.max_time):
            # reset the robot velocity to 0:
            self.cmd_vel_publisher.publish(Twist())
            self._episode_done = True
            self._reset = True
            return True

        return False

    def _get_observation(self):
        """
        Returns the observation.
        """
        self.scan = self.cnn_data.scan
        self.goal = self.cnn_data.goal_cart
        self.vel = self.cnn_data.vel
        
        # ped map:
        # MaxAbsScaler:
        v_min = -2
        v_max = 2
        self.ped_pos = np.asarray(self.ped_pos, dtype=np.float32)
        self.ped_pos = 2 * (self.ped_pos - v_min) / (v_max - v_min) + (-1)

        # scan map:
        # MaxAbsScaler:
        size_slice = int(self.scan_size/self.nb_slice)
        temp = np.array(self.scan, dtype=np.float32)
        temp = temp.reshape(self.scan_history, self.scan_size, 1)
        temp = temp.reshape(self.scan_history, size_slice, self.nb_slice)
        scan_min = np.min(temp, axis=2)
        scan_mean = np.mean(temp, axis=2)
        scan_avg = np.stack((scan_min, scan_mean), axis=1).reshape(2*self.scan_history, size_slice)
        scan_avg = scan_avg.reshape(2*self.scan_history*size_slice)
        self.scan = np.tile(scan_avg, self.scan_tile)
        s_min = 0
        s_max = 10
        self.scan = np.clip(self.scan, 0.0, 10.0)
        self.scan = 2 * (self.scan - s_min) / (s_max - s_min) + (-1)
        
        # goal:
        goal = np.array([
                            self.local_goal_from_robot.x,
                            self.local_goal_from_robot.y
                        ], dtype=np.float32)
        dist_to_goal = np.linalg.norm(
            np.array([
                self.curr_pose.position.x - goal[0],
                self.curr_pose.position.y - goal[1],
            ])
        )

        # observation:
        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("\n\n", throttle_duration_sec=self.config.log.throttle_duration)
            self.node.get_logger().warning("Goal pos/dist: {} / {}".format(goal, dist_to_goal), throttle_duration_sec=self.config.log.throttle_duration)
        
        self.observation = np.concatenate((self.scan, goal, dist_to_goal, self.vel), axis=None)

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Observation Shape = {}".format(self.observation.shape), throttle_duration_sec=self.config.log.throttle_duration)
        
        return self.observation
    
    def _post_information(self):
        """
        Return:
        info: {"init_pose", "goal", "current_pose"}
        """
        self.info = {
            "initial_pose": self.init_pose,
            "goal": self.goal,
            "current_pose": self.curr_pose
            }
        
        return self.info

    def close(self):
        self.node.destroy_node()

    def render(self, mode='human'):
        pass  # ou afficher des infos de debug











    def _compute_reward(self):
        """Calculates the reward to give based on the observations given.
        """
        # reward parameters:
        r_arrival = 20 #15
        r_waypoint = 5.0 #3.2 #2.5 #1.6 #2 #3 #1.6 #6 #2.5 #2.5
        r_collision = -20 #-15
        r_scan = -0.2 #-0.15 #-0.3
        r_angle = 0.2 #0.6 #0.5 #1 #0.8 #1 #0.5
        r_rotation = -0.1 #-0.15 #-0.4 #-0.5 #-0.2 # 0.1

        angle_thresh = np.pi/6
        w_thresh = 1 # 0.7

        # reward parts:
        r_g = self._goal_reached_reward(r_arrival, r_waypoint)
        r_c = self._obstacle_collision_punish(self.cnn_data.scan[-self.scan_size:], r_scan, r_collision)
        r_w = self._angular_velocity_punish(self.curr_vel.angular.z,  r_rotation, w_thresh)
        r_t = self._theta_reward(self.local_goal_from_robot, self.mht_peds, self.curr_vel.linear.x, r_angle, angle_thresh)
        reward = -0.01 + r_g + r_c + r_t #+ r_w #+ r_v # + r_p
        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Current Velocity: \ncurr_vel = {}".format(self.curr_vel.linear.x), throttle_duration_sec=self.config.log.throttle_duration)
            self.node.get_logger().warning("Compute reward done. \nreward = {}".format(reward), throttle_duration_sec=self.config.log.throttle_duration)
            self.node.get_logger().fatal("Compute reward done. \nreward = {}\n    rg: {}\n    rc: {}\n    rw: {}\n    rt: {}".format(reward, r_g, r_c, r_w, r_t))
        return reward

    def _goal_reached_reward(self, r_arrival, r_waypoint):
        """
        Returns positive reward if the robot reaches the goal.
        :param transformed_goal goal position in robot frame
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        dist_to_goal = np.linalg.norm(
            np.array([
                self.curr_pose.position.x - self.final_goal.x,
                self.curr_pose.position.y - self.final_goal.y
            ])
        )

        t_1 = self.num_iterations % 10
        if(self.num_iterations == 0 or self.reset_dist_to_goal_reg):
            self.dist_to_goal_reg = np.ones(10)*dist_to_goal
            self.init_distance = max(dist_to_goal, 1e-6)  # éviter division par zéro
            self.reset_dist_to_goal_reg = False

        reward = 0.0

        if(dist_to_goal <= self.goal_radius):
            reward = r_arrival
            self.reset_dist_to_goal_reg = True
        elif(self.num_iterations >= self.max_iteration):
        # elif(self.start_time < self.get_time() - self.max_time):
            reward = -r_arrival
        else:
            delta = self.dist_to_goal_reg[t_1] - dist_to_goal
            reward = (r_waypoint*delta)
            # reward = (r_waypoint*delta) / self.init_distance
            # reward = np.clip(reward, -r_waypoint, r_waypoint) # GPT didn't test

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Goal reward: {}  {}".format(dist_to_goal, reward), throttle_duration_sec=self.config.log.throttle_duration)
            self.node.get_logger().fatal("Dist : {}     {}".format(t_1, self.dist_to_goal_reg))
            self.node.get_logger().fatal("Goal reward: {}     {}     {}     {}".format(self.dist_to_goal_reg[t_1], dist_to_goal, reward, self.local_goal_from_map))

        self.dist_to_goal_reg[t_1] = dist_to_goal

        
        return reward

    def _obstacle_collision_punish(self, scan, r_scan, r_collision):
        """
        Returns negative reward if the robot collides with obstacles.
        :param scan containing obstacles that should be considered
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        min_scan_dist = np.amin(scan[scan!=0])
        #if(self.bump_flag == True): #or self.pos_valid_flag == False):
        if(min_scan_dist <= self.robot_radius and min_scan_dist >= 0.02):
            reward = r_collision
        elif(min_scan_dist < 3*self.robot_radius):
            reward = r_scan * (3*self.robot_radius - min_scan_dist)
        else:
            reward = 0.0

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Obstacle collision reward: {}".format(reward), throttle_duration_sec=self.config.log.throttle_duration)
        return reward

    def _angular_velocity_punish(self, w_z,  r_rotation, w_thresh):
        """
        Returns negative reward if the robot turns.
        :param w roatational speed of the robot
        :param fac weight of reward punish for turning
        :param thresh rotational speed > thresh will be punished
        :return: returns reward for turning
        """
        if(abs(w_z) > w_thresh):
            reward = abs(w_z) * r_rotation
        else:
            reward = 0.0
        
        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Angular reward: {} {}".format(reward, w_z), throttle_duration_sec=self.config.log.throttle_duration)
        return reward

    def _theta_reward(self, goal, mht_peds, v_x, r_angle, angle_thresh):
        """
        Returns negative reward if the robot turns.
        :param w roatational speed of the robot
        :param fac weight of reward punish for turning
        :param thresh rotational speed > thresh will be punished
        :return: returns reward for turning
        """
        # prefer goal theta:
        theta_pre = np.arctan2(goal.y, goal.x)
        d_theta = theta_pre
        reward = r_angle*(angle_thresh - abs(d_theta))

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Theta reward: {}".format(reward), throttle_duration_sec=self.config.log.throttle_duration)
            self.node.get_logger().fatal("{} {} {} {} {}".format(goal, mht_peds, v_x, r_angle, angle_thresh))
        return reward  




    def get_time(self):
        return self.node.get_clock().now().nanoseconds * 1e-9
    
    def set_curriculum_level(self, level):
        self.curriculum_level = level
        # self.max_iteration = self.config.env.max_iteration * (self.curriculum_level+1)
        self.node.get_logger().info(f"🎯 Changement de niveau de curriculum : {level}")
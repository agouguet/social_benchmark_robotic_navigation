#!/usr/bin/env python3
import subprocess
import time, math, random
import rclpy
from rclpy.node import Node
import gymnasium as gym
import numpy as np
from simulation_msgs.srv import Reset
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from cnn_msgs.msg import CNNdata, MyCNNdata, AllCNNdata
from agents_msgs.msg import AgentArray


import os
import numpy as np
import matplotlib.pyplot as plt

class DRLVOEnv(gym.Env):
    def __init__(self, env_id, config, env_id_display_log=None):
        super().__init__()
        self.config = config
        self.curriculum_level = 0
        
        # env parameters:
        self.env_id = env_id
        self.env_id_display_log = env_id_display_log
        self.prefix = "/env_" + str(self.env_id)
        self.max_iteration = self.config.env.max_iteration #* (self.curriculum_level+1)
        self.max_time = self.config.env.max_time

        # robot parameters:
        self.robot_radius = self.config.env.robot.robot_radius
        self.min_linear_velocity = self.config.env.robot.min_linear_velocity
        self.max_linear_velocity = self.config.env.robot.max_linear_velocity
        self.min_angular_velocity = self.config.env.robot.min_angular_velocity
        self.max_angular_velocity = self.config.env.robot.max_angular_velocity

        # bumper:
        self.bump_num = 0

        # reward:
        self.goal_radius = self.config.env.reward.goal_radius
        self.dist_goal_history_number = self.config.env.reward.goal_dist_history_number
        self.dist_to_goal_reg = np.zeros(self.dist_goal_history_number)
        self.init_distance = 0
        self.reset_dist_to_goal_reg = True
        self.num_iterations = 0
        

        # Initialisation Node
        self.node = Node("ros_env_" + str(self.env_id))

        # === Spaces ===
        self.high_action = np.array([1, 1])
        self.low_action = np.array([-1, -1])
        self.action_space = gym.spaces.Box(low=self.low_action, high=self.high_action, dtype=np.float32)                               # Goal
        self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(19202,), dtype=np.float32)

        self.cnn_data = AllCNNdata()
        self.agents = AgentArray()
        self.scan = []
        self.scan_size = 720
        self.goal = []
        self.mht_peds = []

        # info, initial position and goal position
        self.final_goal = Point()
        self.local_goal_from_robot = None
        
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.info = {}
        # episode done flag:
        self._episode_done = False
        # reset flag:
        self._reset = True

        # === ROS topics ===
        self.node.create_subscription(AllCNNdata, self.prefix + self.config.env.ros.cnn_data, self._cnn_data_callback, 1)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, self.prefix + self.config.env.ros.cmd_vel, 10)

        # === Services ===
        self.reset_client = self.node.create_client(Reset, self.prefix + self.config.env.ros.reset_service)
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
        self.cmd_vel_publisher.publish(Twist())

        if(self._reset): 
            self._reset = False
            req = Reset.Request()
            req.level = self.curriculum_level
            future = self.reset_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

        # initalize info:
        self.cnn_data = None
        self.agents = AgentArray()
        self.scan = []
        self.goal = []
        self.mht_peds = []

        self.curr_pose = None
        self.curr_vel = None
        self.info = {}
        self.final_goal = None
        self.local_goal_from_robot = None

        # reset pose valid flag:
        self.pos_valid_flag = True
        # reset bumper:
        self.bump_num = 0
        # reset the number of iterations:
        self.num_iterations = 0
        # reset distance to goal register:
        # self.dist_to_goal_reg = np.zeros(self.dist_goal_history_number)
        self.reset_dist_to_goal_reg = True
        # reset episode done flag:
        self._episode_done = False

        while self.cnn_data == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)

        dist_to_goal = np.linalg.norm(
            np.array([
                self.local_goal_from_robot.x,
                self.local_goal_from_robot.y
            ])
        )
        self.dist_to_goal_reg = np.ones(10)*dist_to_goal

        # Give the system a little time to finish initialization
        self.node.get_logger().fatal("Finish initialize robot.", once=True)

    def _cnn_data_callback(self, cnn_data_msg):
        self.cnn_data = cnn_data_msg
        self.curr_pose = self.cnn_data.robot_pose.pose
        self.curr_vel = Twist()
        self.curr_vel.linear.x = self.cnn_data.vel[0]
        self.curr_vel.angular.z = self.cnn_data.vel[1]
        self.final_goal = self.cnn_data.global_goal.pose.position
        self.goal = self.cnn_data.goal_cart
        self.local_goal_from_robot = self.cnn_data.local_goal_from_robot
        self.agents = self.cnn_data.agents

    def send_action(self, action):
        cmd_vel = Twist()
        cmd_vel.linear.x = (float(action[0]) + 1) * (self.max_linear_velocity - self.min_linear_velocity) / 2 + self.min_linear_velocity
        cmd_vel.angular.z = (float(action[1]) + 1) * (self.max_angular_velocity - self.min_angular_velocity) / 2 + self.min_angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel)

    def step(self, action):
        step_time = time.time()
        time_tmp = time.time()
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
        reward_time = time.time() - time_tmp

        done = self._is_done(reward)
        truncated = False

        # if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
        #     self.node.get_logger().fatal("\nStep Time: {}\n    action time: {}\n    spin time: {}\n    obs time: {}\n    reward time: {}".format(
        #             time.time() - step_time, action_time, spin_time, obs_time, reward_time
        #         ), 
        #         throttle_duration_sec=self.config.log.throttle_duration)
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

        if(self.dist_to_goal() <= self.goal_radius):
            # reset the robot velocity to 0:
            self.cmd_vel_publisher.publish(Twist())
            self._episode_done = True
            self._reset = True # reset the simulation world
            return True

        # 2) Obstacle collision?
        scan = np.array(self.cnn_data.scan[-self.scan_size:], dtype=np.float32)
        scan = scan[(scan > 0) & np.isfinite(scan)]
        # if scan[scan > 0].size <= 0:
        #     self.node.get_logger().fatal("223 {}".format(scan[scan > 0]))
        min_scan_dist = np.min(scan[scan > 0]) if scan[scan > 0].size > 0 else math.inf
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
        t = time.time()
        self.ped_pos = self.cnn_data.ped_pos_map
        self.scan = self.cnn_data.scan
        self.goal = np.array([
                            self.local_goal_from_robot.x,
                            self.local_goal_from_robot.y
                        ], dtype=np.float32)
        self.vel = self.cnn_data.vel
        
        # ped map:
        # MaxAbsScaler:
        v_min = -2
        v_max = 2
        self.ped_pos = np.array(self.ped_pos, dtype=np.float32)
        self.ped_pos = 2 * (self.ped_pos - v_min) / (v_max - v_min) + (-1)

        # scan map:
        # MaxAbsScaler:

        # size_slice = 80
        # temp = np.array(self.scan, dtype=np.float32)
        # temp = temp.reshape(10, self.scan_size, 1)
        # temp = temp.reshape(10, size_slice, 9)
        # scan_min = np.min(temp, axis=2)
        # scan_mean = np.mean(temp, axis=2)
        # scan_avg = np.stack((scan_min, scan_mean), axis=1).reshape(20, size_slice)
        # scan_avg = scan_avg.reshape(20*size_slice)
        # self.scan = np.tile(scan_avg, 4)
        self.scan = np.array(self.scan, dtype=np.float32)
        t1 = np.min(self.scan)
        t2 = np.max(self.scan)
        temp = np.array(self.scan, dtype=np.float32)
        scan_avg = np.zeros((20,80))
        for n in range(10):
            scan_tmp = temp[n*720:(n+1)*720]
            for i in range(80):
                scan_avg[2*n, i] = np.min(scan_tmp[i*9:(i+1)*9])
                scan_avg[2*n+1, i] = np.mean(scan_tmp[i*9:(i+1)*9])
        scan_avg = scan_avg.reshape(1600)
        scan_avg_map = np.matlib.repmat(scan_avg,1,4)
        self.scan = scan_avg_map.reshape(6400)
        s_min = 0
        s_max = 10
        self.scan = 2 * (self.scan - s_min) / (s_max - s_min) + (-1)

        
        
        # goal:
        # MaxAbsScaler:
        # g_min = -2
        # g_max = 2
        # self.goal = np.array(self.goal, dtype=np.float32)
        # self.goal = 2 * (self.goal - g_min) / (g_max - g_min) + (-1)
        #self.goal = self.goal.tolist()

        self.vel = np.array(self.vel, dtype=np.float32)

        '''
        # vel:
        # MaxAbsScaler:
        vx_min = 0
        vx_max = 0.5
        wz_min = -2
        wz_max = 2
        self.vel = np.array(self.vel, dtype=np.float32)
        self.vel[0] = 2 * (self.vel[0] - vx_min) / (vx_max - vx_min) + (-1)
        self.vel[1] = 2 * (self.vel[1] - wz_min) / (wz_max - wz_min) + (-1)
        '''

        # observation:
        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            # visualize_observation(self.ped_pos, self.scan, self.goal, step=0)
            self.node.get_logger().warning(
                "\nObservation (step {}) => \n    goal: {} \n    vel: {} \n    scan: {} \n    scan_min: {}\n     scan_max: {}\n     ped_pos: {} \n     agents: {}".format(self.num_iterations, self.goal, self.vel, self.scan, t1, t2, self.ped_pos, self.agents), 
                throttle_duration_sec=self.config.log.throttle_duration)
        self.observation = np.concatenate((self.ped_pos, self.scan, self.goal), axis=None) #list(itertools.chain(self.ped_pos, self.scan, self.goal))
        return self.observation
    
    def _post_information(self):
        self.info = {
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
        r_arrival = self.config.env.reward.goal_arrival     #20 #15
        r_waypoint = self.config.env.reward.goal_waypoint   #5.0 #3.2 #2.5 #1.6 #2 #3 #1.6 #6 #2.5 #2.5
        r_collision = self.config.env.reward.collision      #-20 #-15
        r_scan = self.config.env.reward.scan                #0.0 #-0.1 #-0.2 #-0.15 #-0.3
        r_angle = self.config.env.reward.theta_angle        # 0.2 #0.6 #0.5 #1 #0.8 #1 #0.5
        r_rotation = self.config.env.reward.rotation        #-0.1 #-0.15 #-0.4 #-0.5 #-0.2 # 0.1
        r_human = self.config.env.reward.human

        angle_thresh = np.pi/6
        scan_penalty_threshold_factor = self.config.env.reward.scan_penalty_threshold_factor
        w_thresh = 1 # 0.7

        # reward parts:
        r_g = self._goal_reached_reward(r_arrival, r_waypoint)
        r_c = self._obstacle_collision_punish(self.cnn_data.scan[-self.scan_size:], scan_penalty_threshold_factor, r_scan, r_collision)
        r_w = self._angular_velocity_punish(self.curr_vel.angular.z,  r_rotation, w_thresh)
        r_t = self._theta_reward(self.local_goal_from_robot, self.mht_peds, self.curr_vel.linear.x, r_angle, angle_thresh)
        reward = r_g + r_c + r_t + r_w

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("\nreward = {}\n    rg: {}\n    rc: {}\n    rw: {}\n    rt: {}".format(reward, r_g, r_c, r_w, r_t), throttle_duration_sec=self.config.log.throttle_duration)
        return reward

    def dist_to_goal(self):
        return np.linalg.norm(
            np.array([
                self.curr_pose.position.x - self.final_goal.x,
                self.curr_pose.position.y - self.final_goal.y
            ])
        )

    def _goal_reached_reward(self, r_arrival, r_waypoint):
        """
        Returns positive reward if the robot reaches the goal.
        :param transformed_goal goal position in robot frame
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        dist_to_goal = self.dist_to_goal()

        t_1 = self.num_iterations % 10
        if(self.num_iterations == 0 or self.reset_dist_to_goal_reg):
            self.dist_to_goal_reg = np.ones(10)*dist_to_goal
            self.init_distance = max(dist_to_goal, 1e-6)
            self.reset_dist_to_goal_reg = False

        reward = 0.0

        if(dist_to_goal <= self.goal_radius):
            reward = r_arrival
            self.reset_dist_to_goal_reg = True
        elif(self.num_iterations >= self.max_iteration):
            reward = -r_arrival
        else:
            delta = self.dist_to_goal_reg[t_1] - dist_to_goal
            reward = (r_waypoint*delta)

        self.dist_to_goal_reg[t_1] = dist_to_goal

        
        return reward

    def _obstacle_collision_punish(self, scan, scan_penalty_threshold_factor, r_scan, r_collision):
        """
        Returns negative reward if the robot collides with obstacles.
        :param scan containing obstacles that should be considered
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        scan = np.array(scan, dtype=np.float32)
        scan = scan[(scan > 0) & np.isfinite(scan)]
        # if scan[scan > 0].size <= 0:
        #     self.node.get_logger().fatal("426 {}".format(scan[scan > 0]))
        min_scan_dist = np.min(scan[scan > 0]) if scan[scan > 0].size > 0 else math.inf
        #if(self.bump_flag == True): #or self.pos_valid_flag == False):
        if(min_scan_dist <= self.robot_radius and min_scan_dist >= 0.02):
            reward = r_collision
        elif(min_scan_dist < scan_penalty_threshold_factor*self.robot_radius):
            reward = r_scan * (scan_penalty_threshold_factor*self.robot_radius - min_scan_dist)
        else:
            reward = 0.0
        return reward

    def _angular_velocity_punish(self, w_z,  r_rotation, w_thresh):
        if(abs(w_z) > w_thresh):
            reward = abs(w_z) * r_rotation
        else:
            reward = 0.0
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
        # theta_pre = np.arctan2(goal[1], goal[0])
        theta_pre = np.arctan2(goal.y, goal.x)
        d_theta = theta_pre

        # get the pedstrain's position:
        if(len(self.agents.agents) != 0):  # tracker results
            d_theta = np.pi/2 #theta_pre
            N = 60
            theta_min = 1000
            for i in range(N):
                theta = random.uniform(-np.pi, np.pi)
                free = True
                for ped in self.agents.agents:
                    #ped_id = ped.track_id 
                    # create pedestrian's postion costmap: 10*10 m
                    p_x = ped.pose.position.x
                    p_y = ped.pose.position.y
                    p_vx = ped.velocity.linear.x
                    p_vy = ped.velocity.linear.y
                    
                    ped_dis = np.linalg.norm([p_x, p_y])
                    if(ped_dis <= 7):
                        ped_theta = np.arctan2(p_y, p_x)
                        vo_theta = np.arctan2(3*self.robot_radius, np.sqrt(ped_dis**2 - (3*self.robot_radius)**2))
                        # collision cone:
                        theta_rp = np.arctan2(v_x*np.sin(theta)-p_vy, v_x*np.cos(theta) - p_vx)
                        if(theta_rp >= (ped_theta - vo_theta) and theta_rp <= (ped_theta + vo_theta)):
                            free = False
                            break

                # reachable available theta:
                if(free):
                    theta_diff = (theta - theta_pre)**2
                    if(theta_diff < theta_min):
                        theta_min = theta_diff
                        d_theta = theta
                
        else: # no obstacles:
            d_theta = theta_pre

        reward = r_angle*(angle_thresh - abs(d_theta))
        return reward  
    
    def get_time(self):
        return self.node.get_clock().now().nanoseconds * 1e-9
    
    def set_curriculum_level(self, level):
        self.curriculum_level = level
        # self.max_iteration = self.config.env.max_iteration * (self.curriculum_level+1)
        self.node.get_logger().info(f"🎯 Changement de niveau de curriculum : {level}")

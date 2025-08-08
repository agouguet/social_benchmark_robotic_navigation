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


class CrowdHeightEnv(gym.Env):
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

        self.human_number = self.config.env.obs.human_number
        
        # Initialisation Node
        self.node = Node("ros_env_" + str(self.env_id))

        # === Spaces ===
        self.high_action = np.array([1, 1])
        self.low_action = np.array([-1, -1])
        self.action_space = gym.spaces.Box(low=self.low_action, high=self.high_action, dtype=np.float32)

        obs_shape = 0
        obs_shape += 5 # robot (px, py, gx, gy, theta)
        obs_shape += 2 # robot (vx, vy)
        obs_shape += 4 * self.human_number # N human (px, py, vx, vy) pos relative
        obs_shape += 1 # number of detected humans
        obs_shape += self.config.env.obs.scan_dim # scan

        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(obs_shape,), dtype=np.float32)

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

    def send_action(self, action):
        cmd_vel = Twist()
        cmd_vel.linear.x = (float(action[0]) + 1) * (self.max_linear_velocity - self.min_linear_velocity) / 2 + self.min_linear_velocity
        cmd_vel.angular.z = (float(action[1]) + 1) * (self.max_angular_velocity - self.min_angular_velocity) / 2 + self.min_angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel)

    def _get_observation(self):
        ob = {}

        # nodes
        visible_humans, num_visibles, self.human_visibility = self.get_num_human_in_fov()

        ob['robot_node'] = self.robot.get_changing_state_list()


        # edges
        ob['temporal_edges'] = np.array([self.robot.vx, self.robot.vy])

        # ([relative px, relative py, disp_x, disp_y], human id)
        all_spatial_edges = np.ones((max(1, self.max_human_num), 4)) * np.inf

        # robot.vx and vy are in world frame, transform to robot frame first
        v_robot_robFrame = self.world_to_robot([self.robot.vx, self.robot.vy])
        for i in range(self.human_num):
            if self.human_visibility[i]:
                # vector pointing from human i to robot (in world frame)
                relative_pos = np.array(
                    [self.last_human_states[i, 0] - self.robot.px, self.last_human_states[i, 1] - self.robot.py])
                # in self.last_human_states, the human relative positions are in world frame, transform to robot frame for sim2real
                all_spatial_edges[self.humans[i].id, :2] = self.world_to_robot(relative_pos)
                
                # in self.last_human_states, the human velocities are in world frame, transform to robot frame for sim2real
                v_human = self.world_to_robot(self.last_human_states[i, 2:4])
                # absolute velocity
                all_spatial_edges[self.humans[i].id, 2:] = v_human

        # sort all humans by distance (invisible humans will be in the end automatically)
        ob['spatial_edges'] = np.array(sorted(all_spatial_edges, key=lambda x: np.linalg.norm(x[:2])))
        ob['spatial_edges'][np.isinf(ob['spatial_edges'])] = 15

        ob['detected_human_num'] = num_visibles
        # if no human is detected, assume there is one dummy human at (15, 15) to make the pack_padded_sequence work
        if ob['detected_human_num'] == 0:
            ob['detected_human_num'] = 1

        # obstacle representations methods:
        # 1. relative coordinates of 4 vertices w.r.t. the robot

        ob['obstacle_vertices'] = np.ones((max(1, self.max_obs_num), 8,)) * 15

        # obstacle vertices in world frame
        cur_obs_vertices = np.array(self.obstacle_coord) - np.array([self.robot.px, self.robot.py])
        # convert obstacle vertics from world to robot frame
        cur_obs_vertices_rob_frame = np.zeros((self.obs_num, 4, 2))
        for i in range(self.obs_num):
            for j in range(4):
                cur_obs_vertices_rob_frame[i, j] = self.world_to_robot(cur_obs_vertices[i, j])

        ob['obstacle_vertices'][:self.obs_num] = cur_obs_vertices_rob_frame.reshape(self.obs_num, -1)

        # 2. coordinates of bounding boxes (aabb)
        ob['obstacle_num'] = self.obs_num

        # 3. raw lidar point cloud
        # only include obs
        # a. all obs in one lidar scan
        # b. seperate each obs for robot-obstacle attn
        self.ray_test_no_humans()
        ob['point_clouds'] = np.expand_dims(self.closest_hit_dist, axis=0)

        # update self.observed_human_ids
        self.observed_human_ids = np.where(self.human_visibility)[0]
        self.ob = ob

        return ob
    
    def calc_reward(self, action):

        # collision checking
        scan = np.array(scan, dtype=np.float32)
        scan = scan[(scan > 0) & np.isfinite(scan)]
        min_scan_dist = np.min(scan[scan > 0]) if scan[scan > 0].size > 0 else math.inf
        collision = (min_scan_dist <= self.robot_radius and min_scan_dist >= 0.02)

        collision, dmin, _ = self.collision_checker()

        # check if reaching the goal
        reaching_goal = np.linalg.norm(np.array(self.robot.get_position()) - np.array(self.robot.get_goal_position())) < self.config.env.reward.goal_radius

        if(self.num_iterations >= self.max_iteration):
            reward = 0
        elif collision:
            reward = self.config.env.reward.collision
        elif reaching_goal:
            reward = self.config.env.reward.goal_arrival
        elif dmin < self.config.env.reward.human_personal_distance:
            reward = (dmin - self.config.env.reward.human_personal_distance) * self.config.env.reward.human
        else:
            # potential reward
            potential_cur = np.linalg.norm(
                np.array([self.robot.px, self.robot.py]) - np.array(self.robot.get_goal_position()))
            pot_factor = self.pot_factor * 2
            reward = pot_factor * (-abs(potential_cur) - self.potential)
            self.potential = -abs(potential_cur)


        # if the robot is near collision/arrival, it should be able to turn a large angle
        r_spin_coefficient = 0.05
        r_back_coefficient = 0.
        w = self.curr_vel.angular.z
        v = self.curr_vel.linear.x

        # add a rotational penalty
        r_spin = -r_spin_coefficient * w**2

        # add a penalty for going backwards
        if v < 0:
            r_back = -r_back_coefficient * abs(v)
        else:
            r_back = 0.

        reward = reward + r_spin + r_back
        return reward
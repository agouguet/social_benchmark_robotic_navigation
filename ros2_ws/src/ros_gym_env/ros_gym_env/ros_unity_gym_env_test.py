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
from simulation_msgs.srv import Trigger, PausePlay
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from cnn_msgs.msg import CNNdata
from threading import Event
from tf_transformations import euler_from_quaternion
from sklearn.preprocessing import MaxAbsScaler
from ros_gym_env.cnn_data_pub import CnnDataNode


class RosUnityEnvTest(gym.Env):
    def __init__(self, env_id, max_iteration=1024, env_id_display_log=None):
        super().__init__()

        self.env_id = env_id
        self.env_id_display_log = env_id_display_log
        self.prefix = "/env_" + str(self.env_id)
        self.max_iteration = max_iteration

        # robot parameters:
        self.ROBOT_RADIUS = 0.35
        self.GOAL_RADIUS = 0.3 #0.3
        self.DIST_NUM = 10
        self.min_linear_velocity = 0.0
        self.max_linear_velocity = 0.5

        self.min_angular_velocity = -2.0
        self.max_angular_velocity = 2.0


         # bumper:
        self.bump_flag = False
        self.bump_num = 0

        # reward:
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        self.num_iterations = 0

        self.max_steps = 10  # par ex, max steps par épisode
        self.current_step = 0

        # Initialisation Node
        self.node = Node("ros_unity_env_" + str(self.env_id))
        # self.wait_for_message_node = Node("wait_for_message_tester")

        # === Spaces ===
        self.scan_size = 720
        self.scan_obs_size = 6400  # ajustable
        self.ped_size = 12800
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)


        # self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)                                     # Goal
        self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(self.scan_size + 3,), dtype=np.float32)                      # Scan
        # self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(self.scan_obs_size + 2,), dtype=np.float32)                # Scan history
        # self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(self.ped_size + self.scan_size + 2,), dtype=np.float32)    # DRL-VO

        # observation space
        self.cnn_data = CNNdata()
        self.ped_pos = []
        self.scan = []
        self.goal = []
        self.mht_peds = []

        # info, initial position and goal position
        self.init_pose = Pose()
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.goal_position = Point()
        self.info = {}
        # episode done flag:
        self._episode_done = False
        # goal reached flag:
        self._goal_reached = False
        # reset flag:
        self._reset = True

        # self.scan = np.zeros(self.scan_size, dtype=np.float32)
        # self.goal = np.zeros(2, dtype=np.float32)

        # === ROS topics ===
        self.node.create_subscription(Odometry, self.prefix + '/robot_odom', self._robot_vel_callback, 1)
        self.node.create_subscription(PoseStamped, self.prefix + "/robot_pose", self._robot_pose_callback, 1)
        self.node.create_subscription(CNNdata, self.prefix + "/cnn_data", self._cnn_data_callback, 1)
        self.node.create_subscription(PoseStamped, self.prefix + '/global_goal', self._final_goal_callback, 1)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, self.prefix + '/cmd_vel', 10)

        self.obs = None
        self.done = False

        # === Services ===
        self.reset_client = self.node.create_client(Trigger, self.prefix + '/unity/reset')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Attente du service de reset...')

        # self.pause_play_client = self.node.create_client(PausePlay, '/unity/pause_play')
        # while not self.pause_play_client.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('Service /unity/pause_play non disponible, attente...')
        # self.pause = False  # État initial


    def reset(self, *, seed=None, options=None, **kwargs):
        """ 
        obs, info = env.reset() 
        """
        if seed is not None:
            np.random.seed(seed)
        self.current_step = 0
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
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.goal_position = None
        self.info = {}

        # self.init_pose = self.curr_pose # inital_pose.pose.pose
        # self.curr_pose = self.curr_pose # inital_pose.pose.pose
        # self.node.get_logger().warning("Robot was initiated as {}".format(self.init_pose), once=True)

        # reset pose valid flag:
        self.pos_valid_flag = True
        # reset bumper:
        self.bump_flag = False
        # self.bump_num = 0
        # reset the number of iterations:
        self.num_iterations = 0
        # reset distance to goal register:
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        # reset episode done flag:
        self._episode_done = False

        while self.cnn_data == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        # self.node.get_logger().fatal("Time 2 {}".format(time.time() - t))
        while self.goal_position == None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        # self.node.get_logger().fatal("Time 3 {}".format(time.time() - t))
        # time.sleep(2)

        # Give the system a little time to finish initialization
        self.node.get_logger().warning("Finish initialize robot.", once=True)
        
        return self.init_pose, self.goal_position
        
    def _robot_vel_callback(self, robot_vel_msg):
        self.curr_vel = robot_vel_msg.twist.twist

    def _robot_pose_callback(self, robot_pose_msg):
        self.curr_pose = robot_pose_msg.pose

    def _cnn_data_callback(self, cnn_data_msg):
        self.cnn_data = cnn_data_msg
        
    def _final_goal_callback(self, final_goal_msg):
        self.goal_position = final_goal_msg.pose.position

    def send_action(self, action):
        cmd_vel = Twist()

        cmd_vel.linear.x = (float(action[0]) + 1) * (self.max_linear_velocity - self.min_linear_velocity) / 2 + self.min_linear_velocity
        cmd_vel.angular.z = (float(action[1]) + 1) * (self.max_angular_velocity - self.min_angular_velocity) / 2 + self.min_angular_velocity

        self.cmd_vel_publisher.publish(cmd_vel)

    def step(self, action):
        t = time.time()
        self.current_step += 1
        steps_left = self.max_steps - self.current_step
        # if (self.env_id == 0):
        #     self.node.get_logger().fatal("    01 Start {}".format(time.time() - t))
        #     t = time.time()

        # if (self.env_id == 0):
        #     self.node.get_logger().fatal("    02 Send Action {}".format(time.time() - t))
        #     t = time.time()
        self.send_action(action)

        # if (self.env_id == 0):
        #     self.node.get_logger().fatal("    03 Spin {}".format(time.time() - t))
        #     t = time.time()
        rclpy.spin_once(self.node, timeout_sec=0.05)

        # if (self.env_id == 0):
        #     self.node.get_logger().fatal("    04 Get Obs {}".format(time.time() - t))
        #     t = time.time()
        obs = self._get_observation()

        # if (self.env_id == 0):
        #     self.node.get_logger().fatal("    05 Compute Reward {}".format(time.time() - t))
        #     t = time.time()
        reward = self._compute_reward()

        done = self._is_done(reward)
        # done = True

        # if (self.env_id == 0):
        #     self.node.get_logger().fatal("    06 Is Done ? {} {}".format(done, time.time() - t))
        #     t = time.time()

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
            self.curr_pose.position.x - self.goal_position.x,
            self.curr_pose.position.y - self.goal_position.y,
            self.curr_pose.position.z - self.goal_position.z
            ])
        )

        # self.node.get_logger().warning("Dist goal {}    {} ".format(dist_to_goal, self.goal_position))

        if(dist_to_goal <= self.GOAL_RADIUS):
            # reset the robot velocity to 0:
            self.cmd_vel_publisher.publish(Twist())
            self._episode_done = True
            self._reset = True # reset the simulation world
            return True

        # 2) Obstacle collision?
        scan = self.cnn_data.scan[-720:]
        min_scan_dist = np.amin(scan[scan!=0])
        #if(self.bump_flag == True): #or self.pos_valid_flag == False):
        if(min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.00):
            self.bump_num += 1

        # stop and reset if more than 5 collisions: 
        if(self.bump_num >= 3):
            # reset the robot velocity to 0:
            self.cmd_vel_publisher.publish(Twist())
            self.bump_num = 0
            self._episode_done = True
            self._reset = True # reset the simulation world
            return True


        # 4) maximum number of iterations?
        if(self.num_iterations > self.max_iteration):
            # reset the robot velocity to 0:
            self.cmd_vel_publisher.publish(Twist())
            self._episode_done = True
            self._reset = True
            return True

        # self.node.get_logger().warning("Is Done ? dist_to_goal : {}    bump_num : {}     min_scan_dist : {}    num_iterations : {}".format(dist_to_goal, self.bump_num, min_scan_dist, self.num_iterations), throttle_duration_sec=2)
        return False #self._episode_done

    def _get_observation(self):
        """
        Returns the observation.
        """
        # self.node.get_logger().warning("Get obs Method")
        self.ped_pos = self.cnn_data.ped_pos_map
        self.scan = self.cnn_data.scan
        self.goal = self.cnn_data.goal_cart
        # self.vel = self.cnn_data.vel
        
        # ped map:
        # MaxAbsScaler:
        v_min = -2
        v_max = 2
        self.ped_pos = np.asarray(self.ped_pos, dtype=np.float32)
        self.ped_pos = 2 * (self.ped_pos - v_min) / (v_max - v_min) + (-1)

        # scan map:
        # MaxAbsScaler:
        temp = np.array(self.scan, dtype=np.float32)
        temp = temp.reshape(10, 720, 1)
        temp = temp.reshape(10, 80, 9)
        scan_min = np.min(temp, axis=2)
        scan_mean = np.mean(temp, axis=2)
        scan_avg = np.stack((scan_min, scan_mean), axis=1).reshape(20, 80)
        scan_avg = scan_avg.reshape(1600)
        self.scan = np.tile(scan_avg, 4)
        s_min = 0
        s_max = 10
        self.scan = np.clip(self.scan, 0.0, 10.0)
        self.scan = 2 * (self.scan - s_min) / (s_max - s_min) + (-1)


        s_min = 0
        s_max = 10
        self.scan_simple = np.clip(self.cnn_data.scan[-720:], 0.0, 10.0)
        self.scan_simple = 2 * (self.scan_simple - s_min) / (s_max - s_min) + (-1)
        
        # goal:
        # MaxAbsScaler:
        g_min = -2
        g_max = 2
        self.goal = np.array(self.goal, dtype=np.float32)
        self.goal = 2 * (self.goal - g_min) / (g_max - g_min) + (-1)


        dist_to_goal = np.linalg.norm(
            np.array([
                self.curr_pose.position.x - self.goal_position.x,
                self.curr_pose.position.y - self.goal_position.y,
                self.curr_pose.position.z - self.goal_position.z
            ])
        )
        d_min = 0.0
        d_max = 10.0
        dist_to_goal = 2 * (dist_to_goal - d_min) / (d_max - d_min) - 1.0

        # observation:
        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("\n\n")
            self.node.get_logger().warning("Goal pos/dist: {} / {}".format(self.goal, dist_to_goal))
        
        # self.observation = np.concatenate((self.scan_simple, self.goal), axis=None)
        # self.observation = np.concatenate((self.scan, self.goal), axis=None)
        self.observation = np.concatenate((self.scan_simple, self.goal, dist_to_goal), axis=None)
        # self.observation = np.concatenate((self.ped_pos, self.scan, self.goal), axis=None)

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Observation Shape = {}".format(self.observation.shape))
        
        return self.observation
    
    def _post_information(self):
        """
        Return:
        info: {"init_pose", "goal_position", "current_pose"}
        """
        self.info = {
            "initial_pose": self.init_pose,
            "goal_position": self.goal_position,
            "current_pose": self.curr_pose
            }
        
        return self.info

    def close(self):
        self.node.destroy_node()

    def render(self, mode='human'):
        pass  # ou afficher des infos de debug

    def get_yaw_from_quaternion(self, quat):
        orientation_q = [quat.x, quat.y, quat.z, quat.w]
        (_, _, yaw) = euler_from_quaternion(orientation_q)
        return yaw

    def global_goal_from_local(self):
        # goal = self.goal_position
        # 1. Position relative dans le monde (goal - robot)
        delta_x = self.goal_position.x - self.curr_pose.position.x
        delta_y = self.goal_position.y - self.curr_pose.position.y

        # 2. Orientation du robot (en radians) - supposons que tu extrais theta (yaw)
        theta = self.get_yaw_from_quaternion(self.curr_pose.orientation)

        # 3. Transformer dans le repère local du robot
        local_x =  math.cos(-theta) * delta_x - math.sin(-theta) * delta_y
        local_y =  math.sin(-theta) * delta_x + math.cos(-theta) * delta_y

        return np.array([local_x, local_y], dtype=np.float32)











    def _compute_reward(self):
        """Calculates the reward to give based on the observations given.
        """
        # reward parameters:
        r_arrival = 20 #15
        r_waypoint = 5.0 #3.2 #2.5 #1.6 #2 #3 #1.6 #6 #2.5 #2.5
        r_collision = -20 #-15
        r_scan = -0.2 #-0.15 #-0.3
        r_angle = 0.6 #0.5 #1 #0.8 #1 #0.5
        r_rotation = -0.1 #-0.15 #-0.4 #-0.5 #-0.2 # 0.1

        angle_thresh = np.pi/6
        w_thresh = 1 # 0.7

        # reward parts:
        r_g = self._goal_reached_reward(r_arrival, r_waypoint)
        r_c = self._obstacle_collision_punish(self.cnn_data.scan[-720:], r_scan, r_collision)
        r_w = self._angular_velocity_punish(self.curr_vel.angular.z,  r_rotation, w_thresh)
        r_t = self._theta_reward(self.goal, self.mht_peds, self.curr_vel.linear.x, r_angle, angle_thresh)
        reward = r_g + r_c + r_t + r_w #+ r_v # + r_p
        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Current Velocity: \ncurr_vel = {}".format(self.curr_vel.linear.x))
            self.node.get_logger().warning("Compute reward done. \nreward = {}".format(reward))
        return reward

    def _goal_reached_reward(self, r_arrival, r_waypoint):
        """
        Returns positive reward if the robot reaches the goal.
        :param transformed_goal goal position in robot frame
        :param k reward constant
        :return: returns reward colliding with obstacles
        """

        goal = self.goal_position
        # goal.x = self.cnn_data.goal_cart[0]
        # goal.y = self.cnn_data.goal_cart[1]


        # distance to goal:
        dist_to_goal = np.linalg.norm(
            np.array([
                self.curr_pose.position.x - goal.x,
                self.curr_pose.position.y - goal.y,
                self.curr_pose.position.z - goal.z
            ])
        )

        t_1 = self.num_iterations % 10
        if(self.num_iterations == 0):
            self.dist_to_goal_reg = np.ones(10)*dist_to_goal

        reward = 0.0

        if(dist_to_goal <= self.GOAL_RADIUS):
            reward = r_arrival
        elif(self.num_iterations >= self.max_iteration):
            reward = -r_arrival
        else:
            reward = r_waypoint*(self.dist_to_goal_reg[t_1] - dist_to_goal)

        self.dist_to_goal_reg[t_1] = dist_to_goal

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Goal reward: {}  {}".format(dist_to_goal, reward))
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
        if(min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.00):
            reward = r_collision
        elif(min_scan_dist < 3*self.ROBOT_RADIUS):
            reward = r_scan * (3*self.ROBOT_RADIUS - min_scan_dist)
        else:
            reward = 0.0

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Obstacle collision reward: {}".format(reward))
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
            self.node.get_logger().warning("Angular reward: {} {}".format(reward, w_z))
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
        theta_pre = np.arctan2(goal[1], goal[0])
        d_theta = theta_pre

        # get the pedstrain's position:
        if(len(mht_peds) != 0):  # tracker results
            d_theta = np.pi/2 #theta_pre
            N = 60
            theta_min = 1000
            for i in range(N):
                theta = random.uniform(-np.pi, np.pi)
                free = True
                for ped in mht_peds.tracks:
                    #ped_id = ped.track_id 
                    # create pedestrian's postion costmap: 10*10 m
                    p_x = ped.pose.pose.position.x
                    p_y = ped.pose.pose.position.y
                    p_vx = ped.twist.twist.linear.x
                    p_vy = ped.twist.twist.linear.y
                    
                    ped_dis = np.linalg.norm([p_x, p_y])
                    if(ped_dis <= 7):
                        ped_theta = np.arctan2(p_y, p_x)
                        vo_theta = np.arctan2(3*self.ROBOT_RADIUS, np.sqrt(ped_dis**2 - (3*self.ROBOT_RADIUS)**2))
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

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("Theta reward: {}".format(reward))
        return reward  

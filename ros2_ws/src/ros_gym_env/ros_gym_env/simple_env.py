#!/usr/bin/env python3
import subprocess
import time, math
import random
import signal
import atexit
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


class RosUnitySimpleEnv(gym.Env):
    def __init__(self, env_id, max_iteration=1024):
        super().__init__()
        self.env_id = env_id
        self.prefix = "/env_" + str(self.env_id)
        self.max_iteration = max_iteration

        # robot parameters:
        self.ROBOT_RADIUS = 0.35
        self.GOAL_RADIUS = 0.3 #0.3
        self.DIST_NUM = 10
        self.min_linear_velocity = -1.0
        self.max_linear_velocity = 1.0

        self.min_angular_velocity = -2.0
        self.max_angular_velocity = 2.0

         # bumper:
        self.bump_flag = False
        self.bump_num = 0

        # reward:
        self.dist_to_goal_reg = np.zeros(10)
        self.num_iterations = 0

        # Initialisation Node
        self.node = Node("ros_unity_env_"+str(self.env_id))
        self.wait_for_message_node = Node("wait_for_message_tester")

        # === Spaces ===
        self.scan_size = 720  # ajustable
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        # self.observation_space = gym.spaces.Box(low=0.0, high=10.0, shape=(self.scan_size + 2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=0.0, high=10.0, shape=(2,), dtype=np.float32)

        # observation space
        self.scan = np.zeros(self.scan_size, dtype=np.float32)
        self.goal = np.zeros(2, dtype=np.float32)

        # info, initial position and goal position
        self.init_pose = Pose()
        self.curr_pose = Pose()
        self.goal_position = Point()

        # goal reached flag:
        self._goal_reached = False
        # reset flag:
        self._reset = True

        # === ROS topics ===
        self.node.create_subscription(PoseStamped, self.prefix + "/robot_pose", self._robot_pose_callback, 1)
        self.node.create_subscription(LaserScan, self.prefix + '/scan', self._laser_callback, 1)
        self.node.create_subscription(PoseStamped, self.prefix + '/global_goal', self._goal_callback, 1)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, self.prefix + '/cmd_vel', 10)

        self.obs = None
        self.done = False

        # === Services ===
        self.reset_client = self.node.create_client(Trigger, self.prefix + '/unity/reset')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Attente du service de reset...')

        self.t = time.time()
        # self.rate = self.node.create_timer(1, self.node.get_clock)


    def reset(self, *, seed=None, options=None, **kwargs):
        """ 
        obs, info = env.reset() 
        """
        if seed is not None:
            np.random.seed(seed)
        self._set_init()
        obs = self._get_observation()
        info = {}
        return obs, info
    
    def _set_init(self):
        self.node.get_logger().warning("Start initializing robot...", once=True)
        self.cmd_vel_publisher.publish(Twist())

        if(self._reset): 
            self._reset = False
            req = Trigger.Request()
            future = self.reset_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
                
        time.sleep(1)

        # initalize info:
        self.init_pose = self.curr_pose # inital_pose.pose.pose
        self.curr_pose = self.curr_pose # inital_pose.pose.pose
        self.ts_cnt = 0 
        self.node.get_logger().warning("Robot was initiated as {}".format(self.init_pose), once=True)

        # reset pose valid flag:
        self.pos_valid_flag = True
        # reset bumper:
        self.bump_flag = False
        # self.bump_num = 0
        # reset the number of iterations:
        self.num_iterations = 0
        # reset distance to goal register:
        self.dist_to_goal_reg = np.zeros(10)

        # Give the system a little time to finish initialization
        self.node.get_logger().warning("Finish initialize robot.", once=True)
        
        return self.init_pose, self.goal_position
        
    def _robot_pose_callback(self, robot_pose_msg):
        self.curr_pose = robot_pose_msg.pose

    def _laser_callback(self, laser_msg):
        self.scan = np.array(laser_msg.ranges, dtype=np.float32)
        
    def _goal_callback(self, final_goal_msg):
        self.goal_position = final_goal_msg.pose.position
        self.goal = self.global_goal_from_local()
        # self.goal = [final_goal_msg.pose.position.x, final_goal_msg.pose.position.y]

    def send_action(self, action):
        cmd_vel = Twist()

        cmd_vel.linear.x = (float(action[0]) + 1) * (self.max_linear_velocity - self.min_linear_velocity) / 2 + self.min_linear_velocity
        cmd_vel.angular.z = (float(action[1]) + 1) * (self.max_angular_velocity - self.min_angular_velocity) / 2 + self.min_angular_velocity

        self.cmd_vel_publisher.publish(cmd_vel)

    def step(self, action):
        self.send_action(action)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._is_done(reward)
        truncated = False

        # self.node.get_logger().info("Obs : {}       Reward : {}      Done : {}".format(self.goal, reward, done))

        return obs, reward, done, truncated, {}


    def _is_done(self, reward):
        self.num_iterations += 1

        # 1) Goal reached?
        dist_to_goal = np.linalg.norm(self.goal)
        if(dist_to_goal <= self.GOAL_RADIUS):
            self.cmd_vel_publisher.publish(Twist())
            self._reset = True # reset the simulation world
            return True

        # 2) Obstacle collision?
        scan = self.scan
        min_scan_dist = np.amin(scan[scan!=0])
        if(min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.00):
            self.bump_num += 1
        if(self.bump_num >= 3):
            self.cmd_vel_publisher.publish(Twist())
            self.bump_num = 0
            self._reset = True # reset the simulation world
            return True

        # 4) maximum number of iterations?
        if(self.num_iterations > self.max_iteration):
            self.cmd_vel_publisher.publish(Twist())
            self._reset = True
            return True

        return False

    def _get_observation(self):
        """
        Returns the observation.
        """
        # self.node.get_logger().info(str(self.scan) + "    " + str(self.goal))
        # dist_to_goal = np.linalg.norm(self.goal)
        # self.observation = np.concatenate((self.scan, self.goal), axis=None)
        self.observation = self.goal
        return self.observation
    
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
        r_waypoint = 3.2 #2.5 #1.6 #2 #3 #1.6 #6 #2.5 #2.5
        r_collision = -20 #-15
        r_scan = -0.2 #-0.15 #-0.3

        # reward parts:
        r_g = self._goal_reached_reward(r_arrival, r_waypoint)
        r_c = self._obstacle_collision_punish(self.scan, r_scan, r_collision)
        reward = r_g + r_c
        return reward

    def _goal_reached_reward(self, r_arrival, r_waypoint):
        """
        Returns positive reward if the robot reaches the goal.
        :param transformed_goal goal position in robot frame
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        # distance to goal:
        # dist_to_goal = np.linalg.norm(
        #     np.array([
        #     self.curr_pose.position.x - self.goal_position.x,
        #     self.curr_pose.position.y - self.goal_position.y,
        #     self.curr_pose.position.z - self.goal_position.z
        #     ])
        # )
        dist_to_goal = np.linalg.norm(self.goal)

        t_1 = self.num_iterations % 10
        if(self.num_iterations == 0):
            self.dist_to_goal_reg = np.ones(10)*dist_to_goal

        reward = 0.0
        # reward calculation:
        if(dist_to_goal <= self.GOAL_RADIUS):  # goal reached: t = T
            reward = r_arrival
        elif(self.num_iterations >= self.max_iteration):  # failed to the goal
            reward = -r_arrival
        else:   # on the way
            reward = r_waypoint*(self.dist_to_goal_reg[t_1] - dist_to_goal)
            # if self.dist_to_goal_reg is not None:
            #     reward += r_waypoint*(self.dist_to_goal_reg - dist_to_goal)

        # storage the robot pose at t-1:
        # self.dist_to_goal_reg = dist_to_goal
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
        if(min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.00):
            reward = r_collision
        elif(min_scan_dist < 3*self.ROBOT_RADIUS):
            reward = r_scan * (3*self.ROBOT_RADIUS - min_scan_dist)
        else:
            reward = 0.0

        # rospy.logwarn("Obstacle collision reward: {}".format(reward))
        return reward
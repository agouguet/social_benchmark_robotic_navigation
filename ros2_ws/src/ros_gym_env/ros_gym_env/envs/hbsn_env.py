#!/usr/bin/env python3
from ros_gym_env.envs.ros_env import ROSEnv
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
from agents_msgs.msg import AgentArray, Agent
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from cnn_msgs.msg import AllCNNdata
from threading import Event



class HBSNEnv(ROSEnv):
    def __init__(self, env_id, config, env_id_display_log=None):
        super().__init__(env_id, config, env_id_display_log)

        self.goal_radius = self.config.env.reward.goal_radius
        self.dist_goal_history_number = self.config.env.reward.goal_dist_history_number

    def init_info(self):
        self.steps_on_goal = 0
        # reset bumper:
        self.bump_num = 0
        dist_to_goal = self.dist_to_goal()
        self.dist_to_goal_reg = np.ones(10)*dist_to_goal

    def _get_observation(self):
        obs = ()

        # scan:
        if self.config.env.obs.scan:
            scan = np.array(self.scan, dtype=np.float32)
            scan = scan.reshape(self.scan_history*self.scan_size)
            debug_index = self.scan_size

            if self.config.env.obs.scan_avg_pool or self.config.env.obs.scan_min_pool:
                size_slice = int(self.scan_size/self.nb_slice)
                scan = scan.reshape(self.scan_history, size_slice, self.nb_slice)
                stack = ()
                size = 0
                if self.config.env.obs.scan_avg_pool:
                    stack += (np.mean(scan, axis=2),)
                    size += 1
                if self.config.env.obs.scan_min_pool:
                    stack += (np.min(scan, axis=2),)
                    size += 1
                scan_avg = np.stack(stack, axis=1).reshape(size*self.scan_history, size_slice)
                scan = scan_avg.reshape(size*self.scan_history*size_slice)
                debug_index = size_slice
            scan = np.tile(scan, self.scan_tile)
            if self.config.env.obs.scan_norm:
                s_min = 0
                s_max = 10
                scan = np.clip(scan, s_min, s_max)
                scan = scan / s_max
            obs += (scan,)
            if(self.ros_debug):
                self.publish_debug_scan_observation(scan[-debug_index:])
        
        # goal:
        if self.config.env.obs.goal:
            # goal = np.array([
            #                     self.local_goal_from_robot.x,
            #                     self.local_goal_from_robot.y
            #                 ], dtype=np.float32)
            # obs += (goal,)
            angle = np.arctan2(self.local_goal_from_robot.y, self.local_goal_from_robot.x)  # entre -pi et pi
            normalized_angle = angle / np.pi  # normalisé entre -1 et 1
            obs += (normalized_angle,)

        dist_to_goal = 0
        if self.config.env.obs.goal_dist:
            dist_to_goal = self.dist_to_goal()
            dist_to_goal = np.clip(dist_to_goal, 0, 1)
            obs += (dist_to_goal,)

        # vel:
        vel = []
        if self.config.env.obs.robot_velocity:
            vel = np.array([self.curr_vel.linear.x, self.curr_vel.angular.z], dtype=np.float32)
            obs += (vel,)

        # human:
        if self.config.env.obs.human:
            for agent in self.agents:
                obs += (agent,)

        # observation:
        self.observation = np.concatenate((obs), axis=None)

        if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
            self.node.get_logger().warning("\n\n", throttle_duration_sec=self.config.log.throttle_duration)
            self.node.get_logger().warning(
                "Observation ({}) => \n goal: {} \n dist_to_goal: {} \n vel: {} \n agents: {} \n scan: {}".format(self.observation.shape, normalized_angle, dist_to_goal, vel, self.agents, scan), 
                throttle_duration_sec=self.config.log.throttle_duration)
        
        return self.observation
    

    def _is_done(self, reward):
        if(self.dist_to_goal() <= self.goal_radius):
            self.steps_on_goal += 1
            if self.steps_on_goal >= self.config.env.steps_on_goal_required:
                return True
        else:
            self.steps_on_goal = 0

        scan = np.array(self.scan[-self.scan_size:], dtype=np.float32)
        scan = scan[(scan > 0) & np.isfinite(scan)]
        min_scan_dist = np.min(scan) if scan.size > 0 else math.inf

        if(min_scan_dist <= self.robot_radius and min_scan_dist >= 0.02):
            self.bump_num += 1

        if(self.bump_num >= 3):
            return True

        if(self.num_iterations > self.max_iteration):
            return True

        return False
    
    def _compute_reward(self):
        """Calculates the reward to give based on the observations given.
        """
        # reward parameters:
        r_backward = self.config.env.reward.backward
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
        r_b = self._backward_reward(r_backward, self.curr_vel.linear.x)
        r_g = self._goal_reached_reward(r_arrival, r_waypoint)
        r_c = self._obstacle_collision_punish(self.scan[-self.scan_size:], scan_penalty_threshold_factor, r_scan, r_collision)
        r_w = self._angular_velocity_punish(self.curr_vel.angular.z,  r_rotation, w_thresh)
        r_t = self._theta_reward(self.local_goal_from_robot, self.mht_peds, self.curr_vel.linear.x, r_angle, angle_thresh)
        r_h = self._human_reward(r_human)
        reward = self.config.env.reward.constant + r_b + r_g + r_c + r_t + r_h #+ r_w #+ r_v # + r_p
        if self.dist_to_goal() < self.goal_radius*2:
            reward -= self.curr_vel.linear.x * 0.5


        if ((self.env_id_display_log == self.env_id or self.env_id_display_log == None)):
            self.node.get_logger().warning("Compute reward done. \nreward = {}\n    rb: {}\n    rg: {}\n    rc: {}\n    rw: {}\n    rt: {}\n    rh: {}".format(reward, r_b, r_g, r_c, r_w, r_t, r_h)
                                           , throttle_duration_sec=self.config.log.throttle_duration
                                           )
            # self.node.get_logger().warning("Dist Goal Arr: \n{}   {}\n{}   {}".format(self.num_iterations, self.dist_to_goal_reg, self.curr_pose.position, self.final_goal)
            #                                , throttle_duration_sec=self.config.log.throttle_duration
            #                                )
        return reward

    def _backward_reward(self, r_backward, v_x):
        return -r_backward * min(0, v_x)

    def _goal_reached_reward(self, r_arrival, r_waypoint):
        """
        Returns positive reward if the robot reaches the goal.
        :param transformed_goal goal position in robot frame
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        dist_to_goal = self.dist_to_goal()

        t_1 = self.num_iterations % 10
        if(self.num_iterations == 0):
            self.dist_to_goal_reg = np.ones(10)*dist_to_goal

        reward = 0.0

        if(dist_to_goal <= self.goal_radius):
            reward = r_arrival
            if self.steps_on_goal > 0:
                reward += self.steps_on_goal * self.config.env.reward.goal_stay_bonus
        elif(self.num_iterations >= self.max_iteration):
        # elif(self.start_time < self.get_time() - self.max_time):
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
        # prefer goal theta:
        theta_pre = np.arctan2(goal.y, goal.x)
        d_theta = theta_pre
        reward = r_angle*(angle_thresh - abs(d_theta)) * min(0, v_x / self.max_linear_velocity)
        return reward  
    
    def _human_reward(self, r_human):
        reward = 0.0
        hps = self.config.env.reward.human_personal_distance

        # for agent in self.agents:
        #     dist = agent[4]
        #     if dist > 0 and dist < hps:
        #         reward -= (hps - dist) * r_human

        dists = np.array([a[4] for a in self.agents])
        mask = (dists > 0) & (dists < hps)
        reward -= np.sum((hps - dists[mask]) * r_human)

        return reward
    
    def _post_information(self, done):
        self.info = {
            "goal": self.goal,
            "current_pose": self.curr_pose
            }
        
        if (done):
            success = False
            if(self.dist_to_goal() <= self.goal_radius):
                success = True
            self.info["is_success"] = success
        return self.info
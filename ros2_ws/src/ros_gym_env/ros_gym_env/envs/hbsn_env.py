#!/usr/bin/env python3
from ros_gym_env.envs.ros_env import ROSEnv
import time, math
import numpy as np
from hbsn.solver.heuristicfunction import heuristic_score_based
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import gymnasium as gym

class HBSNEnv(ROSEnv):
    def __init__(self, env_id, config, env_id_display_log=None):
        super().__init__(env_id, config, env_id_display_log)

        self.goal_radius = self.config.env.reward.goal_radius
        self.dist_goal_history_number = self.config.env.reward.goal_dist_history_number
        self.goal_social = self.config.env.obs.goal_social

        if self.ros_debug:
            self.debug_social_sub_goal_pub = self.node.create_publisher(MarkerArray, f"{self.prefix}/debug/actions_marker", 10)

    def init_info(self):
        self.steps_on_goal = 0
        self.bump_num = 0
        dist_to_goal = self.dist_to_goal()
        self.dist_to_goal_reg = np.ones(10)*dist_to_goal

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
        self.human_obs_size = self.human_number * 4

        obs_shape = 0
        if use_goal:
            obs_shape += 2 #with heuristic
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

    def generate_actions(self, robot_pos, radius=1.0, num_actions=8):
        angles = np.linspace(0, 2*np.pi, num_actions, endpoint=False)
        return [tuple(robot_pos + np.array([radius * np.cos(a), radius * np.sin(a)])) for a in angles]

    def filter_actions_with_laser(self, actions, scan, robot_pos, robot_radius=0.3, max_distance=10.0):
        filtered = []
        scan = np.array(scan, dtype=np.float32)
        scan = scan[(scan > 0) & np.isfinite(scan)]
        
        if scan.size == 0:
            return actions

        for a in actions:
            dist = np.linalg.norm(np.array(a))
            if dist < robot_radius:  
                continue
            if dist < np.min(scan) + robot_radius:
                continue
            filtered.append(a)
        return filtered

    def _get_observation(self):
        obs = ()
        
        # --- Scan processing ---
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

        # --- Goal info ---
        if self.config.env.obs.goal:
            angle = np.arctan2(self.local_goal_from_robot.y, self.local_goal_from_robot.x)
            normalized_angle = angle / np.pi
            obs += (normalized_angle,)

            # --- Heuristic best action ---
            if self.goal_social:
                robot_pos = np.array([self.curr_pose.position.x, self.curr_pose.position.y])

                radius_action = 1.0
                num_actions = 8

                if self.global_path is not None and len(self.global_path.poses) > 2:
                    next_goal_pose = self.global_path.poses[1].pose.position
                    next_goal = np.array([next_goal_pose.x, next_goal_pose.y])
                    if np.linalg.norm(next_goal - robot_pos) < radius_action:  # seuil à ajuster
                        next_goal = np.array([self.final_goal.x, self.final_goal.y])
                else:
                    next_goal = np.array([self.final_goal.x, self.final_goal.y])

                # self.publish_marker(next_goal, topic_name="sub_goal")
                
                actions = self.generate_actions(robot_pos, radius=radius_action, num_actions=num_actions)
                filtered_actions = actions #self.filter_actions_with_laser(actions, self.scan[-self.scan_size:], robot_pos, robot_radius=self.robot_radius)

                if len(filtered_actions) == 0:
                    filtered_actions = [robot_pos]

                best_action, debug = heuristic_score_based(
                    robot_pos, 
                    next_goal, 
                    filtered_actions,
                    previous_path=[],
                    humans=[[a[0], a[1]] for a in self.agents[-1] if np.linalg.norm([a[0], a[1]]) > 0.01],
                    future_humans=[],
                    return_debug=True,
                    w1=4.0, w2=15.0, w3=1.0, w4=1.0, w5=1.0,
                )
                
                best_action = np.array(best_action, dtype=np.float32)
                # self.publish_action(filtered_actions, best_action)
                # obs += (best_action,)
                angle_best_action = np.arctan2(best_action[1], best_action[0])
                normalized_angle_best_action = angle_best_action / np.pi
                obs += (normalized_angle_best_action,)
            
        
        if self.config.env.obs.goal_dist:
            dist_to_goal = np.clip(self.dist_to_goal(), 0, 1)
            obs += (dist_to_goal,)

        # --- Velocities ---
        if self.config.env.obs.robot_velocity:
            vel = np.array([self.curr_vel.linear.x, self.curr_vel.angular.z], dtype=np.float32)
            obs += (vel,)
        
        # --- Humans ---
        if self.config.env.obs.human:
            for agent_t in self.agents:
                for agent in agent_t:
                    obs += (agent[:-1],)
                # obs += ([agent[0], agent[1], agent[2], agent[3]],)

        # Flatten
        self.observation = np.concatenate((obs), axis=None)
        # if (self.env_id_display_log == self.env_id or self.env_id_display_log == None):
        #     self.node.get_logger().warning(
        #         "\n\nObservation ({})\nRobot => \n    goal: {} {} \n    dist_to_goal: {} \nHuman => \n    agents: {} \nScan => \n    scan: {}".format(self.observation.shape, normalized_angle, normalized_angle_best_action, dist_to_goal, self.agents, scan), 
        #         throttle_duration_sec=self.config.log.throttle_duration)
        #     self.node.get_logger().warning(
        #         "Observation ({}) => \n goal: {} \n dist_to_goal: {} \n vel: {} \n agents: {} \n scan: {}".format(self.observation.shape, normalized_angle, dist_to_goal, vel, self.agents, scan), 
        #         throttle_duration_sec=self.config.log.throttle_duration)
        return self.observation
    

    def _is_done(self, reward):
        if(self.dist_to_goal() <= self.goal_radius):
            self.steps_on_goal += 1
            if self.steps_on_goal >= self.config.env.steps_on_goal_required:
                return True

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
        r_arrival = self.config.env.reward.goal_arrival     
        r_waypoint = self.config.env.reward.goal_waypoint   
        r_collision = self.config.env.reward.collision      
        r_scan = self.config.env.reward.scan                
        r_angle = self.config.env.reward.theta_angle        
        r_rotation = self.config.env.reward.rotation        
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


        # if ((self.env_id_display_log == self.env_id or self.env_id_display_log == None)):
        #     self.node.get_logger().warning("Compute reward done. \nreward = {}     total🪙 = {}\n    rb: {}\n    rg: {}\n    rc: {}\n    rw: {}\n    rt: {}\n    rh: {}".format(reward, self.reward_episode, r_b, r_g, r_c, r_w, r_t, r_h)
        #                                    , throttle_duration_sec=self.config.log.throttle_duration
        #                                    )
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
            reward += r_arrival / max(1, self.config.env.steps_on_goal_required)
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
            "current_pose": self.curr_pose,
            "episode_steps":self.num_iterations
            }
        
        if (done):
            success = False
            if(self.dist_to_goal() <= self.goal_radius):
                success = True
            self.info["is_success"] = success
        return self.info
    
    def publish_action(self, actions, best_action):
        marker_array = MarkerArray()
        for i, a in enumerate(actions):
            marker = Marker()
            marker.header.frame_id = "env_" + str(self.env_id)+"/map"
            if self.raw_last_scan is not None:
                marker.header.stamp = self.raw_last_scan.header.stamp
            else:
                marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "actions"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = a[0]
            marker.pose.position.y = a[1]
            marker.pose.position.z = 0.05  # légèrement au-dessus du sol
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            if np.allclose(a, best_action):
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.6
            marker_array.markers.append(marker)

        self.debug_social_sub_goal_pub.publish(marker_array)
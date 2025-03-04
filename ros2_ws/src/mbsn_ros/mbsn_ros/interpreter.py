#!/usr/bin/env python3
from collections import defaultdict
from itertools import combinations
import math
import os
import random
import time
from abc import ABC, abstractmethod

from matplotlib import pyplot as plt
# from human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction

import mbsn
from mbsn.human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction
from mbsn.mdp.State import State
from mbsn.mdp.MBSN import MBSN
from mbsn.mdp.agent import Agent
from mbsn.polygonal_map.polygonal_map import PolygonalMap
from mbsn.solver.MCTS import MBSNAgentMCTS
from mbsn.solver.heuristicfunction import heuristic_rules_based, heuristic_score_based
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.solver.qtable import QTable
import numpy as np
from scipy.spatial import KDTree, Voronoi

import rclpy # type: ignore
from rclpy.node import Node # type: ignore


from shapely import LineString, MultiPolygon, Point, Polygon, convex_hull, unary_union
from shapely.ops import nearest_points
from geometry_msgs.msg import Pose, Point as RosPoint # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from mbsn_ros.util import ros_point_to_shapely_point, ros_quaternion_to_euler
from visualization_msgs.msg import Marker, MarkerArray # type: ignore
from std_msgs.msg import Header # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from geometry_msgs.msg import PoseArray # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from nav_msgs.msg import Odometry, OccupancyGrid # type: ignore
from nav_msgs.msg import Path # type: ignore
from agents_msgs.msg import AgentArray # type: ignore
from graph_msgs.msg import GraphNav # type: ignore
from simulation_msgs.msg import SceneInfo # type: ignore
from agents_msgs.msg import AgentArray, AgentTrajectories, AgentTrajectory # type: ignore
from ament_index_python.packages import get_package_share_directory

MIN_TIME_OCCUPANCY = 10
TIME_OCCUPANCY = 30
POURCENTAGE_AREA_OF_POLYGON_ACCEPTABLE = 0.5


class Interpreter(Node):

    def __init__(self, name_node="mbsn"):
        super().__init__(name_node)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('polygon_size', 0.7),
                ('polygon_type', "hexagon"),
                ('scenario', ""),
                ('odom', "odom"),
                ('velocity', "cmd_vel"),
                ('agents', "agents"),
                ('agents_trajectory', "agent/trajectories"),
                ('global_goal', "global_goal"),
                ('goal', "goal_pose"),
                ('scene_info', "/social_sim/scene_info"),
                ('polygon_map', "polygon_map"),
                ('robot_visibility_distance', 4.0),
                ('global_path_topic', 'global_path'),
                ('local_path_topic', 'local_path')
            ])

        self.polygon_size = self.get_parameter('polygon_size').value
        self.polygon_type = self.get_parameter('polygon_type').value
        self.scenario = self.get_parameter('scenario').value

        self.get_logger().info('Scenario : ' + str(self.scenario))
        if self.scenario != "":
            self.polygonal_map = PolygonalMap(self.scenario, type=self.polygon_type , polygon_size=self.polygon_size, area_minimum=POURCENTAGE_AREA_OF_POLYGON_ACCEPTABLE)
        else:
            self.polygonal_map = None

        self._odom_topic_name = self.get_parameter('odom').value
        self._velocity_topic_name = self.get_parameter('velocity').value
        self._agents_topic_name = self.get_parameter('agents').value
        self._agents_trajectory_topic_name = self.get_parameter('agents_trajectory').value
        self._global_goal_topic_name = self.get_parameter('global_goal').value
        self._goal_topic_name = self.get_parameter('goal').value
        self._scene_info_topic_name = self.get_parameter('scene_info').value
        self._polygon_map_topic_name = self.get_parameter('polygon_map').value
        self._robot_visibility_distance = self.get_parameter('robot_visibility_distance').value
        self.global_path_topic = self.get_parameter('global_path_topic').value
        self.local_path_topic = self.get_parameter('local_path_topic').value

        self.mdp = None
        self.scenario = None
        
        self.goal = None
        self.path = []
        self.current_state = None
        self.humans = {}
        self.humans_state = []
        self.global_path = None
        self.can_publish_local_goal = True

        self.subsriber_publisher_definition()

    def subsriber_publisher_definition(self):
        # Agents
        self.robot_position_subscription_ = self.create_subscription(Odometry, self._odom_topic_name, self.robot_odom_callback, 1)
        self.velocity_subscription_ = self.create_subscription(Twist, self._velocity_topic_name, self.velocity_callback, 1)
        self.humans_position_subscription_ = self.create_subscription(AgentArray, self._agents_topic_name, self.humans_callback, 1)
        self.humans_trajectories_subscription_  = self.create_subscription(AgentTrajectories, self._agents_trajectory_topic_name, self.humans_trajectory_callback, 1)

        # Goal
        self.goal_subscription_ = self.create_subscription(PoseStamped, self._global_goal_topic_name, self.goal_callback, 10)
        self.goal_publisher_ = self.create_publisher(PoseStamped, self._goal_topic_name, 10)

        # Scene
        self.scene_info_subscriber_ = self.create_subscription(SceneInfo, self._scene_info_topic_name, self.scene_info_callback, 10)

        # Map
        # self.local_map_subscriber_ = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_map_callback, 10)
        self.polygon_publisher_ = self.create_publisher(MarkerArray, self._polygon_map_topic_name, 10)
        self.exterior_polygon_publisher_ = self.create_publisher(MarkerArray, "/polygon_map/exterior", 10)

        # Global Path
        self.global_path_subscriber = self.create_subscription(Path, self.global_path_topic, self.global_path_callback, 10)

        # Local Path
        self.local_path_publisher = self.create_publisher(Path, self.local_path_topic, 10)

    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()
            self.get_logger().info('Scenario received: ' + str(self.scenario))

            if "/" in self.scenario:
                self.polygonal_map = PolygonalMap(self.scenario, type=self.polygon_type , polygon_size=self.polygon_size, area_minimum=POURCENTAGE_AREA_OF_POLYGON_ACCEPTABLE)
            else:
                package_share_directory = get_package_share_directory('assets')
                map_config_path = package_share_directory+"/maps/"+self.scenario+"/"
                self.polygonal_map = PolygonalMap(map_config_path, type=self.polygon_type , polygon_size=self.polygon_size, area_minimum=POURCENTAGE_AREA_OF_POLYGON_ACCEPTABLE)


    def goal_callback(self, msg_goal):
        pos = msg_goal.pose.position
        goal = ros_point_to_shapely_point(pos)
        if self.goal != goal:
            self.goal = goal
            self.get_logger().info('Goal received: ' + str(self.goal))

    @abstractmethod
    def robot_odom_callback(self, msg_odom):
        pass

    @abstractmethod
    def velocity_callback(self, msg_vel):
        pass

    @abstractmethod
    def update(self):
        pass
        
    def global_path_callback(self, msg):
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def humans_callback(self, msg_agents):
        agent_ids = []
        for agent in msg_agents.agents:
            if agent.visible_by_robot:
                agent_ids.append(agent.id)
                if agent.id not in self.humans:
                    self.humans[agent.id] = Agent(ros_point_to_shapely_point(agent.pose.position), ros_quaternion_to_euler(agent.pose.orientation), id=agent.id)
                else:
                    self.humans[agent.id].position = ros_point_to_shapely_point(agent.pose.position)
                    self.humans[agent.id].orientation = ros_quaternion_to_euler(agent.pose.orientation)

        if self.mdp is not None:
            id_to_remove = []
            for id, human in self.humans.items():
                if id not in agent_ids:
                    if self.mdp.get_state_from_continuous_position(human.position) is not None:
                        id_to_remove.append(id)
            for id in id_to_remove:
                del self.humans[id]
    
    def interpolate_path(self, points, max_dist):
        new_points = [points[0]]

        for i in range(1, len(points)):
            p1, p2 = points[i - 1], points[i]
            d = p1.distance(p2)

            if d > max_dist:
                num_extra_points = int(np.ceil(d / max_dist)) - 1
                for j in range(1, num_extra_points + 1):
                    alpha = j / (num_extra_points + 1)
                    new_point = (1 - alpha) * np.array([p1.x, p1.y]) + alpha * np.array([p2.x, p2.y])
                    new_points.append(Point(new_point))

            new_points.append(p2)

        return new_points

    def humans_trajectory_callback(self, msg_traj):
        self.human_trajectories = defaultdict(list)

        for traj in msg_traj.trajectories:
            if traj.id in self.humans:
                trajs = []
                for pose in traj.poses:
                    trajs.append(ros_point_to_shapely_point([pose.x, pose.y]))
                self.humans[traj.id].future_predicted_position = trajs #self.interpolate_path(trajs, 0.3)


    def publish_local_goal(self, position):
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = float(position.x)
        msg.pose.position.y = float(-position.y)

        self.published_local_goal = position
        self.goal_publisher_.publish(msg)
        self.can_publish_local_goal = True

    def publish_local_path(self, path):
        self.path = path
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.local_path_publisher.publish(msg)

    def publish_polygon_map_vizualisation(self):
        self.clear_polygon_map_markers()
        marker_array = MarkerArray()
        markers = []

        id = 0

        for poly in self.mdp.polygons.values():
            # for p in poly.polygon:
            b = poly.polygon.exterior.coords
            linestrings = [LineString(b[k:k+2]) for k in range(len(b) - 1)]
            for l in linestrings:
                marker = Marker()

                marker.header = Header()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "map"

                marker.id = id = id+1
                marker.type = marker.LINE_STRIP

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03

                a, b = l.boundary.geoms

                p1 = RosPoint()
                p1.x = float(a.x)
                p1.y = float(a.y)

                p2 = RosPoint()
                p2.x = float(b.x)
                p2.y = float(b.y)

                points = [p1, p2]

                marker.points = points
                markers.append(marker)
                id += 1
            
            # TEXT
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"

            marker.id = id = id+1
            marker.type = marker.TEXT_VIEW_FACING     

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            pose = Pose()
            c = poly.polygon.centroid
            pose.position.x = float(c.x)
            pose.position.y = float(c.y) - 0.15
            pose.position.z = 1.0
            marker.pose = pose
            marker.text = str(poly.id)
            markers.append(marker)
            # print([list(ls.coords) for ls in linestrings])

        marker_array.markers = markers

        self.polygon_publisher_.publish(marker_array)

    def clear_polygon_map_markers(self):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = marker.DELETEALL
        markers.markers.append(marker)
        self.polygon_publisher_.publish(markers)


    def angle_between_vectors(self, v1, v2):
        """Calcule l'angle (en degrés) entre deux vecteurs v1 et v2"""
        dot_product = np.dot(v1, v2)
        norm_product = np.linalg.norm(v1) * np.linalg.norm(v2)
        angle_rad = np.arccos(np.clip(dot_product / norm_product, -1.0, 1.0))
        return np.degrees(angle_rad)

    def publish_exterior_polygon_map_vizualisation(self):
        self.clear_exterior_polygon_map_markers()
        marker_array = MarkerArray()
        markers = []

        id = 0

        if type(self.exterior_polygon) == Polygon:
            self.exterior_polygon = MultiPolygon([self.exterior_polygon])


        for poly in self.exterior_polygon.geoms:
            b = poly.exterior.coords
            linestrings = [LineString(b[k:k+2]) for k in range(len(b) - 1)]
            for l in linestrings:
                marker = Marker()

                marker.header = Header()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "map"

                marker.id = id = id+1
                marker.type = marker.LINE_STRIP

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03

                a, b = l.boundary.geoms

                p1 = RosPoint()
                p1.x = float(a.x)
                p1.y = float(a.y)

                p2 = RosPoint()
                p2.x = float(b.x)
                p2.y = float(b.y)

                points = [p1, p2]

                marker.points = points
                markers.append(marker)
                id += 1

        marker_array.markers = markers

        self.exterior_polygon_publisher_.publish(marker_array)

    def clear_exterior_polygon_map_markers(self):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = marker.DELETEALL
        markers.markers.append(marker)
        self.exterior_polygon_publisher_.publish(markers)
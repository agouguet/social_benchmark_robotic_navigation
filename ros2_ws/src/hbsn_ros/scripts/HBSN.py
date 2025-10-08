#!/usr/bin/env python3
import time

from hbsn.human_trajectory_prediction.human_trajctory_prediction_model import simple_human_trajectory_prediction
from hbsn.mdp.State import State
from hbsn.mdp.HBSN import HBSN
from hbsn.solver.heuristicfunction import heuristic_score_based
from hbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from hbsn.solver.qtable import QTable
import numpy as np

import rclpy # type: ignore

from shapely import Point
from geometry_msgs.msg import Pose # type: ignore
from hbsn_ros.interpreter import Interpreter
from hbsn_ros.util import ros_point_to_shapely_point
from std_msgs.msg import Header # type: ignore
from nav_msgs.msg import OccupancyGrid # type: ignore


id_points = {}

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if position in id_points:
        return id_points[position]

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.polygon.buffer(0.1).contains(position):
                id_points[position] = id
                return id
    return None

LIMIT_DISTANCE_WITH_HUMAN_TO_MOVING = 0.7

class MBSNNode(Interpreter):

    def __init__(self, name_node="mbsn_sim"):
        super().__init__(name_node)

        self.mdp = None
        self.state = None
        self.local_state_timed = time.time()
        self.qfunction = QTable(default=-1e6)
        self.bandit = UpperConfidenceBounds()
        self.start = False
        self.robot_cell = None

        self._map_publisher = self.create_publisher(OccupancyGrid, 'custom_costmap', 10)

    ################
    ### CALLBACK ###
    ################

    def robot_odom_callback(self, msg_odom):
        if self.goal is not None and self.global_path is not None:
            robot_position = msg_odom.pose.pose.position
            robot_position = ros_point_to_shapely_point(robot_position)
            self.robot_position = robot_position
            self.update()
            # if self.mdp is None or self.need_update():
            #     self.init_trajectory()
            #     # self.update()

    def velocity_callback(self, msg_vel):
        linear = msg_vel.linear
        angular = msg_vel.angular
        if linear.x != 0 or linear.y != 0 or linear.z != 0:
            self.start = True
        if angular.x != 0 or angular.y != 0 or angular.z != 0:
            self.start = True

    ##############
    ### UPDATE ###
    ##############

    def need_update(self):        
        if not self.start:
            return True

        # ROBOT STATE
        old_state_robot = self.state.robot
        new_state_robot = self.mdp.get_state_from_continuous_position(self.robot_position)
        if old_state_robot != new_state_robot:
            return True

        state_of_published_local_goal = self.mdp.get_state_from_continuous_position(self.published_local_goal)
        for _, human in self.humans.items():
            if human.position.distance(self.robot_position) <= self._robot_visibility_distance:
                human_state = self.mdp.get_state_from_continuous_position(human.position)
                if state_of_published_local_goal == human_state:
                    return True

        return False

    def update(self):
        # Safety checks
        if self.polygonal_map is None:
            return
        if self.robot_position is None or self.goal is None or self.global_path is None:
            return
        

        # Rebuild MDP (HBSN) for the current map and robot/goal
        self.mdp = HBSN(
            self.polygonal_map,
            self.robot_position,
            self.goal,
            human_trajectory_prediction_function=simple_human_trajectory_prediction,
            visibility_distance=self._robot_visibility_distance
        )

        # ROBOT/GOAL CELLS (IDs)
        robot_cell  = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, self.robot_position)
        goal_cell = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, self.goal)

        # Publish viz if robot cell changed
        if robot_cell != self.robot_cell:
            self.publish_polygon_map_vizualisation()
        self.robot_cell = robot_cell

        # If we're already at the goal cell, publish trivial local path
        if self.robot_cell == goal_cell:
            self.publish_local_path([self.robot_position, self.goal])
            return

        # LOCAL GOAL: follow global_path as far as possible inside mdp.polygons
        mbsn_goal_id = self.robot_cell 
        path = []
        for pose in self.global_path:
            pose_cell_id = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, pose)
            if pose_cell_id in self.mdp.polygons:
                mbsn_goal_id = pose_cell_id
                path.append(pose_cell_id)
            else:
                break
        mbsn_goal_point = self.mdp.polygons[mbsn_goal_id].polygon.centroid

        # Check if any human is in local scope
        human_in_local_scope = False
        for _, human in self.humans.items():
            if human.position.distance(self.robot_position) <= self._robot_visibility_distance:
                human_in_local_scope = True

        # If no humans in scope, deduplicate path and go
        if not human_in_local_scope:
            path = list(dict.fromkeys(path))
        else:
            # Start building a local path from current robot cell
            path = [self.robot_cell]

            # compute distances to humans and early stop if someone is too close
            humans_distance = {}
            for id, human in self.humans.items():
                humans_distance[id] = human.position.distance(self.robot_position)
                if humans_distance[id] <= LIMIT_DISTANCE_WITH_HUMAN_TO_MOVING:
                    self.publish_local_path([self.robot_position])
                    return

            # sort humans by distance and build ordered humans positions list
            humans_distance = {k: v for k, v in sorted(humans_distance.items(), key=lambda item: item[1])}
            humans_positions_sorted = [tuple(self.humans[id].position.coords[0]) for id in humans_distance.keys()]

            # iterate to extend local path using the new heuristic
            while True:
                current_cell = path[-1]
                # neighbors are Polygon objects stored in self.mdp.polygons[cell].neighbors
                if current_cell not in self.mdp.polygons:
                    break
                neighbors = self.mdp.polygons[current_cell].neighbors
                if not neighbors:
                    break

                # actions are the centroids of neighbor polygons (as tuples)
                actions = [tuple(p.polygon.centroid.coords[0]) for p in neighbors]

                # current robot position for heuristic: use centroid of current_cell polygon
                current_position = tuple(self.mdp.polygons[current_cell].polygon.centroid.coords[0])

                # previous_path as list of coordinate tuples (if self.path exists and is list of cell ids)
                previous_coords = []
                if hasattr(self, "path") and self.path:
                    # self.path may contain cell ids; convert to centroids where available
                    for cell_id in self.path:
                        if cell_id in self.mdp.polygons:
                            previous_coords.append(tuple(self.mdp.polygons[cell_id].polygon.centroid.coords[0]))

                # Call the new heuristic
                best_action = heuristic_score_based(
                    robot=current_position,
                    goal=(mbsn_goal_point.x, mbsn_goal_point.y),
                    actions=actions,
                    previous_path=previous_coords,
                    humans=humans_positions_sorted,
                    w1=4.0,
                    w2=15.0,
                    w3=1.0,
                    w4=1.0,
                    w5=1.0,
                    limit=1.0
                )

                # If heuristic returns None or no valid action, stop
                if best_action is None:
                    break

                # Map the chosen action (coordinate) back to a polygon cell id
                next_cell = get_cell_id_in_dict_from_continuous_position(self.polygonal_map.grid, best_action)

                # If mapping failed or next_cell is None, we cannot continue
                if next_cell is None:
                    break

                # loop prevention: if we go back to previous cell, break
                if len(path) >= 2 and next_cell == path[-2]:
                    break

                path.append(next_cell)

        # Build local_path as sequence of centroids at intersections between successive polygons
        local_path = []
        for idx in range(1, len(path)):
            if path[idx] not in self.mdp.polygons:
                break
            prev_poly = self.mdp.polygons[path[idx-1]].polygon.buffer(0.05)
            next_poly = self.mdp.polygons[path[idx]].polygon.buffer(0.05)
            inter = prev_poly.intersection(next_poly)
            if not inter.is_empty:
                local_path.append(inter.centroid)
            else:
                # fallback to centroid of next polygon if intersection is empty
                local_path.append(self.mdp.polygons[path[idx]].polygon.centroid)

        # Replace first point by actual robot position if we have >1 point
        if len(local_path) > 1:
            local_path[0] = self.robot_position

        # If goal_cell is inside mdp.polygons then append actual goal
        if goal_cell in self.mdp.polygons:
            local_path.append(self.goal)

        self.publish_local_path(local_path)
                
    def publish_costmap(self):
        costmap = OccupancyGrid()
        
        # Remplissage des métadonnées
        costmap.header = Header()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = "map"  # Assurez-vous que Rviz utilise ce frame

        costmap.info.resolution = 0.1  # Taille d'une cellule (mètres par pixel)
        costmap.info.width = 20  # Nombre de colonnes
        costmap.info.height = 20  # Nombre de lignes
        
        # Position de l'origine de la costmap
        costmap.info.origin = Pose()
        costmap.info.origin.position.x = -5.0
        costmap.info.origin.position.y = -5.0
        costmap.info.origin.position.z = 0.0

        # Génération des données de la costmap (valeurs entre 0 et 100, -1 pour inconnu)
        data = []
        for y in range(costmap.info.height):
            for x in range(costmap.info.width):
                if 5 <= x < 15 and 5 <= y < 15:  # Zone occupée au centre
                    data.append(100)  # Zone occupée
                else:
                    data.append(0)  # Zone libre
        
        costmap.data = data  # Affectation des valeurs de la costmap

        # Publication du message
        self._map_publisher.publish(costmap)
        self.get_logger().info("Costmap publiée sur /custom_costmap")


def main(args=None):
    rclpy.init(args=args)

    mbsn_node = MBSNNode()

    rclpy.spin(mbsn_node)

    mbsn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
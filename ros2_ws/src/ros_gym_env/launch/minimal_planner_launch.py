from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('ros_gym_env'), 'config', 'nav2_params.yaml'
    )

    return LaunchDescription([

        # global_costmap et planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='env_0',
            output='screen',
            parameters=[nav2_params],
        ),

        # lifecycle manager pour le planner
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            namespace='env_0',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['planner_server']
            }]
        ),
    ])

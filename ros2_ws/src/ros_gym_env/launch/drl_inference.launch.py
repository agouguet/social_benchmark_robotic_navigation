from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    drl_vo_nav_dir = get_package_share_directory('ros_gym_env')
    model_file_default = os.path.join(drl_vo_nav_dir, 'model', 'test')

    namespace = "env_0"

    return LaunchDescription([
        # Declare model file argument
        DeclareLaunchArgument(
            name='model_file',
            default_value=model_file_default,
            description='Path to trained DRL-VO model'
        ),

        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            output='screen'
        ),

        Node(
            package='ros_gym_env',
            executable='cnn_data_pub.py',
            name=f'cnn_data_pub',
            namespace=namespace,
            output='screen',
        ),

        Node(
            package='pkg-nav',
            executable='velocity_smoother_node.py',
            name=f'velocity_smoother',
            namespace=namespace,
            output='screen',
        ),

        Node(
            package='ros_gym_env',
            executable='rl_inference_node.py',
            name='rl_inference',
            namespace=namespace,
            output='screen',
            parameters=[{
                'model_file': LaunchConfiguration('model_file')
            }]
        ),
    ])

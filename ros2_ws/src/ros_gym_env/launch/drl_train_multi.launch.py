from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    num_instances = int(LaunchConfiguration('num_instances').perform(context))
    speed_time = float(LaunchConfiguration('speed_time').perform(context))
    log_level = LaunchConfiguration('log_level')
    frequency = 20.0 * speed_time

    config_training = os.path.join(
        get_package_share_directory('ros_gym_env'),
        'config',
        'training.yaml'
        )

    config_global_planner = os.path.join(
        get_package_share_directory('base_nav'),
        'config',
        'params.yaml'
        )

    nodes = []

    # Serveur ROS TCP
    nodes.append(Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    ))

    # Boucle sur les instances
    for i in range(100):  # Max théorique
        if i >= num_instances:
            break
        namespace = f'env_{i}'

        # nodes.append(Node(
        #     package='ros_gym_env',
        #     executable='cnn_data_pub.py',
        #     name='cnn_data_pub',
        #     namespace=namespace,
        #     parameters=[{'frequency': frequency}],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     output='screen'
        # ))

        # nodes.append(Node(
        #     package='pkg-nav',
        #     executable='velocity_smoother_node.py',
        #     name='velocity_smoother',
        #     namespace=namespace,
        #     parameters=[{'frequency': frequency}],
        #     arguments=['--ros-args', '--log-level', "fatal"], #log_level],
        #     output='screen'
        # ))

        # nodes.append(Node(
        #     package = 'base_nav',
        #     name = 'global_planner',
        #     executable = 'jps.py',
        #     namespace=namespace,
        #     parameters = [config_global_planner, {'prefix': namespace}],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     output='screen'
        # ))

        # nodes.append(Node(
        #     package = 'ros_gym_env',
        #     name = 'pure_pursuit',
        #     executable = 'pure_pursuit.py',
        #     namespace=namespace,
        #     parameters = [{'prefix': namespace}],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     output='screen'
        # ))

    # Lancement du trainer
    nodes.append(Node(
        package='ros_gym_env',
        executable='rl_trainer_node.py',
        parameters=[{'num_envs': num_instances, 'speed_time': speed_time}],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_instances',
            default_value='1',
            description='Number of agent instances to launch'
        ),
        DeclareLaunchArgument(
            'speed_time',
            default_value='1',
            description='Speed Time'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='error',
            description='Log level: debug, info, warn, error, fatal'
        ),
        OpaqueFunction(function=launch_setup)
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('mbsn_ros'),
        'config',
        'params_sim.yaml'
        )
        
    node=Node(
        package = 'mbsn_ros',
        name = 'mbsn_real',
        executable = 'MBSN_real.py',
        parameters = [config]
    )

    ld.add_action(node)
    return ld
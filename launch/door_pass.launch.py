"""door_pass_node with a parameter file (default: config/wheelchair.yaml)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('pgr_door_pass'), 'config', 'wheelchair.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=cfg),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='pgr_door_pass', executable='door_pass_node', name='door_pass', output='screen',
             parameters=[LaunchConfiguration('params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])

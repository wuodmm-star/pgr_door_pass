"""Gazebo demo: a powered wheelchair with free front casters passes a narrow door with door_pass_node.

ros2 launch pgr_door_pass demo.launch.py door_width:=0.75 x:=-3.2 y:=1.2 yaw:=-0.6 gui:=true
The room is x in [-5, 0], the wall (0.12 m) at x = 0 with the door centred at y = 0; the robot drives through.
Localization in the demo is the wheel odometry (map = odom); the door pose comes from the 2D lidar."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def world_sdf(w, pacer):
    h, L = w / 2, 3.05 - w / 2
    box = lambda n, x, y, sx, sy: (f'<collision name="{n}"><pose>{x} {y} 0.5 0 0 0</pose><geometry><box><size>{sx} {sy} 1.0'
                                   f'</size></box></geometry></collision><visual name="{n}v"><pose>{x} {y} 0.5 0 0 0</pose>'
                                   f'<geometry><box><size>{sx} {sy} 1.0</size></box></geometry></visual>')
    walls = (box('up', 0, h + L / 2, 0.12, L) + box('dn', 0, -h - L / 2, 0.12, L) + box('n', -1, 3.1, 9, 0.1)
             + box('s', -1, -3.1, 9, 0.1) + box('w', -5.1, 0, 0.1, 6.3))
    rate = 0 if pacer else 1000
    pac = f'<plugin name="pacer" filename="{pacer}"><rtf>1.0</rtf></plugin>' if pacer else ''
    return f'''<?xml version="1.0"?><sdf version="1.6"><world name="door_demo">
<physics type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
<real_time_update_rate>{rate}</real_time_update_rate></physics>{pac}
<plugin name="ros_state" filename="libgazebo_ros_state.so"><update_rate>50</update_rate></plugin>
<include><uri>model://sun</uri></include>
<model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal>
<size>40 40</size></plane></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
</collision><visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></visual></link></model>
<model name="walls"><static>true</static><link name="l">{walls}</link></model>
</world></sdf>'''


def setup(context):
    w = float(LaunchConfiguration('door_width').perform(context))
    pacer = LaunchConfiguration('pacer_plugin').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    share = get_package_share_directory('pgr_door_pass')
    fd, world = tempfile.mkstemp(suffix='.world')
    with os.fdopen(fd, 'w') as f:
        f.write(world_sdf(w, pacer))
    urdf = os.path.join(share, 'sim', 'urdf', 'wheelchair.urdf')
    sim_time = {'use_sim_time': True}
    acts = [ExecuteProcess(cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
                           output='screen')]
    if gui:
        acts.append(ExecuteProcess(cmd=['gzclient'], output='screen'))
    acts += [
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'wheelchair', '-file', urdf, '-timeout', '120',
                        '-x', LaunchConfiguration('x').perform(context),
                        '-y', LaunchConfiguration('y').perform(context), '-z', '0.005',
                        '-Y', LaunchConfiguration('yaw').perform(context)]),
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen', parameters=[sim_time],
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen', parameters=[sim_time],
             arguments=['--x', '0.70', '--z', '0.42', '--frame-id', 'base_link', '--child-frame-id', 'laser']),
        TimerAction(period=8.0, actions=[Node(
            package='pgr_door_pass', executable='door_pass_node', name='door_pass', output='screen',
            parameters=[os.path.join(share, 'config', 'wheelchair.yaml'), sim_time, {'autostart': True}])]),
    ]
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('door_width', default_value='0.80'),
        DeclareLaunchArgument('x', default_value='-3.2'),
        DeclareLaunchArgument('y', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='-0.5'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('pacer_plugin', default_value='', description='optional real-time pacer plugin (WSL2)'),
        OpaqueFunction(function=setup),
    ])

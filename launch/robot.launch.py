"""
Starts the whole Solara robot.

  ros2 launch solara_pkg robot.launch.py mode:=mapping
  ros2 launch solara_pkg robot.launch.py mode:=localization map:=~/solara_ws/src/solara-ros/maps/room

Starts: robot_state_publisher (URDF), lidar driver, base_node,
        SLAM Toolbox (mapping or localization), Nav2 (planner, controller,
        behaviors, BT navigator) + lifecycle manager.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

PKG = 'solara_pkg'


def launch_setup(context):
    share = get_package_share_directory(PKG)
    mode = LaunchConfiguration('mode').perform(context)
    map_path = os.path.expanduser(LaunchConfiguration('map').perform(context))

    with open(os.path.join(share, 'urdf', 'solara.urdf')) as f:
        robot_description = f.read()

    nav2_params = os.path.join(share, 'config', 'nav2_params.yaml')
    base_params = os.path.join(share, 'config', 'base_params.yaml')

    # ---------- robot description: base_link -> laser ----------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # ---------- lidar ----------
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port').perform(context),
            'serial_baudrate': int(LaunchConfiguration('lidar_baud').perform(context)),
            'frame_id': 'laser',            # must match the URDF link name
            'angle_compensate': True,
        }],
    )

    # ---------- wheels: cmd_vel <-> Pico, odom -> base_link ----------
    base = Node(package=PKG, executable='base_node', parameters=[base_params])

    # ---------- SLAM Toolbox: map -> odom ----------
    slam_share = get_package_share_directory('slam_toolbox')
    if mode == 'localization':
        slam_params = RewrittenYaml(
            source_file=os.path.join(share, 'config', 'slam_localization_params.yaml'),
            param_rewrites={'map_file_name': map_path},
            convert_types=True,
        )
        slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'localization_launch.py')),
            launch_arguments={'slam_params_file': slam_params}.items(),
        )
    else:
        slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': os.path.join(share, 'config', 'slam_params.yaml'),
            }.items(),
        )

    # ---------- Nav2: only the servers we need ----------
    nav2_nodes = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    nav2 = [
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[nav2_params], output='screen'),   # publishes /cmd_vel directly
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[nav2_params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': nav2_nodes}]),
    ]

    return [robot_state_publisher, lidar, base, slam, *nav2]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping or localization'),
        DeclareLaunchArgument('map', default_value='~/solara_ws/src/solara-ros/maps/room',
                              description='saved map path, no file extension'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200',
                              description='RPLIDAR A1/A2: 115200, A3: 256000, S2: 1000000'),
        OpaqueFunction(function=launch_setup),
    ])

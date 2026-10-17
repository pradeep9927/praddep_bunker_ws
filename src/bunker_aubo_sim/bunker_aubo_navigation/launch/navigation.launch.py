"""Nav2 for the BUNKER + AUBO-i5.

Start the simulation first (and SLAM, unless localization:=amcl), then:

  ros2 launch bunker_aubo_navigation navigation.launch.py                    # with live SLAM
  ros2 launch bunker_aubo_navigation navigation.launch.py localization:=rtabmap
        (SLAM started with slam.launch.py localization:=true database_path:=<db>)
  ros2 launch bunker_aubo_navigation navigation.launch.py localization:=amcl \
        map:=~/bunker_maps/office.yaml

localization
  slam     RTAB-Map is mapping; it publishes /map and map -> odom. Nav2 plans in the growing map.
  rtabmap  RTAB-Map in localization mode on a saved database; same topics, the map is fixed.
  amcl     no RTAB-Map: map_server serves the saved 2D map, AMCL localizes with a 2D slice
           of the VLP-16 (pointcloud_to_laserscan), the LiDAR self filter is started here.
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
             'velocity_smoother', 'bt_navigator', 'waypoint_follower']
LOC_NODES = ['map_server', 'amcl']
MODES = ('slam', 'rtabmap', 'amcl')


def _as_bool(text):
    return str(text).lower() in ('1', 'true', 'yes', 'on')


def setup(context):
    mode = LaunchConfiguration('localization').perform(context).lower()
    if mode not in MODES:
        raise RuntimeError(f"localization must be one of {MODES}, got '{mode}'")
    params = os.path.expanduser(LaunchConfiguration('params_file').perform(context))
    sim = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    autostart = _as_bool(LaunchConfiguration('autostart').perform(context))
    common = [params, {'use_sim_time': sim}]

    actions = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen',
             parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common),
        # smoothed command goes to /cmd_vel, the same input the GUI uses (base_cmd_relay)
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': autostart, 'node_names': NAV_NODES,
                          'bond_timeout': 10.0}]),
    ]

    if mode == 'amcl':
        map_yaml = os.path.expanduser(LaunchConfiguration('map').perform(context))
        if not os.path.isfile(map_yaml):
            actions.insert(0, LogInfo(msg=f'[navigation] map file not found: {map_yaml}'))
        with open(os.path.join(get_package_share_directory('bunker_aubo_slam'), 'config',
                               'slam_nodes.yaml')) as f:
            self_filter = yaml.safe_load(f)['lidar_self_filter']
        actions += [
            Node(package='bunker_aubo_slam', executable='lidar_self_filter', name='lidar_self_filter',
                 output='screen', parameters=[self_filter, {'use_sim_time': sim}],
                 remappings=[('points_in', '/lidar/points'), ('points_out', '/lidar/points_filtered')]),
            Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                 name='pointcloud_to_laserscan', output='screen', parameters=common,
                 remappings=[('cloud_in', '/lidar/points_filtered'), ('scan', '/scan')]),
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=common + [{'yaml_filename': map_yaml}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=common),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': sim, 'autostart': autostart, 'node_names': LOC_NODES,
                              'bond_timeout': 10.0}]),
        ]

    if _as_bool(LaunchConfiguration('rviz').perform(context)):
        rviz = os.path.join(get_package_share_directory('bunker_aubo_navigation'), 'rviz',
                            'navigation.rviz')
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz_navigation', output='log',
                            arguments=['-d', rviz], parameters=[{'use_sim_time': sim}]))

    actions.insert(0, LogInfo(msg=f'[navigation] Nav2 with localization = {mode}'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('bunker_aubo_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='slam',
                              description='slam | rtabmap | amcl (see the file header)'),
        DeclareLaunchArgument('map', default_value='~/bunker_maps/office.yaml',
                              description='saved 2D map for localization:=amcl'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])

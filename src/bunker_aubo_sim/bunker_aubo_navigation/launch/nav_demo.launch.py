"""Everything in one command: simulation + operator GUI + localization + Nav2 + RViz.

  ros2 launch bunker_aubo_navigation nav_demo.launch.py                         # map while navigating
  ros2 launch bunker_aubo_navigation nav_demo.launch.py localization:=rtabmap \\
        database_path:=~/bunker_maps/office.db                                 # saved RTAB-Map db
  ros2 launch bunker_aubo_navigation nav_demo.launch.py localization:=amcl \\
        map:=~/bunker_maps/office.yaml                                         # saved 2D map + AMCL

Goals: "2D Goal Pose" in RViz, or `ros2 run bunker_aubo_navigation go_to north_corridor`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(pkg, name, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=list(args.items()))


def setup(context):
    mode = LaunchConfiguration('localization').perform(context).lower()
    actions = [
        include('bunker_aubo_bringup', 'sim.launch.py',
                {'rviz': 'false', 'gz_gui': LaunchConfiguration('gz_gui')}),
        TimerAction(period=8.0, actions=[include('bunker_aubo_gui', 'gui.launch.py', {})]),
    ]
    if mode in ('slam', 'rtabmap'):
        actions.append(TimerAction(period=12.0, actions=[include(
            'bunker_aubo_slam', 'slam.launch.py',
            {'localization': 'true' if mode == 'rtabmap' else 'false',
             'database_path': LaunchConfiguration('database_path'),
             'rviz': 'false'})]))
    actions.append(TimerAction(period=18.0, actions=[include(
        'bunker_aubo_navigation', 'navigation.launch.py',
        {'localization': mode, 'map': LaunchConfiguration('map'), 'rviz': 'true'})]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='slam', description='slam | rtabmap | amcl'),
        DeclareLaunchArgument('database_path', default_value='~/bunker_maps/rtabmap.db',
                              description='RTAB-Map database (slam: written, rtabmap: loaded)'),
        DeclareLaunchArgument('map', default_value='~/bunker_maps/office.yaml',
                              description='saved 2D map for localization:=amcl'),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        OpaqueFunction(function=setup),
    ])

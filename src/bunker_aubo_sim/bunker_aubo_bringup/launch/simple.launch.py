"""Simple room (walls, 2 pillars, 2 boxes): simulation + GUI + SLAM in one launch.

    ros2 launch bunker_aubo_bringup simple.launch.py                    # everything
    ros2 launch bunker_aubo_bringup simple.launch.py slam:=false        # sim + GUI only
    ros2 launch bunker_aubo_bringup simple.launch.py gui:=false slam:=false   # sim only
    ros2 launch bunker_aubo_bringup simple.launch.py localization:=true # use the saved map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    world = os.path.join(get_package_share_directory('bunker_aubo_gazebo'), 'worlds', 'simple_room.sdf')
    slam_on = LaunchConfiguration('slam')

    def include(pkg, name, args, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', name)),
            launch_arguments=args.items(), condition=condition)

    # one RViz only: the SLAM one when SLAM runs, otherwise the sim one
    sim_rviz = PythonExpression(["'false' if '", slam_on, "'.lower() in ('true', '1') else 'true'"])

    sim = include('bunker_aubo_bringup', 'sim.launch.py',
                  {'world': world, 'gz_gui': LaunchConfiguration('gz_gui'), 'rviz': sim_rviz})
    slam = include('bunker_aubo_slam', 'slam.launch.py',
                   {'world_file': world, 'rviz': 'true',
                    'localization': LaunchConfiguration('localization'),
                    'database_path': LaunchConfiguration('database_path')},
                   IfCondition(slam_on))
    gui = include('bunker_aubo_gui', 'gui.launch.py', {}, IfCondition(LaunchConfiguration('gui')))

    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/bunker_maps/simple_room.db'),
        sim,
        TimerAction(period=8.0, actions=[gui]),     # wait for the robot and controllers
        TimerAction(period=12.0, actions=[slam]),
    ])

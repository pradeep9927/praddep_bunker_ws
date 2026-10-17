import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    def include(pkg, name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', name)),
            launch_arguments=args.items())

    sim = include('bunker_aubo_bringup', 'sim.launch.py',
                  {'rviz': 'false', 'gz_gui': LaunchConfiguration('gz_gui')})
    slam = include('bunker_aubo_slam', 'slam.launch.py',
                   {'localization': LaunchConfiguration('localization'),
                    'database_path': LaunchConfiguration('database_path'),
                    'rviz': 'true'})
    gui = include('bunker_aubo_gui', 'gui.launch.py', {})

    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/bunker_maps/rtabmap.db'),
        sim,
        TimerAction(period=12.0, actions=[slam]),
        TimerAction(period=8.0, actions=[gui]),
    ])

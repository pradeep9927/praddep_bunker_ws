import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory('bunker_aubo_description')
    bringup = get_package_share_directory('bunker_aubo_bringup')
    slam = get_package_share_directory('bunker_aubo_slam')
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(description, 'urdf', 'bunker_aubo.urdf.xacro'), ' sim_gazebo:=false']),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value='~/bunker_maps/replay.db'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[os.path.join(bringup, 'config', 'ekf.yaml'), {'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'slam.launch.py')),
            launch_arguments={'database_path': LaunchConfiguration('database_path'),
                              'eval_dir': '~/bunker_maps/replay_evaluation'}.items()),
    ])

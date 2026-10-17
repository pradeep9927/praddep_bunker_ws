import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    path = os.path.expanduser(LaunchConfiguration('map').perform(context))
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': path, 'use_sim_time': sim, 'topic_name': 'map',
                          'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': ['map_server']}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='~/bunker_maps/office.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])

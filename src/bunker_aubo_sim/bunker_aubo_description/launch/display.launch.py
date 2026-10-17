from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('bunker_aubo_description')
    jsp_gui = LaunchConfiguration('jsp_gui')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'bunker_aubo.urdf.xacro']),
                 ' sim_gazebo:=false']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('jsp_gui', default_value='true',
                              description='Show joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('rviz_config',
                              default_value=PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(jsp_gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(jsp_gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')], output='screen'),
    ])

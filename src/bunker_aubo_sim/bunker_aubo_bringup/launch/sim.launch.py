import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = get_package_share_directory('bunker_aubo_bringup')
    description = get_package_share_directory('bunker_aubo_description')
    gazebo = get_package_share_directory('bunker_aubo_gazebo')

    world = LaunchConfiguration('world')
    gz_gui = LaunchConfiguration('gz_gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = {'use_sim_time': True}

    controllers_yaml = os.path.join(bringup, 'config', 'controllers.yaml')
    bridge_yaml = os.path.join(bringup, 'config', 'ros_gz_bridge.yaml')
    ekf_yaml = os.path.join(bringup, 'config', 'ekf.yaml')

    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(description, 'urdf', 'bunker_aubo.urdf.xacro'),
        ' sim_gazebo:=true',
        ' controllers_config:=', controllers_yaml,
        ' sensors_config:=', LaunchConfiguration('sensors_config'),
    ]), value_type=str)

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(gazebo, 'worlds', 'indoor_office.sdf'),
                              description='Gazebo world (SDF) to load'),
        DeclareLaunchArgument('gz_gui', default_value='true', description='Start the Gazebo GUI client'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(bringup, 'rviz', 'sim.rviz')),
        DeclareLaunchArgument('sensors_config',
                              default_value=os.path.join(description, 'config', 'sensors.yaml'),
                              description='Sensor rates / noise / resolution YAML'),
        DeclareLaunchArgument('x', default_value='-7.5'),
        DeclareLaunchArgument('y', default_value='-1.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('gz_verbosity', default_value='2'),
    ]

    env = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gazebo, 'worlds')),
    ]

    gz_args = PythonExpression([
        "'-r -v ", LaunchConfiguration('gz_verbosity'), " ' + ('' if '", gz_gui,
        "'.lower() in ('true', '1') else '-s --headless-rendering ') + '", world, "'"])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'publish_frequency': 50.0}, use_sim_time])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'bunker_aubo',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge', output='screen',
        parameters=[{'config_file': bridge_yaml}, use_sim_time])

    def spawner(*controllers):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[*controllers, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '120'],
                    parameters=[use_sim_time])

    jsb_spawner = spawner('joint_state_broadcaster')
    motion_spawner = spawner('diff_drive_controller', 'arm_controller', 'gripper_controller')

    start_jsb = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner]))
    start_motion = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[motion_spawner]))

    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               output='screen', parameters=[ekf_yaml, use_sim_time])

    cmd_relay = Node(package='bunker_aubo_bringup', executable='base_cmd_relay', output='screen',
                     parameters=[use_sim_time])
    gripper_server = Node(package='bunker_aubo_bringup', executable='gripper_action_server',
                          output='screen', parameters=[use_sim_time])

    # moving track treads in RViz (the body mesh with the tracks is one rigid visual)
    track_viz = Node(package='bunker_aubo_bringup', executable='track_visualizer', output='log',
                     parameters=[use_sim_time])

    rviz_node = Node(package='rviz2', executable='rviz2', output='log',
                     arguments=['-d', LaunchConfiguration('rviz_config')],
                     parameters=[use_sim_time], condition=IfCondition(rviz))

    return LaunchDescription(args + env + [
        gz_sim, robot_state_publisher, spawn, bridge,
        start_jsb, start_motion,
        ekf, cmd_relay, gripper_server, track_viz, rviz_node,
    ])

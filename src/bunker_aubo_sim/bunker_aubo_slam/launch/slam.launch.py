import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    return str(text).lower() in ('true', '1', 'yes')


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('bunker_aubo_slam')
    cfg_path = LaunchConfiguration('rtabmap_config').perform(context)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    with open(LaunchConfiguration('nodes_config').perform(context)) as f:
        helpers = yaml.safe_load(f)

    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    localization = _as_bool(LaunchConfiguration('localization').perform(context))
    db_path = os.path.expanduser(LaunchConfiguration('database_path').perform(context))
    os.makedirs(os.path.dirname(db_path) or '.', exist_ok=True)

    rtabmap_params = {k: str(v) for k, v in cfg['rtabmap'].items()}   # rtabmap wants strings
    if localization:
        rtabmap_params.update({'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true'})
    else:
        rtabmap_params.update({'Mem/IncrementalMemory': 'true'})
    node_params = dict(cfg['node'])
    node_params.update({
        'use_sim_time': use_sim_time,
        'database_path': db_path,
        'delete_db_on_start': (not localization) and not _as_bool(
            LaunchConfiguration('resume').perform(context)),
    })

    def rgbd_sync(cam):
        return Node(
            package='rtabmap_sync', executable='rgbd_sync', name=f'{cam}_rgbd_sync', namespace=cam,
            output='screen',
            parameters=[helpers['rgbd_sync'], {'use_sim_time': use_sim_time}],
            remappings=[('rgb/image', f'/{cam}/color/image_raw'),
                        ('depth/image', f'/{cam}/depth/image_rect_raw'),
                        ('rgb/camera_info', f'/{cam}/color/camera_info'),
                        ('rgbd_image', f'/{cam}/rgbd_image')])

    multi_sync = Node(
        package='bunker_aubo_slam', executable='multi_rgbd_sync', namespace='rtabmap', output='screen',
        parameters=[helpers['multi_rgbd_sync'], {'use_sim_time': use_sim_time}])

    # removes LiDAR hits on the robot's own arm (otherwise they smear through the map)
    self_filter = Node(
        package='bunker_aubo_slam', executable='lidar_self_filter', name='lidar_self_filter',
        output='screen', parameters=[helpers['lidar_self_filter'], {'use_sim_time': use_sim_time}],
        remappings=[('points_in', '/lidar/points'), ('points_out', '/lidar/points_filtered')])

    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap', output='screen',
        parameters=[node_params, rtabmap_params],
        remappings=[('rgbd_images', '/rtabmap/rgbd_images'),
                    ('scan_cloud', '/lidar/points_filtered'),
                    ('imu', '/imu/data'),
                    ('map', '/map')])

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', namespace='rtabmap',
        output='log', condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        parameters=[node_params, rtabmap_params],
        remappings=[('rgbd_images', '/rtabmap/rgbd_images'), ('scan_cloud', '/lidar/points_filtered')])

    evaluator = Node(
        package='bunker_aubo_slam', executable='slam_evaluator', output='screen',
        condition=IfCondition(LaunchConfiguration('evaluate')),
        parameters=[{'use_sim_time': use_sim_time,
                     'world_file': LaunchConfiguration('world_file').perform(context),
                     'output_dir': LaunchConfiguration('eval_dir').perform(context)}])

    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(share, 'rviz', 'slam.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    mode = 'LOCALIZATION in saved map' if localization else 'MAPPING (new map)'
    return [LogInfo(msg=f'[bunker_aubo_slam] {mode}, database: {db_path}'),
            rgbd_sync('base_camera'), rgbd_sync('wrist_camera'), multi_sync, self_filter,
            rtabmap, rtabmap_viz, evaluator, rviz]


def generate_launch_description():
    share = get_package_share_directory('bunker_aubo_slam')
    try:
        world = os.path.join(get_package_share_directory('bunker_aubo_gazebo'), 'worlds', 'indoor_office.sdf')
    except Exception:  # noqa: BLE001 - evaluation is optional
        world = ''
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = localize in the saved map (read-only)'),
        DeclareLaunchArgument('resume', default_value='false',
                              description='mapping mode: continue the map in database_path instead of starting fresh'),
        DeclareLaunchArgument('database_path', default_value='~/bunker_maps/rtabmap.db',
                              description='RTAB-Map database (this IS the saved map)'),
        DeclareLaunchArgument('rtabmap_config', default_value=os.path.join(share, 'config', 'rtabmap.yaml')),
        DeclareLaunchArgument('nodes_config', default_value=os.path.join(share, 'config', 'slam_nodes.yaml'),
                              description='parameters of rgbd_sync, multi_rgbd_sync and lidar_self_filter'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('evaluate', default_value='true',
                              description='run slam_evaluator (needs /ground_truth/odom)'),
        DeclareLaunchArgument('world_file', default_value=world),
        DeclareLaunchArgument('eval_dir', default_value='~/bunker_maps/evaluation'),
        OpaqueFunction(function=launch_setup),
    ])

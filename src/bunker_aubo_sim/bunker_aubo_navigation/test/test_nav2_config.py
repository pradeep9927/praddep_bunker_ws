import ast
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(ROOT)


def load(*parts):
    with open(os.path.join(*parts)) as f:
        return yaml.safe_load(f)


def nav():
    return load(ROOT, 'config', 'nav2_params.yaml')


def p(node):
    cfg = nav()
    if node in ('local_costmap', 'global_costmap'):
        return cfg[node][node]['ros__parameters']
    return cfg[node]['ros__parameters']


def launch_list(name):
    with open(os.path.join(ROOT, 'launch', 'navigation.launch.py')) as f:
        tree = ast.parse(f.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and node.targets[0].id == name:
            return ast.literal_eval(node.value)
    raise KeyError(name)


def test_every_lifecycle_node_has_parameters():
    cfg = nav()
    for n in launch_list('NAV_NODES') + launch_list('LOC_NODES'):
        assert n in cfg, n
        assert cfg[n]['ros__parameters']['use_sim_time'] is True


def test_frames_match_the_rest_of_the_stack():
    ekf = load(SRC, 'bunker_aubo_bringup', 'config', 'ekf.yaml')
    ekf = next(iter(ekf.values()))['ros__parameters']
    rtab = load(SRC, 'bunker_aubo_slam', 'config', 'rtabmap.yaml')['node']
    base = rtab['frame_id']
    assert base == ekf['base_link_frame'] == 'base_footprint'
    assert p('bt_navigator')['robot_base_frame'] == base
    assert p('local_costmap')['global_frame'] == ekf['odom_frame'] == rtab['odom_frame_id']
    assert p('global_costmap')['global_frame'] == rtab['map_frame_id']
    for n in ('local_costmap', 'global_costmap', 'behavior_server'):
        assert p(n)['robot_base_frame'] == base
    a = p('amcl')
    assert (a['base_frame_id'], a['odom_frame_id'], a['global_frame_id']) == (base, 'odom', 'map')


def test_footprint_covers_the_robot_and_fits_through_doors():
    fp = yaml.safe_load(p('local_costmap')['footprint'])
    assert fp == yaml.safe_load(p('global_costmap')['footprint'])
    xs, ys = [q[0] for q in fp], [q[1] for q in fp]
    assert min(xs) <= -0.52 and max(xs) >= 0.51       # body + tracks, arm stowed
    assert min(ys) <= -0.40 and max(ys) >= 0.40
    width = max(ys) - min(ys) + 2 * p('local_costmap')['footprint_padding']
    assert width < 1.2 - 0.2                          # 1.2 m doors in indoor_office


def test_obstacles_come_from_the_self_filtered_lidar():
    for n in ('local_costmap', 'global_costmap'):
        layer = p(n)['voxel_layer']
        src = layer[layer['observation_sources']]
        assert src['topic'] == '/lidar/points_filtered'
        assert src['data_type'] == 'PointCloud2'
        assert 0.0 < src['min_obstacle_height'] < 0.2      # floor never marks
        assert layer['footprint_clearing_enabled'] is True
        assert layer['z_voxels'] <= 16
        assert layer['z_voxels'] * layer['z_resolution'] >= src['max_obstacle_height']
    assert p('global_costmap')['static_layer']['map_topic'] == '/map'


def test_planner_and_controller_plugins():
    planner = p('planner_server')
    assert planner[planner['planner_plugins'][0]]['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    ctrl = p('controller_server')
    c = ctrl[ctrl['controller_plugins'][0]]
    assert c['plugin'].startswith('nav2_regulated_pure_pursuit_controller::')
    assert c['use_rotate_to_heading'] is True        # tracked base turns on the spot


def test_velocity_limits_stay_inside_the_base_controller():
    ctrl = load(SRC, 'bunker_aubo_bringup', 'config', 'controllers.yaml')['diff_drive_controller']
    ctrl = ctrl['ros__parameters']
    vs = p('velocity_smoother')
    assert vs['max_velocity'][0] <= ctrl['linear.x.max_velocity']
    assert vs['max_velocity'][2] <= ctrl['angular.z.max_velocity']
    rpp = p('controller_server')['FollowPath']
    assert rpp['desired_linear_vel'] <= vs['max_velocity'][0]
    assert rpp['rotate_to_heading_angular_vel'] <= vs['max_velocity'][2]


def test_nav2_output_reaches_the_base_through_the_relay():
    with open(os.path.join(ROOT, 'launch', 'navigation.launch.py')) as f:
        text = f.read()
    assert "('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')" in text
    with open(os.path.join(SRC, 'bunker_aubo_bringup', 'bunker_aubo_bringup', 'base_cmd_relay.py')) as f:
        assert "'/cmd_vel'" in f.read()


def test_launch_files_and_rviz_parse():
    for f in [n for n in os.listdir(os.path.join(ROOT, 'launch')) if n.endswith('.launch.py')]:
        with open(os.path.join(ROOT, 'launch', f)) as fh:
            ast.parse(fh.read())
    rviz = load(ROOT, 'rviz', 'navigation.rviz')
    assert rviz['Visualization Manager']['Global Options']['Fixed Frame'] == 'map'
    tools = [t['Class'] for t in rviz['Visualization Manager']['Tools']]
    assert 'rviz_default_plugins/SetGoal' in tools and 'rviz_default_plugins/SetInitialPose' in tools

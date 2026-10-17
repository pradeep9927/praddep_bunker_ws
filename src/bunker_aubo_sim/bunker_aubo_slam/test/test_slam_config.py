import os
import py_compile

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def cfg():
    with open(os.path.join(ROOT, 'config', 'rtabmap.yaml')) as f:
        return yaml.safe_load(f)


def test_rtabmap_uses_every_sensor():
    c = cfg()
    n, r = c['node'], c['rtabmap']
    assert n['subscribe_rgbd'] and n['rgbd_cameras'] == 0      # N cameras via RGBDImages
    assert n['subscribe_scan_cloud']                           # 3D LiDAR
    assert n['odom_frame_id'] == 'odom' and n['frame_id'] == 'base_footprint'
    assert r['Reg/Strategy'] == '2'                            # visual + ICP registration
    assert r['Grid/Sensor'] == '2'                             # grid from LiDAR and cameras
    assert r['RGBD/NeighborLinkRefining'] == 'true'


def test_library_parameters_are_strings():
    assert all(isinstance(v, str) for v in cfg()['rtabmap'].values())


@pytest.mark.parametrize('f', sorted(f for f in os.listdir(os.path.join(ROOT, 'launch')) if f.endswith('.py')))
def test_launch_files_compile(f, tmp_path):
    py_compile.compile(os.path.join(ROOT, 'launch', f), cfile=str(tmp_path / 'x.pyc'), doraise=True)


def test_helper_node_parameters_live_in_yaml():
    with open(os.path.join(ROOT, 'config', 'slam_nodes.yaml')) as f:
        h = yaml.safe_load(f)
    assert len(h['multi_rgbd_sync']['input_topics']) == 2          # both cameras go to SLAM
    assert h['multi_rgbd_sync']['camera_timeout'] > h['multi_rgbd_sync']['max_dt']
    assert h['rgbd_sync']['approx_sync_max_interval'] < h['multi_rgbd_sync']['max_dt']
    assert h['lidar_self_filter']['base_frame'] == 'base_footprint'
    with open(os.path.join(ROOT, 'launch', 'slam.launch.py')) as f:
        launch = f.read()
    assert 'slam_nodes.yaml' in launch and "'margin'" not in launch

import os

import numpy as np
import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from bunker_aubo_description.urdf_model import UrdfModel

PKG = 'bunker_aubo_description'
SHARE = get_package_share_directory(PKG)
XACRO = os.path.join(SHARE, 'urdf', 'bunker_aubo.urdf.xacro')


def expand(**mappings):
    doc = xacro.process_file(XACRO, mappings={k: str(v) for k, v in mappings.items()})
    return doc.toxml()


@pytest.fixture(scope='module')
def model():
    return UrdfModel(expand(sim_gazebo='true', controllers_config='/tmp/controllers.yaml'))


def test_display_variant_expands():
    m = UrdfModel(expand(sim_gazebo='false'))
    assert not m.ros2_control_joints(), 'display variant must not contain ros2_control'


def test_model_is_valid(model):
    assert model.check(get_package_share_directory) == []
    assert model.root_links == ['base_footprint']


def test_expected_joints(model):
    arm = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
           'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
    for j in arm:
        assert model.joints[j].type == 'revolute'
    for side in ('left', 'right'):
        assert model.joints[f'gripper_{side}_finger_joint'].type == 'prismatic'
        for i in range(1, 5):
            assert model.joints[f'{side}_wheel_{i}_joint'].type == 'continuous'


def test_wheels_touch_ground(model):
    for side in ('left', 'right'):
        for i in range(1, 5):
            z = model.link_pose(f'{side}_wheel_{i}_link')[2, 3]
            assert z == pytest.approx(0.075, abs=1e-6)


def test_every_sensor_frame_exists_and_is_connected(model):
    sensors = model.gazebo_sensors()
    assert {s['type'] for s in sensors.values()} == {'imu', 'gpu_lidar', 'rgbd_camera'}
    rgbd = [s for s in sensors.values() if s['type'] == 'rgbd_camera']
    assert len(rgbd) == 2
    for s in sensors.values():
        assert s['frame_id'] in model.links
        assert model.chain_to_root(s['frame_id'])[0].parent == 'base_footprint'


@pytest.mark.parametrize('cam', ['base_camera', 'wrist_camera'])
def test_optical_frames_follow_rep103(model, cam):
    body = model.link_pose(f'{cam}_link')
    for kind in ('color', 'depth'):
        opt = model.link_pose(f'{cam}_{kind}_optical_frame')
        assert np.allclose(opt[:3, 2], body[:3, 0], atol=1e-6)
        assert np.allclose(opt[:3, 0], -body[:3, 1], atol=1e-6)
        assert np.allclose(opt[:3, 1], -body[:3, 2], atol=1e-6)


def test_ros2_control_interfaces(model):
    rc = model.ros2_control_joints()
    assert len(rc) == 16
    for name, ifaces in rc.items():
        assert name in model.joints
        assert set(ifaces['state']) >= {'position', 'velocity'}
        expected = 'velocity' if 'wheel' in name else 'position'
        assert ifaces['command'] == [expected]


def test_initial_positions_within_limits(model):
    with open(os.path.join(SHARE, 'config', 'initial_positions.yaml')) as f:
        init = yaml.safe_load(f)
    for name, value in init.items():
        j = model.joints[name]
        assert j.lower <= value <= j.upper, name


def test_stowed_arm_stays_above_deck_and_below_lidar(model):
    with open(os.path.join(SHARE, 'config', 'initial_positions.yaml')) as f:
        q = yaml.safe_load(f)
    lidar_z = model.link_pose('lidar_link')[2, 3]
    for link in ('wrist1_Link', 'wrist2_Link', 'wrist3_Link', 'gripper_tcp', 'foreArm_Link'):
        z = model.link_pose(link, q)[2, 3]
        assert 0.45 < z < lidar_z - 0.15, link
    # wrist camera looks forward (helps SLAM) and its image is upright
    cam = model.link_pose('wrist_camera_link', q)
    assert cam[0, 0] > 0.95 and cam[2, 2] > 0.95


def test_total_mass_is_plausible(model):
    assert 120.0 < model.total_mass() < 170.0

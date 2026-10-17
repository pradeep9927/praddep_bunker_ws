import os
import py_compile

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from bunker_aubo_description.urdf_model import UrdfModel

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
CONFIG = os.path.join(PKG_ROOT, 'config')


def load(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def model():
    share = get_package_share_directory('bunker_aubo_description')
    xml = xacro.process_file(
        os.path.join(share, 'urdf', 'bunker_aubo.urdf.xacro'),
        mappings={'sim_gazebo': 'true',
                  'controllers_config': os.path.join(CONFIG, 'controllers.yaml')}).toxml()
    return UrdfModel(xml)


def test_controllers_reference_existing_interfaces(model):
    rc = model.ros2_control_joints()
    c = load('controllers.yaml')
    dd = c['diff_drive_controller']['ros__parameters']
    for j in dd['left_wheel_names'] + dd['right_wheel_names']:
        assert rc[j]['command'] == ['velocity'], j
    for ctrl in ('arm_controller', 'gripper_controller'):
        p = c[ctrl]['ros__parameters']
        for j in p['joints']:
            assert set(p['command_interfaces']) <= set(rc[j]['command']), j
            assert set(p['state_interfaces']) <= set(rc[j]['state']), j


def test_diff_drive_geometry_matches_urdf(model):
    dd = load('controllers.yaml')['diff_drive_controller']['ros__parameters']
    left = model.link_pose('left_wheel_1_link')[1, 3]
    right = model.link_pose('right_wheel_1_link')[1, 3]
    assert dd['wheel_separation'] == pytest.approx(left - right, abs=1e-3)
    assert dd['wheel_radius'] == pytest.approx(model.link_pose('left_wheel_1_link')[2, 3], abs=1e-3)
    assert dd['base_frame_id'] == 'base_footprint'
    assert dd['enable_odom_tf'] is False  # EKF owns odom->base_footprint


def test_bridge_config_is_well_formed():
    entries = load('ros_gz_bridge.yaml')
    required = {'ros_topic_name', 'gz_topic_name', 'ros_type_name', 'gz_type_name', 'direction'}
    ros_topics = set()
    for e in entries:
        assert required <= set(e), e
        assert e['direction'] in ('GZ_TO_ROS', 'ROS_TO_GZ', 'BIDIRECTIONAL')
        ros_topics.add(e['ros_topic_name'])
    expected = {
        '/clock', '/imu/data', '/lidar/points',
        '/base_camera/color/image_raw', '/base_camera/depth/image_rect_raw',
        '/base_camera/color/camera_info', '/base_camera/depth/camera_info',
        '/wrist_camera/color/image_raw', '/wrist_camera/depth/image_rect_raw',
        '/wrist_camera/color/camera_info', '/wrist_camera/depth/camera_info',
    }
    assert expected <= ros_topics


def test_bridge_gz_topics_match_sensor_topics(model):
    sensor_topics = {'/' + s['topic'] for s in model.gazebo_sensors().values()}
    for e in load('ros_gz_bridge.yaml'):
        gz = e['gz_topic_name']
        if gz in ('/clock', '/ground_truth/odom'):
            continue
        assert any(gz == t or gz.startswith(t + '/') for t in sensor_topics), gz


def test_ekf_inputs_and_frames():
    p = load('ekf.yaml')['ekf_filter_node']['ros__parameters']
    assert p['odom0'] == '/odom' and p['imu0'] == '/imu/data'
    assert p['base_link_frame'] == 'base_footprint' and p['world_frame'] == 'odom'
    assert len(p['odom0_config']) == 15 and len(p['imu0_config']) == 15
    assert p['imu0_config'][11] is True          # IMU yaw rate fused
    assert len(p['process_noise_covariance']) == 225


LAUNCH_FILES = sorted(f for f in os.listdir(os.path.join(PKG_ROOT, 'launch')) if f.endswith('.py'))


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_files_compile(launch_file, tmp_path):
    py_compile.compile(os.path.join(PKG_ROOT, 'launch', launch_file),
                       cfile=str(tmp_path / 'x.pyc'), doraise=True)

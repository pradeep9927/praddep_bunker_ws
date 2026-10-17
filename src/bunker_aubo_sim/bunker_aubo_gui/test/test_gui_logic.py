import os
import struct
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from bunker_aubo_gui.arm_model import ArmModel
from bunker_aubo_gui.msg_utils import (colorize_depth, depth_to_metres, image_to_rgb,
                                       pointcloud2_to_xyz)
from bunker_aubo_gui.topic_monitor import NO_DATA, OK, STALE, TopicMonitor

CFG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'gui.yaml')


@pytest.fixture(scope='module')
def cfg():
    with open(CFG_PATH) as f:
        return yaml.safe_load(f)


def make_image(arr, encoding, pad=0):
    h, w = arr.shape[:2]
    row = arr.reshape(h, -1).view(np.uint8)
    step = row.shape[1] + pad
    data = np.zeros((h, step), np.uint8)
    data[:, :row.shape[1]] = row
    return NS(height=h, width=w, step=step, encoding=encoding, is_bigendian=0, data=data.tobytes())


def test_rgb8_roundtrip_with_row_padding():
    rgb = np.random.default_rng(0).integers(0, 255, (4, 5, 3), dtype=np.uint8)
    out = image_to_rgb(make_image(rgb, 'rgb8', pad=3))
    assert np.array_equal(out, rgb)


def test_bgr8_is_swapped():
    bgr = np.zeros((2, 2, 3), np.uint8)
    bgr[..., 0] = 255  # blue
    out = image_to_rgb(make_image(bgr, 'bgr8'))
    assert out[0, 0].tolist() == [0, 0, 255]


def test_unsupported_encoding_returns_none():
    assert image_to_rgb(make_image(np.zeros((2, 2, 3), np.uint8), 'yuv422')) is None


def test_truncated_image_returns_none():
    msg = make_image(np.zeros((4, 4, 3), np.uint8), 'rgb8')
    msg.data = msg.data[:10]
    assert image_to_rgb(msg) is None


def test_depth_encodings():
    d = np.array([[0.5, 1.0], [np.nan, 4.0]], np.float32)
    out = depth_to_metres(make_image(d, '32FC1'))
    assert np.allclose(out[0], [0.5, 1.0]) and np.isnan(out[1, 0])
    mm = np.array([[500, 1000]], np.uint16)
    assert np.allclose(depth_to_metres(make_image(mm, '16UC1')), [[0.5, 1.0]])
    col = colorize_depth(out)
    assert col.shape == (2, 2, 3) and col[1, 0].tolist() == [0, 0, 0]


def make_cloud(points, point_step=32, row_pad=0, height=1):
    fields = [NS(name=n, offset=o, datatype=7, count=1) for n, o in (('x', 0), ('y', 4), ('z', 8))]
    fields.append(NS(name='intensity', offset=16, datatype=7, count=1))
    width = len(points) // height
    row_step = width * point_step + row_pad
    buf = bytearray(row_step * height)
    for i, p in enumerate(points):
        r, c = divmod(i, width)
        struct.pack_into('<fff', buf, r * row_step + c * point_step, *p)
    return NS(fields=fields, width=width, height=height, point_step=point_step, row_step=row_step,
              is_bigendian=False, data=bytes(buf))


def test_pointcloud_parsing_filters_nan():
    pts = [(1, 2, 3), (float('nan'), 0, 0), (-1, -2, -3), (float('inf'), 1, 1)]
    xyz = pointcloud2_to_xyz(make_cloud(pts))
    assert xyz.tolist() == [[1, 2, 3], [-1, -2, -3]]


def test_pointcloud_padded_rows_and_downsampling():
    pts = [(i, 0, 0) for i in range(100)]
    xyz = pointcloud2_to_xyz(make_cloud(pts, row_pad=8, height=4), max_points=10)
    assert len(xyz) == 10 and xyz[0, 0] == 0 and xyz[-1, 0] == 99


def test_pointcloud_without_xyz_is_empty():
    msg = make_cloud([(0, 0, 0)])
    msg.fields = msg.fields[:2]
    assert pointcloud2_to_xyz(msg).shape == (0, 3)


def test_named_poses_are_within_fallback_limits(cfg):
    a = cfg['arm']
    arm = ArmModel(a['joints'], a['fallback_limits'], a['named_poses'])
    for name, q in arm.named_poses.items():
        assert arm.within_limits(q), name
    assert {'home', 'stowed'} <= set(arm.named_poses)


def test_clamp_and_duration(cfg):
    a = cfg['arm']
    arm = ArmModel(a['joints'], a['fallback_limits'], a['named_poses'])
    clamped = arm.clamp([10, -10, 0, 0, 0, 0])
    assert clamped[0] == pytest.approx(3.04) and clamped[1] == pytest.approx(-3.04)
    assert arm.move_duration([0] * 6, [1, 0, 0, 0, 0, 0], 0.3, 1.5) == pytest.approx(1.5)
    assert arm.move_duration([0] * 6, [3, 0, 0, 0, 0, 0], 0.3, 1.5) == pytest.approx(3 / (2.618 * 0.3))


def test_limits_update_from_urdf(cfg):
    a = cfg['arm']
    arm = ArmModel(a['joints'], a['fallback_limits'], a['named_poses'])
    joints = ''.join(
        f'<joint name="{j}" type="revolute"><parent link="a"/><child link="b{i}"/>'
        f'<limit lower="-1" upper="1" effort="1" velocity="2"/></joint><link name="b{i}"/>'
        for i, j in enumerate(a['joints']))
    assert arm.update_from_urdf(f'<robot name="r"><link name="a"/>{joints}</robot>')
    assert arm.clamp([5] * 6) == [1.0] * 6
    assert not arm.update_from_urdf('<robot name="broken"')  # malformed -> keep limits
    assert arm.limits['shoulder_joint'].upper == 1.0


def test_wrong_pose_length_rejected(cfg):
    a = cfg['arm']
    with pytest.raises(ValueError):
        ArmModel(a['joints'], a['fallback_limits'], {'bad': [0, 0]})


def test_topic_monitor_states():
    now = [100.0]
    mon = TopicMonitor(clock=lambda: now[0])
    mon.add('/cam', 10.0)
    assert mon.status('/cam').state == NO_DATA
    for _ in range(20):
        mon.record('/cam')
        now[0] += 0.1
    st = mon.status('/cam')
    assert st.state == OK and st.rate_hz == pytest.approx(10.0, rel=0.05)
    now[0] += 2.0
    assert mon.status('/cam').state == STALE


def test_slow_topic_not_flagged_early():
    now = [0.0]
    mon = TopicMonitor(clock=lambda: now[0])
    mon.add('/map', 0.5)
    mon.record('/map')
    now[0] += 5.0   # 5 s < 5 / 0.5 = 10 s
    assert mon.status('/map').state == OK


def test_arm_collision_guard(cfg):
    xacro = pytest.importorskip('xacro')
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('bunker_aubo_description')
    xml = xacro.process_file(os.path.join(share, 'urdf', 'bunker_aubo.urdf.xacro'),
                             mappings={'sim_gazebo': 'false'}).toxml()
    a = cfg['arm']
    arm = ArmModel(a['joints'], a['fallback_limits'], a['named_poses'])
    assert arm.check_motion(None, [0.0] * 6)[0]          # no URDF yet: never blocks
    assert arm.update_from_urdf(xml) and arm.checker is not None
    stowed = arm.named_poses['stowed']
    for name, q in arm.named_poses.items():
        ok, msg = arm.check_motion(stowed, q)
        assert ok, f'{name}: {msg}'
    ok, msg = arm.check_motion(stowed, [1.5708, 1.9, 0.6, 0.0, 0.0, 0.0])   # onto the left track
    assert not ok and 'COLLISION' in msg
    assert 'COLLIDES' in arm.target_status([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])  # through the mast

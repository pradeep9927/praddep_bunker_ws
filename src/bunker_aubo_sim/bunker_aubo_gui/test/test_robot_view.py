import os

import numpy as np
import pytest

from bunker_aubo_gui.robot_view import (RobotScene, box_triangles, cylinder_triangles,
                                        transform_points)


@pytest.fixture(scope='module')
def scene():
    xacro = pytest.importorskip('xacro')
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('bunker_aubo_description')
    xml = xacro.process_file(os.path.join(share, 'urdf', 'bunker_aubo.urdf.xacro'),
                             mappings={'sim_gazebo': 'false'}).toxml()
    return RobotScene(xml, get_package_share_directory)


def test_primitives_have_the_right_extent():
    box = box_triangles((0.2, 0.4, 0.6)).reshape(-1, 3)
    assert np.allclose(box.min(axis=0), [-0.1, -0.2, -0.3])
    assert np.allclose(box.max(axis=0), [0.1, 0.2, 0.3])
    cyl = cylinder_triangles(0.05, 0.1).reshape(-1, 3)
    assert np.isclose(np.linalg.norm(cyl[:, :2], axis=1).max(), 0.05)
    assert np.isclose(cyl[:, 2].max(), 0.05)


def test_every_physical_link_is_drawn(scene):
    drawn = {p.link for p in scene.parts}
    for name in ('base_link', 'shoulder_Link', 'upperArm_Link', 'wrist3_Link', 'lidar_link',
                 'gripper_left_finger_link', 'base_camera_link', 'wrist_camera_link'):
        assert name in drawn, name
    meshes = [p for p in scene.parts if len(p.triangles) > 100]
    assert len(meshes) >= 8          # BUNKER body + 7 AUBO meshes were found on disk


def test_robot_stands_on_the_floor(scene):
    lows = []
    for part, tf in zip(scene.parts, scene.part_transforms({})):
        v = part.triangles.reshape(-1, 3) @ tf[:3, :3].T + tf[:3, 3]
        lows.append(v[:, 2].min())
    assert -0.01 < min(lows) < 0.02   # wheels touch z = 0 in base_footprint


def test_arm_moves_with_joint_states(scene):
    i = next(k for k, p in enumerate(scene.parts) if p.link == 'wrist3_Link')
    a = scene.part_transforms({'upperArm_joint': 0.0})[i][:3, 3]
    b = scene.part_transforms({'upperArm_joint': 1.0})[i][:3, 3]
    assert np.linalg.norm(a - b) > 0.3


def test_cloud_is_moved_into_base_footprint(scene):
    tf = scene.frame_to_root('lidar_link')
    assert tf is not None and tf[2, 3] > 0.5          # LiDAR sits on the mast
    out = transform_points(np.zeros((1, 3)), tf)
    assert np.allclose(out[0], tf[:3, 3])
    assert scene.frame_to_root('no_such_frame') is None

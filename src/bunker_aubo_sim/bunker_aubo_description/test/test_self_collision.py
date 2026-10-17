"""Arm self-collision guard: named poses are free, known bad poses are caught."""
import math
import os

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from bunker_aubo_description.self_collision import SelfCollisionChecker, describe

SHARE = get_package_share_directory('bunker_aubo_description')
NAMED = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'stowed': [0.0, 1.0, 2.75, 1.6, 1.5708, -1.5708],
    'ready': [0.0, 0.3, 1.85, 0.0, 1.5708, 0.0],
    'inspect': [0.0, 0.5, 1.6, 1.0, 1.5708, 0.0],
}


@pytest.fixture(scope='module')
def checker():
    xml = xacro.process_file(os.path.join(SHARE, 'urdf', 'bunker_aubo.urdf.xacro'),
                             mappings={'sim_gazebo': 'false'}).toxml()
    return SelfCollisionChecker(xml)


def test_spawn_pose_is_collision_free(checker):
    with open(os.path.join(SHARE, 'config', 'initial_positions.yaml')) as f:
        q = yaml.safe_load(f)
    arm = [q[j] for j in ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                          'wrist1_joint', 'wrist2_joint', 'wrist3_joint')]
    assert arm == pytest.approx(NAMED['stowed'])
    assert not checker.contacts(arm), describe(checker.contacts(arm))


@pytest.mark.parametrize('name', sorted(NAMED))
def test_named_pose_is_free(checker, name):
    assert not checker.contacts(NAMED[name]), describe(checker.contacts(NAMED[name]))


@pytest.mark.parametrize('a', sorted(NAMED))
@pytest.mark.parametrize('b', sorted(NAMED))
def test_named_pose_transitions_are_free(checker, a, b):
    if a != b:
        ok, _, contacts = checker.check_path(NAMED[a], NAMED[b])
        assert ok, describe(contacts)


def test_old_stowed_pose_was_in_self_collision(checker):
    links = {c.link_a for c in checker.contacts([0.0, 1.2, 2.77, -1.5708, -1.5708, 0.0])}
    assert 'upperArm_Link' in links


@pytest.mark.parametrize('q,hit', [
    ([math.pi / 2, 1.9, 0.6, 0, 0, 0], 'base_link'),       # down onto the left track
    ([-math.pi / 2, 1.9, 0.6, 0, 0, 0], 'base_link'),      # down onto the right track
    ([0.0, -1.57, 0.0, 0, 0, 0], 'sensor_mast_link'),      # forward through the mast
    ([0.0, 0.0, 2.9, 0, 0, 0], 'shoulder_Link'),           # folded into itself
])
def test_bad_poses_are_caught(checker, q, hit):
    assert hit in {c.link_a for c in checker.contacts(q)}


def test_path_through_the_track_is_blocked(checker):
    ok, frac, _ = checker.check_path([math.pi / 2, 0.0, 0.0, 0, 0, 0], [math.pi / 2, 2.4, 0.3, 0, 0, 0])
    assert not ok and 0.0 < frac < 1.0

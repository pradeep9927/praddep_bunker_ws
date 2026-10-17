import math
import os

import pytest

from bunker_aubo_navigation.goals import clearance, load_goals, world_to_map, yaw_to_quaternion

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(ROOT)
ROBOT_HALF_DIAGONAL = math.hypot(0.54, 0.42)


def goals():
    return load_goals(os.path.join(ROOT, 'config', 'goals.yaml'))


def test_spawn_matches_sim_launch_defaults():
    spawn, named, patrol = goals()
    with open(os.path.join(SRC, 'bunker_aubo_bringup', 'launch', 'sim.launch.py')) as f:
        text = f.read()
    assert f"DeclareLaunchArgument('x', default_value='{spawn[0]}')" in text
    assert f"DeclareLaunchArgument('y', default_value='{spawn[1]}')" in text
    assert f"DeclareLaunchArgument('yaw', default_value='{round(spawn[2], 4)}')" in text
    assert world_to_map(named['start'], spawn) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert patrol[-1] == 'start'


def test_world_to_map_rotation():
    spawn = (-7.5, -1.0, math.pi / 2)
    x, y, yaw = world_to_map((-7.5, 1.0, math.pi / 2), spawn)   # 2 m ahead of the robot
    assert (x, y, yaw) == pytest.approx((2.0, 0.0, 0.0), abs=1e-9)
    assert yaw_to_quaternion(math.pi)[2] == pytest.approx(1.0)


def test_goals_are_free_with_room_to_turn():
    geometry = pytest.importorskip('bunker_aubo_slam.geometry')
    sdf = os.path.join(SRC, 'bunker_aubo_gazebo', 'worlds', 'indoor_office.sdf')
    shapes = geometry.world_footprints(sdf)
    _, named, _ = goals()
    for name, (x, y, _) in named.items():
        assert clearance(x, y, shapes) > ROBOT_HALF_DIAGONAL + 0.05, name

"""Named goals: world coordinates -> map frame, and a clearance check against the world SDF.

No ROS imports, so it is unit tested on its own.
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

import yaml

Pose2D = Tuple[float, float, float]   # x, y, yaw (rad)


def load_goals(path: str) -> Tuple[Pose2D, Dict[str, Pose2D], List[str]]:
    """Returns spawn pose, named goals and patrol order, all in world frame (yaw in rad)."""
    with open(path) as f:
        cfg = yaml.safe_load(f)

    def pose(d):
        return float(d['x']), float(d['y']), math.radians(float(d.get('yaw', 0.0)))

    spawn = pose(cfg['spawn'])
    goals = {name: pose(d) for name, d in cfg['goals'].items()}
    patrol = list(cfg.get('patrol', []))
    for name in patrol:
        if name not in goals:
            raise KeyError(f'patrol uses unknown goal {name!r}')
    return spawn, goals, patrol


def world_to_map(p: Pose2D, spawn: Pose2D) -> Pose2D:
    """Map frame = robot start pose (where RTAB-Map starts its map)."""
    dx, dy = p[0] - spawn[0], p[1] - spawn[1]
    c, s = math.cos(spawn[2]), math.sin(spawn[2])
    yaw = math.atan2(math.sin(p[2] - spawn[2]), math.cos(p[2] - spawn[2]))
    return c * dx + s * dy, -s * dx + c * dy, yaw


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def clearance(x: float, y: float, shapes: Sequence[tuple]) -> float:
    """Distance from (x, y) to the nearest obstacle footprint ('box' or 'circle' tuples as
    produced by bunker_aubo_slam.geometry.world_footprints)."""
    best = float('inf')
    for s in shapes:
        if s[0] == 'circle':
            _, cx, cy, r = s
            d = math.hypot(x - cx, y - cy) - r
        else:
            _, cx, cy, yaw, sx, sy = s
            c, sn = math.cos(yaw), math.sin(yaw)
            lx = c * (x - cx) + sn * (y - cy)
            ly = -sn * (x - cx) + c * (y - cy)
            qx, qy = abs(lx) - sx / 2.0, abs(ly) - sy / 2.0
            d = math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0)
        best = min(best, d)
    return best

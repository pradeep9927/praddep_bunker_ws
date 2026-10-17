import math
import os

import numpy as np
import pytest

from bunker_aubo_slam.geometry import (align_2d, apply_se2, associate, map_accuracy, rasterize,
                                       se2, trajectory_errors, world_footprints)

WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                     'bunker_aubo_gazebo', 'worlds', 'indoor_office.sdf')


def test_align_recovers_rigid_transform():
    rng = np.random.default_rng(1)
    est = rng.uniform(-5, 5, (50, 2))
    t = se2(1.5, -2.0, 0.7)
    gt = apply_se2(t, est)
    assert np.allclose(align_2d(est, gt), t, atol=1e-9)


def test_trajectory_errors_zero_for_perfect_estimate_and_detects_drift():
    s = np.linspace(0, 2 * math.pi, 200)
    gt = np.stack([3 * np.cos(s), 2 * np.sin(s)], axis=1)
    est = apply_se2(se2(0.3, 0.1, 0.2), gt)          # same shape, different frame
    e = trajectory_errors(est, gt)
    assert e['ate_rmse'] < 1e-9
    drift = gt + np.linspace(0, 1, len(gt))[:, None] * np.array([0.5, 0.0])
    assert trajectory_errors(drift, gt)['ate_rmse'] > 0.05


def test_associate_nearest_within_tolerance():
    ie, ig = associate([0.0, 1.02, 5.0], [0.0, 0.5, 1.0, 1.5])
    assert ie.tolist() == [0, 1] and ig.tolist() == [0, 2]


@pytest.mark.skipif(not os.path.isfile(WORLD), reason='world file not in source tree')
def test_perfect_map_scores_perfectly():
    shapes = world_footprints(WORLD)
    assert len(shapes) > 100
    res = 0.05
    spawn = se2(-7.5, -1.0, math.pi / 2)            # world_T_map = robot start pose
    origin = (-12.0, -12.0)
    h = w = 480
    xs = origin[0] + (np.arange(w) + 0.5) * res
    ys = origin[1] + (np.arange(h) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    wxy = apply_se2(spawn, np.stack([gx.ravel(), gy.ravel()], 1))
    truth_world = rasterize(shapes, (-10.5, -7.5), (300, 420), res)
    ci = ((wxy[:, 0] + 10.5) / res).astype(int)
    ri = ((wxy[:, 1] + 7.5) / res).astype(int)
    inside = (ci >= 0) & (ci < 420) & (ri >= 0) & (ri < 300)
    grid = np.full(h * w, -1, np.int8)
    grid[inside] = 0
    grid[inside] = np.where(truth_world[ri[inside], ci[inside]], 100, 0)
    grid = grid.reshape(h, w)
    m = map_accuracy(grid, res, origin, spawn, shapes)
    assert m['median_error_m'] <= 0.03
    assert m['within_10cm'] > 0.95
    assert m['wall_coverage'] > 0.9
    bad = map_accuracy(grid, res, origin, spawn @ se2(0, 0, math.radians(3)), shapes)
    assert bad['within_10cm'] < m['within_10cm'] - 0.2

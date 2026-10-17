import math

from bunker_aubo_bringup.track_visualizer import TrackPath


def test_path_is_closed_and_treads_are_evenly_spaced():
    p = TrackPath()
    assert 2.0 < p.length < 3.0
    t = p.treads(0.0, 0.06)
    assert 35 <= len(t) <= 45


def test_belt_moves_like_a_real_track():
    p = TrackPath()
    x0, z0, _ = p.at(0.0)            # bottom run, front
    x1, _, _ = p.at(0.05)
    assert z0 < 0.02 and x1 < x0      # forward driving: lower run moves backwards
    s_top = next(s / 100 for s in range(int(p.length * 100)) if p.at(s / 100)[1] > 0.395)
    assert p.at(s_top + 0.05)[0] > p.at(s_top)[0]   # upper run moves forwards


def test_full_wheel_turn_moves_treads_by_one_circumference():
    p = TrackPath()
    a = p.at(1.0)
    b = p.at(1.0 + 2 * math.pi * 0.075 + p.length)   # wraps around
    c = p.at(1.0 + 2 * math.pi * 0.075)
    assert b == c and a != c

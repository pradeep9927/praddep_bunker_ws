import math

from bunker_aubo_bringup.gripper_logic import (HOLDING, REACHED, STOPPED, UNKNOWN, clamp_gap,
                                               classify, move_duration)


def test_clamp_gap():
    assert clamp_gap(-0.01, 0.08) == 0.0
    assert clamp_gap(0.2, 0.08) == 0.08
    assert clamp_gap(0.04, 0.08) == 0.04


def test_move_duration_uses_finger_travel():
    assert math.isclose(move_duration(0.08, 0.0, 0.08, 0.08), 0.5)     # 40 mm per finger
    assert move_duration(0.04, 0.04, 0.08, 0.08) == 0.3                # minimum
    assert math.isclose(move_duration(float('nan'), 0.0, 0.08, 0.08), 0.5)


def test_classify_outcomes():
    tol = 0.003
    assert classify(0.08, 0.04, 0.041, tol) == REACHED     # slider to 40 mm
    assert classify(0.04, 0.08, 0.079, tol) == REACHED     # open again
    assert classify(0.08, 0.0, 0.035, tol) == HOLDING      # closed on a 35 mm object
    assert classify(0.0, 0.08, 0.05, tol) == STOPPED       # opening blocked: not "holding"
    assert classify(0.08, 0.0, float('nan'), tol) == UNKNOWN

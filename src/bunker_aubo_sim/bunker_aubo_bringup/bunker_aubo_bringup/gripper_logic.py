"""Pure helpers of the gripper action server (no ROS imports, unit tested)."""
import math

REACHED = 'reached'
HOLDING = 'holding'
STOPPED = 'stopped'
UNKNOWN = 'unknown'


def clamp_gap(gap: float, max_gap: float) -> float:
    return min(max(float(gap), 0.0), float(max_gap))


def move_duration(start_gap: float, target_gap: float, finger_speed: float, max_gap: float,
                  minimum: float = 0.3) -> float:
    """Time for the trajectory: each finger travels half of the gap change."""
    if math.isnan(start_gap):
        travel = max_gap / 2.0
    else:
        travel = abs(target_gap - start_gap) / 2.0
    return max(minimum, travel / finger_speed)


def classify(start_gap: float, target_gap: float, final_gap: float, tolerance: float) -> str:
    """How a gripper move ended.

    reached  final gap within tolerance of the target
    holding  the fingers were closing and stopped early: something is between them
    stopped  any other early stop (e.g. opening against an obstacle)
    """
    if math.isnan(final_gap):
        return UNKNOWN
    if abs(final_gap - target_gap) <= tolerance:
        return REACHED
    closing = math.isnan(start_gap) or target_gap < start_gap - tolerance
    if closing and final_gap > target_gap + tolerance:
        return HOLDING
    return STOPPED

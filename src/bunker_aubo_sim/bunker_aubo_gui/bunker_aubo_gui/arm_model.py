from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence


@dataclass
class JointLimit:
    name: str
    lower: float
    upper: float
    velocity: float

    def clamp(self, value: float) -> float:
        return min(max(value, self.lower), self.upper)


class ArmModel:
    def __init__(self, joints: Sequence[str], fallback: Dict[str, List[float]],
                 named_poses: Dict[str, List[float]]):
        self.joints = list(joints)
        self.limits: Dict[str, JointLimit] = {
            j: JointLimit(j, fallback['lower'][i], fallback['upper'][i], fallback['velocity'][i])
            for i, j in enumerate(self.joints)
        }
        self.source = 'gui.yaml fallback'
        self.checker = None          # SelfCollisionChecker, built from /robot_description
        self.named_poses = {}
        for name, q in named_poses.items():
            if len(q) != len(self.joints):
                raise ValueError(f'named pose "{name}" has {len(q)} values, expected {len(self.joints)}')
            self.named_poses[name] = [float(v) for v in q]

    def update_from_urdf(self, urdf_xml: str) -> bool:
        try:
            from bunker_aubo_description.urdf_model import UrdfModel
            model = UrdfModel(urdf_xml)
        except Exception:  # noqa: BLE001 - malformed description must not kill the GUI
            return False
        found = {}
        for j in self.joints:
            uj = model.joints.get(j)
            if uj is None or uj.lower is None or uj.upper is None:
                return False
            found[j] = JointLimit(j, uj.lower, uj.upper, uj.velocity or self.limits[j].velocity)
        self.limits = found
        self.source = '/robot_description (URDF)'
        try:
            from bunker_aubo_description.self_collision import SelfCollisionChecker
            self.checker = SelfCollisionChecker(urdf_xml)
        except Exception:  # noqa: BLE001 - no collision guard is better than no GUI
            self.checker = None
        return True

    def check_motion(self, current: Optional[Sequence[float]], target: Sequence[float],
                     gripper: Optional[Sequence[float]] = None):
        """(ok, message). Blocks targets/paths that hit the tracks, deck, mast, floor or the arm itself."""
        if self.checker is None:
            return True, 'collision check not available (no URDF yet)'
        from bunker_aubo_description.self_collision import describe
        ok, frac, contacts = self.checker.check_path(current, target, gripper)
        if ok:
            return True, 'collision-free'
        where = 'target pose' if frac is None or frac >= 0.999 else f'{100 * frac:.0f}% along the path'
        return False, f'COLLISION at {where}: {describe(contacts)}'

    def target_status(self, target: Sequence[float], gripper: Optional[Sequence[float]] = None) -> str:
        if self.checker is None:
            return 'target: collision check not available yet'
        from bunker_aubo_description.self_collision import describe
        c = self.checker.contacts(target, gripper)
        return 'target: collision-free' if not c else f'target: COLLIDES ({describe(c, 2)})'

    def clamp(self, positions: Sequence[float]) -> List[float]:
        return [self.limits[j].clamp(v) for j, v in zip(self.joints, positions)]

    def within_limits(self, positions: Sequence[float], tol: float = 1e-9) -> bool:
        return all(self.limits[j].lower - tol <= v <= self.limits[j].upper + tol
                   for j, v in zip(self.joints, positions))

    def move_duration(self, current: Optional[Sequence[float]], target: Sequence[float],
                      velocity_scale: float, min_time: float) -> float:
        if current is None:
            return max(min_time, 4.0)
        t = min_time
        for j, c, g in zip(self.joints, current, target):
            vmax = max(1e-3, self.limits[j].velocity * velocity_scale)
            t = max(t, abs(g - c) / vmax)
        return t

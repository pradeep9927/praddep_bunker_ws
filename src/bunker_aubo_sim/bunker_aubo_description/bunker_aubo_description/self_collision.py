"""Arm self-collision checking for the BUNKER + AUBO-i5 (pure Python + numpy, no ROS needed).

Gazebo runs the model with self-collision OFF (the default for a URDF), so nothing physically
stops the arm from swinging through the tracks, the deck, the sensor mast or the floor. This
module is the guard that the operator GUI (and anything else that commands the arm) uses before
a goal is sent, the same idea as MoveIt's planning-scene check, without needing MoveIt.

How it works
  * Every <collision> of the URDF becomes an oriented box in its link frame
    (box -> itself, cylinder -> enclosing box, mesh -> its axis-aligned bounding box).
  * Every link that moves with the arm is also sampled as a cloud of surface points
    (mesh triangles are area-sampled, primitives are sampled on their faces).
  * Links get a "segment" number = how many arm joints lie between them and the chassis.
    The chassis, tracks, wheels, mast, LiDAR, base camera and the arm mounting plate are
    segment 0, the shoulder is 1, the upper arm 2 ... the fingers 7.
  * A configuration is in collision when a surface point of one link lies inside the
    (inflated) box of another link at least two segments away, or when an arm point is
    below the floor clearance. Neighbouring links (one joint apart) are never compared,
    because they touch at the joint by design.

A path between two configurations is checked by sampling it in joint space (the trajectory
controller interpolates in joint space as well).
"""
from __future__ import annotations

import math
import os
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .urdf_model import UrdfModel, make_tf

ARM_JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
              'wrist1_joint', 'wrist2_joint', 'wrist3_joint')
FINGER_JOINTS = ('gripper_left_finger_joint', 'gripper_right_finger_joint')
# Links that sit under the shoulder joint but do not move with it. They get segment 1, so the
# upper arm (segment 2), whose hub sits right on top of them, is not compared with them.
SHOULDER_SUPPORT = ('arm_mount_link', 'aubo_base_link')
# Pairs that the AUBO wrist geometry keeps apart over the whole joint range (checked by sweeping
# the wrist joints: the closest surfaces stay 3.6 mm, 19 mm, 40 mm ... apart). Their bounding
# boxes do overlap, so comparing them would only produce false alarms.
NEVER_COLLIDE = {
    frozenset(('foreArm_Link', 'wrist2_Link')),
    frozenset(('wrist1_Link', 'wrist3_Link')),
    frozenset(('wrist1_Link', 'gripper_base_link')),
    frozenset(('wrist1_Link', 'wrist_camera_link')),
    frozenset(('wrist1_Link', 'gripper_left_finger_link')),
    frozenset(('wrist1_Link', 'gripper_right_finger_link')),
    frozenset(('wrist2_Link', 'gripper_left_finger_link')),
    frozenset(('wrist2_Link', 'gripper_right_finger_link')),
}


# ------------------------------------------------------------------------------------ meshes
def load_stl(path: str) -> np.ndarray:
    """Triangles of an STL file (binary or ASCII) as an (N, 3, 3) array."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype(
                [('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
            return rec['v'].reshape(-1, 3, 3).astype(float)
    verts = [list(map(float, line.split()[1:4])) for line in data.decode(errors='ignore').splitlines()
             if line.strip().startswith('vertex')]
    return np.asarray(verts, dtype=float).reshape(-1, 3, 3)


def sample_triangles(tris: np.ndarray, spacing: float, max_points: int, seed: int = 0) -> np.ndarray:
    """Area-weighted surface samples (deterministic) plus all vertices."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    total = float(area.sum())
    verts = np.unique(tris.reshape(-1, 3).round(5), axis=0)
    if total <= 0.0:
        return verts
    n = int(min(max_points, max(50, total / (spacing * spacing))))
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(tris), size=n, p=area / total)
    r1 = np.sqrt(rng.random(n))[:, None]
    r2 = rng.random(n)[:, None]
    pts = (1 - r1) * a[idx] + r1 * (1 - r2) * b[idx] + r1 * r2 * c[idx]
    if len(verts) > max_points:
        verts = verts[rng.choice(len(verts), size=max_points, replace=False)]
    return np.vstack([pts, verts])


def box_surface_points(half: np.ndarray, spacing: float) -> np.ndarray:
    """Points on the six faces of a centred box with half extents `half`."""
    pts = []
    for axis in range(3):
        u, v = [i for i in range(3) if i != axis]
        nu = max(2, int(math.ceil(2 * half[u] / spacing)) + 1)
        nv = max(2, int(math.ceil(2 * half[v] / spacing)) + 1)
        gu, gv = np.meshgrid(np.linspace(-half[u], half[u], nu), np.linspace(-half[v], half[v], nv))
        for sign in (-1.0, 1.0):
            p = np.zeros((gu.size, 3))
            p[:, axis] = sign * half[axis]
            p[:, u] = gu.ravel()
            p[:, v] = gv.ravel()
            pts.append(p)
    return np.vstack(pts)


# ------------------------------------------------------------------------------------ shapes
@dataclass
class Shape:
    link: str
    T: np.ndarray            # shape frame in the link frame (4x4), box centred at its origin
    half: np.ndarray         # half extents of the (enclosing) box
    points: np.ndarray       # surface samples in the LINK frame, (N, 3)


def _resolve_mesh(filename: str, resolver=None) -> Optional[str]:
    if filename.startswith('file://'):
        return filename[len('file://'):]
    if filename.startswith('package://'):
        pkg, rel = filename[len('package://'):].split('/', 1)
        if resolver is None:
            try:
                from ament_index_python.packages import get_package_share_directory as resolver
            except ImportError:
                return None
        return os.path.join(resolver(pkg), rel)
    return filename


def _floats(text, default):
    return np.array([float(v) for v in text.split()]) if text else np.array(default, dtype=float)


def mesh_slab_boxes(pts: np.ndarray, slab: float) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Axis-aligned boxes around consecutive slabs (thickness ~`slab`) along the longest axis."""
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    axis = int(np.argmax(hi - lo))
    n = max(1, int(math.ceil((hi[axis] - lo[axis]) / slab)))
    edges = np.linspace(lo[axis], hi[axis], n + 1)
    boxes = []
    for i in range(n):
        sel = (pts[:, axis] >= edges[i] - 1e-9) & (pts[:, axis] <= edges[i + 1] + 1e-9)
        if np.count_nonzero(sel) < 4:
            continue
        boxes.append((pts[sel].min(axis=0), pts[sel].max(axis=0)))
    return boxes or [(lo, hi)]


def collision_shapes(urdf_xml: str, spacing: float = 0.012, max_points: int = 1500,
                     slab: float = 0.05, resolver=None) -> Dict[str, List[Shape]]:
    root = ET.fromstring(urdf_xml)
    shapes: Dict[str, List[Shape]] = {}
    for le in root.findall('link'):
        name = le.get('name')
        for k, ce in enumerate(le.findall('collision')):
            origin = ce.find('origin')
            T = make_tf(_floats(origin.get('xyz') if origin is not None else None, (0, 0, 0)),
                        _floats(origin.get('rpy') if origin is not None else None, (0, 0, 0)))
            geom = ce.find('geometry')
            if geom is None or len(geom) == 0:
                continue
            g = geom[0]
            if g.tag == 'box':
                half = _floats(g.get('size'), (0, 0, 0)) / 2.0
                pts = box_surface_points(half, spacing)
                shapes.setdefault(name, []).append(Shape(name, T, half, (T[:3, :3] @ pts.T).T + T[:3, 3]))
            elif g.tag in ('cylinder', 'sphere'):
                r = float(g.get('radius'))
                h = float(g.get('length')) if g.tag == 'cylinder' else 2 * r
                half = np.array([r, r, h / 2.0])
                n = max(12, int(2 * math.pi * r / spacing))
                ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
                zs = np.linspace(-h / 2, h / 2, max(2, int(h / spacing) + 1))
                ring = np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)
                side = np.array([[x, y, z] for z in zs for x, y in ring])
                pts = np.vstack([side, np.c_[ring * 0.5, np.full(n, h / 2)], np.c_[ring * 0.5, np.full(n, -h / 2)]])
                shapes.setdefault(name, []).append(Shape(name, T, half, (T[:3, :3] @ pts.T).T + T[:3, 3]))
            elif g.tag == 'mesh':
                path = _resolve_mesh(g.get('filename'), resolver)
                if not path or not os.path.isfile(path):
                    continue
                scale = _floats(g.get('scale'), (1, 1, 1))
                tris = load_stl(path) * scale
                pts = sample_triangles(tris, spacing, max_points, seed=k + len(name))
                link_pts = (T[:3, :3] @ pts.T).T + T[:3, 3]
                # One bounding box per slab along the longest axis: a long link (upper arm,
                # forearm) is then wrapped tightly instead of by one fat box.
                for lo, hi in mesh_slab_boxes(pts, slab):
                    Tb = T @ make_tf((lo + hi) / 2.0, (0, 0, 0))
                    shapes.setdefault(name, []).append(Shape(name, Tb, (hi - lo) / 2.0, link_pts))
                    link_pts = np.zeros((0, 3))   # points are stored once per link
    return shapes


# ------------------------------------------------------------------------------------ checker
@dataclass
class Contact:
    link_a: str     # the link whose box is penetrated (or 'floor')
    link_b: str     # the moving link whose surface point is inside it
    depth: float    # how deep the deepest point is inside the inflated box [m]

    def __str__(self):
        return f'{self.link_b} <-> {self.link_a}'


class SelfCollisionChecker:
    """Collision check of arm configurations against the robot body, the arm itself and the floor."""

    def __init__(self, urdf_xml: str, base_frame: str = 'base_footprint', body_margin: float = 0.025,
                 arm_margin: float = 0.005, floor_clearance: float = 0.03, resolver=None):
        self.model = UrdfModel(urdf_xml)
        self.base = base_frame
        self.body_margin = body_margin
        self.arm_margin = arm_margin
        self.floor_clearance = floor_clearance
        self.shapes = collision_shapes(urdf_xml, resolver=resolver)
        self.segment = {name: self._segment(name) for name in self.shapes}
        self.moving = [n for n in self.shapes if self.segment[n] >= 1 and n not in SHOULDER_SUPPORT]
        # pairs (obstacle, mover): mover's points tested against obstacle's boxes
        self.pairs: List[Tuple[str, str]] = []
        names = sorted(self.shapes, key=lambda n: self.segment[n])
        for a in names:
            for b in names:
                if self.segment[b] - self.segment[a] >= 2 and frozenset((a, b)) not in NEVER_COLLIDE:
                    self.pairs.append((a, b))
        # the two fingers are the same segment but must not cross each other: not needed (they
        # are driven symmetrically and stop at the centre line), so they are not paired.

    def _segment(self, link: str) -> int:
        if link in SHOULDER_SUPPORT:
            return 1
        seg = 0
        for j in self.model.chain_to_root(link):
            if j.name in ARM_JOINTS or j.name in FINGER_JOINTS:
                seg += 1
        return seg

    # -------------------------------------------------------------- kinematics helpers
    def positions(self, arm: Sequence[float], gripper: Optional[Sequence[float]] = None) -> Dict[str, float]:
        q = {j: float(v) for j, v in zip(ARM_JOINTS, arm)}
        g = gripper if gripper is not None else (0.04, 0.04)
        q.update({j: float(v) for j, v in zip(FINGER_JOINTS, g)})
        return q

    def link_poses(self, q: Dict[str, float]) -> Dict[str, np.ndarray]:
        """Pose of every link with collision geometry in the base frame (one pass over the tree)."""
        cache: Dict[str, np.ndarray] = {}

        def pose(link):
            if link in cache:
                return cache[link]
            j = self.model.child_joint.get(link)
            t = np.eye(4) if j is None else pose(j.parent) @ self.model.joint_transform(j, q.get(j.name, 0.0))
            cache[link] = t
            return t

        return {n: pose(n) for n in self.shapes}

    def _boxes(self, names, poses):
        """Stack the boxes of several links: rotations (K,3,3), centres (K,3), half extents (K,3)."""
        R, c, h, owner = [], [], [], []
        for n in names:
            for s in self.shapes[n]:
                Tw = poses[n] @ s.T
                R.append(Tw[:3, :3])
                c.append(Tw[:3, 3])
                h.append(s.half)
                owner.append(n)
        return np.array(R), np.array(c), np.array(h), owner

    def _setup_groups(self):
        """For every moving link, the obstacle links it must be compared with (static part cached)."""
        self._mover_points = {n: np.vstack([s.points for s in self.shapes[n]]) for n in self.moving}
        self._obstacles = {}
        for b in self.moving:
            self._obstacles[b] = [a for a, bb in self.pairs if bb == b]

    # -------------------------------------------------------------- queries
    def contacts(self, arm: Sequence[float], gripper: Optional[Sequence[float]] = None,
                 first_only: bool = False) -> List[Contact]:
        if not hasattr(self, '_obstacles'):
            self._setup_groups()
        poses = self.link_poses(self.positions(arm, gripper))
        found: List[Contact] = []
        for b in self.moving:
            pts = (poses[b][:3, :3] @ self._mover_points[b].T).T + poses[b][:3, 3]
            if self.segment[b] >= 2:
                zmin = float(pts[:, 2].min())
                if zmin < self.floor_clearance:
                    found.append(Contact('floor', b, self.floor_clearance - zmin))
                    if first_only:
                        return found
            obstacles = self._obstacles[b]
            if not obstacles:
                continue
            R, ctr, half, owner = self._boxes(obstacles, poses)
            margin = np.array([[self.body_margin if self.segment[o] == 0 else self.arm_margin]
                               for o in owner])
            # broad phase: bounding sphere of each box against the bounding sphere of the link
            mc = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
            mr = float(np.linalg.norm(pts.max(axis=0) - mc))
            near = np.linalg.norm(ctr - mc, axis=1) <= np.linalg.norm(half + margin, axis=1) + mr
            if not near.any():
                continue
            R, ctr, half, margin = R[near], ctr[near], half[near], margin[near]
            owner = [o for o, k in zip(owner, near) if k]
            # points in every box frame: (K, N, 3)
            local = np.matmul(pts[None, :, :] - ctr[:, None, :], R)
            slack = (half + margin)[:, None, :] - np.abs(local)
            inside = np.all(slack >= 0.0, axis=2)              # (K, N)
            hit = np.flatnonzero(inside.any(axis=1))
            reported = set()
            for k in hit:
                if owner[k] in reported:
                    continue
                reported.add(owner[k])
                depth = float(np.max(np.min(slack[k][inside[k]], axis=1)))
                found.append(Contact(owner[k], b, depth))
                if first_only:
                    return found
        return found

    def in_collision(self, arm: Sequence[float], gripper: Optional[Sequence[float]] = None) -> bool:
        return bool(self.contacts(arm, gripper, first_only=True))

    def check_path(self, start: Optional[Sequence[float]], goal: Sequence[float],
                   gripper: Optional[Sequence[float]] = None,
                   step: float = math.radians(2.0)) -> Tuple[bool, Optional[float], List[Contact]]:
        """Check the joint-space straight line start -> goal.

        Returns (ok, fraction_of_path_where_it_first_collides, contacts). If start is None only
        the goal is checked. A start that is already in collision does not block a motion that
        moves out of it: only the first colliding sample after the arm has been free counts.
        """
        goal = np.asarray(goal, dtype=float)
        if start is None:
            c = self.contacts(goal, gripper)
            return (not c, 1.0 if c else None, c)
        start = np.asarray(start, dtype=float)
        n = max(1, int(math.ceil(float(np.max(np.abs(goal - start))) / step)))
        was_free = not self.in_collision(start, gripper)
        for i in range(1, n + 1):
            s = i / n
            q = start + s * (goal - start)
            c = self.contacts(q, gripper, first_only=True)
            if c and was_free:
                return False, s, self.contacts(q, gripper)
            if not c:
                was_free = True
        c = self.contacts(goal, gripper)
        return (not c, 1.0 if c else None, c)


def arm_boxes(checker: 'SelfCollisionChecker', joints: Dict[str, float], frame_T: np.ndarray,
              margin: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Oriented boxes of every moving arm link (+ gripper, wrist camera), expressed in the frame
    whose pose in the base frame is frame_T. Returns rotations (K,3,3), centres (K,3), half extents (K,3)."""
    poses = checker.link_poses(joints)
    inv = np.linalg.inv(frame_T)
    R, c, h = [], [], []
    for name in checker.moving:
        for s in checker.shapes[name]:
            T = inv @ poses[name] @ s.T
            R.append(T[:3, :3])
            c.append(T[:3, 3])
            h.append(s.half + margin)
    return np.array(R), np.array(c), np.array(h)


def points_in_boxes(pts: np.ndarray, R: np.ndarray, ctr: np.ndarray, half: np.ndarray) -> np.ndarray:
    """Boolean mask of points (N,3) inside any of the oriented boxes."""
    mask = np.zeros(len(pts), dtype=bool)
    for k in range(len(R)):
        local = (pts - ctr[k]) @ R[k]
        mask |= np.all(np.abs(local) <= half[k], axis=1)
    return mask


def describe(contacts: Sequence[Contact], limit: int = 3) -> str:
    seen, parts = set(), []
    for c in contacts:
        key = (c.link_a, c.link_b)
        if key in seen:
            continue
        seen.add(key)
        parts.append(str(c))
        if len(parts) == limit:
            break
    return ', '.join(parts)

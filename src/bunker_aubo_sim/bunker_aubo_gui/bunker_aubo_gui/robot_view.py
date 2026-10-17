"""Robot model for the 3D LiDAR view of the operator GUI.

The URDF from /robot_description is turned into a list of triangle meshes, one per <visual>
(STL meshes, boxes, cylinders and spheres), each with its colour and its fixed offset inside the
link. At draw time every part is placed with forward kinematics from the latest /joint_states,
so the arm, gripper and wheel hubs move in the GUI the same way they do in Gazebo.

Everything is drawn in the base_footprint frame (floor at z = 0). The LiDAR cloud is moved
into that frame with the fixed mount transform from the same URDF, so cloud and robot line up.

This file has no Qt or ROS imports, so it can be unit tested on its own.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

import numpy as np

from bunker_aubo_description.self_collision import load_stl
from bunker_aubo_description.urdf_model import UrdfModel, make_tf

DEFAULT_RGBA = (0.7, 0.7, 0.7, 1.0)
ROOT_FRAME = 'base_footprint'


@dataclass
class VisualPart:
    link: str
    offset: np.ndarray      # 4x4, visual origin inside the link
    triangles: np.ndarray   # (M, 3, 3) in the visual frame
    rgba: tuple


# ----------------------------------------------------------------------------- primitives
def box_triangles(size) -> np.ndarray:
    sx, sy, sz = (0.5 * float(v) for v in size)
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    # corners indexed as 4*ix + 2*iy + iz
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, cc, d in quads:
        tris.append([c[a], c[b], c[cc]])
        tris.append([c[a], c[cc], c[d]])
    return np.asarray(tris, dtype=float)


def cylinder_triangles(radius: float, length: float, segments: int = 24) -> np.ndarray:
    """Closed cylinder along z, centred at the origin (URDF convention)."""
    h = 0.5 * length
    ang = np.linspace(0.0, 2.0 * math.pi, segments + 1)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    tris = []
    for i in range(segments):
        (x0, y0), (x1, y1) = ring[i], ring[i + 1]
        tris.append([[x0, y0, -h], [x1, y1, -h], [x1, y1, h]])
        tris.append([[x0, y0, -h], [x1, y1, h], [x0, y0, h]])
        tris.append([[0, 0, h], [x0, y0, h], [x1, y1, h]])
        tris.append([[0, 0, -h], [x1, y1, -h], [x0, y0, -h]])
    return np.asarray(tris, dtype=float)


def sphere_triangles(radius: float, rows: int = 10, cols: int = 16) -> np.ndarray:
    th = np.linspace(0.0, math.pi, rows + 1)
    ph = np.linspace(0.0, 2.0 * math.pi, cols + 1)

    def p(i, j):
        return [radius * math.sin(th[i]) * math.cos(ph[j]),
                radius * math.sin(th[i]) * math.sin(ph[j]),
                radius * math.cos(th[i])]
    tris = []
    for i in range(rows):
        for j in range(cols):
            if i > 0:
                tris.append([p(i, j), p(i + 1, j), p(i, j + 1)])
            if i < rows - 1:
                tris.append([p(i + 1, j), p(i + 1, j + 1), p(i, j + 1)])
    return np.asarray(tris, dtype=float)


# ----------------------------------------------------------------------------- URDF parsing
def resolve_mesh_path(uri: str, share_dir: Optional[Callable[[str], str]] = None) -> Optional[str]:
    if uri.startswith('file://'):
        return uri[len('file://'):]
    if uri.startswith('package://'):
        if share_dir is None:
            return None
        pkg, rel = uri[len('package://'):].split('/', 1)
        try:
            return os.path.join(share_dir(pkg), rel)
        except Exception:  # noqa: BLE001 - unknown package
            return None
    return uri


def _floats(text, default):
    return tuple(float(v) for v in text.split()) if text else default


def load_visuals(urdf_text: str, share_dir: Optional[Callable[[str], str]] = None,
                 max_triangles_per_mesh: int = 0) -> List[VisualPart]:
    """All <visual> elements of the URDF as triangle meshes. Missing meshes are skipped."""
    root = ET.fromstring(urdf_text)
    colours: Dict[str, tuple] = {}
    for m in root.findall('material'):
        col = m.find('color')
        if col is not None and m.get('name'):
            colours[m.get('name')] = _floats(col.get('rgba'), DEFAULT_RGBA)

    stl_cache: Dict[str, np.ndarray] = {}
    parts: List[VisualPart] = []
    for link in root.findall('link'):
        for vis in link.findall('visual'):
            geo = vis.find('geometry')
            if geo is None or len(geo) == 0:
                continue
            g = geo[0]
            if g.tag == 'box':
                tris = box_triangles(_floats(g.get('size'), (0.1, 0.1, 0.1)))
            elif g.tag == 'cylinder':
                tris = cylinder_triangles(float(g.get('radius')), float(g.get('length')))
            elif g.tag == 'sphere':
                tris = sphere_triangles(float(g.get('radius')))
            elif g.tag == 'mesh':
                path = resolve_mesh_path(g.get('filename', ''), share_dir)
                if not path or not os.path.isfile(path) or not path.lower().endswith('.stl'):
                    continue
                if path not in stl_cache:
                    t = load_stl(path)
                    if max_triangles_per_mesh and len(t) > max_triangles_per_mesh:
                        # crude thinning for very slow machines (leaves small gaps in the surface)
                        t = t[np.linspace(0, len(t) - 1, max_triangles_per_mesh).astype(int)]
                    stl_cache[path] = t
                tris = stl_cache[path] * np.asarray(_floats(g.get('scale'), (1.0, 1.0, 1.0)))
            else:
                continue

            origin = vis.find('origin')
            offset = make_tf(_floats(origin.get('xyz') if origin is not None else None, (0.0, 0.0, 0.0)),
                             _floats(origin.get('rpy') if origin is not None else None, (0.0, 0.0, 0.0)))
            rgba = DEFAULT_RGBA
            mat = vis.find('material')
            if mat is not None:
                col = mat.find('color')
                if col is not None:
                    rgba = _floats(col.get('rgba'), DEFAULT_RGBA)
                else:
                    rgba = colours.get(mat.get('name'), DEFAULT_RGBA)
            parts.append(VisualPart(link.get('name'), offset, tris, tuple(rgba)))
    return parts


# ----------------------------------------------------------------------------- kinematics
class RobotScene:
    """URDF visuals plus forward kinematics, all relative to base_footprint."""

    def __init__(self, urdf_text: str, share_dir: Optional[Callable[[str], str]] = None,
                 root_frame: str = ROOT_FRAME):
        self.model = UrdfModel(urdf_text)
        self.root_frame = root_frame if root_frame in self.model.links else self.model.root_links[0]
        self.parts = load_visuals(urdf_text, share_dir)

    def link_poses(self, positions: Optional[Dict[str, float]] = None) -> Dict[str, np.ndarray]:
        """Pose of every link in the root frame (one pass down the tree)."""
        positions = positions or {}
        root_inv = np.linalg.inv(self.model.link_pose(self.root_frame))
        poses: Dict[str, np.ndarray] = {}

        def pose(link):
            if link not in poses:
                j = self.model.child_joint.get(link)
                if j is None:
                    poses[link] = root_inv.copy()
                else:
                    poses[link] = pose(j.parent) @ self.model.joint_transform(j, positions.get(j.name, 0.0))
            return poses[link]

        for name in self.model.links:
            pose(name)
        return poses

    def part_transforms(self, positions: Optional[Dict[str, float]] = None) -> List[np.ndarray]:
        poses = self.link_poses(positions)
        return [poses[p.link] @ p.offset for p in self.parts]

    def frame_to_root(self, frame: str) -> Optional[np.ndarray]:
        """Fixed transform of a sensor frame into the root frame (None if unknown)."""
        if frame not in self.model.links:
            return None
        return self.link_poses()[frame]

    def outline_xy(self, positions: Optional[Dict[str, float]] = None, step: int = 7) -> np.ndarray:
        """Sub-sampled vertices projected to the floor, for the 2D fallback view."""
        pts = []
        for part, tf in zip(self.parts, self.part_transforms(positions)):
            v = part.triangles.reshape(-1, 3)[::step]
            pts.append(v @ tf[:3, :3].T + tf[:3, 3])
        return np.concatenate(pts)[:, :2] if pts else np.zeros((0, 2))


def transform_points(xyz: np.ndarray, tf: Optional[np.ndarray]) -> np.ndarray:
    if tf is None or xyz is None or len(xyz) == 0:
        return xyz
    return xyz @ tf[:3, :3].T + tf[:3, 3]

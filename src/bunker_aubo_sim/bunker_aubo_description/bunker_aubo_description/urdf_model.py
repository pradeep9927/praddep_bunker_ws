from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

VIRTUAL_LINK_SUFFIXES = ('_optical_frame', '_color_frame', '_depth_frame')
VIRTUAL_LINKS = {'base_footprint', 'tool0', 'gripper_tcp'}


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(r: np.ndarray) -> Tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock
        roll = 0.0
        yaw = math.atan2(-r[0, 1], r[1, 1])
    return roll, pitch, yaw


def make_tf(xyz, rpy) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def axis_angle(axis, angle) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _floats(text: Optional[str], default=(0.0, 0.0, 0.0)):
    if text is None:
        return tuple(default)
    return tuple(float(v) for v in text.split())


@dataclass
class Inertial:
    mass: float
    com: Tuple[float, float, float]
    inertia: np.ndarray  # 3x3


@dataclass
class Link:
    name: str
    visuals: int = 0
    collisions: int = 0
    inertial: Optional[Inertial] = None
    meshes: List[str] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    lower: Optional[float] = None
    upper: Optional[float] = None
    effort: Optional[float] = None
    velocity: Optional[float] = None

    @property
    def movable(self) -> bool:
        return self.type in ('revolute', 'continuous', 'prismatic')

    def clamp(self, value: float) -> float:
        if self.type in ('revolute', 'prismatic') and self.lower is not None:
            return min(max(value, self.lower), self.upper)
        return value


class UrdfModel:
    def __init__(self, xml_text: str):
        self.root = ET.fromstring(xml_text)
        self.name = self.root.get('name', '')
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self._parse()

    def _parse(self):
        for le in self.root.findall('link'):
            link = Link(le.get('name'))
            link.visuals = len(le.findall('visual'))
            link.collisions = len(le.findall('collision'))
            for mesh in le.iter('mesh'):
                link.meshes.append(mesh.get('filename'))
            ie = le.find('inertial')
            if ie is not None:
                mass = float(ie.find('mass').get('value'))
                origin = ie.find('origin')
                com = _floats(origin.get('xyz') if origin is not None else None)
                i = ie.find('inertia')
                g = {k: float(i.get(k, 0.0)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
                tensor = np.array([[g['ixx'], g['ixy'], g['ixz']],
                                   [g['ixy'], g['iyy'], g['iyz']],
                                   [g['ixz'], g['iyz'], g['izz']]])
                link.inertial = Inertial(mass, com, tensor)
            self.links[link.name] = link

        for je in self.root.findall('joint'):
            origin = je.find('origin')
            axis = je.find('axis')
            limit = je.find('limit')
            j = Joint(
                name=je.get('name'),
                type=je.get('type'),
                parent=je.find('parent').get('link'),
                child=je.find('child').get('link'),
                xyz=_floats(origin.get('xyz') if origin is not None else None),
                rpy=_floats(origin.get('rpy') if origin is not None else None),
                axis=_floats(axis.get('xyz'), (1.0, 0.0, 0.0)) if axis is not None else (1.0, 0.0, 0.0),
            )
            if limit is not None:
                j.lower = float(limit.get('lower')) if limit.get('lower') is not None else None
                j.upper = float(limit.get('upper')) if limit.get('upper') is not None else None
                j.effort = float(limit.get('effort')) if limit.get('effort') is not None else None
                j.velocity = float(limit.get('velocity')) if limit.get('velocity') is not None else None
            self.joints[j.name] = j

        self.child_joint = {j.child: j for j in self.joints.values()}

    @property
    def root_links(self) -> List[str]:
        return [n for n in self.links if n not in self.child_joint]

    def chain_to_root(self, link: str) -> List[Joint]:
        chain = []
        while link in self.child_joint:
            j = self.child_joint[link]
            chain.append(j)
            link = j.parent
        return list(reversed(chain))

    def is_virtual(self, link: str) -> bool:
        return link in VIRTUAL_LINKS or link.endswith(VIRTUAL_LINK_SUFFIXES)

    def joint_transform(self, j: Joint, q: float = 0.0) -> np.ndarray:
        t = make_tf(j.xyz, j.rpy)
        if j.type in ('revolute', 'continuous'):
            m = np.eye(4)
            m[:3, :3] = axis_angle(j.axis, q)
            t = t @ m
        elif j.type == 'prismatic':
            m = np.eye(4)
            m[:3, 3] = np.asarray(j.axis) * q
            t = t @ m
        return t

    def link_pose(self, link: str, positions: Optional[Dict[str, float]] = None,
                  base: Optional[str] = None) -> np.ndarray:
        positions = positions or {}
        t = np.eye(4)
        for j in self.chain_to_root(link):
            t = t @ self.joint_transform(j, positions.get(j.name, 0.0))
        if base is not None:
            t = np.linalg.inv(self.link_pose(base, positions)) @ t
        return t

    def check(self, package_share_resolver=None) -> List[str]:
        errors: List[str] = []
        roots = self.root_links
        if len(roots) != 1:
            errors.append(f'expected exactly one root link, found {roots}')
        for j in self.joints.values():
            if j.parent not in self.links:
                errors.append(f'joint {j.name}: unknown parent {j.parent}')
            if j.child not in self.links:
                errors.append(f'joint {j.name}: unknown child {j.child}')
            if j.movable:
                if j.effort is None or j.effort <= 0 or j.velocity is None or j.velocity <= 0:
                    errors.append(f'joint {j.name}: missing/invalid effort or velocity limit')
                if j.type in ('revolute', 'prismatic'):
                    if j.lower is None or j.upper is None or not j.lower < j.upper:
                        errors.append(f'joint {j.name}: invalid position limits')

        for link in self.links.values():
            if self.is_virtual(link.name):
                continue
            if link.visuals == 0:
                errors.append(f'link {link.name}: no <visual>')
            if link.collisions == 0:
                errors.append(f'link {link.name}: no <collision>')
            if link.inertial is None:
                errors.append(f'link {link.name}: no <inertial>')
                continue
            if link.inertial.mass <= 0:
                errors.append(f'link {link.name}: mass must be > 0')
            eig = np.linalg.eigvalsh(link.inertial.inertia)
            if np.any(eig <= 0):
                errors.append(f'link {link.name}: inertia tensor not positive definite')
            else:
                a, b, c = sorted(eig)
                if a + b < c * (1 - 1e-6):
                    errors.append(f'link {link.name}: principal moments violate triangle inequality')
            for m in link.meshes:
                path = m
                if m.startswith('file://'):
                    path = m[len('file://'):]
                elif m.startswith('package://') and package_share_resolver:
                    pkg, rel = m[len('package://'):].split('/', 1)
                    path = os.path.join(package_share_resolver(pkg), rel)
                if not os.path.isfile(path):
                    errors.append(f'link {link.name}: mesh not found: {m}')
        return errors

    def total_mass(self) -> float:
        return sum(lk.inertial.mass for lk in self.links.values() if lk.inertial)

    def ros2_control_joints(self) -> Dict[str, Dict[str, List[str]]]:
        out = {}
        for rc in self.root.findall('ros2_control'):
            for je in rc.findall('joint'):
                out[je.get('name')] = {
                    'command': [c.get('name') for c in je.findall('command_interface')],
                    'state': [s.get('name') for s in je.findall('state_interface')],
                }
        return out

    def gazebo_sensors(self) -> Dict[str, Dict[str, str]]:
        out = {}
        for gz in self.root.findall('gazebo'):
            for s in gz.findall('sensor'):
                fid = s.find('gz_frame_id')
                topic = s.find('topic')
                out[s.get('name')] = {
                    'type': s.get('type'),
                    'reference': gz.get('reference'),
                    'frame_id': fid.text if fid is not None else None,
                    'topic': topic.text if topic is not None else None,
                }
        return out

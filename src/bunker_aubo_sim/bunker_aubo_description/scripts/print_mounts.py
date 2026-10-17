#!/usr/bin/env python3
import math
import os
import sys

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from bunker_aubo_description.urdf_model import UrdfModel, matrix_to_rpy

FRAMES = [
    ('imu_link', 'base_link'),
    ('lidar_link', 'base_link'),
    ('base_camera_link', 'base_link'),
    ('base_camera_color_optical_frame', 'base_camera_link'),
    ('base_camera_depth_optical_frame', 'base_camera_link'),
    ('wrist_camera_link', 'tool0'),
    ('wrist_camera_color_optical_frame', 'wrist_camera_link'),
    ('wrist_camera_depth_optical_frame', 'wrist_camera_link'),
    ('aubo_base_link', 'base_link'),
    ('gripper_tcp', 'tool0'),
    ('base_link', 'base_footprint'),
]


def fmt(v):
    return ' '.join(f'{x:+.4f}' for x in v)


def main():
    share = get_package_share_directory('bunker_aubo_description')
    xml = xacro.process_file(os.path.join(share, 'urdf', 'bunker_aubo.urdf.xacro'),
                             mappings={'sim_gazebo': 'false'}).toxml()
    m = UrdfModel(xml)
    with open(os.path.join(share, 'config', 'initial_positions.yaml')) as f:
        q = yaml.safe_load(f)

    out = sys.stdout
    out.write('# Sensor mounting transforms\n\n')
    out.write('Generated from the URDF by `ros2 run bunker_aubo_description print_mounts.py`.\n')
    out.write('Translation in metres, rotation as roll/pitch/yaw in degrees '
              '(fixed-axis XYZ, as in URDF `<origin rpy>`).\n\n')
    out.write('| Frame | Parent | xyz [m] | rpy [deg] |\n|---|---|---|---|\n')
    for child, parent in FRAMES:
        t = m.link_pose(child, q, base=parent)
        rpy = [math.degrees(a) for a in matrix_to_rpy(t[:3, :3])]
        out.write(f'| `{child}` | `{parent}` | {fmt(t[:3, 3])} | {fmt(rpy)} |\n')

    out.write('\n## Height above ground (base_footprint), arm stowed\n\n')
    out.write('| Frame | x [m] | y [m] | z [m] |\n|---|---|---|---|\n')
    for child in ('imu_link', 'lidar_link', 'base_camera_link', 'wrist_camera_link', 'gripper_tcp'):
        t = m.link_pose(child, q)
        out.write(f'| `{child}` | {t[0, 3]:.3f} | {t[1, 3]:.3f} | {t[2, 3]:.3f} |\n')


if __name__ == '__main__':
    main()

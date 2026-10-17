#!/usr/bin/env python3
"""Write the TF tree of the robot as a Graphviz file (and PNG/SVG if `dot` is installed).

Frames come from the URDF. map -> odom (RTAB-Map) and odom -> base_footprint (EKF) are added
on top, so the picture shows the full tree as it is while SLAM runs.

    ros2 run xacro xacro bunker_aubo.urdf.xacro sim_gazebo:=false > /tmp/robot.urdf
    python3 tf_tree_diagram.py /tmp/robot.urdf docs/images/tf_tree
"""
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

MOVING = '#1565c0'
FIXED = '#757575'
RUNTIME = '#c62828'


def build_dot(urdf_text: str) -> str:
    root = ET.fromstring(urdf_text)
    out = ['digraph tf {', '  rankdir=TB;', '  node [shape=box, style="rounded,filled", fillcolor="#f5f5f5",',
           '        fontname="Helvetica", fontsize=10];', '  edge [fontname="Helvetica", fontsize=8];',
           '  map [fillcolor="#ffebee"]; odom [fillcolor="#ffebee"];',
           f'  map -> odom [color="{RUNTIME}", label="RTAB-Map"];',
           f'  odom -> base_footprint [color="{RUNTIME}", label="EKF"];']
    for j in root.findall('joint'):
        parent, child = j.find('parent').get('link'), j.find('child').get('link')
        moving = j.get('type') != 'fixed'
        colour = MOVING if moving else FIXED
        label = j.get('name') if moving else ''
        style = 'bold' if moving else 'solid'
        out.append(f'  "{parent}" -> "{child}" [color="{colour}", style={style}, label="{label}"];')
    out.append('  subgraph cluster_legend { label="legend"; fontsize=9; style=dashed;')
    out.append('    l1 [label="SLAM / EKF (runtime)", fillcolor="#ffebee"];')
    out.append('    l2 [label="moving joint (joint_states)", fillcolor="#e3f2fd"];')
    out.append('    l3 [label="fixed joint (URDF)"]; }')
    out.append('}')
    return '\n'.join(out) + '\n'


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1]) as f:
        dot = build_dot(f.read())
    base = sys.argv[2]
    os.makedirs(os.path.dirname(base) or '.', exist_ok=True)
    with open(base + '.dot', 'w') as f:
        f.write(dot)
    if shutil.which('dot'):
        for fmt in ('png', 'svg'):
            subprocess.run(['dot', f'-T{fmt}', '-Gdpi=110' if fmt == 'png' else '-Gdpi=72',
                            base + '.dot', '-o', f'{base}.{fmt}'], check=True)
    print('written', base + '.*')


if __name__ == '__main__':
    main()

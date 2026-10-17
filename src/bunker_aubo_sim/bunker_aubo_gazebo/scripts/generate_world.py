#!/usr/bin/env python3
import math

WALL_H = 2.5
WALL_T = 0.15

WALL_COLOURS = {
    'outer': '0.85 0.85 0.82',
    'north': '0.95 0.90 0.70',
    'central': '0.88 0.78 0.66',
    'rooms': '0.72 0.84 0.95',
    'south': '0.95 0.95 0.95',
}


def colour_material(rgb, alpha='1'):
    return (f'<material><ambient>{rgb} {alpha}</ambient><diffuse>{rgb} {alpha}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def box_elems(name, size, pose, rgb, collide=True):
    sx, sy, sz = size
    x, y, z, yaw = pose
    geo = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    p = f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>'
    s = f'<visual name="{name}_v">{p}{geo}{colour_material(rgb)}</visual>'
    if collide:
        s += f'<collision name="{name}_c">{p}{geo}</collision>'
    return s


def cyl_elems(name, r, h, pose, rgb):
    x, y, z = pose
    geo = f'<geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>'
    p = f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
    return (f'<visual name="{name}_v">{p}{geo}{colour_material(rgb)}</visual>'
            f'<collision name="{name}_c">{p}{geo}</collision>')


def static_model(name, body, pose=(0, 0, 0, 0)):
    x, y, z, yaw = pose
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>'
            f'<link name="link">{body}</link></model>\n')


def wall(name, x1, y1, x2, y2, colour):
    length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    return box_elems(name, (length + WALL_T, WALL_T, WALL_H), (cx, cy, WALL_H / 2, yaw),
                     WALL_COLOURS[colour])


def wall_with_doors(name, x1, y1, x2, y2, doors, colour):
    horizontal = abs(y2 - y1) < 1e-6
    a0, a1 = (x1, x2) if horizontal else (y1, y2)
    cuts = [a0]
    for d0, d1 in sorted(doors):
        cuts += [d0, d1]
    cuts.append(a1)
    s = ''
    for i in range(0, len(cuts), 2):
        s0, s1 = cuts[i], cuts[i + 1]
        if s1 - s0 < 0.05:
            continue
        if horizontal:
            s += wall(f'{name}_{i // 2}', s0, y1, s1, y1, colour)
        else:
            s += wall(f'{name}_{i // 2}', x1, s0, x1, s1, colour)
    return s


def build_walls():
    s = ''
    s += wall('outer_s', -10, -7, 10, -7, 'outer')
    s += wall('outer_n', -10, 7, 10, 7, 'outer')
    s += wall('outer_w', -10, -7, -10, 7, 'outer')
    s += wall('outer_e', 10, -7, 10, 7, 'outer')
    s += wall_with_doors('north_rooms', -10, 4.5, 10, 4.5,
                         [(-7.2, -6.0), (-0.6, 0.6), (6.0, 7.2)], 'north')
    s += wall('north_part_w', -3.5, 4.5, -3.5, 7, 'rooms')
    s += wall('north_part_e', 3.5, 4.5, 3.5, 7, 'rooms')
    s += wall_with_doors('central_n', -5, 2.5, 5, 2.5, [(1.8, 3.0)], 'central')
    s += wall_with_doors('central_s', -5, -2.5, 5, -2.5, [(-2.6, -1.4)], 'central')
    s += wall('central_w', -5, -2.5, -5, 2.5, 'central')
    s += wall('central_e', 5, -2.5, 5, 2.5, 'central')
    s += wall_with_doors('central_mid', 0.5, -2.5, 0.5, 2.5, [(-0.6, 0.6)], 'rooms')
    return static_model('office_walls', s)


POSTERS = [
    (-10 + 0.09, -5.0, 0.0, 1), (-10 + 0.09, 0.75, 0.0, 2), (-10 + 0.09, 5.8, 0.0, 3),
    (10 - 0.09, -5.2, math.pi, 4), (10 - 0.09, 0.5, math.pi, 5), (10 - 0.09, 5.8, math.pi, 6),
    (-5 - 0.09, -1.2, math.pi, 7), (-5 - 0.09, 1.3, math.pi, 8),
    (5 + 0.09, -1.0, 0.0, 9), (5 + 0.09, 1.5, 0.0, 10),
    (-3.0, 2.5 + 0.09, math.pi / 2, 11), (0.5, 2.5 + 0.09, math.pi / 2, 12),
    (-8.5, 4.5 - 0.09, -math.pi / 2, 13), (3.5, 4.5 - 0.09, -math.pi / 2, 14),
    (-4.0, -2.5 - 0.09, -math.pi / 2, 15), (2.5, -2.5 - 0.09, -math.pi / 2, 16),
    (-5.2, -7 + 0.09, math.pi / 2, 17), (1.0, -7 + 0.09, math.pi / 2, 18),
    (6.5, -7 + 0.09, math.pi / 2, 19), (2.0, 7 - 0.09, -math.pi / 2, 20),
]


def textured_panel(name, size, pose_xyz_rpy, texture):
    sx, sy, sz = size
    x, y, z, r, p, yw = pose_xyz_rpy
    return (f'<visual name="{name}">'
            f'<pose>{x:.3f} {y:.3f} {z:.3f} {r:.4f} {p:.4f} {yw:.4f}</pose>'
            f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
            f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular>'
            f'<pbr><metal><albedo_map>textures/{texture}.png</albedo_map>'
            f'<roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr>'
            f'</material></visual>')


def build_posters():
    s = ''
    for i, (x, y, yaw, tex) in enumerate(POSTERS):
        s += textured_panel(f'poster_{i:02d}', (0.01, 1.0, 1.0),
                            (x, y, 1.05, 0.0, 0.0, yaw), f'poster_{tex:02d}')
    return static_model('wall_posters', s)


def build_rugs():
    s = ''
    rugs = [(-7.5, -1.0, 1), (0.0, 3.5, 2), (7.5, 1.5, 3), (-1.0, -3.6, 4)]
    for i, (x, y, tex) in enumerate(rugs):
        s += textured_panel(f'rug_{i}', (1.6, 1.2, 0.004), (x, y, 0.002, 0, 0, 0.3 * i),
                            f'rug_{tex:02d}')
    return static_model('floor_rugs', s)


def table(name, x, y, yaw, w=1.4, d=0.8, h=0.75, colour='0.55 0.38 0.22'):
    s = box_elems('top', (w, d, 0.04), (0, 0, h - 0.02, 0), colour)
    for i, (lx, ly) in enumerate([(w / 2 - 0.05, d / 2 - 0.05), (-w / 2 + 0.05, d / 2 - 0.05),
                                  (w / 2 - 0.05, -d / 2 + 0.05), (-w / 2 + 0.05, -d / 2 + 0.05)]):
        s += box_elems(f'leg{i}', (0.05, 0.05, h - 0.04), (lx, ly, (h - 0.04) / 2, 0), '0.2 0.2 0.2')
    return static_model(name, s, (x, y, 0, yaw))


def desk_with_monitor(name, x, y, yaw):
    s = box_elems('top', (1.2, 0.7, 0.04), (0, 0, 0.73, 0), '0.85 0.85 0.85')
    s += box_elems('side_l', (0.04, 0.7, 0.71), (0.58, 0, 0.355, 0), '0.3 0.3 0.3')
    s += box_elems('side_r', (0.04, 0.7, 0.71), (-0.58, 0, 0.355, 0), '0.3 0.3 0.3')
    s += box_elems('drawer', (0.4, 0.6, 0.55), (0.35, 0, 0.3, 0), '0.4 0.45 0.55')
    s += box_elems('monitor', (0.55, 0.05, 0.35), (0, 0.2, 0.95, 0), '0.05 0.05 0.05')
    s += box_elems('chair_seat', (0.45, 0.45, 0.06), (0, -0.6, 0.45, 0), '0.15 0.25 0.6')
    s += box_elems('chair_back', (0.45, 0.06, 0.5), (0, -0.82, 0.73, 0), '0.15 0.25 0.6')
    s += box_elems('chair_post', (0.06, 0.06, 0.42), (0, -0.6, 0.21, 0), '0.2 0.2 0.2')
    return static_model(name, s, (x, y, 0, yaw))


def shelf(name, x, y, yaw, length=1.8, depth=0.45, height=1.9, seed=0):
    import random
    rnd = random.Random(seed)
    frame = '0.35 0.35 0.38'
    s = ''
    for i, sx in enumerate((-length / 2 + 0.02, length / 2 - 0.02)):
        s += box_elems(f'post{i}', (0.04, depth, height), (sx, 0, height / 2, 0), frame)
    for k, z in enumerate((0.1, 0.55, 1.0, 1.45, 1.88)):
        s += box_elems(f'shelf{k}', (length, depth, 0.03), (0, 0, z, 0), frame)
        if k < 4:
            cx = -length / 2 + 0.12
            while cx < length / 2 - 0.25:
                w = rnd.uniform(0.15, 0.35)
                h = rnd.uniform(0.12, 0.38)
                c = f'{rnd.uniform(0.1, 0.95):.2f} {rnd.uniform(0.1, 0.95):.2f} {rnd.uniform(0.1, 0.95):.2f}'
                s += box_elems(f'box{k}_{cx:.2f}', (w, depth * 0.8, h), (cx + w / 2, 0, z + 0.015 + h / 2, 0), c,
                               collide=False)
                cx += w + rnd.uniform(0.03, 0.15)
    return static_model(name, s, (x, y, 0, yaw))


def cabinet(name, x, y, yaw, colour):
    s = box_elems('body', (0.9, 0.5, 1.1), (0, 0, 0.55, 0), colour)
    s += box_elems('handle1', (0.02, 0.02, 0.2), (-0.1, -0.26, 0.8, 0), '0.8 0.8 0.8', collide=False)
    s += box_elems('handle2', (0.02, 0.02, 0.2), (0.1, -0.26, 0.8, 0), '0.8 0.8 0.8', collide=False)
    return static_model(name, s, (x, y, 0, yaw))


def pillar(name, x, y):
    return static_model(name, cyl_elems('p', 0.25, WALL_H, (0, 0, WALL_H / 2), '0.7 0.7 0.72'), (x, y, 0, 0))


def crate_stack(name, x, y, yaw):
    s = box_elems('c1', (0.6, 0.4, 0.4), (0, 0, 0.2, 0), '0.62 0.45 0.25')
    s += box_elems('c2', (0.6, 0.4, 0.4), (0.05, 0.02, 0.6, 0.2), '0.55 0.40 0.22')
    s += box_elems('c3', (0.5, 0.5, 0.5), (0.75, 0, 0.25, 0), '0.25 0.45 0.25')
    return static_model(name, s, (x, y, 0, yaw))


def bin_(name, x, y, colour):
    return static_model(name, cyl_elems('b', 0.2, 0.7, (0, 0, 0.35), colour), (x, y, 0, 0))


def sofa(name, x, y, yaw):
    c = '0.5 0.15 0.2'
    s = box_elems('seat', (1.8, 0.8, 0.42), (0, 0, 0.21, 0), c)
    s += box_elems('back', (1.8, 0.2, 0.45), (0, 0.3, 0.64, 0), c)
    s += box_elems('arm_l', (0.2, 0.8, 0.25), (0.8, 0, 0.545, 0), c)
    s += box_elems('arm_r', (0.2, 0.8, 0.25), (-0.8, 0, 0.545, 0), c)
    return static_model(name, s, (x, y, 0, yaw))


def plant(name, x, y):
    s = cyl_elems('pot', 0.18, 0.35, (0, 0, 0.175), '0.55 0.3 0.15')
    s += cyl_elems('leaves', 0.3, 0.7, (0, 0, 0.7), '0.15 0.5 0.15')
    return static_model(name, s, (x, y, 0, 0))


def build_furniture():
    m = ''
    for i, x in enumerate((-7.0, -4.2, -1.4, 1.4, 4.2, 7.0)):
        m += desk_with_monitor(f'desk_a{i}', x, -5.0, 0.0)
        if i % 2 == 0:
            m += desk_with_monitor(f'desk_b{i}', x + 0.6, -6.5, math.pi)
    for i, x in enumerate((-5.6, 0.0, 5.6)):
        m += pillar(f'pillar_{i}', x, -4.3)
    m += plant('plant_sw', -9.5, -6.5)
    m += plant('plant_se', 9.5, -6.5)
    m += bin_('bin_s1', -8.8, -4.2, '0.1 0.4 0.8')
    m += bin_('bin_s2', 9.3, -5.8, '0.8 0.6 0.1')
    m += shelf('shelf_w1', -9.65, -2.8, math.pi / 2, seed=1)
    m += shelf('shelf_w2', -9.65, 2.2, math.pi / 2, seed=2)
    m += cabinet('cabinet_w', -9.65, -0.3, math.pi / 2, '0.3 0.5 0.3')
    m += crate_stack('crates_w', -6.0, 3.6, 0.3)
    m += shelf('shelf_e1', 9.65, -1.5, -math.pi / 2, seed=3)
    m += sofa('sofa_e', 9.5, 2.6, -math.pi / 2)
    m += cabinet('cabinet_e', 9.7, -3.8, -math.pi / 2, '0.6 0.3 0.3')
    m += plant('plant_ne', 9.4, 4.0)
    m += bin_('bin_n1', -2.0, 4.15, '0.2 0.7 0.3')
    m += cabinet('cabinet_n', 2.2, 4.15, math.pi, '0.35 0.35 0.6')
    m += plant('plant_nw', -9.4, 4.0)
    m += shelf('shelf_store1', -8.0, 6.7, math.pi, seed=4)
    m += crate_stack('crates_store', -5.5, 5.8, 1.2)
    m += table('lab_bench1', -1.5, 6.3, 0.0, w=2.0, d=0.7, h=0.9, colour='0.8 0.8 0.8')
    m += table('lab_bench2', 1.5, 5.4, 0.0, w=1.6, d=0.7, h=0.9, colour='0.8 0.8 0.8')
    m += table('kitchen_table', 8.0, 5.9, 0.0, w=1.2, d=1.2)
    m += cabinet('kitchen_counter', 9.0, 6.7, math.pi, '0.9 0.9 0.9')
    m += table('meeting_table', -2.3, 0.0, 0.0, w=2.6, d=1.2)
    for i, x in enumerate((-3.3, -2.3, -1.3)):
        m += static_model(f'mchair_n{i}', box_elems('s', (0.45, 0.45, 0.45), (0, 0, 0.225, 0), '0.2 0.2 0.25'),
                          (x, 1.0, 0, 0))
        m += static_model(f'mchair_s{i}', box_elems('s', (0.45, 0.45, 0.45), (0, 0, 0.225, 0), '0.2 0.2 0.25'),
                          (x, -1.0, 0, 0))
    for i, y in enumerate((-1.6, -0.4, 0.8)):
        m += static_model(f'rack_{i}', box_elems('r', (0.8, 0.6, 2.0), (0, 0, 1.0, 0), '0.1 0.1 0.12'),
                          (3.8, y, 0, 0))
    return m


HEADER = """<?xml version="1.0" ?>
<!-- GENERATED by scripts/generate_world.py - edit the generator, not this file. -->
<sdf version="1.9">
  <world name="indoor_office">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <grid>false</grid>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <light type="point" name="ceiling_south">
      <cast_shadows>false</cast_shadows>
      <pose>0 -4.5 2.4 0 0 0</pose>
      <diffuse>0.4 0.4 0.38 1</diffuse>
      <attenuation><range>12</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation>
    </light>
    <light type="point" name="ceiling_north">
      <cast_shadows>false</cast_shadows>
      <pose>0 5.5 2.4 0 0 0</pose>
      <diffuse>0.4 0.4 0.38 1</diffuse>
      <attenuation><range>12</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.55 0.55 0.52 1</ambient><diffuse>0.55 0.55 0.52 1</diffuse>
                    <specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


def main():
    print(HEADER + build_walls() + build_posters() + build_rugs() + build_furniture() + FOOTER)


if __name__ == '__main__':
    main()

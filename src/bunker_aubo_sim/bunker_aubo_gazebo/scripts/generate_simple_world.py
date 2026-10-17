#!/usr/bin/env python3
"""Writes worlds/simple_room.sdf: outer walls, two pillars, two boxes. Nothing else.

    python3 scripts/generate_simple_world.py > worlds/simple_room.sdf
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_world import FOOTER, HEADER, box_elems, pillar, static_model, wall  # noqa: E402

# same 20 x 14 m outline as the office, so the default spawn (-7.5, -1.0) still works
X, Y = 10.0, 7.0

PILLARS = [(-3.0, 0.0), (3.0, 0.0)]
BOXES = [(0.0, 3.5, 1.0, 1.0, 1.0), (0.0, -3.5, 1.2, 0.8, 0.8)]  # x, y, sx, sy, sz


def build():
    s = ''
    s += wall('wall_s', -X, -Y, X, -Y, 'outer')
    s += wall('wall_n', -X, Y, X, Y, 'outer')
    s += wall('wall_w', -X, -Y, -X, Y, 'outer')
    s += wall('wall_e', X, -Y, X, Y, 'outer')
    m = static_model('room_walls', s)

    for i, (x, y) in enumerate(PILLARS):
        m += pillar(f'pillar_{i}', x, y)

    for i, (x, y, sx, sy, sz) in enumerate(BOXES):
        m += static_model(f'box_{i}', box_elems('b', (sx, sy, sz), (0, 0, sz / 2, 0), '0.6 0.45 0.3'),
                          (x, y, 0, 0))
    return m


def main():
    header = HEADER.replace('generate_world.py', 'generate_simple_world.py') \
                   .replace('name="indoor_office"', 'name="simple_room"')
    print(header + build() + FOOTER)


if __name__ == '__main__':
    main()

"""Offline accuracy check of a saved 2D map against the Gazebo world geometry.

    ros2 run bunker_aubo_slam evaluate_map maps/office.yaml \
        --world $(ros2 pkg prefix bunker_aubo_gazebo)/share/bunker_aubo_gazebo/worlds/indoor_office.sdf \
        --start -7.5 -1.0 1.5708

--start is the robot's true start pose in the world (the spawn pose in
sim.launch.py). The SLAM map frame starts there. The alignment is fixed; no
fitting is done, so any map rotation or offset counts as error.
"""
import argparse
import json
import os

import yaml

from .geometry import map_accuracy, se2, world_footprints
from .map_io import load_pgm, pgm_to_grid


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('--world', required=True)
    ap.add_argument('--start', nargs=3, type=float, default=[-7.5, -1.0, 1.5708], metavar=('X', 'Y', 'YAW'))
    args = ap.parse_args(argv)

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)
    img = load_pgm(os.path.join(os.path.dirname(os.path.abspath(args.map_yaml)), meta['image']))
    grid = pgm_to_grid(img)
    res = map_accuracy(grid, float(meta['resolution']), tuple(meta['origin'][:2]),
                       se2(*args.start), world_footprints(args.world))
    print(json.dumps(res, indent=2))


if __name__ == '__main__':
    main()

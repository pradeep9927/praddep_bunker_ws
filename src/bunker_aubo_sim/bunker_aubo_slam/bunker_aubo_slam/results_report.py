"""Copy the outputs of a recorded run into docs/results and write docs/results/README.md.

Only numbers that the evaluator actually produced are written. Anything that was not measured
is shown as "not measured" so the report never contains made-up values.

    ros2 run bunker_aubo_slam fill_results --repo ~/bunker_ws/src/bunker_aubo_sim \
        --maps ~/bunker_maps --name office --video <link> --bag <link-or-path>
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
from typing import Optional

DB_INLINE_LIMIT_MB = 50.0
MAP_FILES = ('{n}.pgm', '{n}.yaml', '{n}_cloud.ply')
EVAL_FILES = ('evaluation.md', 'evaluation.json', 'trajectory.png', 'trajectory.csv', 'loop_closures.csv')


def _num(value, fmt='{:.3f}', scale=1.0) -> str:
    if value is None:
        return 'not measured'
    try:
        v = float(value) * scale
    except (TypeError, ValueError):
        return 'not measured'
    return 'not measured' if math.isnan(v) else fmt.format(v)


def copy_outputs(maps_dir: str, name: str, out_dir: str) -> dict:
    """Copy map + evaluation files. Returns {'copied': [...], 'missing': [...], 'db': ...}."""
    os.makedirs(out_dir, exist_ok=True)
    copied, missing = [], []
    for pattern in MAP_FILES:
        f = pattern.format(n=name)
        src = os.path.join(maps_dir, f)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(out_dir, f))
            copied.append(f)
        else:
            missing.append(f)
    eval_dir = os.path.join(maps_dir, 'evaluation')
    for f in EVAL_FILES:
        src = os.path.join(eval_dir, f)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(out_dir, f))
            copied.append(f)
    db = os.path.join(maps_dir, f'{name}.db')
    db_info = None
    if os.path.isfile(db):
        size_mb = os.path.getsize(db) / 1e6
        if size_mb <= DB_INLINE_LIMIT_MB:
            shutil.copy2(db, os.path.join(out_dir, f'{name}.db'))
            copied.append(f'{name}.db')
            db_info = f'`maps/{name}.db` ({size_mb:.1f} MB)'
        else:
            db_info = f'{size_mb:.0f} MB, too large for git, keep it next to the bag'
    else:
        missing.append(f'{name}.db')
    return {'copied': copied, 'missing': missing, 'db': db_info}


def render_readme(evaluation: Optional[dict], name: str, links: dict, files: dict) -> str:
    ev = evaluation or {}
    s = ev.get('slam_anchored', {})
    o = ev.get('odom_anchored', {})
    m = ev.get('map_2d', {})
    yaw = ev.get('slam_yaw_error_deg', {})

    def link(key):
        return links.get(key) or 'add link'

    lines = [
        '# Results',
        '',
        'Outputs of the final recorded run in the `indoor_office` world. The numbers below are copied',
        'from `evaluation.json`, written by `slam_evaluator` against the Gazebo ground truth pose.',
        '',
        '## Recordings',
        '',
        '| What | Where |',
        '|---|---|',
        f'| Demo video (5 to 10 min) | {link("video")} |',
        f'| ROS 2 bag (mcap, zstd) | {link("bag")} |',
        f'| RTAB-Map database `{name}.db` | {files.get("db") or link("db")} |',
        '',
        '## Saved maps',
        '',
        f'* `maps/{name}.pgm`, `maps/{name}.yaml`: 2D occupancy grid (load with `load_map.launch.py`)',
        f'* `maps/{name}_cloud.ply`: registered 3D cloud (MeshLab / CloudCompare)',
        f'* `maps/{name}.db`: RTAB-Map database, reload with '
        f'`slam.launch.py localization:=true database_path:=<path>/{name}.db`',
        '* `maps/trajectory.png`, `maps/evaluation.md`: trajectory plot and full report',
        '',
    ]
    if files.get('missing'):
        lines += ['Not found when the report was written: ' + ', '.join(f'`{f}`' for f in files['missing']), '']

    lines += [
        '## Accuracy against Gazebo ground truth',
        '',
        '| Metric | Value |',
        '|---|---|',
        f'| Path length | {_num(s.get("path_length"), "{:.1f}")} m |',
        f'| Loop closures / proximity detections | {ev.get("loop_closures", "not measured")} / '
        f'{ev.get("proximity_detections", "not measured")} |',
        f'| SLAM ATE RMSE (anchored at start) | {_num(s.get("ate_rmse"))} m |',
        f'| SLAM final position error | {_num(s.get("final_error"))} m '
        f'({_num(s.get("final_error_percent"), "{:.2f}")} % of path) |',
        f'| SLAM heading error, mean / max | {_num(yaw.get("mean"), "{:.2f}")} / '
        f'{_num(yaw.get("max"), "{:.2f}")} deg |',
        f'| EKF odometry only, ATE RMSE (anchored) | {_num(o.get("ate_rmse"))} m |',
        f'| EKF odometry only, final error | {_num(o.get("final_error"))} m |',
        f'| Largest trajectory jump beyond ground truth | {_num(ev.get("slam_max_step_minus_truth_m"))} m |',
        f'| Map: median distance of occupied cells to true obstacles | {_num(m.get("median_error_m"))} m |',
        f'| Map: occupied cells within 10 cm | {_num(m.get("within_10cm"), "{:.1f}", 100.0)} % |',
        f'| Map: wall coverage (15 cm tolerance) | {_num(m.get("wall_coverage"), "{:.1f}", 100.0)} % |',
        '',
    ]
    if not ev:
        lines += ['No `evaluation.json` was found, so no accuracy numbers are reported yet.', '']
    return '\n'.join(lines)


def fill(repo: str, maps_dir: str, name: str, links: dict) -> str:
    results = os.path.join(repo, 'docs', 'results')
    files = copy_outputs(os.path.expanduser(maps_dir), name, os.path.join(results, 'maps'))
    ev_path = os.path.join(results, 'maps', 'evaluation.json')
    evaluation = None
    if os.path.isfile(ev_path):
        with open(ev_path) as f:
            evaluation = json.load(f)
    text = render_readme(evaluation, name, links, files)
    with open(os.path.join(results, 'README.md'), 'w') as f:
        f.write(text)
    return text


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--repo', default='.', help='repository root (contains docs/results)')
    p.add_argument('--maps', default='~/bunker_maps', help='directory given to save_map.sh')
    p.add_argument('--name', default='office')
    p.add_argument('--video', default='')
    p.add_argument('--bag', default='')
    p.add_argument('--db', default='')
    a, _ = p.parse_known_args(argv)
    if not os.path.isdir(os.path.join(a.repo, 'docs')):
        p.error(f'{a.repo} does not look like the repository root (no docs/ folder)')
    fill(a.repo, a.maps, a.name, {'video': a.video, 'bag': a.bag, 'db': a.db})
    print(f'wrote {os.path.join(a.repo, "docs", "results", "README.md")}')


if __name__ == '__main__':
    main()

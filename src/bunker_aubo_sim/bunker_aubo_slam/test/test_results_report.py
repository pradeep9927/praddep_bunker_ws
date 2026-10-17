import json
import os

from bunker_aubo_slam.results_report import fill, render_readme


def test_unmeasured_values_are_never_invented():
    text = render_readme(None, 'office', {}, {'missing': ['office.pgm']})
    assert 'not measured' in text
    assert 'No `evaluation.json`' in text
    assert '_ m' not in text


def test_fill_copies_maps_and_reports_real_numbers(tmp_path):
    maps = tmp_path / 'maps'
    (maps / 'evaluation').mkdir(parents=True)
    for f in ('office.pgm', 'office.yaml', 'office_cloud.ply', 'office.db'):
        (maps / f).write_bytes(b'x')
    ev = {'samples': 900, 'loop_closures': 3, 'proximity_detections': 5,
          'slam_anchored': {'ate_rmse': 0.0812, 'final_error': 0.05, 'final_error_percent': 0.09,
                            'path_length': 61.4},
          'odom_anchored': {'ate_rmse': 0.6, 'final_error': 1.1},
          'map_2d': {'median_error_m': 0.031, 'within_10cm': 0.91, 'wall_coverage': 0.87}}
    (maps / 'evaluation' / 'evaluation.json').write_text(json.dumps(ev))
    repo = tmp_path / 'repo'
    (repo / 'docs' / 'results').mkdir(parents=True)
    text = fill(str(repo), str(maps), 'office', {'video': 'https://example.org/v'})
    assert os.path.isfile(repo / 'docs' / 'results' / 'maps' / 'office.pgm')
    assert '| SLAM ATE RMSE (anchored at start) | 0.081 m |' in text
    assert '| Loop closures / proximity detections | 3 / 5 |' in text
    assert '91.0 %' in text and 'https://example.org/v' in text

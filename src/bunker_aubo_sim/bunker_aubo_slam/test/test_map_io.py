import numpy as np
import yaml

from bunker_aubo_slam.map_io import load_pgm, pgm_to_grid, save_occupancy_pgm, save_ply


def test_pgm_roundtrip(tmp_path):
    grid = np.full((20, 30), -1, np.int16)
    grid[5:10, :] = 0
    grid[7, 3:20] = 100
    pgm, yml = save_occupancy_pgm(str(tmp_path / 'm'), grid, 0.05, (-1.0, -2.0, 0.0))
    meta = yaml.safe_load(open(yml))
    assert meta['image'] == 'm.pgm' and meta['resolution'] == 0.05 and meta['origin'] == [-1.0, -2.0, 0.0]
    back = pgm_to_grid(load_pgm(pgm))
    assert np.array_equal(back, grid)


def test_ply_header_and_size(tmp_path):
    xyz = np.random.default_rng(0).normal(size=(100, 3)).astype(np.float32)
    rgb = np.full((100, 3), 200, np.uint8)
    p = save_ply(str(tmp_path / 'c.ply'), xyz, rgb)
    data = open(p, 'rb').read()
    head, body = data.split(b'end_header\n', 1)
    assert b'element vertex 100' in head and b'property uchar red' in head
    assert len(body) == 100 * 15

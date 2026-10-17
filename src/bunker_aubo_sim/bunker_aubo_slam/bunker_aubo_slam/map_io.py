import os

import numpy as np
import yaml


def save_occupancy_pgm(path_base: str, grid: np.ndarray, resolution: float, origin_xyyaw,
                       occupied_thresh: float = 0.65, free_thresh: float = 0.25):
    img = np.full(grid.shape, 205, np.uint8)            # unknown
    known = grid >= 0
    occ = np.clip(grid, 0, 100).astype(np.float32) / 100.0
    img[known & (occ >= occupied_thresh)] = 0
    img[known & (occ <= free_thresh)] = 254
    mid = known & (occ > free_thresh) & (occ < occupied_thresh)
    img[mid] = (255 * (1.0 - occ[mid])).astype(np.uint8)
    img = np.flipud(img)
    h, w = img.shape
    with open(path_base + '.pgm', 'wb') as f:
        f.write(f'P5\n# CREATOR: bunker_aubo_slam {resolution:.3f} m/pix\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    meta = {
        'image': os.path.basename(path_base) + '.pgm',
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(origin_xyyaw[0]), float(origin_xyyaw[1]), float(origin_xyyaw[2])],
        'negate': 0,
        'occupied_thresh': occupied_thresh,
        'free_thresh': free_thresh,
    }
    with open(path_base + '.yaml', 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return path_base + '.pgm', path_base + '.yaml'


def load_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        data = f.read()
    parts = []
    idx = 0
    while len(parts) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            idx = data.index(b'\n', idx) + 1
            continue
        end = idx
        while not data[end:end + 1].isspace():
            end += 1
        parts.append(data[idx:end])
        idx = end
    idx += 1
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[idx:idx + w * h], np.uint8).reshape(h, w)


def pgm_to_grid(img: np.ndarray) -> np.ndarray:
    img = np.flipud(img)
    grid = np.full(img.shape, -1, np.int16)
    grid[img == 0] = 100
    grid[img >= 254] = 0
    return grid


def save_ply(path: str, xyz: np.ndarray, rgb: np.ndarray = None):
    xyz = np.asarray(xyz, np.float32)
    n = len(xyz)
    header = ['ply', 'format binary_little_endian 1.0', 'comment bunker_aubo_slam cloud_map',
              f'element vertex {n}', 'property float x', 'property float y', 'property float z']
    if rgb is not None:
        header += ['property uchar red', 'property uchar green', 'property uchar blue']
    header.append('end_header')
    with open(path, 'wb') as f:
        f.write(('\n'.join(header) + '\n').encode())
        if rgb is None:
            f.write(xyz.astype('<f4').tobytes())
        else:
            rec = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                     ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
            rec['x'], rec['y'], rec['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            rgb = np.asarray(rgb, np.uint8)
            rec['r'], rec['g'], rec['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
            f.write(rec.tobytes())
    return path


def cloud_fields_to_arrays(msg):
    offs = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = bytes(msg.data)
    step = msg.point_step
    base = np.frombuffer(raw, np.uint8)[:n * step].reshape(n, step)

    def f32(off):
        return base[:, off:off + 4].copy().view('<f4').ravel()
    xyz = np.stack([f32(offs['x']), f32(offs['y']), f32(offs['z'])], axis=1)
    rgb = None
    key = 'rgb' if 'rgb' in offs else ('rgba' if 'rgba' in offs else None)
    if key:
        packed = base[:, offs[key]:offs[key] + 4].copy().view('<u4').ravel()
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=1).astype(np.uint8)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], (rgb[ok] if rgb is not None else None)

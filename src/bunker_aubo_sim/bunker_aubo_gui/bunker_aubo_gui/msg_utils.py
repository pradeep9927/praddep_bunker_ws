from typing import Optional

import numpy as np

_CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}


def _buffer(data):
    if isinstance(data, (list, tuple)):
        return bytes(data)
    return data


def image_to_rgb(msg) -> Optional[np.ndarray]:
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    if h == 0 or w == 0:
        return None
    if enc not in _CHANNELS:
        return None
    ch = _CHANNELS[enc]
    buf = np.frombuffer(_buffer(msg.data), dtype=np.uint8)
    if buf.size < h * step:
        return None
    img = buf[:h * step].reshape(h, step)[:, :w * ch].reshape(h, w, ch)
    if enc == 'mono8':
        return np.repeat(img, 3, axis=2)
    if enc.startswith('bgr'):
        img = img[:, :, 2::-1] if ch == 3 else img[:, :, [2, 1, 0]]
    else:
        img = img[:, :, :3]
    return np.ascontiguousarray(img)


def depth_to_metres(msg) -> Optional[np.ndarray]:
    enc = msg.encoding.upper()
    h, w, step = msg.height, msg.width, msg.step
    raw = _buffer(msg.data)
    if enc == '32FC1':
        dtype, scale, bpp = np.float32, 1.0, 4
    elif enc in ('16UC1', 'MONO16'):
        dtype, scale, bpp = np.uint16, 0.001, 2
    else:
        return None
    endian = '>' if getattr(msg, 'is_bigendian', False) else '<'
    arr = np.frombuffer(raw, dtype=np.dtype(dtype).newbyteorder(endian))
    row = step // bpp
    if arr.size < h * row:
        return None
    return arr[:h * row].reshape(h, row)[:, :w].astype(np.float32) * scale


def colorize_depth(depth: np.ndarray, near: float = 0.1, far: float = 8.0) -> np.ndarray:
    d = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    valid = (d > near) & (d < far)
    t = np.clip((d - near) / (far - near), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0, 1)
    g = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0, 1)
    b = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0, 1)
    rgb = (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)
    rgb[~valid] = 0
    return rgb


_PF_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def pointcloud2_to_xyz(msg, max_points: int = 0) -> np.ndarray:
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            offsets[f.name] = (f.offset, _PF_DTYPES.get(f.datatype, 'f4'))
    if len(offsets) != 3:
        return np.zeros((0, 3), np.float32)
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), np.float32)
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + offsets[k][1] for k in ('x', 'y', 'z')],
        'offsets': [offsets[k][0] for k in ('x', 'y', 'z')],
        'itemsize': msg.point_step,
    })
    raw = _buffer(msg.data)
    if msg.row_step * msg.height == memoryview(raw).nbytes and msg.row_step == msg.width * msg.point_step:
        pts = np.frombuffer(raw, dtype=dtype, count=n)
    else:  # padded rows
        rows = [np.frombuffer(raw, dtype=dtype, count=msg.width, offset=r * msg.row_step)
                for r in range(msg.height)]
        pts = np.concatenate(rows)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if max_points and len(xyz) > max_points:
        idx = np.linspace(0, len(xyz) - 1, max_points).astype(np.int64)
        xyz = xyz[idx]
    return xyz


def height_colors(z: np.ndarray, zmin: float = -1.0, zmax: float = 1.5) -> np.ndarray:
    t = np.clip((z - zmin) / (zmax - zmin), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0, 1)
    g = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0, 1)
    b = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0, 1)
    return np.stack([r, g, b, np.ones_like(t)], axis=-1).astype(np.float32)

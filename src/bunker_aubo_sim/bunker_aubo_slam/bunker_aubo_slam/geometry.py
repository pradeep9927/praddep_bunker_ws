import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

import numpy as np


def se2(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def apply_se2(t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    return pts @ t[:2, :2].T + t[:2, 2]


def align_2d(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    est = np.asarray(est, float)
    gt = np.asarray(gt, float)
    mu_e, mu_g = est.mean(axis=0), gt.mean(axis=0)
    h = (est - mu_e).T @ (gt - mu_g)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, d]) @ u.T
    t = np.eye(3)
    t[:2, :2] = r
    t[:2, 2] = mu_g - r @ mu_e
    return t


def trajectory_errors(est_xy: np.ndarray, gt_xy: np.ndarray, align: bool = True) -> Dict[str, float]:
    est_xy = np.asarray(est_xy, float)
    gt_xy = np.asarray(gt_xy, float)
    if len(est_xy) < 2:
        return {'n': len(est_xy), 'ate_rmse': float('nan'), 'ate_mean': float('nan'),
                'ate_max': float('nan'), 'final_error': float('nan'), 'path_length': 0.0}
    if align:
        est_xy = apply_se2(align_2d(est_xy, gt_xy), est_xy)
    err = np.linalg.norm(est_xy - gt_xy, axis=1)
    length = float(np.sum(np.linalg.norm(np.diff(gt_xy, axis=0), axis=1)))
    return {
        'n': int(len(err)),
        'ate_rmse': float(np.sqrt(np.mean(err ** 2))),
        'ate_mean': float(err.mean()),
        'ate_max': float(err.max()),
        'final_error': float(err[-1]),
        'path_length': length,
        'final_error_percent': float(100.0 * err[-1] / length) if length > 0 else float('nan'),
    }


def associate(t_est: np.ndarray, t_gt: np.ndarray, max_dt: float = 0.05) -> Tuple[np.ndarray, np.ndarray]:
    t_est = np.asarray(t_est, float)
    t_gt = np.asarray(t_gt, float)
    if len(t_gt) == 0 or len(t_est) == 0:
        return np.zeros(0, int), np.zeros(0, int)
    j = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    left, right = t_gt[j - 1], t_gt[j]
    j = np.where(np.abs(t_est - left) <= np.abs(right - t_est), j - 1, j)
    ok = np.abs(t_gt[j] - t_est) <= max_dt
    return np.nonzero(ok)[0], j[ok]


def _pose(el) -> List[float]:
    p = el.find('pose')
    vals = [float(v) for v in p.text.split()] if p is not None and p.text else [0.0] * 6
    return (vals + [0.0] * 6)[:6]


def world_footprints(sdf_path: str, z_min: float = 0.10, z_max: float = 2.0):
    root = ET.parse(sdf_path).getroot()
    shapes = []
    for model in root.iter('model'):
        mx, my, mz, _, _, myaw = _pose(model)
        for col in model.iter('collision'):
            geom = col.find('geometry')
            if geom is None or len(geom) == 0:
                continue
            g = geom[0]
            px, py, pz, _, _, pyaw = _pose(col)
            c, s = math.cos(myaw), math.sin(myaw)
            cx, cy, cz = mx + c * px - s * py, my + s * px + c * py, mz + pz
            yaw = myaw + pyaw
            if g.tag == 'box':
                sx, sy, sz = (float(v) for v in g.find('size').text.split())
                if cz + sz / 2 < z_min or cz - sz / 2 > z_max:
                    continue
                shapes.append(('box', cx, cy, yaw, sx, sy))
            elif g.tag == 'cylinder':
                r = float(g.find('radius').text)
                h = float(g.find('length').text)
                if cz + h / 2 < z_min or cz - h / 2 > z_max:
                    continue
                shapes.append(('circle', cx, cy, r))
    return shapes


def rasterize(shapes, origin: Tuple[float, float], size: Tuple[int, int], res: float) -> np.ndarray:
    h, w = size
    xs = origin[0] + (np.arange(w) + 0.5) * res
    ys = origin[1] + (np.arange(h) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    occ = np.zeros((h, w), bool)
    for s in shapes:
        if s[0] == 'box':
            _, cx, cy, yaw, sx, sy = s
            c, sn = math.cos(yaw), math.sin(yaw)
            dx, dy = gx - cx, gy - cy
            lx, ly = c * dx + sn * dy, -sn * dx + c * dy
            occ |= (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
        else:
            _, cx, cy, r = s
            occ |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
    return occ


def distance_field(occ: np.ndarray, res: float) -> np.ndarray:
    try:
        from scipy import ndimage
        return ndimage.distance_transform_edt(~occ) * res
    except ImportError:  # pragma: no cover - brute-force fallback, fine for small maps
        pts = np.argwhere(occ)
        out = np.full(occ.shape, np.inf)
        if len(pts) == 0:
            return out
        idx = np.argwhere(np.ones_like(occ))
        for chunk in np.array_split(idx, max(1, len(idx) // 20000)):
            d = np.sqrt(((chunk[:, None, :] - pts[None, :, :]) ** 2).sum(-1)).min(1)
            out[chunk[:, 0], chunk[:, 1]] = d * res
        return out


def map_accuracy(grid: np.ndarray, res: float, map_origin_xy: Tuple[float, float],
                 world_T_map: np.ndarray, shapes, occupied_thresh: int = 65,
                 coverage_tol: float = 0.15) -> Dict[str, float]:
    occ_rc = np.argwhere(grid >= occupied_thresh)
    if len(occ_rc) == 0:
        return {'occupied_cells': 0}
    map_xy = np.stack([map_origin_xy[0] + (occ_rc[:, 1] + 0.5) * res,
                       map_origin_xy[1] + (occ_rc[:, 0] + 0.5) * res], axis=1)
    world_xy = apply_se2(world_T_map, map_xy)

    gres = 0.02
    lo = world_xy.min(axis=0) - 2.0
    hi = world_xy.max(axis=0) + 2.0
    size = (int(math.ceil((hi[1] - lo[1]) / gres)), int(math.ceil((hi[0] - lo[0]) / gres)))
    truth = rasterize(shapes, (lo[0], lo[1]), size, gres)
    dist = distance_field(truth, gres)
    ci = np.clip(((world_xy[:, 0] - lo[0]) / gres).astype(int), 0, size[1] - 1)
    ri = np.clip(((world_xy[:, 1] - lo[1]) / gres).astype(int), 0, size[0] - 1)
    d = dist[ri, ci]

    boundary = truth & ~_erode(truth)
    known_world = _known_mask_world(grid, res, map_origin_xy, world_T_map, lo, size, gres)
    b_rc = np.argwhere(boundary & known_world)
    covered = float('nan')
    if len(b_rc):
        map_occ = np.zeros(size, bool)
        map_occ[ri, ci] = True
        dmap = distance_field(map_occ, gres)
        covered = float(np.mean(dmap[b_rc[:, 0], b_rc[:, 1]] <= coverage_tol))
    return {
        'occupied_cells': int(len(d)),
        'mean_error_m': float(d.mean()),
        'median_error_m': float(np.median(d)),
        'p95_error_m': float(np.percentile(d, 95)),
        'within_5cm': float(np.mean(d <= 0.05)),
        'within_10cm': float(np.mean(d <= 0.10)),
        'wall_coverage': covered,
    }


def _erode(mask: np.ndarray) -> np.ndarray:
    m = mask.copy()
    m[1:, :] &= mask[:-1, :]
    m[:-1, :] &= mask[1:, :]
    m[:, 1:] &= mask[:, :-1]
    m[:, :-1] &= mask[:, 1:]
    return m


def _known_mask_world(grid, res, map_origin_xy, world_T_map, lo, size, gres):
    known = np.argwhere(grid >= 0)
    xy = np.stack([map_origin_xy[0] + (known[:, 1] + 0.5) * res,
                   map_origin_xy[1] + (known[:, 0] + 0.5) * res], axis=1)
    w = apply_se2(world_T_map, xy)
    ci = ((w[:, 0] - lo[0]) / gres).astype(int)
    ri = ((w[:, 1] - lo[1]) / gres).astype(int)
    ok = (ci >= 0) & (ci < size[1]) & (ri >= 0) & (ri < size[0])
    mask = np.zeros(size, bool)
    mask[ri[ok], ci[ok]] = True
    k = max(1, int(round(res / gres)) + 2)
    out = mask.copy()
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            out |= np.roll(np.roll(mask, dr, 0), dc, 1)
    return out

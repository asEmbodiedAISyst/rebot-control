"""Calibration poses: random joint configurations that keep the arm and tool clear of the table and the base.

The clearance check is a simple kinematic one (no meshes): points along the segments between joint origins and the
TCP must stay above ``table_z`` and outside a cylinder around the base. It is deliberately conservative; the pose
list is printed before anything moves, and every move is a slow joint-space move with stall detection.
"""

from __future__ import annotations

import numpy as np

from ..model import RobotModel

# Joint ranges (deg) for calibration poses: well inside the soft limits, arm in front of the base.
DEFAULT_RANGES_DEG = np.array([[-60, 60], [15, 120], [25, 160], [-70, 80], [-80, 80], [-120, 120]], dtype=float)


def segment_points(pts: np.ndarray, per_segment: int = 4) -> np.ndarray:
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        for s in np.linspace(0.0, 1.0, per_segment + 1)[1:]:
            out.append(a + s * (b - a))
    return np.array(out)


def is_clear(model: RobotModel, q: np.ndarray, table_z: float = 0.04, base_radius: float = 0.09,
             base_height: float = 0.20, data=None) -> bool:
    """True when the arm + tool at ``q`` stays above the table and away from the base column."""
    pts = model.link_positions(q, data)  # joint origins 1..n, then the TCP
    # skip the base joints (1, 2): they sit on the base by construction
    pts = segment_points(pts[2:])
    if (pts[:, 2] < table_z).any():
        return False
    r = np.linalg.norm(pts[:, :2], axis=1)
    return not ((r < base_radius) & (pts[:, 2] < base_height)).any()


def path_is_clear(model: RobotModel, q0: np.ndarray, q1: np.ndarray, steps: int = 12, **kw) -> bool:
    return all(is_clear(model, q0 + s * (q1 - q0), **kw) for s in np.linspace(0.0, 1.0, steps))


def generate_poses(model: RobotModel, n: int, start: np.ndarray, rng: np.random.Generator | None = None,
                   ranges_deg: np.ndarray | None = None, max_tries: int = 20000, **clear_kw) -> np.ndarray:
    """``n`` random clear poses ordered as a short tour from ``start`` (every segment is also clear)."""
    rng = rng or np.random.default_rng(0)
    rngs = np.radians(DEFAULT_RANGES_DEG if ranges_deg is None else np.asarray(ranges_deg, dtype=float))
    lo = np.maximum(rngs[:, 0], model.lower + 0.05)
    hi = np.minimum(rngs[:, 1], model.upper - 0.05)
    cands = []
    for _ in range(max_tries):
        q = rng.uniform(lo, hi)
        if is_clear(model, q, **clear_kw):
            cands.append(q)
            if len(cands) >= 3 * n:
                break
    if len(cands) < n:
        raise RuntimeError(f"only {len(cands)} clear poses found; widen the ranges or lower table_z")
    # spread: farthest point sampling, then a greedy nearest-neighbour tour with clear segments
    cands = np.array(cands)
    chosen = [int(np.argmax(np.abs(cands - start).max(axis=1)))]
    dmin = np.abs(cands - cands[chosen[0]]).max(axis=1)
    while len(chosen) < n:
        i = int(np.argmax(dmin))
        chosen.append(i)
        dmin = np.minimum(dmin, np.abs(cands - cands[i]).max(axis=1))
    todo = [cands[i] for i in chosen]
    tour, cur = [], np.asarray(start, dtype=float)
    while todo:
        order = np.argsort([np.abs(q - cur).max() for q in todo])
        for k in order:
            if path_is_clear(model, cur, todo[k], **clear_kw):
                cur = todo.pop(int(k))
                tour.append(cur)
                break
        else:  # nothing reachable on a straight line from here: drop the nearest
            todo.pop(int(order[0]))
    return np.array(tour)

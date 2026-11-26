"""TCP position by pivot calibration.

The tool tip is put in a fixed point (a small dimple or a cone in a block fixed to the table) and the arm is moved
by hand (gravity mode) through several orientations around it. For every recorded flange pose ``(p_i, R_i)``::

    R_i t + p_i = c        (t: tip in the flange frame, c: the pivot point in the base frame)

which is linear in ``(t, c)``: ``[R_i  -I] [t; c] = -p_i``, solved by least squares. The residual tells how well
the tip stayed in the dimple. Orientations should spread over at least 30-40 degrees in two directions.

Only the TCP *position* is identified; the TCP orientation stays as configured in the tool YAML.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from ..pose import matrix_to_rotvec


@dataclass
class PivotFit:
    tcp_xyz: np.ndarray  # tip in the flange frame (m)
    pivot: np.ndarray  # pivot point in the base frame (m)
    rms: float  # RMS distance between the predicted tip positions and the pivot (m)
    max_err: float
    spread_deg: float  # largest angle between two recorded orientations

    def summary(self) -> str:
        return (f"TCP in flange frame: {np.round(self.tcp_xyz, 5).tolist()} m\n"
                f"pivot point (base): {np.round(self.pivot, 4).tolist()} m\n"
                f"residual RMS {self.rms * 1000:.2f} mm, max {self.max_err * 1000:.2f} mm, "
                f"orientation spread {self.spread_deg:.0f} deg")


def solve_pivot(positions: np.ndarray, rotations: np.ndarray) -> PivotFit:
    """Least-squares pivot solution from flange poses (``positions`` (N, 3), ``rotations`` (N, 3, 3))."""
    P, R = np.asarray(positions, dtype=float), np.asarray(rotations, dtype=float)
    if len(P) < 4:
        raise ValueError("need at least 4 poses")
    A = np.vstack([np.hstack([Ri, -np.eye(3)]) for Ri in R])
    b = -P.reshape(-1)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    t, c = x[:3], x[3:]
    tips = np.einsum("nij,j->ni", R, t) + P
    err = np.linalg.norm(tips - c, axis=1)
    spread = max(np.linalg.norm(matrix_to_rotvec(Ri.T @ Rj)) for Ri in R for Rj in R)
    sv = np.linalg.svd(A, compute_uv=False)
    if sv[-1] < 1e-3 * sv[0]:
        raise ValueError("orientations too similar: the pivot problem is ill-conditioned (tilt the tool more)")
    return PivotFit(tcp_xyz=t, pivot=c, rms=float(np.sqrt(np.mean(err ** 2))), max_err=float(err.max()),
                    spread_deg=float(np.degrees(spread)))


def collect_pivot(arm, n_poses: int = 8, average_s: float = 0.3, prompt: Callable[[str], str] = input,
                  progress: Callable[[str], None] = print) -> tuple[np.ndarray, np.ndarray]:
    """Interactive: gravity mode, the user places the tip in the dimple and presses Enter for each pose."""
    arm.gravity_mode()
    P, R = [], []
    try:
        while len(P) < n_poses:
            ans = prompt(f"[{len(P) + 1}/{n_poses}] tip in the dimple, new orientation, hold still, Enter "
                         f"(q = finish early): ")
            if ans.strip().lower() == "q":
                break
            with arm.recording() as rec:
                arm.sleep(average_s)
            if np.abs(rec["dq"]).max() > np.radians(5.0):
                progress("  the arm was moving: pose skipped, try again")
                continue
            q = rec["q"].mean(axis=0)
            p, Rf = arm.model.flange_pose(q)
            P.append(p)
            R.append(Rf)
            progress(f"  flange at {np.round(p, 4).tolist()}")
    finally:
        arm.hold()
    return np.array(P), np.array(R)

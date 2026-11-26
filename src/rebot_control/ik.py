"""Damped least-squares (DLS) inverse kinematics of the TCP.

The algorithm, with kinematics from Pinocchio:

* error = [position error; ``rot_weight`` * orientation error], the orientation error being the small-angle
  rotation vector ``0.5 * sum_i r_i x r_i*`` over the columns of the current / target rotation;
* step ``dq = J^T (J J^T + lambda I)^-1 e`` plus a null-space term pulling towards a preferred ``posture``
  (avoids drifting into odd configurations and singularities);
* the step norm is capped at ``max_step`` and the result is clipped to the joint limits after every iteration.

Warm-started from the previous solution, 30 iterations per 30 Hz control cycle are enough for streaming targets.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model import RobotModel


@dataclass
class IKResult:
    q: np.ndarray
    pos_err: float  # m
    rot_err: float  # rad
    success: bool
    iterations: int


def small_angle_rot_error(R: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """Orientation error as a rotation vector in the base frame (small-angle approximation, sin(theta) * axis)."""
    return 0.5 * (np.cross(R[:, 0], R_target[:, 0]) + np.cross(R[:, 1], R_target[:, 1]) + np.cross(R[:, 2], R_target[:, 2]))


class DiffIK:
    """Position (+ optional orientation) IK of the model's TCP frame.

    Args:
        model: The arm + tool model; its own Pinocchio data is created here (one solver per thread).
        posture: Preferred joint configuration for the null-space term (default: the middle of the joint range).
        damping: DLS damping lambda.
        posture_gain: Null-space gain towards ``posture``.
        rot_weight: Weight of the orientation error relative to the position error (m vs rad).
        success_pos_tol / success_rot_tol: A solution further than this from the target is reported as failed.
    """

    def __init__(self, model: RobotModel, posture: np.ndarray | None = None, damping: float = 1e-3,
                 posture_gain: float = 0.05, rot_weight: float = 0.3, success_pos_tol: float = 5e-3,
                 success_rot_tol: float = 5e-2) -> None:
        self.model = model
        self.data = model.new_data()
        self.lower = model.lower + 1e-3
        self.upper = model.upper - 1e-3
        self.posture = 0.5 * (model.lower + model.upper) if posture is None else np.asarray(posture, dtype=float)
        self.damping = damping
        self.posture_gain = posture_gain
        self.rot_weight = rot_weight
        self.success_pos_tol = success_pos_tol
        self.success_rot_tol = success_rot_tol

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return self.model.fk(q, self.data)

    def solve(self, target_pos: np.ndarray, target_rot: np.ndarray | None, q_init: np.ndarray, max_iters: int = 50,
              pos_tol: float = 1e-4, rot_tol: float = 1e-3, max_step: float = 0.2) -> IKResult:
        """Joint angles that bring the TCP to ``target_pos`` (and ``target_rot`` unless it is None)."""
        n = self.model.n
        q = np.clip(np.asarray(q_init, dtype=float).copy(), self.lower, self.upper)
        target_pos = np.asarray(target_pos, dtype=float)
        pos_err = rot_err = np.inf
        it = 0
        for it in range(1, max_iters + 1):  # noqa: B007 - `it` is reported after the loop
            pos, R = self.model.fk(q, self.data)
            e_pos = target_pos - pos
            pos_err = float(np.linalg.norm(e_pos))
            if target_rot is not None:
                e_rot = small_angle_rot_error(R, target_rot)
                rot_err = float(np.linalg.norm(e_rot))
            else:
                e_rot, rot_err = np.zeros(3), 0.0
            if pos_err < pos_tol and rot_err < rot_tol:
                break
            J6 = self.model.jacobian(q, self.data)
            if target_rot is not None:
                J = np.vstack([J6[:3], self.rot_weight * J6[3:]])
                e = np.concatenate([e_pos, self.rot_weight * e_rot])
            else:
                J, e = J6[:3], e_pos
            JJt = J @ J.T + self.damping * np.eye(J.shape[0])
            J_pinv = J.T @ np.linalg.inv(JJt)
            dq = J_pinv @ e
            dq += (np.eye(n) - J_pinv @ J) @ (self.posture_gain * (self.posture - q))
            norm = np.linalg.norm(dq)
            if norm > max_step:
                dq *= max_step / norm
            q = np.clip(q + dq, self.lower, self.upper)
        else:  # loop ran out: evaluate the final iterate
            pos, R = self.model.fk(q, self.data)
            pos_err = float(np.linalg.norm(target_pos - pos))
            rot_err = float(np.linalg.norm(small_angle_rot_error(R, target_rot))) if target_rot is not None else 0.0
        success = pos_err < self.success_pos_tol and rot_err < self.success_rot_tol
        return IKResult(q=q, pos_err=pos_err, rot_err=rot_err, success=success, iterations=it)

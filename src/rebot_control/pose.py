"""Pose helpers (pure numpy).

A pose is ``(p, R)``: position (3,) in m and rotation matrix (3, 3) whose columns are the frame's axes expressed in
the parent frame (the robot base frame unless stated otherwise). Rotation increments are rotation vectors
(axis * angle, rad).

:func:`pose_delta` / :func:`apply_delta` express a motion in the frame of its start pose, which does not depend on
the world frame (useful for learning from data recorded in other frames, e.g. a handheld tool tracked by a phone).
"""

from __future__ import annotations

import numpy as np

WORLD_DOWN = np.array([0.0, 0.0, -1.0])


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rotvec_to_matrix(r: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle) -> rotation matrix (Rodrigues)."""
    r = np.asarray(r, dtype=float)
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3) + skew(r)
    k = skew(r / theta)
    return np.eye(3) + np.sin(theta) * k + (1.0 - np.cos(theta)) * (k @ k)


def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> rotation vector (axis * angle), angle in [0, pi]."""
    R = np.asarray(R, dtype=float)
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if theta < 1e-6:
        return 0.5 * w
    if np.pi - theta < 1e-4:  # near 180 deg: use the diagonal for the axis
        axis = np.sqrt(np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None))
        axis *= np.sign(w + (w == 0))
        return theta * axis / np.linalg.norm(axis)
    return theta / (2.0 * np.sin(theta)) * w


def rotation_error(R: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """Rotation vector (world frame) that turns ``R`` into ``R_target``."""
    return matrix_to_rotvec(np.asarray(R_target) @ np.asarray(R).T)


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """URDF fixed-axis roll / pitch / yaw (rad) -> rotation matrix ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_to_rpy(R: np.ndarray) -> np.ndarray:
    """Inverse of :func:`rpy_to_matrix` (pitch in [-pi/2, pi/2])."""
    R = np.asarray(R, dtype=float)
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) < 1e-9:  # gimbal lock: put everything in yaw
        return np.array([0.0, pitch, np.arctan2(-R[0, 1], R[1, 1])])
    return np.array([np.arctan2(R[2, 1], R[2, 2]), pitch, np.arctan2(R[1, 0], R[0, 0])])


def pose_delta(p0: np.ndarray, R0: np.ndarray, p1: np.ndarray, R1: np.ndarray) -> np.ndarray:
    """Motion from pose 0 to pose 1 expressed in the frame of pose 0: [dp (3), drot (3)]."""
    return np.concatenate([R0.T @ (np.asarray(p1) - np.asarray(p0)), matrix_to_rotvec(R0.T @ R1)])


def apply_delta(p0: np.ndarray, R0: np.ndarray, delta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Inverse of :func:`pose_delta`: pose 0 moved by ``delta`` (expressed in the frame of pose 0)."""
    delta = np.asarray(delta, dtype=float)
    return np.asarray(p0) + R0 @ delta[:3], R0 @ rotvec_to_matrix(delta[3:])


def tilt_angle(R: np.ndarray, axis: int = 2) -> float:
    """Angle (rad) between the tool axis ``axis`` (default z, the approach axis) and straight down."""
    return float(np.arccos(np.clip(np.asarray(R)[:, axis] @ WORLD_DOWN, -1.0, 1.0)))


def yaw_of(R: np.ndarray) -> float:
    """Heading (rad) of the tool x axis projected on the horizontal plane."""
    x = np.asarray(R)[:, 0]
    return float(np.arctan2(x[1], x[0]))


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def limit_tilt(R: np.ndarray, max_tilt: float, axis: int = 2) -> np.ndarray:
    """Rotate ``R`` about a horizontal axis so that its tool axis is at most ``max_tilt`` from straight down."""
    tilt = tilt_angle(R, axis)
    if tilt <= max_tilt:
        return R
    a = R[:, axis]
    rot_axis = np.cross(a, WORLD_DOWN)
    n = np.linalg.norm(rot_axis)
    if n < 1e-9:  # pointing straight up: any horizontal axis works
        other = R[:, (axis + 1) % 3]
        rot_axis = other - (other @ WORLD_DOWN) * WORLD_DOWN
        n = np.linalg.norm(rot_axis)
    return rotvec_to_matrix(rot_axis / n * (tilt - max_tilt)) @ R


def to_rot6d(R: np.ndarray) -> np.ndarray:
    """Continuous 6-D rotation representation: the first two columns of ``R`` (Zhou et al., CVPR 2019)."""
    R = np.asarray(R)
    return np.concatenate([R[:, 0], R[:, 1]])


def from_rot6d(r: np.ndarray) -> np.ndarray:
    """Inverse of :func:`to_rot6d` (Gram-Schmidt, tolerant to non-orthonormal inputs)."""
    r = np.asarray(r, dtype=float)
    x = r[:3] / max(np.linalg.norm(r[:3]), 1e-9)
    y = r[3:] - (x @ r[3:]) * x
    y /= max(np.linalg.norm(y), 1e-9)
    return np.column_stack([x, y, np.cross(x, y)])

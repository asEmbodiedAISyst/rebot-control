"""Cartesian streaming control of the TCP.

Every control cycle (:meth:`CartesianController.step`):

1. **limits** on the desired pose: workspace box (base frame), optional tilt limit (angle between the tool axis
   and straight down) and yaw limit (heading of the tool x axis), then linear / angular speed limits relative to
   the previous *commanded* pose (not the measured one);
2. **lag slow-down**: when the arm lags its joint target by more than ``safety.slow_lag``, the speed limits are
   scaled down linearly (frozen at twice that lag) so that it catches up without stop-and-go;
3. **IK** (:class:`rebot_control.ik.DiffIK`) warm-started from the previous joint command; a failed solution
   (out of reach, joint limit) is refused and the previous pose is kept (the arm holds);
4. the joint target goes to :meth:`rebot_control.arm.Arm.command_joint` (jump guard, rate limit, filter, servo).

The same limits are meant to be applied to recorded demonstrations, so that data and execution agree.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import TYPE_CHECKING

import numpy as np

from .ik import DiffIK
from .pose import limit_tilt, matrix_to_rotvec, rot_z, rotvec_to_matrix, yaw_of

if TYPE_CHECKING:
    from .arm import Arm


@dataclass
class CartesianLimits:
    """Limits on the commanded TCP pose (base frame, m / rad; tilt and yaw in degrees, None = not limited)."""

    x: tuple[float, float] = (0.10, 0.50)
    y: tuple[float, float] = (-0.30, 0.30)
    z: tuple[float, float] = (0.02, 0.40)
    max_speed: float = 0.20  # m/s
    max_ang_speed: float = 1.5  # rad/s
    max_tilt_deg: float | None = None
    max_yaw_deg: float | None = None
    tilt_axis: int = 2  # tool axis that should point down (z = approach axis)

    @classmethod
    def from_dict(cls, d: dict | None) -> CartesianLimits:
        d = dict(d or {})
        names = {f.name for f in fields(cls)}
        unknown = set(d) - names
        if unknown:
            raise ValueError(f"unknown cartesian limit keys: {sorted(unknown)}")
        for k in ("x", "y", "z"):
            if k in d:
                d[k] = (float(d[k][0]), float(d[k][1]))
        return cls(**d)

    def clip_position(self, p: np.ndarray) -> np.ndarray:
        return np.array([np.clip(p[0], *self.x), np.clip(p[1], *self.y), np.clip(p[2], *self.z)])

    def clip_orientation(self, R: np.ndarray) -> np.ndarray:
        if self.max_tilt_deg is not None:
            R = limit_tilt(R, np.radians(self.max_tilt_deg), self.tilt_axis)
        if self.max_yaw_deg is not None:
            yaw, m = yaw_of(R), np.radians(self.max_yaw_deg)
            if abs(yaw) > m:
                R = rot_z(np.clip(yaw, -m, m) - yaw) @ R
        return R

    def contains(self, p: np.ndarray) -> bool:
        return bool(np.allclose(self.clip_position(p), p))


@dataclass
class CartesianStatus:
    """What one control cycle did."""

    pos: np.ndarray  # commanded TCP position after limits (m)
    rot: np.ndarray  # commanded TCP rotation after limits
    q: np.ndarray  # joint target sent to the arm (after the joint filter)
    ik_ok: bool  # False: the step was refused (unreachable), the previous pose is held
    clipped: bool  # the desired pose was changed by the workspace / tilt / yaw / speed limits
    lag_deg: float  # largest joint lag behind the target before this step
    speed_factor: float  # 1 = full speed, < 1 = slowed down because the arm lags
    info: dict = field(default_factory=dict)


class CartesianController:
    """Streams TCP pose targets to an :class:`~rebot_control.arm.Arm` at the arm's control rate.

    Typical use::

        cart = arm.cartesian()           # starts from the measured TCP pose
        for t in ...:
            st = cart.step(p_des, R_des)  # R_des=None keeps the current orientation
            arm.sleep()
    """

    def __init__(self, arm: Arm, limits: CartesianLimits | None = None, posture: np.ndarray | None = None,
                 ik_iters: int = 30) -> None:
        self.arm = arm
        self.limits = limits or CartesianLimits.from_dict(arm.tool.cartesian)
        self.dt = 1.0 / arm.cfg.control_hz
        if posture is None and arm.tool.home is not None:
            posture = arm.tool.home
        self.ik = DiffIK(arm.model, posture=posture)
        self.ik_iters = ik_iters
        self.pos = np.zeros(3)
        self.rot = np.eye(3)
        self.q_cmd = np.zeros(arm.n)
        self.reset()

    def reset(self) -> None:
        """Restart from the measured pose (call after any joint-space move or mode change)."""
        q = self.arm.joint_target()
        if q is None:
            q = self.arm.q()
        self.q_cmd = q.copy()
        self.pos, self.rot = self.ik.fk(q)

    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Current commanded TCP pose."""
        return self.pos.copy(), self.rot.copy()

    def limit(self, p_des: np.ndarray, R_des: np.ndarray | None, speed_factor: float = 1.0) -> tuple[np.ndarray, np.ndarray, bool]:
        """Apply the workspace / orientation / speed limits to a desired pose relative to the commanded pose."""
        lim = self.limits
        p_des = np.asarray(p_des, dtype=float)
        p = lim.clip_position(p_des)
        clipped = not np.allclose(p, p_des, atol=1e-9)
        step = p - self.pos
        n = np.linalg.norm(step)
        vmax = lim.max_speed * speed_factor * self.dt
        if n > vmax:
            step *= vmax / max(n, 1e-12)
            clipped = True
        p = self.pos + step
        R = self.rot
        if R_des is not None:
            R_des = np.asarray(R_des, dtype=float)
            R_lim = lim.clip_orientation(R_des)
            clipped |= not np.allclose(R_lim, R_des, atol=1e-9)
            dr = matrix_to_rotvec(R_lim @ self.rot.T)  # increment in the base frame
            a = np.linalg.norm(dr)
            amax = lim.max_ang_speed * speed_factor * self.dt
            if a > amax:
                dr *= amax / a
                clipped = True
            R = rotvec_to_matrix(dr) @ self.rot
        return p, R, clipped

    def step(self, p_des: np.ndarray, R_des: np.ndarray | None = None) -> CartesianStatus:
        """One control cycle towards ``(p_des, R_des)``. Call :meth:`Arm.sleep` between steps."""
        arm = self.arm
        arm.check()
        lag = arm.lag()
        slow_lag = arm.cfg.safety.slow_lag
        speed = float(np.clip((2.0 * slow_lag - lag) / slow_lag, 0.0, 1.0)) if lag > slow_lag else 1.0
        arm.check_stall()
        ik_ok, clipped = True, False
        if speed > 0.0:
            p, R, clipped = self.limit(p_des, R_des, speed)
            res = self.ik.solve(p, R if R_des is not None else self.rot, self.q_cmd, max_iters=self.ik_iters)
            if res.success:
                self.pos, self.rot, self.q_cmd = p, R, res.q
            else:
                ik_ok = False
        q = arm.command_joint(self.q_cmd)
        return CartesianStatus(pos=self.pos.copy(), rot=self.rot.copy(), q=q, ik_ok=ik_ok, clipped=clipped,
                               lag_deg=float(np.degrees(lag)), speed_factor=speed)

    def hold(self) -> CartesianStatus:
        """Keep the commanded pose (e.g. while an operator is disengaged)."""
        return self.step(self.pos, self.rot)

    def reachable(self, p: np.ndarray, R: np.ndarray | None = None) -> bool:
        """IK check only (no motion): can the TCP reach ``(p, R)`` from the current command?"""
        return self.ik.solve(p, R, self.q_cmd, max_iters=200).success

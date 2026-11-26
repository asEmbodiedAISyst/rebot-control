"""Joint-target conditioning between the caller (IK, joint commands) and the servo thread.

Per control cycle, in this order:

1. clip to the soft joint limits;
2. refuse a target more than ``max_jump`` from the previous target (:class:`CommandRejected`): a jump that large in
   one cycle is a bug or a bad IK solution, never a valid command;
3. rate limit: move the target at most ``max_joint_speed * dt`` from the previous *target* (not from the measured
   position, which would cap the torque instead of the speed);
4. first-order low-pass filter with time constant ``filter_tau_s`` (takes input jitter and IK steps out of the motion).
"""

from __future__ import annotations

import math

import numpy as np

from .errors import CommandRejected


class JointTargetFilter:
    def __init__(self, lower: np.ndarray, upper: np.ndarray, dt: float, max_jump: float, max_speed: float,
                 filter_tau: float, joint_names: list[str] | None = None) -> None:
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.dt = dt
        self.max_jump = max_jump
        self.max_step = max_speed * dt
        self.keep = math.exp(-dt / filter_tau) if filter_tau > 0 else 0.0
        self.names = joint_names or [f"joint{i + 1}" for i in range(len(self.lower))]
        self.q_prev: np.ndarray | None = None  # last rate-limited target (the "commanded" pose for lag checks)
        self.q_filt: np.ndarray | None = None  # last filtered target (sent to the servo)

    def reset(self, q: np.ndarray | None) -> None:
        """Start from ``q`` (the measured pose), or forget the history with ``None``."""
        self.q_prev = None if q is None else np.asarray(q, dtype=float).copy()
        self.q_filt = None if q is None else np.asarray(q, dtype=float).copy()

    def __call__(self, q_des: np.ndarray, q_meas: np.ndarray) -> np.ndarray:
        q = np.clip(np.asarray(q_des, dtype=float), self.lower, self.upper)
        if self.q_prev is None:
            self.reset(q_meas)
        err = np.abs(q - self.q_prev)
        j = int(err.argmax())
        if err[j] > self.max_jump:
            raise CommandRejected(
                f"{self.names[j]} target jumps from {math.degrees(self.q_prev[j]):.1f} to {math.degrees(q[j]):.1f} deg "
                f"({math.degrees(err[j]):.1f} > {math.degrees(self.max_jump):.0f} deg); refused (IK or input fault)")
        q = self.q_prev + np.clip(q - self.q_prev, -self.max_step, self.max_step)
        self.q_prev = q
        self.q_filt = self.keep * self.q_filt + (1.0 - self.keep) * q
        return self.q_filt.copy()

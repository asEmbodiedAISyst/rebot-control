"""Backend interface: the motors (real or simulated), in joint units.

A backend only moves MIT frames and feedback; all control logic (interpolation, feedforward, safety) lives above it
in :class:`rebot_control.servo.ServoLoop`, so the real arm and the simulation run the same code.

Units: rad, rad/s, N*m, N*m/rad, N*m*s/rad, deg C. Sign conventions (motor direction / offset) are handled inside
the backend: everything crossing this interface is in URDF joint coordinates.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


@dataclass
class JointState:
    t: float  # backend time (s): time.perf_counter() on the real arm, simulation time in the simulator
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray  # torque reported by the motors (RobStride: estimated from the phase current)
    temp: np.ndarray  # driver MOS temperature (deg C)
    valid: np.ndarray  # per joint: feedback received at least once
    enabled: np.ndarray = field(default=None)  # per joint: the motor reports that it is enabled (torque can flow)

    def __post_init__(self) -> None:
        if self.enabled is None:
            self.enabled = np.zeros(len(self.q), dtype=bool)

    def copy(self) -> JointState:
        return JointState(self.t, self.q.copy(), self.dq.copy(), self.tau.copy(), self.temp.copy(), self.valid.copy(),
                          self.enabled.copy())


class Backend(ABC):
    is_sim: bool = False
    n: int

    @abstractmethod
    def connect(self) -> None:
        """Open the bus / build the simulation. Sends nothing that changes what the motors are doing: motors left
        enabled by an earlier program keep executing their last frame."""

    @abstractmethod
    def close(self, disable: bool = True) -> None:
        """Release the bus. ``disable=True``: torque off first. ``disable=False``: leave the motors exactly as they
        are, so enabled motors keep executing the last frame they received (see ``Arm.detach``)."""

    @abstractmethod
    def read_positions(self) -> np.ndarray:
        """Joint angles read without sending any command frame, so a motor that is holding keeps holding.

        This is how a new program looks at an arm before it knows whether the motors are enabled."""

    @abstractmethod
    def enable(self) -> None:
        """Switch the motors that are not enabled yet to MIT mode and enable them. Motors that are already enabled
        are not touched. Newly enabled motors produce no torque until they receive a frame with gains."""

    @abstractmethod
    def disable(self) -> None:
        """Torque off (motors coast; the arm falls unless supported)."""

    @abstractmethod
    def send_mit(self, q: np.ndarray, dq: np.ndarray, kp: np.ndarray, kd: np.ndarray, tau: np.ndarray) -> None:
        """One MIT frame per motor: ``tau_out = kp (q - q_meas) + kd (dq - dq_meas) + tau``."""

    def send_passive(self) -> None:
        """Zero-gain frames: no output torque, but every motor replies with its state (RobStride feedback trick)."""
        z = np.zeros(self.n)
        self.send_mit(z, z, z, z, z)

    @abstractmethod
    def read(self) -> JointState:
        """Latest feedback (processes pending replies first)."""

    @abstractmethod
    def now(self) -> float:
        """Backend clock (s)."""

    def set_zero(self, joints: list[int]) -> None:
        """Store the current position as zero in the given motors (real arm only)."""
        raise NotImplementedError

    def advance(self, dt: float) -> None:
        """Simulation only: advance physics by ``dt`` seconds."""
        raise NotImplementedError

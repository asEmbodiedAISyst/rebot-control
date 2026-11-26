"""Real arm: RobStride motors over SocketCAN through ``motorbridge`` (``pip install rebot-control[real]``).

Notes on motorbridge and the RobStride motors (verified on the B601-RS):

* motorbridge updates a RobStride motor's state only from the reply to a command; ``request_feedback()`` is a no-op
  for RobStride and the motors' periodic active-report frames are not parsed. So feedback always comes from replies:
  the servo thread sends a frame to every motor every tick (zero-gain frames when torque is off), and reads the
  replies of the previous tick (``poll_feedback_once`` + ``get_state``).
* Disabled motors still answer MIT frames (within ~3 ms) and stay disabled.
* The SocketCAN interface comes up DOWN after a reboot or a re-plug unless a systemd unit brings it up;
  :func:`check_can_up` fails early with the command to fix it.

Holding through a dead program (``docs/user/safety.md``): with the drivers' CAN timeout at 0 (the value found on
the B601-RS), an enabled motor keeps executing the last MIT frame it received for as long as it has power. The
library relies on this to keep the arm where it is when a program ends abnormally, and never sends a zero-gain
frame to a motor that may be enabled:

* :meth:`RobStrideBackend.read_positions` reads the angles through parameter reads, which do not replace the MIT
  frame a motor is executing;
* the feedback frames carry the driver's mode; :meth:`RobStrideBackend.read` reports it as ``JointState.enabled``;
* :meth:`RobStrideBackend.close` with ``disable=False`` releases the bus without sending anything.

One program at a time: two programs streaming frames to the same motors fight over them (each replaces the other's
frame 150 times a second). :meth:`RobStrideBackend.connect` therefore takes an exclusive lock per CAN channel
(``flock`` on a file in the temp directory). The operating system drops it when the process ends, however it ends.
"""

from __future__ import annotations

import fcntl
import logging
import os
import sys
import tempfile
import threading
import time

import numpy as np

from ..config import ArmConfig
from ..errors import BackendError
from .base import Backend, JointState

log = logging.getLogger(__name__)

# RobStride parameter indices (type 17 / 18 frames), as used with motorbridge. Read on the B601-RS on 2026-11-16.
PARAM_MECH_POS = 0x7019      # f32, rad: mechanical angle of the output shaft
PARAM_CAN_TIMEOUT = 0x7028   # u32: 0 = no timeout (the motor keeps its last frame when frames stop)


def check_can_up(port: str = "can0", sysfs: str = "/sys/class/net") -> None:
    """Raise with a clear hint when the SocketCAN interface is missing or DOWN. Non-Linux hosts are not checked."""
    base = os.path.join(sysfs, port)
    if not os.path.isdir(sysfs) or "/" in port:
        return
    if not os.path.isdir(base):
        raise BackendError(f"CAN interface {port} not found: is the USB-CAN adapter plugged in? (ip -br link)")
    try:
        with open(os.path.join(base, "flags")) as f:
            flags = int(f.read().strip(), 16)
    except (OSError, ValueError):
        return
    if not flags & 0x1:  # IFF_UP
        raise BackendError(
            f"CAN interface {port} is DOWN. Bring it up with:\n"
            f"    sudo ip link set {port} type can bitrate 1000000 restart-ms 100 && sudo ip link set {port} up")


class RobStrideBackend(Backend):
    is_sim = False

    def __init__(self, cfg: ArmConfig) -> None:
        self.cfg = cfg
        self.n = cfg.n
        self.channel = cfg.bus.channel
        self.direction = np.array([m.direction for m in cfg.motors])
        self.offset = np.array([m.offset for m in cfg.motors])
        self.bus = None
        self.motors: list = []
        self._lock = threading.Lock()  # every bus access goes through it (servo thread + caller)
        self._state = JointState(0.0, np.zeros(self.n), np.zeros(self.n), np.zeros(self.n), np.zeros(self.n),
                                 np.zeros(self.n, dtype=bool), np.zeros(self.n, dtype=bool))

    def connect(self) -> None:
        try:
            from motorbridge import Controller
        except ImportError as e:  # pragma: no cover - optional dependency
            raise BackendError("motorbridge is not installed: pip install 'rebot-control[real]'") from e
        check_can_up(self.channel)
        self._take_bus_lock()
        self.bus = Controller(self.channel)
        self.motors = [self.bus.add_robstride_motor(m.can_id, m.feedback_id, m.model) for m in self.cfg.motors]
        log.info("connected %d RobStride motors on %s", len(self.motors), self.channel)
        self._check_can_timeout()

    def _take_bus_lock(self) -> None:
        """Refuse to connect while another program controls the motors on this channel."""
        path = os.path.join(tempfile.gettempdir(), f"rebot-control-{self.channel}.lock")
        f = open(path, "a+")  # noqa: SIM115 - kept open for as long as the bus is
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as e:
            f.seek(0)
            owner = f.read().strip() or "unknown"
            f.close()
            raise BackendError(f"another program is connected to the arm on {self.channel} ({owner}). Two programs "
                               "must not command the motors at the same time: wait for it or stop it first.") from e
        try:
            os.chmod(path, 0o666)  # any user on this machine may take the lock later
        except OSError:
            pass
        f.seek(0)
        f.truncate()
        f.write(f"pid {os.getpid()}: {' '.join(sys.argv)[:200]}\n")
        f.flush()
        self._lock_file = f

    def _release_bus_lock(self) -> None:
        f, self._lock_file = getattr(self, "_lock_file", None), None
        if f is not None:
            f.close()  # closing the file drops the flock

    def _check_can_timeout(self) -> None:
        """Holding through a dead program needs the drivers' CAN timeout to be off; say so if it is not."""
        try:
            with self._lock:
                timeouts = [int(m.robstride_get_param_u32(PARAM_CAN_TIMEOUT, 200)) for m in self.motors]
        except Exception:  # noqa: BLE001 - informational only
            log.warning("could not read the drivers' CAN timeout")
            return
        if any(timeouts):
            log.warning("CAN timeout is set on some motors (%s): they will NOT hold their pose when this program stops "
                        "sending frames", dict(zip([m.can_id for m in self.cfg.motors], timeouts)))

    def read_positions(self) -> np.ndarray:
        q = np.zeros(self.n)
        with self._lock:
            for i, motor in enumerate(self.motors):
                for attempt in range(3):
                    try:
                        raw = float(motor.robstride_get_param_f32(PARAM_MECH_POS, 300))
                        break
                    except Exception as e:  # noqa: BLE001 - motorbridge raises CallError on timeouts
                        if attempt == 2:
                            raise BackendError(f"no answer from the motor with CAN id {self.cfg.motors[i].can_id} "
                                               "(power, wiring, CAN ids?)") from e
                q[i] = (raw - self.offset[i]) * self.direction[i]
        return q

    def enable(self) -> None:
        from motorbridge import Mode

        with self._lock:
            # Only the motors that are not enabled: a motor that is holding must not be disabled (the mode switch
            # needs a disabled motor) or sent a zero-gain frame, or the joint drops.
            todo = [i for i in range(self.n) if not self._state.enabled[i]]
            for i in todo:
                motor = self.motors[i]
                motor.disable()
                for attempt in range(10):
                    try:
                        motor.ensure_mode(Mode.MIT)
                        break
                    except Exception as e:  # noqa: BLE001 - motorbridge raises CallError on timeouts
                        if attempt == 9:
                            raise BackendError(f"motor {self.cfg.motors[i].can_id}: could not switch to MIT mode") from e
                        time.sleep(0.01)
            for i in todo:
                self.motors[i].enable()
                # enabled with zero gains: no output until the servo sends gains
                self.motors[i].send_mit(0.0, 0.0, 0.0, 0.0, 0.0)

    def disable(self) -> None:
        with self._lock:
            if self.bus is not None:
                self.bus.disable_all()

    def close(self, disable: bool = True) -> None:
        with self._lock:
            if self.bus is None:
                return
            for motor in self.motors:
                try:
                    if disable:
                        motor.disable()
                        motor.clear_error()
                    motor.close()
                except Exception:  # noqa: BLE001 - best effort on shutdown
                    log.exception("closing motor failed")
            self.bus.close()
            self.bus = None
            self.motors = []
            self._release_bus_lock()

    def send_mit(self, q, dq, kp, kd, tau) -> None:
        q_raw = self.direction * np.asarray(q, float) + self.offset
        dq_raw = self.direction * np.asarray(dq, float)
        tau_raw = self.direction * np.asarray(tau, float)
        with self._lock:
            for i, motor in enumerate(self.motors):
                motor.send_mit(float(q_raw[i]), float(dq_raw[i]), float(kp[i]), float(kd[i]), float(tau_raw[i]))

    def read(self) -> JointState:
        with self._lock:
            try:
                self.bus.poll_feedback_once()
            except Exception:  # noqa: BLE001 - a failed poll keeps the previous state
                log.warning("CAN poll failed")
            st = self._state
            st.t = time.perf_counter()
            for i, motor in enumerate(self.motors):
                s = motor.get_state()
                if s is None:
                    continue
                st.q[i] = (s.pos - self.offset[i]) * self.direction[i]
                st.dq[i] = s.vel * self.direction[i]
                st.tau[i] = s.torq * self.direction[i]
                st.temp[i] = s.t_mos
                st.valid[i] = True
                # The driver's mode from the feedback frame: 0 while disabled (seen on the arm). Anything else counts
                # as enabled, which errs towards "may be holding" (never send such a motor a zero-gain frame).
                st.enabled[i] = s.status_code != 0
            return st.copy()

    def now(self) -> float:
        return time.perf_counter()

    def set_zero(self, joints: list[int]) -> None:
        """Store the current position as zero (same as Motorbridge Studio). Torque must be off; the arm must be in
        the zero pose. Overwrites the zero stored in the motors."""
        if self._state.enabled.any():
            raise BackendError("set_zero needs torque off on every motor: release the arm first")
        with self._lock:
            self.bus.disable_all()
            for j in joints:
                self.motors[j].set_zero_position()
                time.sleep(0.1)
        log.warning("zero position stored in motors %s", [self.cfg.motors[j].can_id for j in joints])

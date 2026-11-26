"""Servo loop: streams MIT frames to the motors at ``servo_hz`` (150 Hz by default).

Every tick (:meth:`ServoLoop.tick`):

1. read the latest feedback (replies to the previous tick's frames);
2. safety: motor temperature (warn / torque off);
3. compute the frame for the current mode:

   * ``PASSIVE``: zero-gain frames (no output torque; keeps feedback flowing);
   * ``HOLD``: position target interpolated linearly from the previous target to the newest one over one control
     period (evaluated half a servo period ahead, see :meth:`ServoLoop.tick`), with the matching velocity as
     feedforward (the motor's kd term then damps deviations only, it does not brake the motion), the target clamped to ``measured +- max_error`` (torque cap = kp * max_error), and the
     feedforward ``gravity * gravity_scale + friction(dq_target) * friction_scale`` (clipped);
   * ``GRAVITY``: kp = 0, small kd, gravity (+ friction) feedforward only: the arm floats and can be guided by hand.

4. gains and feedforward ramp in over ``ramp_s`` after a mode change from PASSIVE (no torque step at enable).

The loop never switches torque off on its own except for an over-temperature stop with
``safety.on_overtemp: disable``. When it has to stop (an error in the thread), it first holds the measured pose
(:meth:`ServoLoop.freeze`): the motors then keep executing that frame, see ``docs/user/safety.md``.

On the real arm a background thread calls ``tick`` (:meth:`start`); in simulation the caller steps it
(:meth:`rebot_control.arm.Arm.sleep`), so both run the same code.
"""

from __future__ import annotations

import enum
import logging
import threading
import time

import numpy as np

from .backends.base import Backend, JointState
from .config import ArmConfig
from .errors import SafetyStop
from .model import RobotModel

log = logging.getLogger(__name__)


FREEZE_KEEP = np.radians(3.0)        # freeze: a joint this close to its target keeps that target
ADOPT_MAX_OFFSET = np.radians(3.0)   # take-over: largest target offset derived from the reported torque


class ServoMode(enum.Enum):
    PASSIVE = "passive"
    HOLD = "hold"
    GRAVITY = "gravity"


class ServoLoop:
    def __init__(self, backend: Backend, model: RobotModel, cfg: ArmConfig) -> None:
        self.backend = backend
        self.model = model
        self.cfg = cfg
        self.n = cfg.n
        self.data = model.new_data()  # this thread's Pinocchio data
        self.period = 1.0 / cfg.servo_hz
        self.control_period = 1.0 / cfg.control_hz
        self.kp = cfg.gains.kp.copy()
        self.kd = cfg.gains.kd.copy()
        self.mode = ServoMode.PASSIVE
        self._lock = threading.RLock()
        self._from: np.ndarray | None = None
        self._to: np.ndarray | None = None
        self._t_set = 0.0
        self._ramp_t0 = 0.0
        self._last_sent: np.ndarray | None = None
        self.state: JointState | None = None
        self.last_cmd: dict[str, np.ndarray] = {}
        self.error: BaseException | None = None
        self._temp_warned = False
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._recording = False
        self._log: list[tuple] = []
        self.overruns = 0

    # ---------------------------------------------------------------------------------------------- commands
    def set_target(self, q: np.ndarray, t: float, step: bool = False) -> None:
        """New position target: interpolated from where the last frame was to ``q`` over one control period.

        ``step=True`` jumps to ``q`` at the next frame (step-response tests only; still capped by ``max_error``).
        """
        with self._lock:
            q = np.asarray(q, dtype=float).copy()
            start = self._last_sent if self._last_sent is not None else (self.state.q if self.state is not None else q)
            self._from = q.copy() if step else start.copy()
            self._to = q
            self._t_set = t

    def set_mode(self, mode: ServoMode, t: float) -> None:
        """Switch mode. Entering HOLD from another mode holds the measured pose (no jump towards an old target)."""
        with self._lock:
            if mode == self.mode:
                return
            if self.mode == ServoMode.PASSIVE:
                self._ramp_t0 = t  # gains and feedforward ramp in from zero
            if mode == ServoMode.HOLD:
                if self.state is None:
                    raise RuntimeError("no feedback yet: cannot hold")
                q = self.state.q.copy()
                self._from, self._to, self._t_set, self._last_sent = q.copy(), q, t, None
            else:
                self._from = self._to = self._last_sent = None
            self.mode = mode

    def hold_measured(self, t: float) -> None:
        """Replace the target by the measured pose (HOLD mode only; no gain ramp, no passive frame)."""
        with self._lock:
            q = self.state.q.copy()
            self._from, self._to, self._t_set, self._last_sent = q.copy(), q, t, None

    def adopt(self, q: np.ndarray, t: float) -> None:
        """Enter HOLD at ``q`` with full gains at once (no ramp): for motors that are already enabled and holding
        there, as found at connect, or to stop where the arm is from any mode."""
        with self._lock:
            q = np.asarray(q, dtype=float).copy()
            self._from, self._to, self._t_set, self._last_sent = q.copy(), q, t, None
            self._ramp_t0 = t - self.cfg.gains.ramp_s
            self.mode = ServoMode.HOLD

    def freeze(self, t: float) -> None:
        """Stop and hold from now on, whatever the mode was, with zero target velocity.

        A joint that is following its target (within ``FREEZE_KEEP`` of it) keeps the target it had: re-targeting to
        the measured angle would move the equilibrium by the joint's steady-state error every time. A joint far from
        its target (blocked, or no target at all in gravity mode) holds its measured angle."""
        if self.state is None:
            return
        with self._lock:
            q = self.state.q.copy()
            if self.mode == ServoMode.HOLD and self._last_sent is not None:
                near = np.abs(self._last_sent - q) < FREEZE_KEEP
                q[near] = self._last_sent[near]
            self.adopt(q, t)

    def adopt_running(self, st: JointState, t: float) -> None:
        """Take over motors that are already holding the arm at ``st.q`` (found enabled at connect).

        The target is placed so that this controller asks for the torque the motors report right now,
        ``kp (q* - q) + ff(q) = tau``: the equilibrium stays where it is although the gravity model here may differ
        from what the previous frame used. A plain "hold the measured angle" would shift the arm by its steady-state
        error at every take-over."""
        z = np.zeros(self.n)
        with self._lock:
            offset = (st.tau - self.feedforward(st.q, z)) / np.maximum(self.kp, 1e-6)
            self.adopt(st.q + np.clip(offset, -ADOPT_MAX_OFFSET, ADOPT_MAX_OFFSET), t)

    def send_hold(self, q: np.ndarray) -> None:
        """One full-gain frame that holds ``q`` (target velocity zero, gravity feedforward at ``q``).

        Sent once at connect, before it is known whether the motors are enabled: an enabled motor that sits at
        ``q`` keeps holding, a disabled motor ignores it. Both answer with their state."""
        z = np.zeros(self.n)
        q = np.asarray(q, dtype=float)
        self.backend.send_mit(q, z, self.kp, self.kd, self.feedforward(q, z))

    def set_gains(self, kp: np.ndarray | None = None, kd: np.ndarray | None = None) -> None:
        with self._lock:
            if kp is not None:
                self.kp = np.asarray(kp, dtype=float).copy()
            if kd is not None:
                self.kd = np.asarray(kd, dtype=float).copy()

    def target(self) -> np.ndarray | None:
        with self._lock:
            return None if self._to is None else self._to.copy()

    # -------------------------------------------------------------------------------------------------- tick
    def feedforward(self, q: np.ndarray, dq_des: np.ndarray) -> np.ndarray:
        ff = self.cfg.feedforward
        tau = np.zeros(self.n)
        if ff.gravity:
            tau += ff.gravity_scale * self.model.gravity(q, self.data)
        if ff.friction_scale:
            tau += ff.friction_scale * self.cfg.friction.torque(dq_des)
        if ff.ripple_scale and self.cfg.ripple is not None:
            tau += ff.ripple_scale * self.cfg.ripple.torque(q)  # at the measured angle: see RippleParams
        return np.clip(tau, -ff.tau_max, ff.tau_max)

    def tick(self, t: float) -> JointState:
        st = self.backend.read()
        self.state = st
        stop = self._check_temperature(st, t)  # may switch torque off or freeze; raised after this tick's frame
        z = np.zeros(self.n)
        with self._lock:
            mode = self.mode
            ramp = 1.0
            if self.cfg.gains.ramp_s > 0:
                ramp = float(np.clip((t - self._ramp_t0) / self.cfg.gains.ramp_s, 0.0, 1.0))
            if mode == ServoMode.PASSIVE:
                self.backend.send_passive()
                self.last_cmd = {"q": st.q, "dq": z, "kp": z, "kd": z, "tau": z}
            elif mode == ServoMode.HOLD:
                # Evaluate the interpolation half a servo period ahead: the motor holds each frame's position for one
                # servo period (zero-order hold), so this centres the held target on the desired trajectory instead
                # of lagging it by half a period. Keep the velocity feedforward a little past the end of the segment
                # so that a slightly late next target (real-time jitter) does not cause a one-tick velocity dropout.
                a_raw = (t + 0.5 * self.period - self._t_set) / self.control_period
                a = min(1.0, a_raw)
                q_cmd = self._from + a * (self._to - self._from)
                dq_cmd = (self._to - self._from) / self.control_period if a_raw < 1.25 else z
                lim = self.cfg.safety.max_error
                q_cmd = st.q + np.clip(q_cmd - st.q, -lim, lim)  # torque cap: kp * max_error
                tau = self.feedforward(st.q, dq_cmd) * ramp
                kp, kd = self.kp * ramp, self.kd * ramp
                self.backend.send_mit(q_cmd, dq_cmd, kp, kd, tau)
                self._last_sent = q_cmd
                self.last_cmd = {"q": q_cmd, "dq": dq_cmd, "kp": kp, "kd": kd, "tau": tau}
            else:  # GRAVITY
                tau = self.feedforward(st.q, st.dq) * ramp
                kd = self.cfg.gains.gravity_mode_kd * ramp
                self.backend.send_mit(st.q, z, z, kd, tau)
                self.last_cmd = {"q": st.q, "dq": z, "kp": z, "kd": kd, "tau": tau}
            if self._recording:
                c = self.last_cmd
                self._log.append((t, st.q.copy(), st.dq.copy(), st.tau.copy(), st.temp.copy(), c["q"].copy(),
                                  c["dq"].copy(), c["kp"].copy(), c["kd"].copy(), c["tau"].copy()))
        if stop is not None:
            raise stop
        return st

    def _check_temperature(self, st: JointState, t: float) -> SafetyStop | None:
        s = self.cfg.safety
        hot = st.valid & (st.temp > s.temp_stop_c)
        if hot.any():
            j = int(np.argmax(np.where(st.valid, st.temp, -np.inf)))
            msg = f"{self.cfg.joints[j]} driver at {st.temp[j]:.0f} C > {s.temp_stop_c:.0f} C"
            if s.on_overtemp == "hold":
                if self.mode != ServoMode.PASSIVE:
                    self.freeze(t)
                return SafetyStop(msg + ": holding the measured pose (safety.on_overtemp: hold)")
            self.backend.disable()
            self.mode = ServoMode.PASSIVE
            return SafetyStop(msg + ": torque off")
        if not self._temp_warned and (st.valid & (st.temp > s.temp_warn_c)).any():
            self._temp_warned = True
            log.warning("motor temperature high: %s C", np.round(st.temp, 0))
        return None

    # ------------------------------------------------------------------------------------------- recording
    def start_recording(self) -> None:
        with self._lock:
            self._log = []
            self._recording = True

    def stop_recording(self) -> dict[str, np.ndarray]:
        """Samples since :meth:`start_recording`: t, q, dq, tau, temp, q_cmd, dq_cmd, kp, kd, tau_ff (one row per tick)."""
        with self._lock:
            self._recording = False
            rows, self._log = self._log, []
        keys = ("t", "q", "dq", "tau", "temp", "q_cmd", "dq_cmd", "kp", "kd", "tau_ff")
        if not rows:
            return {k: np.zeros((0,) if k == "t" else (0, self.n)) for k in keys}
        return {k: np.array([r[i] for r in rows]) for i, k in enumerate(keys)}

    # ------------------------------------------------------------------------------------------ thread (real)
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self.error = None
        self._thread = threading.Thread(target=self._run, name="rebot-servo", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        next_t = time.perf_counter()
        try:
            while not self._stop.is_set():
                now = time.perf_counter()
                self.tick(now)
                next_t += self.period
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    self.overruns += 1
                    next_t = time.perf_counter()
        except BaseException as e:  # noqa: BLE001 - surfaced to the caller by Arm.check()
            self.error = e
            log.exception("servo thread stopped")
            if not isinstance(e, SafetyStop) and self.mode != ServoMode.PASSIVE:
                # Never torque off here: the arm would fall. Send one last frame that holds the measured pose; the
                # motors keep executing it. If the bus is the problem, they keep the frame they already have.
                try:
                    self.freeze(time.perf_counter())
                    self.tick(time.perf_counter())
                except BaseException:  # noqa: BLE001
                    log.exception("could not send a hold frame after the servo error; the motors keep their last frame")

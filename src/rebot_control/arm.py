"""The :class:`Arm` facade: one object for the real arm and the simulator.

Layers (bottom up, see ``docs/developer/architecture.md``)::

    Arm (this file)        modes, joint / Cartesian commands, slow moves, stall check, recording
    JointTargetFilter      soft limits, jump guard, rate limit, low-pass           (control rate, 30 Hz)
    ServoLoop              interpolation, torque cap, gravity / friction feedforward (servo rate, 150 Hz)
    Backend                RobStride over CAN (motorbridge) | MuJoCo simulation

Life cycle::

    arm = Arm.from_config("b601_rs", tool="pusher_ip17", backend="real")
    with arm:                   # connect(): look at the arm without changing what the motors do
        arm.enable()            # torque on, holds the measured pose (gains ramp in over ramp_s)
        arm.move_home()         # unfold in stages to the tool's home pose
        ...
    # leaving the block normally: fold back to the zero pose in stages, then torque off

Torque is switched off only on purpose: at the normal end of the block (after folding), by :meth:`Arm.release` /
:meth:`Arm.disable_torque`, or by an over-temperature stop (``safety.on_overtemp``). When the block is left by an
exception or Ctrl-C the arm **holds where it is and stays enabled** (:meth:`Arm.detach`), and the motors keep
executing that last frame after the program is gone. The next program finds them enabled in :meth:`Arm.connect`
and takes over the hold without a jump (``arm.taken_over``); the operator ends it with ``rebot-control release``.
See ``docs/user/safety.md``.

Timing: the caller runs its loop at ``control_hz`` and calls :meth:`Arm.sleep` once per cycle. On the real arm
the servo thread streams frames in the background and ``sleep`` just keeps the rate; in simulation ``sleep``
advances the physics (and the servo loop) by one control period, so scripts run unchanged in both.
"""

from __future__ import annotations

import contextlib
import logging
import math
import time
from pathlib import Path

import numpy as np

from .backends.base import Backend, JointState
from .config import ArmConfig, Calibration, ToolConfig
from .errors import BackendError, CommandRejected, SafetyStop
from .joint_filter import JointTargetFilter
from .model import RobotModel
from .servo import ServoLoop, ServoMode

log = logging.getLogger(__name__)


def make_backend(kind: str, cfg: ArmConfig, tool: ToolConfig, calibration: Calibration | None, **kwargs) -> Backend:
    if kind == "sim":
        from .backends.sim import SimBackend

        return SimBackend(cfg, tool, calibration, **kwargs)
    if kind == "real":
        if cfg.bus.type != "robstride":
            raise BackendError(f"unsupported bus type {cfg.bus.type!r}")
        from .backends.robstride import RobStrideBackend

        return RobStrideBackend(cfg)
    raise ValueError(f"backend must be 'real' or 'sim', got {kind!r}")


class Arm:
    def __init__(self, cfg: ArmConfig, tool: ToolConfig | None = None, calibration: Calibration | None = None,
                 backend: str | Backend = "real", **backend_kwargs) -> None:
        self.cfg = cfg.copy()
        self.calibration = calibration
        if calibration is not None:
            calibration.apply_to(self.cfg)
        self.tool = (tool or ToolConfig(mount_frame=self.cfg.flange_frame)).copy()
        self.model = RobotModel(self.cfg, self.tool, calibration)
        self.n = self.cfg.n
        self.backend = backend if isinstance(backend, Backend) else make_backend(backend, self.cfg, self.tool, calibration,
                                                                                  **backend_kwargs)
        self.is_sim = self.backend.is_sim
        if self.cfg.ripple is not None:  # a wrong ripple model must not be able to cancel the position gain
            self.cfg.ripple.check(self.cfg.gains.kp, self.cfg.feedforward.ripple_scale)
        self.servo = ServoLoop(self.backend, self.model, self.cfg)
        self.dt = 1.0 / self.cfg.control_hz
        s = self.cfg.safety
        self.filter = JointTargetFilter(self.model.lower, self.model.upper, self.dt, s.max_jump, s.max_joint_speed,
                                        s.filter_tau_s, self.cfg.joints)
        self.connected = False
        self.enabled = False
        self.taken_over = False  # the motors were already enabled (holding) when this Arm connected
        self._commanded = False  # this Arm has switched torque or commanded the arm since connecting
        self._next_t: float | None = None
        self._lag_since: float | None = None
        self._lag_q0: np.ndarray | None = None

    @classmethod
    def from_config(cls, arm: str | Path = "b601_rs", tool: str | Path | ToolConfig | None = "none",
                    calibration: str | Path | Calibration | None = None, backend: str = "real",
                    overrides: dict | None = None, **backend_kwargs) -> Arm:
        """Build from config names or YAML paths (``arm``: ``configs/arms``, ``tool``: ``configs/tools``)."""
        cfg = ArmConfig.load(arm, overrides)
        if tool is None:
            tool_cfg = ToolConfig(mount_frame=cfg.flange_frame)
        else:
            tool_cfg = tool if isinstance(tool, ToolConfig) else ToolConfig.load(tool)
        cal = calibration
        if cal is not None and not isinstance(cal, Calibration):
            cal = Calibration.load(cal, cfg.n)
        return cls(cfg, tool_cfg, cal, backend, **backend_kwargs)

    # ------------------------------------------------------------------------------------------- life cycle
    def connect(self) -> Arm:
        """Open the bus (or build the simulation) and start streaming feedback, without changing what the motors do.

        The motors may still be enabled and holding the arm from a program that ended abnormally (see
        :meth:`detach`). So nothing here sends a zero-gain frame before the state of the motors is known:

        1. read the joint angles without any command frame (:meth:`Backend.read_positions`);
        2. send one full-gain frame that holds exactly those angles: an enabled motor keeps holding, a disabled
           one ignores it, and both answer with their state;
        3. motors enabled: take over the hold (``taken_over``, ``enabled``); all disabled: torque stays off.
        """
        if self.connected:
            return self
        self.backend.connect()
        self.connected = True
        try:
            q = self.backend.read_positions()
            st = None
            for _ in range(5):
                self.servo.send_hold(q)
                if not self.is_sim:
                    time.sleep(max(self.cfg.bus.feedback_wait_s, 0.005))
                st = self.backend.read()
                if st.valid.all():
                    break
            if st is None or not st.valid.all():
                missing = [self.cfg.motors[i].can_id for i in range(self.n) if st is None or not st.valid[i]]
                raise BackendError(f"no feedback from motors with CAN id {missing} (power, wiring, CAN ids?)")
        except BaseException:
            self.backend.close(disable=False)  # the motors are in an unknown state: leave them as they are
            self.connected = False
            raise
        self.servo.state = st
        self.taken_over = bool(st.enabled.any())
        self.enabled = self.taken_over
        self._commanded = False
        if self.taken_over:
            self.servo.adopt_running(st, self.now())
            self.filter.reset(self.servo.target())
            self._lag_since = None
            off = [self.cfg.joints[i] for i in range(self.n) if not st.enabled[i]]
            log.warning("the motors are enabled: taking over, holding the arm where it is%s",
                        f" (NOT enabled: {off}; call enable() to switch them on)" if off else "")
        if self.is_sim:
            self._sim_run(2 * self.servo.period)
        else:
            self.servo.start()
        log.info("connected (%s): q = %s deg, torque %s", "sim" if self.is_sim else self.cfg.bus.channel,
                 np.round(np.degrees(self.q()), 1), "ON (taken over)" if self.taken_over else "off")
        return self

    def enable(self) -> None:
        """Torque on and hold the measured pose; gains and feedforward ramp in over ``gains.ramp_s``."""
        self._require_connected()
        self._commanded = True
        all_on = bool(self.servo.state.enabled.all()) if self.is_enabled_known() else self.enabled
        if self.enabled and all_on and self.servo.mode == ServoMode.HOLD:
            return
        if self.enabled and not all_on:
            # Some motors hold, some are off (taken over after a fault): hold the measured pose first, so the
            # motors that come on now start at their own angle and nothing jumps.
            self.servo.freeze(self.now())
            self.backend.enable()
        elif not self.enabled:
            self.backend.enable()
            self.enabled = True
        self.servo.set_mode(ServoMode.HOLD, self.now())
        self.filter.reset(self.q())
        self._lag_since = None
        self.wait(self.cfg.gains.ramp_s)

    def is_enabled_known(self) -> bool:
        return self.servo.state is not None and bool(self.servo.state.valid.all())

    def hold(self) -> None:
        """Hold the measured pose with the PD gains (from gravity mode or after a stall in 'hold' mode)."""
        self.enable()

    def gravity_mode(self) -> None:
        """Gravity (+ friction) compensation only, kp = 0: the arm floats and can be moved by hand."""
        self._require_connected()
        self._commanded = True
        if not self.enabled:
            self.backend.enable()
            self.enabled = True
        self.servo.set_mode(ServoMode.GRAVITY, self.now())
        self.filter.reset(None)

    def disable_torque(self) -> None:
        """Torque off immediately. The arm falls unless it is supported (or folded at zero)."""
        self._commanded = True
        # disable before the servo goes passive: zero-gain frames must never reach a motor that is still enabled
        if self.connected:
            self.backend.disable()
        self.servo.set_mode(ServoMode.PASSIVE, self.now() if self.connected else 0.0)
        self.enabled = False
        self.filter.reset(None)

    def release(self, fold: bool = False) -> None:
        """The operator's explicit torque-off. ``fold=True`` folds to the zero pose first; otherwise the arm goes
        limp where it is and must be supported by hand."""
        self._require_connected()
        if fold and self.enabled:
            self.hold()
            self.fold()
        self.disable_torque()

    def freeze(self) -> None:
        """Hold the measured pose with the PD gains, from any mode, and stay enabled."""
        self._require_connected()
        if self.enabled:
            self.servo.freeze(self.now())
            self.filter.reset(self.servo.target())

    def detach(self) -> None:
        """Stop commanding and release the bus **without** switching torque off.

        If torque is on, the arm first holds the measured pose (zero target velocity); the motors then keep
        executing that frame for as long as they have power (the drivers' CAN timeout must be 0). Nobody watches
        the arm after this: no stall check, no temperature guard. The next :meth:`connect` takes the hold over."""
        if not self.connected:
            return
        try:
            if self.enabled:
                error = self.servo.error
                if not self.is_sim:
                    self.servo.stop()
                if error is None or isinstance(error, SafetyStop):
                    self.servo.freeze(self.now())
                    try:
                        self.servo.tick(self.now())  # the last frame the motors get: hold here
                    except SafetyStop:
                        pass  # that frame was sent before the stop was raised
                log.warning("detached with torque ON: the arm holds q = %s deg until another program takes over "
                            "or the operator releases it", np.round(np.degrees(self.servo.state.q), 1))
            elif not self.is_sim:
                self.servo.stop()
        except Exception:  # noqa: BLE001 - whatever happened, do not switch torque off
            log.exception("could not send the final hold frame; the motors keep their last frame")
        finally:
            self.backend.close(disable=False)
            self.connected = False

    def close(self, fold: bool = True) -> None:
        """Normal end: fold back to the zero pose (if torque is on and ``fold``), torque off, release the bus.

        An arm that was found enabled and never commanded by this program is left as found (:meth:`detach`)."""
        if not self.connected:
            return
        if self.enabled and not self._commanded:
            self.detach()
            return
        try:
            if fold and self.enabled and self.servo.error is None:
                if self.servo.mode != ServoMode.HOLD:
                    self.hold()
                self.fold()
        except BaseException:
            # folding failed half way: do not drop the arm, hold it where it is
            log.exception("folding to zero failed; holding the arm where it is")
            self.detach()
            raise
        self.disable_torque()
        if not self.is_sim:
            self.servo.stop()
        self.backend.close()
        self.connected = False

    def __enter__(self) -> Arm:
        return self.connect()

    def __exit__(self, exc_type, exc, tb) -> None:
        if exc_type is None:
            self.close(fold=True)
        else:
            # An error or Ctrl-C: never torque off (the arm would fall) and never move. Hold here, stay enabled.
            self.detach()

    # ------------------------------------------------------------------------------------------------ state
    def now(self) -> float:
        return self.backend.now()

    def state(self) -> JointState:
        self.check()
        st = self.servo.state
        if st is None:
            raise BackendError("not connected")
        return st.copy()

    def q(self) -> np.ndarray:
        """Measured joint angles (rad)."""
        return self.state().q

    def tcp_pose(self, q: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """TCP pose (p, R) at the measured (or given) joint angles."""
        return self.model.fk(self.q() if q is None else q)

    def joint_target(self) -> np.ndarray | None:
        """Last rate-limited joint target (None when not holding)."""
        return None if self.filter.q_prev is None else self.filter.q_prev.copy()

    def lag(self) -> float:
        """Largest ``|target - measured|`` over the joints (rad); 0 when not holding."""
        if self.filter.q_prev is None or self.servo.state is None:
            return 0.0
        return float(np.abs(self.filter.q_prev - self.servo.state.q).max())

    def check(self) -> None:
        """Raise the servo thread's error, if any (temperature stop, bus failure)."""
        e = self.servo.error
        if e is not None:
            # the servo thread has stopped: torque is off only if it went passive (over-temperature stop with
            # on_overtemp: disable); otherwise the motors hold the last frame
            self.enabled = self.servo.mode != ServoMode.PASSIVE
            raise e

    def check_stall(self) -> None:
        """Safety stop when a joint lags more than ``safety.lag`` for ``stall_time_s`` without making progress
        (collision, blocked joint, not enough torque). ``safety.on_stall``: ``disable`` (torque off) or ``hold``."""
        s = self.cfg.safety
        if self.filter.q_prev is None or self.servo.state is None:
            return
        q = self.servo.state.q
        err = np.abs(self.filter.q_prev - q)
        now = self.now()
        if err.max() <= s.lag:
            self._lag_since = self._lag_q0 = None
            return
        if self._lag_since is None:
            self._lag_since, self._lag_q0 = now, q.copy()
            return
        progress = np.abs(q - self._lag_q0)
        if now - self._lag_since > s.stall_time_s and progress.max() < s.stall_progress:
            j = int(err.argmax())
            msg = (f"{self.cfg.joints[j]} stalled: {math.degrees(err[j]):.1f} deg behind its target, moved "
                   f"{math.degrees(progress[j]):.1f} deg in {s.stall_time_s:.1f} s (collision / blocked / not enough torque)")
            self._lag_since = None
            if s.on_stall == "disable":
                self.disable_torque()
                raise SafetyStop(msg + "; torque off")
            self.servo.hold_measured(now)
            self.filter.reset(self.servo.state.q)
            raise SafetyStop(msg + "; holding the measured pose")
        if progress.max() >= s.stall_progress:  # still moving: restart the window
            self._lag_since, self._lag_q0 = now, q.copy()

    # ------------------------------------------------------------------------------------------- commands
    def command_joint(self, q: np.ndarray) -> np.ndarray:
        """Stream one joint target (rad) for this control cycle. Returns the filtered target sent to the servo.

        Raises :class:`CommandRejected` for a target more than ``safety.max_jump`` from the previous one.
        """
        self.check()
        if not self.enabled or self.servo.mode != ServoMode.HOLD:
            raise CommandRejected("torque is not on in hold mode: call enable() first")
        self._commanded = True
        q_f = self.filter(q, self.servo.state.q)
        self.servo.set_target(q_f, self.now())
        return q_f

    def sleep(self, dt: float | None = None) -> None:
        """Wait until the next control cycle (real) or advance the simulation by one cycle (sim)."""
        dt = self.dt if dt is None else dt
        if self.is_sim:
            self._sim_run(dt)
        else:
            now = time.perf_counter()
            if self._next_t is None or now - self._next_t > dt:  # first call or fell behind: resync
                self._next_t = now
            self._next_t += dt
            delay = self._next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
        self.check()

    def wait(self, seconds: float) -> None:
        """Keep streaming the current target for ``seconds`` (holds the pose; checks for stalls)."""
        for _ in range(max(1, int(round(seconds / self.dt)))):
            if self.filter.q_prev is not None and self.servo.mode == ServoMode.HOLD:
                self.command_joint(self.filter.q_prev)
                self.check_stall()
            self.sleep()

    def _sim_run(self, dt: float) -> None:
        steps = max(1, int(round(dt / self.servo.period)))
        for _ in range(steps):
            self.servo.tick(self.backend.now())
            self.backend.advance(self.servo.period)

    def move_to_joint(self, q: np.ndarray, speed: float | None = None, settle: float = 0.5) -> None:
        """Slow joint-space move from the current target (or measured pose) to ``q`` (rad).

        Smoothstep profile (zero velocity at both ends); ``speed`` (rad/s) is the average speed of the joint that
        moves most (default ``poses.move_speed``). Checks for stalls on the way.
        """
        if not self.enabled or self.servo.mode != ServoMode.HOLD:
            self.enable()
        speed = self.cfg.poses.move_speed if speed is None else speed
        q1 = np.clip(np.asarray(q, dtype=float), self.model.lower, self.model.upper)
        q0 = self.filter.q_prev.copy() if self.filter.q_prev is not None else self.q()
        span = float(np.abs(q1 - q0).max())
        steps = max(1, math.ceil(span / speed / self.dt))
        for i in range(1, steps + 1):
            a = i / steps
            s = a * a * (3.0 - 2.0 * a)
            self.command_joint(q0 + s * (q1 - q0))
            self.sleep()
            self.check_stall()
        self.wait(settle)

    def move_to_joint_staged(self, q: np.ndarray, first: list[int] | None = None, speed: float | None = None) -> None:
        """Move the ``first`` joints (default ``poses.unfold_first``: shoulder + elbow) before the others.

        From the folded zero pose a straight joint-space line flexes the wrist while the arm is still closed and
        rubs the upper arm; lifting shoulder and elbow first avoids it.
        """
        first = self.cfg.poses.unfold_first if first is None else first
        q = np.asarray(q, dtype=float)
        stage = self.filter.q_prev.copy() if self.filter.q_prev is not None else self.q()
        stage[first] = q[first]
        self.move_to_joint(stage, speed, settle=0.2)
        self.move_to_joint(q, speed)

    def is_folded(self, tol: float = math.radians(15.0)) -> bool:
        return bool(np.abs(self.q() - self.cfg.poses.zero).max() < tol)

    def move_home(self, speed: float | None = None) -> None:
        """Go to the tool's home pose (staged when starting from the folded pose)."""
        if self.tool.home is None:
            raise ValueError(f"tool {self.tool.name!r} has no home_deg")
        if self.is_folded():
            self.move_to_joint_staged(self.tool.home, speed=speed)
        else:
            self.move_to_joint(self.tool.home, speed)

    def fold(self, speed: float | None = None) -> None:
        """Return to the zero pose in ``poses.fold_stages`` order (base + wrist first, then shoulder + elbow)."""
        zero = self.cfg.poses.zero
        target = self.filter.q_prev.copy() if self.filter.q_prev is not None else self.q()
        for stage in self.cfg.poses.fold_stages:
            target[stage] = zero[stage]
            self.move_to_joint(target.copy(), speed, settle=0.2)

    def move_to_pose(self, pos: np.ndarray, rot: np.ndarray | None = None, speed: float | None = None,
                     tol: float = 1e-3, timeout: float = 20.0) -> bool:
        """Straight-line TCP move (Cartesian controller) to ``(pos, rot)``; ``speed`` in m/s (default: the limit).

        Returns False when the target is not reached within ``timeout`` (e.g. outside the workspace limits).
        """
        cart = self.cartesian()
        if speed is not None:
            cart.limits.max_speed = min(cart.limits.max_speed, speed)
        pos = np.asarray(pos, dtype=float)
        t_end = self.now() + timeout
        while self.now() < t_end:
            st = cart.step(pos, rot)
            self.sleep()
            done = np.linalg.norm(st.pos - pos) < tol
            if rot is not None:
                done &= np.linalg.norm(np.asarray(rot) - st.rot) < 1e-3
            if done:
                self.wait(0.5)
                return True
        return False

    def cartesian(self, limits=None):
        """A :class:`~rebot_control.cartesian.CartesianController` starting at the current target pose."""
        from .cartesian import CartesianController

        if not self.enabled or self.servo.mode != ServoMode.HOLD:
            self.enable()
        return CartesianController(self, limits)

    # ------------------------------------------------------------------------------------------ tool / gains
    def set_tool(self, tool: str | Path | ToolConfig) -> None:
        """Swap the tool (TCP + payload) without reconnecting; takes effect at the next servo tick."""
        tool = tool if isinstance(tool, ToolConfig) else ToolConfig.load(tool)
        with self.servo._lock:
            self.model.set_tool(tool)
            self.servo.data = self.model.new_data()
        self.tool = tool.copy()

    def set_gains(self, kp: np.ndarray | None = None, kd: np.ndarray | None = None) -> None:
        if kp is not None and self.cfg.ripple is not None:
            self.cfg.ripple.check(np.asarray(kp, dtype=float), self.cfg.feedforward.ripple_scale)
        self.servo.set_gains(kp, kd)

    # -------------------------------------------------------------------------------------------- recording
    def start_recording(self) -> None:
        self.servo.start_recording()

    def stop_recording(self) -> dict[str, np.ndarray]:
        return self.servo.stop_recording()

    @contextlib.contextmanager
    def recording(self):
        """``with arm.recording() as rec: ...`` then ``rec["q"]`` etc. (filled when the block exits)."""
        out: dict[str, np.ndarray] = {}
        self.start_recording()
        try:
            yield out
        finally:
            out.update(self.stop_recording())

    def _require_connected(self) -> None:
        if not self.connected:
            raise BackendError("not connected: call connect() (or use `with arm:`)")
        self.check()

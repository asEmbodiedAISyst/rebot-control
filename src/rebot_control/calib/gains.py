"""MIT gain (kp / kd) checks and tuning from small step responses.

Background (``docs/user/gains.md``): the motor runs ``tau = kp (q* - q) + kd (dq* - dq) + tau_ff`` internally, so
the joint behaves like a mass-spring-damper with stiffness ``kp``, damping ``kd`` (plus friction) and inertia
``J = M(q)_jj + rotor inertia``::

    omega_n = sqrt(kp / J),    zeta = kd / (2 sqrt(kp J))

``kp`` and ``kd`` cannot be raised freely: delay (CAN round trip, 150 Hz frames), encoder quantisation and velocity
noise make the loop ring, buzz or squeal above some limit. The procedure here:

1. :func:`step_response`: a small target step (default 2 deg) on one joint while the others hold; metrics:
   overshoot, rise / settling time, steady-state error, residual velocity (limit cycle) and torque ripple (buzz).
2. :func:`find_kp_max`: raise kp geometrically (kd follows the damping-ratio rule) until a limit criterion trips or
   the safety cap is reached. The last passing kp is ``kp_max``.
3. Recommended working gains: ``kp = margin * kp_max`` (default 0.6) and ``kd`` for ``zeta`` (default 0.8),
   both capped; verified with one more step.

Every step is small and the torque stays capped by ``safety.max_error``; a step that moves the joint by more than
three times the amplitude aborts.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import asdict, dataclass

import numpy as np

from ..errors import SafetyStop


@dataclass
class StepMetrics:
    joint: int
    kp: float
    kd: float
    amplitude_deg: float
    overshoot: float  # fraction of the step
    rise_s: float  # 10 -> 90 %
    settle_s: float  # into +-5 % for good
    ss_error_deg: float
    residual_vel_deg_s: float  # RMS velocity in the last 30 % (limit cycle / ringing)
    torque_ripple: float  # std of the joint torque in the last 30 % (N*m), buzz / squeal indicator
    zeta_est: float  # from the overshoot (nan when there is none)
    zeta_model: float  # kd / (2 sqrt(kp J))
    omega_n: float  # sqrt(kp / J) (rad/s)

    def line(self) -> str:
        return (f"kp {self.kp:7.1f} kd {self.kd:6.2f} | overshoot {100 * self.overshoot:5.1f} % rise {1000 * self.rise_s:5.0f} ms "
                f"settle {1000 * self.settle_s:5.0f} ms ss {self.ss_error_deg:+.3f} deg | residual vel "
                f"{self.residual_vel_deg_s:5.2f} deg/s ripple {self.torque_ripple:.3f} N*m | zeta est "
                f"{self.zeta_est:4.2f} model {self.zeta_model:4.2f}")


def effective_inertia(arm, q: np.ndarray, joint: int) -> float:
    return float(arm.model.mass_matrix(q)[joint, joint])


def analyze_step(t: np.ndarray, qj: np.ndarray, dqj: np.ndarray, tauj: np.ndarray, q_start: float, amplitude: float
                 ) -> dict[str, float]:
    t = t - t[0]
    y = (qj - q_start) / amplitude
    over = max(0.0, float(y.max() - 1.0))
    try:
        t10 = t[np.argmax(y >= 0.1)] if (y >= 0.1).any() else np.nan
        t90 = t[np.argmax(y >= 0.9)] if (y >= 0.9).any() else np.nan
        rise = float(t90 - t10)
    except IndexError:
        rise = np.nan
    outside = np.flatnonzero(np.abs(y - 1.0) > 0.05)
    settle = float(t[outside[-1]]) if len(outside) else 0.0
    if len(outside) and outside[-1] == len(t) - 1:
        settle = np.inf
    tail = slice(int(0.7 * len(t)), None)
    ss = float((y[tail].mean() - 1.0) * math.degrees(amplitude))
    rv = float(math.degrees(np.sqrt(np.mean(dqj[tail] ** 2))))
    ripple = float(np.std(tauj[tail]))
    if over > 0.005:
        L = math.log(over)
        zeta = -L / math.sqrt(math.pi ** 2 + L ** 2)
    else:
        zeta = float("nan")
    return {"overshoot": over, "rise_s": rise, "settle_s": settle, "ss_error_deg": ss, "residual_vel_deg_s": rv,
            "torque_ripple": ripple, "zeta_est": zeta}


def step_response(arm, joint: int, kp: float | None = None, kd: float | None = None,
                  amplitude: float = math.radians(2.0), duration: float = 0.8) -> list[StepMetrics]:
    """Step ``joint`` by +amplitude and back with the given gains (others unchanged); returns both responses."""
    kp_all, kd_all = arm.servo.kp.copy(), arm.servo.kd.copy()
    if kp is not None:
        kp_all[joint] = kp
    if kd is not None:
        kd_all[joint] = kd
    q0 = arm.joint_target()
    if q0 is None:
        raise RuntimeError("arm must be holding (enable() first)")
    J = effective_inertia(arm, q0, joint)
    out = []
    arm.set_gains(kp_all, kd_all)
    try:
        arm.wait(0.3)
        for sign in (+1.0, -1.0):
            start = arm.joint_target()
            q1 = start.copy()
            q1[joint] += sign * amplitude
            with arm.recording() as rec:
                arm.servo.set_target(q1, arm.now(), step=True)
                arm.filter.reset(q1)
                arm.sleep()  # let the servo send the step before the next (interpolated) target replaces it
                arm.wait(duration)
            if np.abs(rec["q"][:, joint] - start[joint]).max() > 3 * amplitude + math.radians(1.0):
                raise SafetyStop(f"{arm.cfg.joints[joint]}: step response diverged (kp {kp_all[joint]}, kd {kd_all[joint]})")
            m = analyze_step(rec["t"], rec["q"][:, joint], rec["dq"][:, joint], rec["tau"][:, joint], start[joint],
                             sign * amplitude)
            out.append(StepMetrics(joint=joint, kp=float(kp_all[joint]), kd=float(kd_all[joint]),
                                   amplitude_deg=math.degrees(sign * amplitude),
                                   zeta_model=float(kd_all[joint] / (2 * math.sqrt(kp_all[joint] * J))),
                                   omega_n=float(math.sqrt(kp_all[joint] / J)), **m))
    except BaseException:
        arm.set_gains(arm.cfg.gains.kp, arm.cfg.gains.kd)
        raise
    return out


@dataclass
class Limits:
    max_overshoot: float = 0.30
    max_residual_vel_deg_s: float = 1.0
    max_ripple_frac: float = 0.01  # of the joint effort limit
    max_settle_s: float = 0.6


def failed(m: StepMetrics, lim: Limits, effort: float) -> str | None:
    if m.overshoot > lim.max_overshoot:
        return f"overshoot {100 * m.overshoot:.0f} %"
    if m.residual_vel_deg_s > lim.max_residual_vel_deg_s:
        return f"residual velocity {m.residual_vel_deg_s:.1f} deg/s (ringing / limit cycle)"
    if m.torque_ripple > lim.max_ripple_frac * effort:
        return f"torque ripple {m.torque_ripple:.3f} N*m (buzz)"
    if not m.settle_s <= lim.max_settle_s:
        return f"does not settle within {lim.max_settle_s:.1f} s"
    return None


def kd_for(kp: float, J: float, zeta: float) -> float:
    return 2.0 * zeta * math.sqrt(kp * J)


@dataclass
class TuneResult:
    joint: int
    kp_max: float
    stop_reason: str
    kp: float
    kd: float
    trials: list[dict]
    check: list[dict]


def find_kp_max(arm, joint: int, kp_cap: float, kd_cap: float, zeta: float = 0.8, factor: float = 1.25,
                margin: float = 0.6, limits: Limits | None = None, amplitude: float = math.radians(2.0),
                progress: Callable[[str], None] = print) -> TuneResult:
    """Raise kp from the configured value until a limit trips or ``kp_cap``; recommend ``margin * kp_max``."""
    lim = limits or Limits()
    effort = float(arm.model.effort_limit[joint])
    J = effective_inertia(arm, arm.joint_target(), joint)
    kp = float(arm.cfg.gains.kp[joint])
    trials, kp_ok, reason = [], None, f"reached the cap kp = {kp_cap:g}"
    try:
        while True:
            kd = min(kd_for(kp, J, zeta), kd_cap)
            ms = step_response(arm, joint, kp, kd, amplitude)
            worst = max(ms, key=lambda m: (m.overshoot, m.residual_vel_deg_s))
            trials.append(asdict(worst))
            why = next((w for w in (failed(m, lim, effort) for m in ms) if w), None)
            progress(f"  {worst.line()}" + (f"  <- {why}" if why else ""))
            if why:
                reason = why
                break
            kp_ok = kp
            if kp >= kp_cap:
                break
            kp = min(kp * factor, kp_cap)
    finally:
        arm.set_gains(arm.cfg.gains.kp, arm.cfg.gains.kd)
    if kp_ok is None:
        raise SafetyStop(f"{arm.cfg.joints[joint]}: the configured kp already fails ({reason}); lower it in the arm config")
    kp_rec = margin * kp_ok
    kd_rec = min(kd_for(kp_rec, J, zeta), kd_cap)
    check = step_response(arm, joint, kp_rec, kd_rec, amplitude)
    arm.set_gains(arm.cfg.gains.kp, arm.cfg.gains.kd)
    progress(f"  recommended: {check[0].line()}")
    return TuneResult(joint=joint, kp_max=kp_ok, stop_reason=reason, kp=kp_rec, kd=kd_rec, trials=trials,
                      check=[asdict(m) for m in check])

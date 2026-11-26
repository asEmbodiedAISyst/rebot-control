"""Joint friction identification from constant-velocity sweeps.

One joint at a time moves back and forth over ``[center - amplitude, center + amplitude]`` at a list of speeds
while the other joints hold. In the constant-velocity part of each sweep (``ddq = 0``)::

    tau_meas - rnea(q, dq, 0) = fc * tanh(v / vs) + fv * v + residual(q_j)

The rigid-body term uses the current model (gravity + Coriolis); its error along the sweep (only ``q_j`` varies) is
absorbed by ``a sin q_j + b cos q_j + c`` so that a not-yet-perfect gravity model does not bias the friction.
Forward and backward passes over the same angles make Coulomb friction and the gravity error separable.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from ..config import FrictionParams
from ..model import RobotModel
from .poses import path_is_clear

DEFAULT_SPEEDS_DEG = (5.0, 10.0, 20.0, 40.0, 60.0)
DEFAULT_AMPLITUDE_DEG = np.array([30.0, 15.0, 20.0, 30.0, 30.0, 40.0])


def sweep_trajectory(q0: float, amplitude: float, speed: float, dt: float, accel: float) -> tuple[np.ndarray, np.ndarray]:
    """Joint angle samples of one sweep q0-A -> q0+A -> q0-A with trapezoidal velocity; returns (q, v_nominal)."""
    def leg(a: float, b: float) -> tuple[np.ndarray, np.ndarray]:
        dist = abs(b - a)
        sgn = np.sign(b - a)
        t_acc = speed / accel
        d_acc = 0.5 * accel * t_acc ** 2
        if 2 * d_acc > dist:  # triangle profile
            t_acc = np.sqrt(dist / accel)
            v_peak, t_flat = accel * t_acc, 0.0
        else:
            v_peak, t_flat = speed, (dist - 2 * d_acc) / speed
        T = 2 * t_acc + t_flat
        t = np.arange(1, int(np.ceil(T / dt)) + 1) * dt
        t = np.minimum(t, T)
        s = np.where(t < t_acc, 0.5 * accel * t ** 2,
                     np.where(t < t_acc + t_flat, 0.5 * accel * t_acc ** 2 + v_peak * (t - t_acc),
                              dist - 0.5 * accel * (T - t) ** 2))
        v = np.where((t >= t_acc) & (t <= t_acc + t_flat), v_peak, 0.0)  # nominal: only the flat part counts
        return a + sgn * s, sgn * v
    up_q, up_v = leg(q0 - amplitude, q0 + amplitude)
    dn_q, dn_v = leg(q0 + amplitude, q0 - amplitude)
    return np.concatenate([up_q, dn_q]), np.concatenate([up_v, dn_v])


def collect_sweeps(arm, joint: int, center: np.ndarray, amplitude: float, speeds: tuple[float, ...],
                   accel: float = np.radians(200.0), progress: Callable[[str], None] = print) -> dict[str, np.ndarray]:
    """Run the sweeps of one joint (rad, rad/s) and return the recorded servo samples plus the nominal velocity."""
    center = np.asarray(center, dtype=float)
    lo, hi = center.copy(), center.copy()
    lo[joint] -= amplitude
    hi[joint] += amplitude
    if not (path_is_clear(arm.model, lo, hi) and path_is_clear(arm.model, center, lo)):
        raise ValueError(f"{arm.cfg.joints[joint]}: sweep of +-{np.degrees(amplitude):.0f} deg around "
                         f"{np.round(np.degrees(center), 1)} is not clear of the table / base")
    vmax = arm.cfg.safety.max_joint_speed
    out: dict[str, list] = {k: [] for k in ("t", "q", "dq", "tau", "v_nom")}
    arm.move_to_joint(lo)
    for v in speeds:
        if v > vmax:
            progress(f"  skip {np.degrees(v):.0f} deg/s (> max_joint_speed)")
            continue
        qj, vn = sweep_trajectory(center[joint], amplitude, v, arm.dt, accel)
        q = center.copy()
        with arm.recording() as rec:
            t_start = arm.now()
            times = []
            for k in range(len(qj)):
                q[joint] = qj[k]
                arm.command_joint(q)
                times.append(arm.now() - t_start)
                arm.sleep()
                arm.check_stall()
        # nominal velocity for each servo sample (the joint filter delays the motion by about filter_tau_s)
        delay = arm.cfg.safety.filter_tau_s
        ts = rec["t"] - t_start
        idx = np.clip(np.searchsorted(np.asarray(times) + delay, ts) - 1, 0, len(vn) - 1)
        out["t"].append(rec["t"])
        out["q"].append(rec["q"])
        out["dq"].append(rec["dq"])
        out["tau"].append(rec["tau"])
        out["v_nom"].append(vn[idx])
        arm.wait(0.3)
        progress(f"  {arm.cfg.joints[joint]} at {np.degrees(v):.0f} deg/s: {len(rec['t'])} samples")
    return {k: np.concatenate(v) for k, v in out.items()} | {"joint": np.array(joint)}


@dataclass
class JointFrictionFit:
    joint: int
    fc_pos: float
    fc_neg: float
    fv: float
    rms: float
    samples: int


def fit_joint_friction(model: RobotModel, data: dict[str, np.ndarray], vs: float = 0.02,
                       vel_tol: float = 0.2) -> JointFrictionFit:
    """Fit Coulomb (per direction) + viscous friction of one joint from :func:`collect_sweeps` data."""
    j = int(data["joint"])
    q, dq, tau, v_nom = data["q"], data["dq"], data["tau"], data["v_nom"]
    v = dq[:, j]
    keep = (np.abs(v_nom) > 0) & (np.abs(v - v_nom) < vel_tol * np.abs(v_nom) + np.radians(0.5))
    if keep.sum() < 20:
        raise ValueError(f"joint {j}: only {keep.sum()} constant-velocity samples; slower speeds or larger amplitude")
    d = model.new_data()
    params = model.dynamic_params()
    rb = np.array([model.regressor(q[i], dq[i], None, d)[j] @ params for i in np.flatnonzero(keep)])
    y = tau[keep, j] - rb
    vk, qk = v[keep], q[keep, j]
    # Coulomb friction is fitted symmetric: a direction-dependent part is indistinguishable from a constant
    # torque offset (and from the gravity-model error), which the sin / cos / constant terms absorb.
    A = np.column_stack([np.tanh(vk / vs), vk, np.sin(qk), np.cos(qk), np.ones_like(vk)])
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    rms = float(np.sqrt(np.mean((A @ x - y) ** 2)))
    return JointFrictionFit(joint=j, fc_pos=float(x[0]), fc_neg=float(x[0]), fv=float(x[1]), rms=rms, samples=int(keep.sum()))


def to_friction_params(fits: list[JointFrictionFit], n: int, base: FrictionParams | None = None,
                       vs: float = 0.02) -> FrictionParams:
    fp = FrictionParams.zeros(n) if base is None else FrictionParams(*(np.array(getattr(base, k), dtype=float).copy()
                                                                       for k in ("fc_pos", "fc_neg", "fv", "vs", "bias")))
    for f in fits:
        fp.fc_pos[f.joint], fp.fc_neg[f.joint], fp.fv[f.joint], fp.vs[f.joint] = max(f.fc_pos, 0.0), max(f.fc_neg, 0.0), \
            max(f.fv, 0.0), vs
    return fp

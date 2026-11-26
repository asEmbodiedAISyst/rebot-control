"""Torque ripple that repeats with the joint angle, identified from slow constant-velocity sweeps.

Geared BLDC actuators need a torque that varies periodically with the rotor angle to turn evenly (cogging, one
period per electrical cycle: pole pairs x gear ratio periods per turn of the joint). At low speed the position
loop cannot reject it and the joint visibly speeds up and slows down; at high speed the inertia filters it out.

The sweeps are the ones of the friction calibration (:func:`~rebot_control.calib.friction.collect_sweeps`), run
slowly. In the constant-velocity part of each pass the motor torque is::

    tau_meas - gravity(q) = offset(direction, speed) + ripple(q_j)
    ripple(q) = a1 sin(N q) + b1 cos(N q) + a2 sin(2 N q) + b2 cos(2 N q)

The reported torque is the position loop's reaction to the ripple, so it lags the ripple a little. Seen over the
angle, that lag has opposite signs in the two directions of travel: the model is the mean of the two directions'
coefficients, which cancels it. ``N`` is the integer that explains most of the torque when it is not given.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import RippleParams
from ..model import RobotModel


@dataclass
class JointRippleFit:
    joint: int
    cycles: int
    a1: float
    b1: float
    a2: float
    b2: float
    amplitude: float            # of the fundamental of the model (N*m)
    amplitude_dir: tuple[float, float]   # fundamental fitted in the positive / negative direction alone
    phase_dir: tuple[float, float]       # its phase (deg)
    explained: float            # share of the torque variance (offsets removed) the model explains
    explained_dir: float        # the same with each direction's own fit (upper bound)
    rms_before: float           # N*m, torque around the offsets
    rms_after: float
    samples: int


def _design(q: np.ndarray, n: float) -> np.ndarray:
    x = n * q
    return np.column_stack([np.sin(x), np.cos(x), np.sin(2 * x), np.cos(2 * x)])


def fit_joint_ripple(model: RobotModel, data: dict[str, np.ndarray], cycles: int | None = None,
                     search: tuple[int, int] = (40, 300), vel_tol: float = 0.5) -> JointRippleFit:
    """Fit the ripple of one joint from :func:`~rebot_control.calib.friction.collect_sweeps` data."""
    j = int(data["joint"])
    q, dq, tau, v_nom = data["q"], data["dq"], data["tau"], data["v_nom"]
    keep = (np.abs(v_nom) > 0) & (np.abs(dq[:, j] - v_nom) < vel_tol * np.abs(v_nom) + np.radians(3.0))
    if keep.sum() < 200:
        raise ValueError(f"joint {j}: only {keep.sum()} constant-velocity samples; sweep further or more slowly")
    d = model.new_data()
    y = tau[keep, j] - np.array([model.gravity(q[i], d)[j] for i in np.flatnonzero(keep)])
    qj, v = q[keep, j], v_nom[keep]
    span = float(np.ptp(qj))
    passes = []                                   # per direction: (angles, torque with the slow part removed)
    for sign in (1.0, -1.0):
        m = np.sign(v) == sign
        if m.sum() < 100:
            raise ValueError(f"joint {j}: the sweeps do not cover both directions")
        speeds = np.unique(np.round(np.abs(v[m]), 6))
        base = np.column_stack([np.abs(v[m]) == s for s in speeds] + [qj[m] - qj.mean(), (qj[m] - qj.mean()) ** 2]).astype(float)
        passes.append((qj[m], y[m], base))

    def solve(n: float):
        coefs, parts = [], []
        for a, yy, base in passes:
            X = np.column_stack([_design(a, n), base])
            c = np.linalg.lstsq(X, yy, rcond=None)[0]
            coefs.append(c[:4])
            parts.append((a, yy - base @ c[4:]))       # torque around the offsets
        return coefs, parts

    def explained(n: float, coefs) -> tuple[float, float, float]:
        _, parts = solve(n)
        tot = sum(float((r ** 2).sum()) for _, r in parts)
        res = sum(float(((r - _design(a, n) @ c) ** 2).sum()) for (a, r), c in zip(parts, coefs))
        cnt = sum(len(r) for _, r in parts)
        return 1.0 - res / tot, float(np.sqrt(tot / cnt)), float(np.sqrt(res / cnt))

    if cycles is None:
        if span * search[0] < 4 * np.pi:
            raise ValueError(f"joint {j}: the sweep covers {np.degrees(span):.0f} deg, too little to find the period")
        best = (-1.0, search[0])
        for n in range(search[0], search[1] + 1):
            # score the fundamental alone: with its second harmonic, N / 2 would explain as much as N
            coefs = [np.r_[c[:2], 0.0, 0.0] for c in solve(float(n))[0]]
            best = max(best, (explained(float(n), coefs)[0], n))
        cycles = best[1]
    n = float(cycles)
    coefs, _ = solve(n)
    mean = 0.5 * (coefs[0] + coefs[1])
    share, rms_before, rms_after = explained(n, [mean, mean])
    share_dir = explained(n, coefs)[0]
    return JointRippleFit(joint=j, cycles=int(cycles), a1=float(mean[0]), b1=float(mean[1]), a2=float(mean[2]), b2=float(mean[3]),
                          amplitude=float(np.hypot(mean[0], mean[1])),
                          amplitude_dir=(float(np.hypot(*coefs[0][:2])), float(np.hypot(*coefs[1][:2]))),
                          phase_dir=(float(np.degrees(np.arctan2(coefs[0][1], coefs[0][0]))), float(np.degrees(np.arctan2(coefs[1][1], coefs[1][0])))),
                          explained=share, explained_dir=share_dir, rms_before=rms_before, rms_after=rms_after, samples=int(keep.sum()))


def to_ripple_params(fits: list[JointRippleFit], n: int, base: RippleParams | None = None) -> RippleParams:
    """Ripple model with the fitted joints replaced (the others keep ``base``, or zero)."""
    rp = RippleParams.zeros(n) if base is None else RippleParams(*(np.array(getattr(base, k), dtype=float).copy()
                                                                   for k in ("cycles", "a1", "b1", "a2", "b2")))
    for f in fits:
        rp.cycles[f.joint], rp.a1[f.joint], rp.b1[f.joint], rp.a2[f.joint], rp.b2[f.joint] = f.cycles, f.a1, f.b1, f.a2, f.b2
    return rp

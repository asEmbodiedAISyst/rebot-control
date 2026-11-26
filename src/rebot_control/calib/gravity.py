"""Gravity and payload identification from static poses.

Data (:func:`collect_static`): at each pose the arm holds still (HOLD mode) and the torque reported by the motors is
averaged. Each pose is approached once from above and once from below (``approach``): static friction makes the
holding torque depend on the approach direction, and averaging both sides cancels it to first order.

Model: at rest ``tau = Y(q) theta + b``, with ``Y`` the static part of the joint torque regressor (Pinocchio) and
``theta`` the mass and first moment ``m, m*c`` (4 parameters) of every body. Only some combinations of ``theta``
are identifiable from static data; the fit is a MAP / ridge estimate around the prior (URDF or previous
calibration), so unidentifiable directions stay at the prior instead of drifting. ``b`` is a per-joint torque offset
(motor torque estimate bias, cable forces), fitted but not used for control.

* :func:`fit_gravity`: all link masses / centres of mass (tool attached or not; the tool payload from the tool
  config is subtracted from the flange body).
* :func:`fit_payload`: only the tool payload (mass + centre of mass) with the arm links fixed.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from ..model import N_PARAMS, RobotModel

log = logging.getLogger(__name__)
STATIC = 4  # m, m*cx, m*cy, m*cz


# ------------------------------------------------------------------------------------------------------ data
def collect_static(arm, poses: np.ndarray, approach: float = np.radians(3.0), settle_s: float = 1.0,
                   average_s: float = 0.5, speed: float | None = None,
                   progress: Callable[[str], None] = print) -> dict[str, np.ndarray]:
    """Visit ``poses`` and record the averaged joint angles and torques at rest, from both approach sides.

    Returns ``{"q": (2N, n), "tau": (2N, n), "tau_std": (2N, n), "side": (2N,)}``.
    """
    qs, taus, stds, sides = [], [], [], []
    for i, q in enumerate(poses):
        for side in (+1.0, -1.0):
            pre = np.clip(q + side * approach, arm.model.lower, arm.model.upper)
            arm.move_to_joint(pre, speed, settle=0.0)
            arm.move_to_joint(q, speed, settle=settle_s)
            with arm.recording() as rec:
                arm.wait(average_s)
            qs.append(rec["q"].mean(axis=0))
            taus.append(rec["tau"].mean(axis=0))
            stds.append(rec["tau"].std(axis=0))
            sides.append(side)
        progress(f"pose {i + 1}/{len(poses)}: q = {np.round(np.degrees(q), 1)} deg, "
                 f"tau = {np.round(0.5 * (taus[-1] + taus[-2]), 2)} N*m")
    return {"q": np.array(qs), "tau": np.array(taus), "tau_std": np.array(stds), "side": np.array(sides)}


# ------------------------------------------------------------------------------------------------------- fit
def static_columns(n: int) -> np.ndarray:
    return np.concatenate([np.arange(k * N_PARAMS, k * N_PARAMS + STATIC) for k in range(n)])


def static_regressor(model: RobotModel, qs: np.ndarray, data=None) -> np.ndarray:
    """Stacked static regressors, shape (N * n, 4 n)."""
    cols = static_columns(model.n)
    d = model.new_data() if data is None else data
    return np.vstack([model.regressor(q, data=d)[:, cols] for q in qs])


def _payload_static(model: RobotModel) -> np.ndarray:
    """Payload (m, m*c) in the flange joint frame."""
    return model.payload_inertia().toDynamicParameters()[:STATIC] if model.tool.mass > 0 else np.zeros(STATIC)


def _link_static(model: RobotModel) -> np.ndarray:
    return np.concatenate([model.link_inertias[j].toDynamicParameters()[:STATIC] for j in model.joint_ids])


def _to_link_params(model: RobotModel, theta_links: np.ndarray, min_mass: float = 1e-3) -> dict[str, dict]:
    out = {}
    for k, j in enumerate(model.joint_ids):
        m, mc = theta_links[4 * k], theta_links[4 * k + 1:4 * k + 4]
        name = model.link_name(j)
        if m < min_mass:
            log.warning("%s: identified mass %.4f kg is not physical; keeping the prior", name, m)
            m, com = model.link_inertias[j].mass, model.link_inertias[j].lever
        else:
            com = mc / m
        out[name] = {"mass": round(float(m), 6), "com": np.round(com, 6).tolist()}
    return out


@dataclass
class GravityFit:
    link_params: dict[str, dict]
    bias: np.ndarray  # per-joint torque offset (N*m)
    rms_before: np.ndarray  # per-joint residual RMS with the prior model (N*m)
    rms_after: np.ndarray  # per-joint residual RMS after the fit (N*m)
    theta: np.ndarray  # identified static parameters (4 per body, payload included on the flange body)
    info: dict = field(default_factory=dict)

    def summary(self, joints: list[str]) -> str:
        lines = ["joint     RMS before  RMS after   bias  (N*m)"]
        for j, name in enumerate(joints):
            lines.append(f"{name:8s}  {self.rms_before[j]:9.3f}  {self.rms_after[j]:9.3f}  {self.bias[j]:6.3f}")
        lines.append("link      mass (kg)   com (m)")
        for link, p in self.link_params.items():
            lines.append(f"{link:8s}  {p['mass']:9.4f}   {np.round(p['com'], 4).tolist()}")
        return "\n".join(lines)


def fit_gravity(model: RobotModel, data: dict[str, np.ndarray], prior_rel: float = 0.3, prior_abs_kg: float = 0.02,
                lever_m: float = 0.05, bias_std: float = 0.5, noise_std: float = 0.05, fit_bias: bool = True) -> GravityFit:
    """MAP fit of every body's mass and first moment around the current model (see the module docstring).

    Prior standard deviations: mass ``prior_rel * m0 + prior_abs_kg``, first moment that times ``lever_m``; torque
    noise ``noise_std``. Lower ``prior_rel`` to trust the URDF more.
    """
    n = model.n
    qs, tau = np.asarray(data["q"]), np.asarray(data["tau"])
    Y = static_regressor(model, qs)
    theta0 = _link_static(model)
    payload = _payload_static(model)
    fl = model.joint_ids.index(model.flange_joint)
    theta0_total = theta0.copy()
    theta0_total[4 * fl:4 * fl + 4] += payload
    m0 = np.repeat([model.link_inertias[j].mass for j in model.joint_ids], STATIC)
    sd = prior_rel * m0 + prior_abs_kg
    sd = sd * np.tile([1.0, lever_m, lever_m, lever_m], n)
    B = np.tile(np.eye(n), (len(qs), 1)) if fit_bias else np.zeros((len(qs) * n, 0))
    A = np.hstack([Y, B]) / noise_std
    r = (tau.reshape(-1) - Y @ theta0_total) / noise_std
    prior_sd = np.concatenate([sd, np.full(B.shape[1], bias_std)])
    A_aug = np.vstack([A, np.diag(1.0 / prior_sd)])
    r_aug = np.concatenate([r, np.zeros(len(prior_sd))])
    x, *_ = np.linalg.lstsq(A_aug, r_aug, rcond=None)
    theta = theta0_total + x[:Y.shape[1]]
    bias = x[Y.shape[1]:] if fit_bias else np.zeros(n)
    res0 = (tau.reshape(-1) - Y @ theta0_total).reshape(-1, n)
    res1 = (tau.reshape(-1) - Y @ theta).reshape(-1, n) - bias  # B @ bias; B has no columns when fit_bias is False
    theta_links = theta.copy()
    theta_links[4 * fl:4 * fl + 4] -= payload
    sv = np.linalg.svd(Y, compute_uv=False)
    rank = int((sv > sv[0] * 1e-6).sum())
    return GravityFit(link_params=_to_link_params(model, theta_links), bias=bias,
                      rms_before=np.sqrt((res0 ** 2).mean(axis=0)), rms_after=np.sqrt((res1 ** 2).mean(axis=0)),
                      theta=theta, info={"samples": len(qs), "regressor_rank": rank, "parameters": Y.shape[1]})


@dataclass
class PayloadFit:
    mass: float
    com: np.ndarray  # in the tool mount frame (flange)
    bias: np.ndarray
    rms_before: np.ndarray
    rms_after: np.ndarray

    def summary(self) -> str:
        return (f"payload mass {self.mass:.4f} kg, com {np.round(self.com, 4).tolist()} m (flange frame)\n"
                f"residual RMS before {np.round(self.rms_before, 3).tolist()} N*m\n"
                f"residual RMS after  {np.round(self.rms_after, 3).tolist()} N*m")


def fit_payload(model: RobotModel, data: dict[str, np.ndarray], fit_bias: bool = False, ridge: float = 1e-6) -> PayloadFit:
    """Least-squares payload (m, m*c) with the arm link parameters fixed (calibrate gravity first)."""
    n = model.n
    qs, tau = np.asarray(data["q"]), np.asarray(data["tau"])
    Y = static_regressor(model, qs)
    fl = model.joint_ids.index(model.flange_joint)
    theta_links = _link_static(model)
    r = tau.reshape(-1) - Y @ theta_links
    Yp = Y[:, 4 * fl:4 * fl + 4]
    B = np.tile(np.eye(n), (len(qs), 1)) if fit_bias else np.zeros((len(qs) * n, 0))
    A = np.hstack([Yp, B])
    x = np.linalg.solve(A.T @ A + ridge * np.eye(A.shape[1]), A.T @ r)
    p, bias = x[:4], (x[4:] if fit_bias else np.zeros(n))
    res0 = (r - Yp @ _payload_static(model)).reshape(-1, n)
    res1 = (r - A @ x).reshape(-1, n)
    m = float(p[0])
    if m <= 1e-4:
        raise ValueError(f"identified payload mass {m:.4f} kg: no payload, or not enough excitation")
    c_joint = p[1:4] / m
    flange = model.model.frames[model.flange_frame_id].placement
    com = flange.rotation.T @ (c_joint - flange.translation)
    return PayloadFit(mass=m, com=com, bias=bias, rms_before=np.sqrt((res0 ** 2).mean(axis=0)),
                      rms_after=np.sqrt((res1 ** 2).mean(axis=0)))


def gravity_residual(model: RobotModel, data: dict[str, np.ndarray]) -> np.ndarray:
    """Per-joint RMS of measured - predicted static torque (N*m) for the current model."""
    d = model.new_data()
    res = np.array([tau - model.gravity(q, d) for q, tau in zip(data["q"], data["tau"])])
    return np.sqrt((res ** 2).mean(axis=0))


__all__ = ["GravityFit", "PayloadFit", "collect_static", "fit_gravity", "fit_payload", "gravity_residual",
           "static_regressor"]

"""Simulated arm: MuJoCo physics of the same Pinocchio model (``pip install rebot-control[sim]``).

The MuJoCo model is generated from :class:`rebot_control.model.RobotModel` (bodies, joint axes, inertias, payload),
so kinematics and dynamics match the controller's model exactly unless a different "truth" is given on purpose
(``truth_link_params`` / ``truth_tool``), which is how the calibration tools are tested.

The motors are emulated at the physics rate: each step applies ``tau = kp (q* - q) + kd (dq* - dq) + tau_ff``
clipped to the URDF effort limit, like the MIT loop inside a RobStride driver. Joint damping and dry friction come
from ``sim:`` in the arm config. The arm itself has no collision geometry (no table, no self-collision): on its own
the simulator is for checking control logic, not contact.

A scene can be added around the arm with ``scene=``: a function that receives the ``mujoco.MjSpec`` after the arm has
been built and adds what it likes to it (floor, objects with free joints, collision geometry on the tool, cameras,
lights). The arm's joints stay the first ``n`` degrees of freedom, and the backend only ever touches those. The
spec takes inertias from the bodies, not from their geoms (``inertiafromgeom`` is off), so a moving body added by
the scene needs ``mass``, ``ipos``, ``inertia`` and ``explicitinertial = True`` (without ``ipos`` MuJoCo puts the
centre of mass at an offset equal to the body's position).

Time is simulation time: nothing runs in a background thread; :meth:`rebot_control.arm.Arm.sleep` advances the
physics tick by tick.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
import pinocchio as pin

from ..config import ArmConfig, Calibration, RippleParams, ToolConfig
from ..model import RobotModel
from .base import Backend, JointState


def build_mjspec(model: RobotModel, cfg: ArmConfig):
    """MuJoCo spec of the arm + tool from the Pinocchio model (no actuators: torques go through qfrc_applied)."""
    import mujoco

    pm = model.model
    pd = pm.createData()
    q0 = np.zeros(pm.nq)
    pin.computeJointJacobians(pm, pd, q0)
    spec = mujoco.MjSpec()
    spec.modelname = f"{cfg.name}_{model.tool.name}"
    spec.option.timestep = 1.0 / cfg.sim.physics_hz
    spec.option.gravity = [0.0, 0.0, -9.81]
    spec.compiler.degree = False  # joint ranges in rad
    spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
    bodies = {0: spec.worldbody}
    for j in model.joint_ids:
        M = pm.jointPlacements[j]
        parent = bodies[pm.parents[j]]
        name = model.link_name(j)
        quat = pin.Quaternion(M.rotation)
        body = parent.add_body(name=name, pos=M.translation.tolist(), quat=[quat.w, quat.x, quat.y, quat.z])
        I = pm.inertias[j]
        body.mass = max(float(I.mass), 1e-6)
        body.ipos = I.lever.tolist()
        Ic = I.inertia
        body.fullinertia = [Ic[0, 0], Ic[1, 1], Ic[2, 2], Ic[0, 1], Ic[0, 2], Ic[1, 2]]
        if hasattr(body, "explicitinertial"):
            body.explicitinertial = True
        axis = pin.getJointJacobian(pm, pd, j, pin.ReferenceFrame.LOCAL)[3:, pm.joints[j].idx_v]
        i = j - 1
        body.add_joint(name=pm.names[j], type=mujoco.mjtJoint.mjJNT_HINGE, axis=axis.tolist(),
                       range=[float(pm.lowerPositionLimit[i]), float(pm.upperPositionLimit[i])],
                       limited=mujoco.mjtLimited.mjLIMITED_TRUE, armature=float(cfg.rotor_inertia[i]),
                       damping=float(cfg.sim.damping[i]), frictionloss=float(cfg.sim.frictionloss[i]))
        # visual only: a thin capsule towards each child joint
        for c in range(1, pm.njoints):
            if pm.parents[c] == j:
                p = pm.jointPlacements[c].translation
                if np.linalg.norm(p) > 1e-4:
                    body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0, *p.tolist()], size=[0.02, 0, 0],
                                  contype=0, conaffinity=0, rgba=[0.55, 0.75, 0.55, 1])
        bodies[j] = body
    tcp = pm.frames[model.tcp_frame_id].placement
    qt = pin.Quaternion(tcp.rotation)
    flange = bodies[model.flange_joint]
    flange.add_site(name="tcp", pos=tcp.translation.tolist(), quat=[qt.w, qt.x, qt.y, qt.z], size=[0.006, 0, 0],
                    rgba=[1, 0.2, 0.2, 1])
    if np.linalg.norm(tcp.translation) > 1e-4:
        flange.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0, *tcp.translation.tolist()],
                        size=[0.006, 0, 0], contype=0, conaffinity=0, rgba=[0.8, 0.8, 0.8, 1])
    return spec


class SimBackend(Backend):
    is_sim = True

    def __init__(self, cfg: ArmConfig, tool: ToolConfig | None = None, calibration: Calibration | None = None,
                 truth_link_params: dict | None = None, truth_tool: ToolConfig | None = None,
                 truth_friction: dict | None = None, truth_ripple: dict | None = None,
                 q0: np.ndarray | None = None, seed: int = 0,
                 viewer: bool = False, realtime: bool | None = None,
                 scene: Callable | None = None) -> None:
        """
        Args:
            cfg, tool, calibration: The arm as the controller knows it.
            truth_link_params / truth_tool: Override the simulated link masses / tool (to test identification).
            truth_friction: ``{"damping": [...], "frictionloss": [...]}`` overriding ``cfg.sim``.
            truth_ripple: a :class:`~rebot_control.config.RippleParams` dict: the simulated motors need this extra
                torque to move evenly (it is subtracted from what they produce), to test the ripple feedforward.
            q0: Initial joint angles (default: the folded zero pose, like a powered-up real arm).
            viewer: Open the MuJoCo passive viewer (needs a display).
            scene: ``scene(spec)`` is called with the arm's ``mujoco.MjSpec`` before it is compiled, to add the
                surroundings. The base frame of the arm is the world frame.
            realtime: Pace the simulation to wall-clock time (default: on with the viewer, off otherwise).
        """
        self.cfg = cfg.copy()
        if truth_friction:
            for k, v in truth_friction.items():
                setattr(self.cfg.sim, k, np.asarray(v, dtype=float))
        self.n = cfg.n
        self.truth = RobotModel(self.cfg, truth_tool or tool, calibration)
        if truth_link_params:
            self.truth.set_link_params(truth_link_params)
        self.truth_ripple = RippleParams.from_dict(truth_ripple, self.n) if truth_ripple else None
        self.q0 = self.cfg.poses.zero.copy() if q0 is None else np.asarray(q0, dtype=float)
        self.scene = scene
        self.rng = np.random.default_rng(seed)
        self.mj_model = self.mj_data = None
        self.enabled = False
        self.temperature = 30.0  # reported driver temperature (deg C); tests raise it
        z = np.zeros(self.n)
        self._cmd = (z.copy(), z.copy(), z.copy(), z.copy(), z.copy())
        self._tau_out = z.copy()
        self.want_viewer = viewer
        self.realtime = viewer if realtime is None else realtime
        self.viewer = None
        self._wall0: float | None = None

    def connect(self) -> None:
        import mujoco

        if self.mj_model is not None:  # a new Arm attaching to the same simulated arm: keep its state
            return
        spec = build_mjspec(self.truth, self.cfg)
        if self.scene is not None:
            self.scene(spec)
        self.mj_model = spec.compile()
        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_data.qpos[:self.n] = self.q0
        mujoco.mj_forward(self.mj_model, self.mj_data)
        self.effort = np.asarray(self.truth.effort_limit, dtype=float)
        if self.want_viewer:
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)

    def close(self, disable: bool = True) -> None:
        if disable:
            self.enabled = False
        if self.viewer is not None and disable:
            self.viewer.close()
            self.viewer = None

    def read_positions(self) -> np.ndarray:
        return self.mj_data.qpos[:self.n].copy()

    def enable(self) -> None:
        if self.enabled:  # like the real motors: already enabled, keep executing the last frame
            return
        self.enabled = True
        self.send_passive()

    def disable(self) -> None:
        self.enabled = False

    def send_mit(self, q, dq, kp, kd, tau) -> None:
        self._cmd = tuple(np.asarray(v, dtype=float).copy() for v in (q, dq, kp, kd, tau))

    def read(self) -> JointState:
        d = self.mj_data
        noise = self.cfg.sim.torque_noise * self.rng.standard_normal(self.n) if self.cfg.sim.torque_noise else 0.0
        return JointState(float(d.time), d.qpos[:self.n].copy(), d.qvel[:self.n].copy(), self._tau_out + noise,
                          np.full(self.n, self.temperature),
                          np.ones(self.n, dtype=bool), np.full(self.n, self.enabled, dtype=bool))

    def now(self) -> float:
        return float(self.mj_data.time)

    def advance(self, dt: float) -> None:
        import mujoco

        m, d = self.mj_model, self.mj_data
        steps = max(1, int(round(dt / m.opt.timestep)))
        q_c, dq_c, kp, kd, tau_ff = self._cmd
        tau_sum = np.zeros(self.n)
        for _ in range(steps):
            if self.enabled:
                tau = kp * (q_c - d.qpos[:self.n]) + kd * (dq_c - d.qvel[:self.n]) + tau_ff
                tau = np.clip(tau, -self.effort, self.effort)
            else:
                tau = np.zeros(self.n)
            tau_sum += tau
            if self.truth_ripple is not None:
                tau = tau - self.truth_ripple.torque(d.qpos[:self.n])
            d.qfrc_applied[:self.n] = tau
            mujoco.mj_step(m, d)
        # reported torque = mean over the step (the drivers report a filtered current, not an instantaneous sample)
        self._tau_out = tau_sum / steps
        if self.realtime:
            import time

            now = time.perf_counter()
            if self._wall0 is None:
                self._wall0 = now - d.time
            ahead = d.time - (now - self._wall0)
            if ahead > 0:
                time.sleep(ahead)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def set_zero(self, joints: list[int]) -> None:
        pass

    def set_q(self, q: np.ndarray) -> None:
        """Teleport the simulated arm (tests)."""
        import mujoco

        self.mj_data.qpos[:self.n] = q
        self.mj_data.qvel[:self.n] = 0.0
        mujoco.mj_forward(self.mj_model, self.mj_data)

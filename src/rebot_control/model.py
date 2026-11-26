"""Kinematic and dynamic model of the arm + tool (Pinocchio).

One :class:`RobotModel` holds the Pinocchio model built from the arm URDF, with

* a ``tcp`` frame attached to the flange at the tool's TCP pose;
* the tool payload added to the flange body's inertia;
* optionally, identified link masses / centres of mass (:class:`rebot_control.config.Calibration`).

Pinocchio ``Data`` objects are not thread safe: every thread that evaluates the model (the servo thread, the
caller's thread) uses its own, from :meth:`RobotModel.new_data`.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from .config import ArmConfig, Calibration, ToolConfig
from .pose import rpy_to_matrix

LWA = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
N_PARAMS = 10  # dynamic parameters per body: m, m*c (3), inertia about the joint frame origin (6)


class RobotModel:
    def __init__(self, cfg: ArmConfig, tool: ToolConfig | None = None, calibration: Calibration | None = None) -> None:
        self.cfg = cfg
        model = pin.buildModelFromUrdf(str(cfg.urdf))
        names = [model.names[i] for i in range(1, model.njoints)]
        if names != list(cfg.joints):
            raise ValueError(f"URDF joints {names} do not match the config joints {cfg.joints}")
        self.model = model
        self.n = model.nv
        self.joint_ids = list(range(1, model.njoints))
        self.flange_joint = model.frames[model.getFrameId(cfg.flange_frame)].parentJoint
        self.flange_frame_id = model.getFrameId(cfg.flange_frame)
        # Joint limits: URDF ∩ configured soft limits
        lo = np.maximum(model.lowerPositionLimit, cfg.safety.soft_limits[:, 0])
        hi = np.minimum(model.upperPositionLimit, cfg.safety.soft_limits[:, 1])
        if np.any(lo >= hi):
            raise ValueError("empty joint range after intersecting URDF and soft limits")
        self.lower, self.upper = lo, hi
        self.effort_limit = np.asarray(model.effortLimit, dtype=float).copy()
        self.rotor_inertia = np.asarray(cfg.rotor_inertia, dtype=float)
        self.urdf_inertias = [pin.Inertia(model.inertias[i]) for i in range(model.njoints)]
        self.link_inertias = [pin.Inertia(I) for I in self.urdf_inertias]  # URDF or identified, without payload
        self.tool = ToolConfig()
        self.tcp_frame_id = -1
        if calibration is not None and calibration.link_params:
            self.set_link_params(calibration.link_params)  # needs self.tool: it re-applies the payload
        self.set_tool(tool or ToolConfig())

    # ------------------------------------------------------------------------------------------------ setup
    def link_name(self, joint_id: int) -> str:
        """Name of the URDF link moved by joint ``joint_id`` (1-based Pinocchio joint id)."""
        for f in self.model.frames:
            if f.type == pin.FrameType.BODY and f.parentJoint == joint_id and f.parentFrame and \
                    self.model.frames[f.parentFrame].type == pin.FrameType.JOINT:
                return f.name
        return self.model.names[joint_id]

    def set_link_params(self, params: dict[str, dict]) -> None:
        """Replace the mass and COM of links (``{link: {"mass": m, "com": [x, y, z]}}``, COM in the link frame).

        The rotational inertia about the COM is kept from the URDF. Links are matched by URDF link name.
        """
        by_link = {self.link_name(j): j for j in self.joint_ids}
        for link, p in params.items():
            if link not in by_link:
                raise KeyError(f"unknown link {link!r}; links: {list(by_link)}")
            j = by_link[link]
            base = self.urdf_inertias[j]
            # The URDF inertia of a link is expressed at the joint frame; the link frame coincides with it here.
            self.link_inertias[j] = pin.Inertia(float(p["mass"]), np.asarray(p["com"], dtype=float), base.inertia.copy())
        self._apply_inertias()

    def link_params(self) -> dict[str, dict]:
        return {self.link_name(j): {"mass": round(float(self.link_inertias[j].mass), 6),
                                    "com": np.round(self.link_inertias[j].lever, 6).tolist()}
                for j in self.joint_ids}

    def set_tool(self, tool: ToolConfig) -> None:
        """Attach a tool: TCP frame on the flange + payload inertia. Safe to call again to swap tools."""
        if tool.mount_frame != self.cfg.flange_frame:
            raise ValueError(f"tool mount frame {tool.mount_frame!r} != arm flange {self.cfg.flange_frame!r}")
        self.tool = tool.copy()
        flange = self.model.frames[self.flange_frame_id]
        tcp = pin.SE3(rpy_to_matrix(tool.tcp_rpy), np.asarray(tool.tcp_xyz, dtype=float))
        placement = flange.placement * tcp
        if self.model.existFrame("tcp"):
            fid = self.model.getFrameId("tcp")
            f = self.model.frames[fid]
            f.placement = placement
            self.model.frames[fid] = f
        else:
            fid = self.model.addFrame(pin.Frame("tcp", self.flange_joint, self.flange_frame_id, placement, pin.FrameType.OP_FRAME))
        self.tcp_frame_id = fid
        self._apply_inertias()
        self.data = self.model.createData()

    def payload_inertia(self) -> pin.Inertia:
        """Tool payload as a Pinocchio inertia in the flange joint frame."""
        t = self.tool
        I = np.zeros((3, 3)) if t.inertia is None else np.asarray(t.inertia, dtype=float)
        flange = self.model.frames[self.flange_frame_id].placement
        return flange.act(pin.Inertia(float(t.mass), np.asarray(t.com, dtype=float), I))

    def _apply_inertias(self) -> None:
        for j in self.joint_ids:
            self.model.inertias[j] = pin.Inertia(self.link_inertias[j])
        if self.tool.mass > 0.0:
            self.model.inertias[self.flange_joint] = self.link_inertias[self.flange_joint] + self.payload_inertia()

    def new_data(self) -> pin.Data:
        return self.model.createData()

    # ------------------------------------------------------------------------------------------- kinematics
    def fk(self, q: np.ndarray, data: pin.Data | None = None, frame: int | str | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Pose (p, R) of the TCP (default) or of another frame, in the base frame."""
        d = self.data if data is None else data
        fid = self.tcp_frame_id if frame is None else (self.model.getFrameId(frame) if isinstance(frame, str) else frame)
        pin.forwardKinematics(self.model, d, np.asarray(q, dtype=float))
        T = pin.updateFramePlacement(self.model, d, fid)
        return T.translation.copy(), T.rotation.copy()

    def flange_pose(self, q: np.ndarray, data: pin.Data | None = None) -> tuple[np.ndarray, np.ndarray]:
        return self.fk(q, data, self.flange_frame_id)

    def jacobian(self, q: np.ndarray, data: pin.Data | None = None, frame: int | None = None) -> np.ndarray:
        """6 x n Jacobian of the TCP (or ``frame``): rows = [linear; angular] velocity in the base frame."""
        d = self.data if data is None else data
        fid = self.tcp_frame_id if frame is None else frame
        return pin.computeFrameJacobian(self.model, d, np.asarray(q, dtype=float), fid, LWA).copy()

    def link_positions(self, q: np.ndarray, data: pin.Data | None = None) -> np.ndarray:
        """Origins of all joint frames + the TCP (k x 3), for simple workspace / table-clearance checks."""
        d = self.data if data is None else data
        pin.forwardKinematics(self.model, d, np.asarray(q, dtype=float))
        pts = [d.oMi[j].translation.copy() for j in self.joint_ids]
        pts.append(pin.updateFramePlacement(self.model, d, self.tcp_frame_id).translation.copy())
        return np.array(pts)

    # --------------------------------------------------------------------------------------------- dynamics
    def gravity(self, q: np.ndarray, data: pin.Data | None = None) -> np.ndarray:
        """Joint torques (N*m) that hold the arm + payload against gravity at ``q``."""
        d = self.data if data is None else data
        return pin.computeGeneralizedGravity(self.model, d, np.asarray(q, dtype=float)).copy()

    def mass_matrix(self, q: np.ndarray, data: pin.Data | None = None, rotor: bool = True) -> np.ndarray:
        d = self.data if data is None else data
        M = pin.crba(self.model, d, np.asarray(q, dtype=float)).copy()
        M = np.triu(M) + np.triu(M, 1).T
        if rotor:
            M += np.diag(self.rotor_inertia)
        return M

    def regressor(self, q: np.ndarray, dq: np.ndarray | None = None, ddq: np.ndarray | None = None,
                  data: pin.Data | None = None) -> np.ndarray:
        """Joint torque regressor Y (n x 10 n): ``tau = Y @ params`` with :meth:`dynamic_params` ordering."""
        d = self.data if data is None else data
        z = np.zeros(self.n)
        Y = pin.computeJointTorqueRegressor(self.model, d, np.asarray(q, float), z if dq is None else dq, z if ddq is None else ddq)
        return np.asarray(Y).copy()

    def dynamic_params(self) -> np.ndarray:
        """Stacked per-body dynamic parameters (10 per joint, Pinocchio ``Inertia.toDynamicParameters`` order)."""
        return np.concatenate([self.model.inertias[j].toDynamicParameters() for j in self.joint_ids])

    def static_torque(self, q: np.ndarray, data: pin.Data | None = None) -> np.ndarray:
        """Same as :meth:`gravity`, via the regressor (used to check consistency in tests)."""
        return self.regressor(q, data=data) @ self.dynamic_params()

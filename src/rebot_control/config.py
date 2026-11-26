"""Configuration objects: the arm (hardware, gains, safety) and the tool (TCP + payload).

Both are plain dataclasses loaded from YAML. Angles are stored in **radians** in memory (YAML files use degrees,
see the ``*_deg`` keys); see ``docs/developer/conventions.md`` for every unit and frame convention.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

PACKAGE_DIR = Path(__file__).resolve().parent
ASSET_DIR = PACKAGE_DIR / "assets"
CONFIG_DIR = PACKAGE_DIR / "configs"


def _resolve(name_or_path: str | Path, kind: str) -> Path:
    """A built-in config name (``"b601_rs"``) or a path to a YAML file."""
    p = Path(name_or_path)
    if p.suffix in (".yaml", ".yml"):
        if not p.is_file():
            raise FileNotFoundError(f"{kind} config not found: {p}")
        return p
    builtin = CONFIG_DIR / kind / f"{name_or_path}.yaml"
    if not builtin.is_file():
        available = sorted(f.stem for f in (CONFIG_DIR / kind).glob("*.yaml"))
        raise FileNotFoundError(f"unknown {kind[:-1]} config {name_or_path!r}; built-in: {available}")
    return builtin


def _vec(values, n: int, name: str) -> np.ndarray:
    a = np.asarray(values if np.ndim(values) else [values] * n, dtype=float)
    if a.shape != (n,):
        raise ValueError(f"{name}: expected {n} values, got {a.shape}")
    return a


# ----------------------------------------------------------------------------------------------------------- arm
@dataclass
class MotorConfig:
    can_id: int
    model: str
    feedback_id: int = 0xFD
    direction: float = 1.0  # raw motor angle = direction * joint angle + offset
    offset: float = 0.0  # rad


@dataclass
class BusConfig:
    type: str = "robstride"
    channel: str = "can0"
    feedback_wait_s: float = 0.003


@dataclass
class GainConfig:
    kp: np.ndarray
    kd: np.ndarray
    gravity_mode_kd: np.ndarray
    ramp_s: float = 0.5


@dataclass
class FeedforwardConfig:
    gravity: bool = True
    gravity_scale: float = 1.0
    friction_scale: float = 0.0
    ripple_scale: float = 1.0  # multiplies the calibrated torque ripple model (zero until calibrated)
    tau_max: np.ndarray = field(default_factory=lambda: np.full(6, 10.0))


@dataclass
class SafetyConfig:
    soft_limits: np.ndarray  # (n, 2) rad
    max_jump: float = np.radians(25.0)
    max_joint_speed: float = np.radians(90.0)
    filter_tau_s: float = 0.1
    max_error: float = np.radians(10.0)
    lag: float = np.radians(10.0)
    stall_time_s: float = 1.0
    stall_progress: float = np.radians(1.0)
    on_stall: str = "hold"
    on_overtemp: str = "disable"
    slow_lag: float = np.radians(20.0)
    temp_warn_c: float = 115.0
    temp_stop_c: float = 125.0


@dataclass
class PoseConfig:
    zero: np.ndarray
    unfold_first: list[int] = field(default_factory=lambda: [1, 2])
    fold_stages: list[list[int]] = field(default_factory=lambda: [[0, 3, 4, 5], [1, 2]])
    move_speed: float = np.radians(20.0)


@dataclass
class SimConfig:
    physics_hz: float = 1050.0
    damping: np.ndarray = field(default_factory=lambda: np.full(6, 0.1))
    frictionloss: np.ndarray = field(default_factory=lambda: np.full(6, 0.05))
    torque_noise: float = 0.0


@dataclass
class FrictionParams:
    """Per-joint friction model ``tau = fc * tanh(v / vs) + fv * v + bias`` (fc may differ by direction)."""

    fc_pos: np.ndarray
    fc_neg: np.ndarray
    fv: np.ndarray
    vs: np.ndarray
    bias: np.ndarray

    @classmethod
    def zeros(cls, n: int) -> FrictionParams:
        return cls(np.zeros(n), np.zeros(n), np.zeros(n), np.full(n, 0.02), np.zeros(n))

    def torque(self, v: np.ndarray) -> np.ndarray:
        v = np.asarray(v, dtype=float)
        s = np.tanh(v / self.vs)
        fc = np.where(v >= 0.0, self.fc_pos, self.fc_neg)
        return fc * s + self.fv * v

    def to_dict(self) -> dict:
        return {k: np.round(getattr(self, k), 6).tolist() for k in ("fc_pos", "fc_neg", "fv", "vs", "bias")}

    @classmethod
    def from_dict(cls, d: dict, n: int) -> FrictionParams:
        z = cls.zeros(n)
        return cls(*(_vec(d.get(k, getattr(z, k)), n, k) for k in ("fc_pos", "fc_neg", "fv", "vs", "bias")))


@dataclass
class RippleParams:
    """Torque ripple that repeats with the joint angle: ``tau = a1 sin(N q) + b1 cos(N q) + a2 sin(2 N q) + b2 cos(2 N q)``.

    ``cycles`` (N) is the number of periods per turn of the joint, e.g. pole pairs x gear ratio. The model is the
    extra torque the motor must produce to move evenly; it is fed forward at the **measured** angle. That makes it
    a position feedback of up to ``N * amplitude`` N*m/rad, negative when the phase is wrong, so :meth:`check`
    refuses a model that could cancel more than ``max_share`` of the position gain. The phase belongs to the
    joint's zero: after re-zeroing a motor the model must be calibrated again.
    """

    cycles: np.ndarray
    a1: np.ndarray
    b1: np.ndarray
    a2: np.ndarray
    b2: np.ndarray

    @classmethod
    def zeros(cls, n: int) -> RippleParams:
        return cls(np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n))

    def torque(self, q: np.ndarray) -> np.ndarray:
        x = self.cycles * np.asarray(q, dtype=float)
        return self.a1 * np.sin(x) + self.b1 * np.cos(x) + self.a2 * np.sin(2.0 * x) + self.b2 * np.cos(2.0 * x)

    def amplitude(self) -> np.ndarray:
        """Largest torque of the model per joint (upper bound: both harmonics in phase)."""
        return np.hypot(self.a1, self.b1) + np.hypot(self.a2, self.b2)

    def stiffness(self) -> np.ndarray:
        """Largest ``|d tau / d q|`` per joint (N*m/rad)."""
        return self.cycles * (np.hypot(self.a1, self.b1) + 2.0 * np.hypot(self.a2, self.b2))

    def check(self, kp: np.ndarray, scale: float = 1.0, max_share: float = 0.75) -> None:
        bad = np.flatnonzero(abs(scale) * self.stiffness() > max_share * np.asarray(kp, dtype=float))
        if len(bad):
            raise ValueError(f"torque ripple model too strong for the position gain on joint(s) {(bad + 1).tolist()}: "
                             f"N * amplitude = {np.round(abs(scale) * self.stiffness()[bad], 1).tolist()} N*m/rad, "
                             f"allowed {max_share:g} x kp = {np.round(max_share * np.asarray(kp)[bad], 1).tolist()}")

    def to_dict(self) -> dict:
        return {k: np.round(getattr(self, k), 6).tolist() for k in ("cycles", "a1", "b1", "a2", "b2")}

    @classmethod
    def from_dict(cls, d: dict, n: int) -> RippleParams:
        z = cls.zeros(n)
        return cls(*(_vec(d.get(k, getattr(z, k)), n, k) for k in ("cycles", "a1", "b1", "a2", "b2")))


@dataclass
class ArmConfig:
    name: str
    urdf: Path
    flange_frame: str
    joints: list[str]
    bus: BusConfig
    motors: list[MotorConfig]
    control_hz: float
    servo_hz: float
    gains: GainConfig
    feedforward: FeedforwardConfig
    rotor_inertia: np.ndarray
    safety: SafetyConfig
    poses: PoseConfig
    sim: SimConfig
    friction: FrictionParams
    ripple: RippleParams | None = None  # None: no model (same as zeros)
    source: Path | None = None

    @property
    def n(self) -> int:
        return len(self.joints)

    @classmethod
    def load(cls, name_or_path: str | Path = "b601_rs", overrides: dict | None = None) -> ArmConfig:
        path = _resolve(name_or_path, "arms")
        data = yaml.safe_load(path.read_text())
        if overrides:
            data = _deep_merge(data, overrides)
        return cls.from_dict(data, source=path)

    @classmethod
    def from_dict(cls, d: dict[str, Any], source: Path | None = None) -> ArmConfig:
        joints = list(d["joints"])
        n = len(joints)
        urdf = Path(d["urdf"])
        if not urdf.is_absolute():
            urdf = ASSET_DIR / urdf
        motors = [MotorConfig(can_id=int(m["can_id"]), model=str(m["model"]), feedback_id=int(m.get("feedback_id", 0xFD)),
                              direction=float(m.get("direction", 1.0)), offset=np.radians(float(m.get("offset_deg", 0.0))))
                  for m in d["motors"]]
        if len(motors) != n:
            raise ValueError(f"{len(motors)} motors for {n} joints")
        g = d.get("gains", {})
        gains = GainConfig(kp=_vec(g["kp"], n, "gains.kp"), kd=_vec(g["kd"], n, "gains.kd"),
                           gravity_mode_kd=_vec(g.get("gravity_mode_kd", 0.5), n, "gains.gravity_mode_kd"),
                           ramp_s=float(g.get("ramp_s", 0.5)))
        f = d.get("feedforward", {})
        ff = FeedforwardConfig(gravity=bool(f.get("gravity", True)), gravity_scale=float(f.get("gravity_scale", 1.0)),
                               friction_scale=float(f.get("friction_scale", 0.0)),
                               ripple_scale=float(f.get("ripple_scale", 1.0)),
                               tau_max=_vec(f.get("tau_max", 10.0), n, "feedforward.tau_max"))
        s = d.get("safety", {})
        lim = np.radians(np.asarray(s["soft_limits_deg"], dtype=float))
        if lim.shape != (n, 2):
            raise ValueError(f"safety.soft_limits_deg: expected ({n}, 2), got {lim.shape}")
        on_stall = str(s.get("on_stall", "hold"))
        if on_stall not in ("disable", "hold"):
            raise ValueError("safety.on_stall must be 'disable' or 'hold'")
        on_overtemp = str(s.get("on_overtemp", "disable"))
        if on_overtemp not in ("disable", "hold"):
            raise ValueError("safety.on_overtemp must be 'disable' or 'hold'")
        safety = SafetyConfig(
            soft_limits=lim, max_jump=np.radians(s.get("max_jump_deg", 25.0)),
            max_joint_speed=np.radians(s.get("max_joint_speed_deg_s", 90.0)), filter_tau_s=float(s.get("filter_tau_s", 0.1)),
            max_error=np.radians(s.get("max_error_deg", 10.0)), lag=np.radians(s.get("lag_deg", 10.0)),
            stall_time_s=float(s.get("stall_time_s", 1.0)), stall_progress=np.radians(s.get("stall_progress_deg", 1.0)),
            on_stall=on_stall, on_overtemp=on_overtemp, slow_lag=np.radians(s.get("slow_lag_deg", 20.0)),
            temp_warn_c=float(s.get("temp_warn_c", 115.0)), temp_stop_c=float(s.get("temp_stop_c", 125.0)))
        p = d.get("poses", {})
        poses = PoseConfig(zero=np.radians(_vec(p.get("zero_deg", 0.0), n, "poses.zero_deg")),
                           unfold_first=list(p.get("unfold_first", [1, 2])),
                           fold_stages=[list(st) for st in p.get("fold_stages", [[0, 3, 4, 5], [1, 2]])],
                           move_speed=np.radians(float(p.get("move_speed_deg_s", 20.0))))
        sm = d.get("sim", {})
        sim = SimConfig(physics_hz=float(sm.get("physics_hz", 1050.0)), damping=_vec(sm.get("damping", 0.1), n, "sim.damping"),
                        frictionloss=_vec(sm.get("frictionloss", 0.05), n, "sim.frictionloss"),
                        torque_noise=float(sm.get("torque_noise", 0.0)))
        b = d.get("bus", {})
        bus = BusConfig(type=str(b.get("type", "robstride")), channel=str(b.get("channel", "can0")),
                        feedback_wait_s=float(b.get("feedback_wait_s", 0.003)))
        rates = d.get("rates", {})
        friction = FrictionParams.from_dict(d.get("friction", {}), n)
        ripple = RippleParams.from_dict(d["ripple"], n) if d.get("ripple") else None
        return cls(name=str(d.get("name", "arm")), urdf=urdf, flange_frame=str(d.get("flange_frame", "link6")),
                   joints=joints, bus=bus, motors=motors, control_hz=float(rates.get("control_hz", 30.0)),
                   servo_hz=float(rates.get("servo_hz", 150.0)), gains=gains, feedforward=ff,
                   rotor_inertia=_vec(d.get("rotor_inertia", 0.0), n, "rotor_inertia"), safety=safety, poses=poses,
                   sim=sim, friction=friction, ripple=ripple, source=source)

    def copy(self) -> ArmConfig:
        return copy.deepcopy(self)


def _deep_merge(a: dict, b: dict) -> dict:
    out = dict(a)
    for k, v in b.items():
        out[k] = _deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


# ---------------------------------------------------------------------------------------------------------- tool
@dataclass
class ToolConfig:
    """An end effector: TCP pose and payload, both expressed in ``mount_frame`` (the flange, link6)."""

    name: str = "none"
    mount_frame: str = "link6"
    tcp_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tcp_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))  # rad, URDF fixed-axis roll/pitch/yaw
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    inertia: np.ndarray | None = None  # 3x3 about the COM, or None (point mass)
    home: np.ndarray | None = None  # rad
    cartesian: dict = field(default_factory=dict)  # default CartesianLimits fields
    source: Path | None = None
    extra: dict = field(default_factory=dict)  # unknown keys are kept (e.g. calibration metadata)

    @classmethod
    def load(cls, name_or_path: str | Path = "none") -> ToolConfig:
        path = _resolve(name_or_path, "tools")
        return cls.from_dict(yaml.safe_load(path.read_text()), source=path)

    @classmethod
    def from_dict(cls, d: dict, source: Path | None = None) -> ToolConfig:
        tcp = d.get("tcp", {})
        pl = d.get("payload", {})
        inertia = pl.get("inertia")
        if inertia is not None:
            ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in inertia)
            inertia = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        known = {"name", "mount_frame", "tcp", "payload", "home_deg", "cartesian"}
        return cls(name=str(d.get("name", "tool")), mount_frame=str(d.get("mount_frame", "link6")),
                   tcp_xyz=np.asarray(tcp.get("xyz", [0, 0, 0]), dtype=float),
                   tcp_rpy=np.radians(np.asarray(tcp.get("rpy_deg", [0, 0, 0]), dtype=float)),
                   mass=float(pl.get("mass_kg", 0.0)), com=np.asarray(pl.get("com_xyz", [0, 0, 0]), dtype=float),
                   inertia=inertia, home=None if d.get("home_deg") is None else np.radians(np.asarray(d["home_deg"], float)),
                   cartesian=dict(d.get("cartesian", {})), source=source,
                   extra={k: v for k, v in d.items() if k not in known})

    def to_dict(self) -> dict:
        d: dict[str, Any] = {
            "name": self.name, "mount_frame": self.mount_frame,
            "tcp": {"xyz": np.round(self.tcp_xyz, 6).tolist(), "rpy_deg": np.round(np.degrees(self.tcp_rpy), 4).tolist()},
            "payload": {"mass_kg": round(float(self.mass), 6), "com_xyz": np.round(self.com, 6).tolist()},
        }
        if self.inertia is not None:
            I = self.inertia
            d["payload"]["inertia"] = [round(float(v), 8) for v in (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])]
        if self.home is not None:
            d["home_deg"] = np.round(np.degrees(self.home), 3).tolist()
        if self.cartesian:
            d["cartesian"] = self.cartesian
        d.update(self.extra)
        return d

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(yaml.safe_dump(self.to_dict(), sort_keys=False, default_flow_style=None))

    def copy(self) -> ToolConfig:
        return copy.deepcopy(self)


# --------------------------------------------------------------------------------------------------- calibration
@dataclass
class Calibration:
    """Results of the self-calibration tools for one physical arm (see ``rebot-calib``).

    ``link_params``: identified mass and centre of mass per link (``{link: {"mass": m, "com": [x, y, z]}}``,
    COM in the link frame); ``friction``: :class:`FrictionParams`; ``ripple``: :class:`RippleParams`;
    ``gains``: tuned ``kp`` / ``kd`` (rad units); ``feedforward``: scales that go with these models on this arm
    (``friction_scale``, ``ripple_scale``, ``gravity_scale``), overriding the arm config's.
    """

    link_params: dict[str, dict] = field(default_factory=dict)
    friction: FrictionParams | None = None
    ripple: RippleParams | None = None
    feedforward: dict[str, float] = field(default_factory=dict)
    gains: dict[str, list[float]] = field(default_factory=dict)
    meta: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: str | Path, n: int = 6) -> Calibration:
        d = yaml.safe_load(Path(path).read_text()) or {}
        fr, rp = d.get("friction"), d.get("ripple")
        return cls(link_params=d.get("link_params", {}) or {},
                   friction=FrictionParams.from_dict(fr, n) if fr else None,
                   ripple=RippleParams.from_dict(rp, n) if rp else None,
                   feedforward={k: float(v) for k, v in (d.get("feedforward") or {}).items()},
                   gains=d.get("gains", {}) or {}, meta=d.get("meta", {}) or {})

    def save(self, path: str | Path) -> None:
        d: dict[str, Any] = {"meta": self.meta}
        if self.link_params:
            d["link_params"] = self.link_params
        if self.friction is not None:
            d["friction"] = self.friction.to_dict()
        if self.ripple is not None:
            d["ripple"] = self.ripple.to_dict()
        if self.feedforward:
            d["feedforward"] = dict(self.feedforward)
        if self.gains:
            d["gains"] = self.gains
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(yaml.safe_dump(d, sort_keys=False, default_flow_style=None))

    def merge(self, other: Calibration) -> Calibration:
        """Fields set in ``other`` replace the ones here."""
        return Calibration(link_params=other.link_params or self.link_params,
                           friction=other.friction if other.friction is not None else self.friction,
                           ripple=other.ripple if other.ripple is not None else self.ripple,
                           feedforward={**self.feedforward, **other.feedforward},
                           gains=other.gains or self.gains, meta={**self.meta, **other.meta})

    def apply_to(self, cfg: ArmConfig) -> None:
        """Copy friction and gains into an arm config (link params are applied by the model)."""
        if self.friction is not None:
            cfg.friction = self.friction
        if self.ripple is not None:
            cfg.ripple = self.ripple
        for k, v in self.feedforward.items():
            if k not in ("friction_scale", "ripple_scale", "gravity_scale"):
                raise ValueError(f"calibration.feedforward: unknown key {k!r}")
            setattr(cfg.feedforward, k, float(v))
        if "kp" in self.gains:
            cfg.gains.kp = _vec(self.gains["kp"], cfg.n, "calibration.gains.kp")
        if "kd" in self.gains:
            cfg.gains.kd = _vec(self.gains["kd"], cfg.n, "calibration.gains.kd")

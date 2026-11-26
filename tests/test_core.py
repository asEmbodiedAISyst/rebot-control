"""Pure-model tests (no simulator needed): config loading, poses, kinematics, dynamics, IK, joint filter, limits."""

import math

import numpy as np
import pytest

from rebot_control import ArmConfig, Calibration, CartesianLimits, CommandRejected, DiffIK, RobotModel, ToolConfig
from rebot_control.calib.poses import generate_poses, is_clear
from rebot_control.calib.tcp import solve_pivot
from rebot_control.joint_filter import JointTargetFilter
from rebot_control.pose import (
    apply_delta,
    from_rot6d,
    limit_tilt,
    matrix_to_rotvec,
    matrix_to_rpy,
    pose_delta,
    rot_z,
    rotvec_to_matrix,
    rpy_to_matrix,
    tilt_angle,
    to_rot6d,
    yaw_of,
)


@pytest.fixture(scope="module")
def cfg():
    return ArmConfig.load("b601_rs")


@pytest.fixture(scope="module")
def pusher():
    return ToolConfig.load("pusher_ip17")


def test_config_units(cfg):
    assert cfg.n == 6
    assert cfg.safety.max_jump == pytest.approx(math.radians(25))
    assert cfg.safety.soft_limits[2, 1] == pytest.approx(math.radians(200))
    assert cfg.urdf.is_file()


def test_overrides():
    c = ArmConfig.load("b601_rs", {"gains": {"kp": [10] * 6}})
    assert np.allclose(c.gains.kp, 10)


def test_tool_roundtrip(tmp_path, pusher):
    p = tmp_path / "t.yaml"
    pusher.save(p)
    t2 = ToolConfig.load(p)
    assert np.allclose(t2.tcp_xyz, pusher.tcp_xyz)
    assert np.allclose(t2.tcp_rpy, pusher.tcp_rpy)
    assert np.allclose(t2.home, pusher.home)
    assert t2.cartesian == pusher.cartesian


def test_calibration_roundtrip(tmp_path, cfg):
    cal = Calibration(link_params={"link2": {"mass": 1.6, "com": [0.0, 0.0, 0.0]}}, gains={"kp": [1, 2, 3, 4, 5, 6]})
    p = tmp_path / "c.yaml"
    cal.save(p)
    c2 = Calibration.load(p)
    assert c2.link_params["link2"]["mass"] == 1.6
    c = cfg.copy()
    c2.apply_to(c)
    assert np.allclose(c.gains.kp, [1, 2, 3, 4, 5, 6])


def test_pose_helpers():
    rng = np.random.default_rng(0)
    for _ in range(20):
        r = rng.normal(size=3)
        r *= min(1.0, 3.0 / np.linalg.norm(r))
        R = rotvec_to_matrix(r)
        assert np.allclose(matrix_to_rotvec(R), r, atol=1e-8)
        assert np.allclose(rpy_to_matrix(matrix_to_rpy(R)), R, atol=1e-8)
        assert np.allclose(from_rot6d(to_rot6d(R)), R)
        p0, p1 = rng.normal(size=3), rng.normal(size=3)
        R1 = rotvec_to_matrix(rng.normal(size=3) * 0.5)
        p2, R2 = apply_delta(p0, R, pose_delta(p0, R, p1, R1))
        assert np.allclose(p2, p1) and np.allclose(R2, R1)
    down = np.diag([1.0, -1.0, -1.0])  # z axis pointing down
    assert tilt_angle(down) == pytest.approx(0.0)
    tilted = rotvec_to_matrix([0.0, math.radians(50), 0.0]) @ down
    assert tilt_angle(limit_tilt(tilted, math.radians(30))) == pytest.approx(math.radians(30))
    assert yaw_of(rot_z(0.4) @ down) == pytest.approx(0.4)


def test_pusher_home_fk(cfg, pusher):
    m = RobotModel(cfg, pusher)
    p, R = m.fk(pusher.home)
    assert np.allclose(p, [0.22, 0.0, 0.10], atol=2e-3)
    assert math.degrees(tilt_angle(R)) < 1.0  # rod straight down


def test_jacobian_matches_finite_differences(cfg, pusher):
    m = RobotModel(cfg, pusher)
    q = pusher.home + 0.1
    J = m.jacobian(q)
    p0, R0 = m.fk(q)
    eps = 1e-6
    for i in range(m.n):
        dq = np.zeros(m.n)
        dq[i] = eps
        p1, R1 = m.fk(q + dq)
        assert np.allclose((p1 - p0) / eps, J[:3, i], atol=1e-5)
        assert np.allclose(matrix_to_rotvec(R1 @ R0.T) / eps, J[3:, i], atol=1e-5)


def test_regressor_matches_gravity(cfg, pusher):
    m = RobotModel(cfg, pusher)
    rng = np.random.default_rng(1)
    for _ in range(5):
        q = rng.uniform(m.lower, m.upper)
        assert np.allclose(m.static_torque(q), m.gravity(q), atol=1e-9)


def test_payload_adds_gravity(cfg, pusher):
    bare, tooled = RobotModel(cfg), RobotModel(cfg, pusher)
    q = pusher.home
    assert np.abs(tooled.gravity(q) - bare.gravity(q)).max() > 0.2
    tooled.set_tool(ToolConfig.load("none"))
    assert np.allclose(tooled.gravity(q), bare.gravity(q))


def test_link_params_roundtrip(cfg):
    m = RobotModel(cfg)
    lp = m.link_params()
    lp["link3"]["mass"] *= 1.5
    m.set_link_params(lp)
    assert m.link_params()["link3"]["mass"] == pytest.approx(lp["link3"]["mass"], rel=1e-5)


def test_calibration_with_link_params_loads(tmp_path, cfg, pusher):
    """A saved gravity calibration must load into a model (and an Arm) and change its gravity torques."""
    from rebot_control import Arm, Calibration

    base = RobotModel(cfg, pusher)
    params = base.link_params()
    params["link3"]["mass"] *= 1.5
    path = tmp_path / "cal.yaml"
    Calibration(link_params=params).save(path)
    cal = Calibration.load(path, cfg.n)
    q = pusher.home
    assert not np.allclose(RobotModel(cfg, pusher, cal).gravity(q), base.gravity(q))
    arm = Arm.from_config("b601_rs", tool="pusher_ip17", calibration=str(path), backend="sim")
    assert np.allclose(arm.model.gravity(q), RobotModel(cfg, pusher, cal).gravity(q))
    assert arm.model.link_params()["link3"]["mass"] == pytest.approx(params["link3"]["mass"])


def test_gravity_fit_without_bias(cfg, pusher):
    from rebot_control.calib.gravity import fit_gravity

    model = RobotModel(cfg, pusher)
    rng = np.random.default_rng(0)
    qs = rng.uniform(model.lower, model.upper, size=(30, model.n))
    data = {"q": qs, "tau": np.array([model.gravity(q) for q in qs])}
    for fit_bias in (True, False):
        fit = fit_gravity(model, data, fit_bias=fit_bias)
        assert fit.rms_after.max() < 1e-6 and np.abs(fit.bias).max() < 1e-6


def test_ik_reaches_fk_targets(cfg, pusher):
    m = RobotModel(cfg, pusher)
    ik = DiffIK(m, posture=pusher.home)
    rng = np.random.default_rng(2)
    for _ in range(10):
        q_t = pusher.home + rng.uniform(-0.3, 0.3, m.n)
        q_t = np.clip(q_t, m.lower, m.upper)
        p, R = m.fk(q_t)
        res = ik.solve(p, R, pusher.home, max_iters=300)
        assert res.success, (res.pos_err, res.rot_err)
        assert res.pos_err < 1e-3


def test_joint_filter_limits(cfg):
    f = JointTargetFilter(np.full(6, -3.0), np.full(6, 3.0), 1 / 30, math.radians(25), math.radians(90), 0.1)
    q0 = np.zeros(6)
    out = f(q0 + math.radians(10), q0)
    assert np.all(f.q_prev <= math.radians(3) + 1e-12)  # 90 deg/s at 30 Hz
    assert np.all(out < f.q_prev)  # low-pass behind the rate-limited target
    with pytest.raises(CommandRejected):
        f(q0 + math.radians(40), q0)
    g = JointTargetFilter(np.full(6, -0.05), np.full(6, 0.05), 1 / 30, math.radians(25), math.radians(90), 0.0)
    for _ in range(5):
        out = g(np.full(6, 0.3), q0)  # beyond the soft limit: clipped
    assert np.allclose(out, 0.05)


def test_cartesian_limits():
    lim = CartesianLimits.from_dict({"x": [0.2, 0.5], "z": [0.0, 0.2], "max_tilt_deg": 30.0, "max_yaw_deg": 45})
    assert np.allclose(lim.clip_position([0.0, 0.0, 1.0]), [0.2, 0.0, 0.2])
    down = np.diag([1.0, -1.0, -1.0])
    R = rot_z(math.radians(90)) @ rotvec_to_matrix([0.0, math.radians(60), 0.0]) @ down
    Rl = lim.clip_orientation(R)
    assert math.degrees(tilt_angle(Rl)) <= 30.0 + 1e-6
    assert abs(math.degrees(yaw_of(Rl))) <= 45.0 + 1e-6
    with pytest.raises(ValueError):
        CartesianLimits.from_dict({"speed": 1})


def test_calibration_poses_are_clear(cfg, pusher):
    m = RobotModel(cfg, pusher)
    poses = generate_poses(m, 15, pusher.home, np.random.default_rng(0))
    assert len(poses) >= 12
    assert all(is_clear(m, q) for q in poses)
    assert not is_clear(m, np.radians([0, 120, 10, 0, 0, 0]))  # reaching down into the table


def test_pivot_solution():
    rng = np.random.default_rng(3)
    t, c = np.array([-0.09, 0.01, 0.17]), np.array([0.3, 0.05, 0.01])
    R = np.array([rotvec_to_matrix(rng.normal(size=3) * 0.4) for _ in range(8)])
    P = np.array([c - Ri @ t for Ri in R])
    fit = solve_pivot(P, R)
    assert np.allclose(fit.tcp_xyz, t, atol=1e-9) and np.allclose(fit.pivot, c, atol=1e-9)
    with pytest.raises(ValueError):
        solve_pivot(P[:4] * 0 + P[0], np.array([R[0]] * 4))


def test_calibration_carries_feedforward_scales(tmp_path, cfg):
    from rebot_control import FrictionParams

    fr = FrictionParams.zeros(cfg.n)
    fr.fc_pos[:] = fr.fc_neg[:] = 0.3
    Calibration(friction=fr, feedforward={"friction_scale": 0.7}).save(tmp_path / "c.yaml")
    cal = Calibration.load(tmp_path / "c.yaml")
    c = cfg.copy()
    cal.apply_to(c)
    assert c.feedforward.friction_scale == 0.7 and c.friction.fc_pos[1] == 0.3
    with pytest.raises(ValueError, match="unknown key"):
        Calibration(feedforward={"typo_scale": 1.0}).apply_to(cfg.copy())

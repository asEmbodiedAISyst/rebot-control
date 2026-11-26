"""Closed-loop tests in the MuJoCo simulator: servo, Arm life cycle, Cartesian control, safety, calibration."""

import math

import numpy as np
import pytest

pytest.importorskip("mujoco")

from rebot_control import Arm, ArmConfig, Calibration, RippleParams, RobotModel, SafetyStop, ServoMode, ToolConfig  # noqa: E402
from rebot_control.calib.friction import collect_sweeps, fit_joint_friction  # noqa: E402
from rebot_control.calib.gains import step_response  # noqa: E402
from rebot_control.calib.gravity import collect_static, fit_gravity, fit_payload  # noqa: E402
from rebot_control.calib.poses import generate_poses  # noqa: E402
from rebot_control.calib.ripple import fit_joint_ripple  # noqa: E402


def make_arm(tool="pusher_ip17", **kw):
    return Arm.from_config("b601_rs", tool=tool, backend="sim", **kw)


def test_sim_matches_model():
    arm = make_arm()
    with arm:
        d, m = arm.backend.mj_data, arm.backend.mj_model
        import mujoco

        q = arm.tool.home
        d.qpos[:] = q
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        assert np.allclose(d.qfrc_bias, arm.model.gravity(q), atol=1e-6)


def test_enable_hold_and_fold():
    arm = make_arm()
    with arm:
        arm.enable()
        assert arm.servo.mode == ServoMode.HOLD
        arm.move_home()
        assert np.abs(arm.q() - arm.tool.home).max() < math.radians(0.3)
    assert not arm.connected
    assert np.abs(arm.servo.state.q - arm.cfg.poses.zero).max() < math.radians(1.0)


def test_cartesian_move_and_limits():
    arm = make_arm()
    with arm:
        arm.enable()
        arm.move_home()
        p, R = arm.tcp_pose()
        assert arm.move_to_pose(p + [0.08, 0.05, 0.02], R)
        p2, _ = arm.tcp_pose()
        assert np.linalg.norm(p2 - (p + [0.08, 0.05, 0.02])) < 2e-3
        cart = arm.cartesian()
        st = cart.step(p2 + [0.0, 0.0, -1.0], R)  # below the table: clipped to z_min, speed limited
        assert st.clipped
        assert np.linalg.norm(st.pos - p2) <= cart.limits.max_speed * arm.dt + 1e-9


def test_speed_limit_is_respected():
    arm = make_arm()
    with arm:
        arm.enable()
        arm.move_home()
        cart = arm.cartesian()
        p0, R0 = cart.pose()
        prev = p0
        for _ in range(30):
            st = cart.step(p0 + [0.3, 0.0, 0.0], R0)
            arm.sleep()
            assert np.linalg.norm(st.pos - prev) <= cart.limits.max_speed * arm.dt + 1e-9
            prev = st.pos


def test_gravity_mode_floats():
    arm = make_arm()
    with arm:
        arm.enable()
        arm.move_home()
        arm.gravity_mode()
        q0 = arm.q()
        arm.sleep(2.0)
        assert np.abs(arm.q() - q0).max() < math.radians(1.0)
        arm.hold()


def _block_shoulder_and_stall(arm):
    arm.enable()
    arm.move_home()
    arm.backend.effort[1] = 0.5  # block joint 2 by making the simulated motor too weak to move
    q = arm.tool.home.copy()
    q[1] += math.radians(30)
    arm.move_to_joint(q, speed=math.radians(30))


def test_stall_holds_by_default():
    arm = make_arm()
    assert arm.cfg.safety.on_stall == "hold"
    with pytest.raises(SafetyStop):
        with arm:
            _block_shoulder_and_stall(arm)
    assert arm.enabled and arm.backend.enabled  # still holding, not dropped
    assert not arm.connected


def test_stall_can_disable():
    arm = make_arm(overrides={"safety": {"on_stall": "disable"}})
    with pytest.raises(SafetyStop):
        with arm:
            _block_shoulder_and_stall(arm)
    assert not arm.enabled and not arm.backend.enabled


def _run_physics(backend, seconds):
    """What happens after the controlling program is gone: only the motors and physics run."""
    for _ in range(int(seconds * 150)):
        backend.advance(1.0 / 150)


def _crashed_arm(exc=RuntimeError):
    """An arm left in mid-air, mid-move, by a program that died with ``exc``."""
    arm = make_arm()
    q_mid = None
    with pytest.raises(exc):
        with arm:
            arm.enable()
            arm.move_home()
            target = arm.tool.home.copy()
            target[[0, 2]] += np.radians([20.0, 25.0])
            for i in range(1, 21):  # stop half way through a move, while the joints are moving
                arm.command_joint(arm.tool.home + i / 40.0 * (target - arm.tool.home))
                arm.sleep()
            q_mid = arm.q().copy()
            raise exc("program failed")
    return arm, q_mid


@pytest.mark.parametrize("exc", [RuntimeError, KeyboardInterrupt])
def test_abnormal_exit_freezes(exc):
    arm, q_mid = _crashed_arm(exc)
    backend = arm.backend
    assert backend.enabled, "torque must stay on after an abnormal exit"
    assert not arm.connected
    _run_physics(backend, 3.0)
    q = backend.mj_data.qpos
    assert np.abs(q - q_mid).max() < math.radians(1.0), "the arm must hold where the program died"
    assert np.abs(backend.mj_data.qvel).max() < math.radians(0.5)
    assert q[1] > math.radians(15), "it must not have folded or fallen"


def test_next_program_takes_over_without_a_jump():
    arm, _ = _crashed_arm()
    backend = arm.backend
    _run_physics(backend, 1.0)
    q_held = backend.mj_data.qpos.copy()

    arm2 = Arm(arm.cfg, arm.tool, backend=backend)
    with arm2:
        assert arm2.taken_over and arm2.enabled and backend.enabled
        assert arm2.servo.mode == ServoMode.HOLD
        with arm2.recording() as rec:
            arm2.sleep(1.0)
        assert np.abs(rec["q"] - q_held).max() < math.radians(0.3), "taking over must not move the arm"
        arm2.enable()  # the new program takes command
        arm2.move_to_joint(arm2.tool.home)
        assert np.abs(arm2.q() - arm2.tool.home).max() < math.radians(0.3)
    assert not backend.enabled  # a normal end: folded, torque off
    assert np.abs(backend.mj_data.qpos - arm2.cfg.poses.zero).max() < math.radians(1.0)


def test_repeated_takeovers_do_not_creep():
    """With a gravity model that is off, the arm sits away from its target. Taking over must not add that error."""
    heavy = ToolConfig.load("pusher_ip17")
    heavy.mass = 3 * heavy.mass  # the real tool is heavier than the controller thinks
    arm = make_arm(truth_tool=heavy)
    with pytest.raises(RuntimeError):
        with arm:
            arm.enable()
            arm.move_home()
            arm.sleep(1.0)
            assert np.abs(arm.q() - arm.tool.home).max() > math.radians(0.2), "the test needs a visible static error"
            raise RuntimeError("program failed")
    backend = arm.backend
    _run_physics(backend, 1.0)
    q_held = backend.mj_data.qpos.copy()
    for _ in range(5):  # five programs look at the arm, one after the other
        watcher = Arm(arm.cfg, arm.tool, backend=backend)
        with watcher:
            watcher.sleep(1.0)
        _run_physics(backend, 0.5)
    assert np.abs(backend.mj_data.qpos - q_held).max() < math.radians(0.1)


def test_looking_at_a_held_arm_leaves_it_held():
    arm, _ = _crashed_arm()
    backend = arm.backend
    _run_physics(backend, 1.0)
    q_held = backend.mj_data.qpos.copy()
    watcher = Arm(arm.cfg, arm.tool, backend=backend)
    with watcher:  # e.g. `rebot-control state`: reads, commands nothing
        assert watcher.taken_over
        for _ in range(10):
            watcher.sleep(0.1)
            watcher.state()
    assert backend.enabled, "a program that only looks must not switch torque off"
    _run_physics(backend, 2.0)
    assert np.abs(backend.mj_data.qpos - q_held).max() < math.radians(0.5)


def test_operator_release():
    arm, _ = _crashed_arm()
    backend = arm.backend
    op = Arm(arm.cfg, arm.tool, backend=backend)
    with op:
        assert op.taken_over
        op.release(fold=True)  # fold first, then torque off
        assert not op.enabled and not backend.enabled
        assert np.abs(op.q() - op.cfg.poses.zero).max() < math.radians(1.0)

    arm, _ = _crashed_arm()
    op = Arm(arm.cfg, arm.tool, backend=arm.backend)
    with op:
        op.release()  # torque off where it is (the operator supports the arm)
        assert not op.enabled and not arm.backend.enabled
        assert op.servo.mode == ServoMode.PASSIVE


def test_connect_does_not_disturb_a_disabled_arm():
    arm = make_arm()
    with arm:
        assert not arm.taken_over and not arm.enabled
        assert arm.servo.mode == ServoMode.PASSIVE
        assert not arm.backend.enabled


def test_overtemperature_policy():
    for policy, torque_on in (("disable", False), ("hold", True)):
        arm = make_arm(overrides={"safety": {"on_overtemp": policy}})
        with pytest.raises(SafetyStop):
            with arm:
                arm.enable()
                arm.move_home()
                arm.backend.temperature = 200.0  # a driver overheats while the arm is unfolded
                arm.sleep(0.5)
        assert arm.backend.enabled is torque_on
        if torque_on:
            _run_physics(arm.backend, 2.0)
            assert np.abs(arm.backend.mj_data.qpos - arm.tool.home).max() < math.radians(1.0)


def test_scene_hook_adds_objects_without_disturbing_the_arm():
    """A scene with a floor and a free box: the arm still controls its own joints, and the box obeys physics."""
    import mujoco

    def scene(spec):
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.1], pos=[0, 0, 0])
        box = spec.worldbody.add_body(name="box", pos=[0.4, 0.0, 0.2])
        box.add_freejoint()
        box.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.02, 0.02, 0.02])
        # the arm's spec takes inertias from the bodies, not from geoms: give them explicitly
        box.mass, box.ipos, box.inertia, box.explicitinertial = 0.05, [0, 0, 0], [1.3e-5, 1.3e-5, 1.3e-5], True

    arm = make_arm(scene=scene)
    with arm:
        assert arm.backend.mj_model.nq == arm.n + 7
        arm.enable()
        arm.move_home()
        assert np.abs(arm.q() - arm.tool.home).max() < math.radians(0.3)
        d = arm.backend.mj_data
        assert abs(d.qpos[arm.n + 2] - 0.02) < 2e-3, "the box must have fallen onto the floor and stopped"
        assert np.abs(d.qpos[arm.n:arm.n + 2] - [0.4, 0.0]).max() < 5e-3


def test_jump_is_rejected():
    from rebot_control import CommandRejected

    arm = make_arm()
    with arm:
        arm.enable()
        with pytest.raises(CommandRejected):
            arm.command_joint(arm.q() + math.radians(40))


def test_gravity_and_payload_identification():
    cfg, tool = ArmConfig.load("b601_rs"), ToolConfig.load("pusher_ip17")
    truth = RobotModel(cfg, tool).link_params()
    truth["link2"]["mass"] *= 1.2
    truth["link3"]["com"][0] += 0.02
    arm = Arm(cfg, tool, backend="sim", truth_link_params=truth)
    with arm:
        arm.enable()
        arm.move_home()
        data = collect_static(arm, generate_poses(arm.model, 15, arm.q(), np.random.default_rng(0)),
                              progress=lambda s: None)
    fit = fit_gravity(arm.model, data)
    assert np.all(fit.rms_after[1:3] < 0.3 * fit.rms_before[1:3])
    m = RobotModel(cfg, tool)
    m.set_link_params(truth)
    pf = fit_payload(m, data)
    assert pf.mass == pytest.approx(tool.mass, rel=0.05)


def test_friction_identification():
    truth = {"damping": [0.1, 0.1, 0.1, 0.12, 0.1, 0.06], "frictionloss": [0.05, 0.05, 0.05, 0.1, 0.05, 0.04]}
    arm = make_arm(truth_friction=truth)
    with arm:
        arm.enable()
        arm.move_home()
        center = arm.tool.home + np.radians([0, 15, 25, 0, 0, 0])
        arm.move_to_joint(center)
        d = collect_sweeps(arm, 3, center, math.radians(30), tuple(np.radians([10, 30, 60])), progress=lambda s: None)
    f = fit_joint_friction(arm.model, d)
    assert f.fc_pos == pytest.approx(0.1, abs=0.02)
    assert f.fv == pytest.approx(0.12, abs=0.03)


def test_step_response_metrics():
    arm = make_arm()
    with arm:
        arm.enable()
        arm.move_home()
        ms = step_response(arm, 1)
    assert len(ms) == 2
    for m in ms:
        assert m.overshoot < 0.1
        assert m.settle_s < 0.5
        assert abs(m.ss_error_deg) < 0.1


RIPPLE_TRUTH = {"cycles": [0, 126, 126, 0, 0, 0], "a1": [0, 0.30, -0.25, 0, 0, 0], "b1": [0, -0.20, 0.40, 0, 0, 0]}


def _ripple_sweep(arm, joint):
    center = arm.tool.home + np.radians([0, 15, 25, 0, 0, 0])
    arm.move_to_joint(center)
    return collect_sweeps(arm, joint, center, math.radians(12), tuple(np.radians([3, 6])), progress=lambda s: None)


def test_ripple_identification():
    arm = make_arm(truth_ripple=RIPPLE_TRUTH)
    with arm:
        arm.enable()
        arm.move_home()
        data = {j: _ripple_sweep(arm, j) for j in (1, 2)}
    for j in (1, 2):
        f = fit_joint_ripple(arm.model, data[j])
        assert f.cycles == 126
        assert f.a1 == pytest.approx(RIPPLE_TRUTH["a1"][j], abs=0.06)
        assert f.b1 == pytest.approx(RIPPLE_TRUTH["b1"][j], abs=0.06)
        assert f.explained > 0.7


def test_ripple_feedforward_smooths_slow_motion():
    def speed_ripple(calibration):
        arm = make_arm(truth_ripple=RIPPLE_TRUTH, calibration=calibration)
        with arm:
            arm.enable()
            arm.move_home()
            d = _ripple_sweep(arm, 2)
        v, v_nom = d["dq"][:, 2], d["v_nom"]
        steady = np.abs(v_nom) > 0
        steady[1:] &= np.abs(np.diff(v_nom)) < 1e-9          # constant commanded speed
        return float(np.std((v - v_nom)[steady]))            # the motor torque still ripples: now the feedforward makes it

    rp = RippleParams.from_dict(RIPPLE_TRUTH, 6)
    without = speed_ripple(None)
    with_ff = speed_ripple(Calibration(ripple=rp))
    assert with_ff < 0.4 * without


def test_ripple_model_cannot_cancel_the_position_gain():
    rp = RippleParams.from_dict({"cycles": [0, 126, 0, 0, 0, 0], "a1": [0, 1.0, 0, 0, 0, 0]}, 6)   # 126 N*m/rad > 0.75 * 120
    with pytest.raises(ValueError, match="too strong"):
        make_arm(calibration=Calibration(ripple=rp))

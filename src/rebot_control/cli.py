"""``rebot-control`` command line: bring-up checks, simple motions and the calibration tools.

Every command works on the real arm (default) and in simulation (``--sim``). Commands that move the arm ask for
confirmation on the real arm unless ``--yes`` is given. Run ``rebot-control <command> -h`` for the options.

Bring-up order on a new arm (``docs/user/hardware-test-plan.md``)::

    rebot-control state                  # torque off: angles, temperatures, TCP pose
    rebot-control hold --seconds 10      # torque on at the folded pose, gravity feedforward
    rebot-control float --seconds 30     # gravity mode: move the arm by hand
    rebot-control home                   # unfold to the tool's home pose and fold back
    rebot-control square                 # Cartesian square at the home height
    rebot-control calib gains --check    # step responses with the configured gains
    rebot-control calib gravity          # static poses -> link masses / COM
    rebot-control calib friction         # velocity sweeps -> Coulomb + viscous friction
    rebot-control calib payload          # tool mass / COM (after gravity)
    rebot-control calib tcp              # pivot calibration of the tool tip

Torque goes off only at the normal end of a command (after folding) or with ``rebot-control release``. A command
that fails or is interrupted leaves the arm holding where it is, enabled; the next command takes that hold over.
``rebot-control state`` shows whether the motors are enabled and changes nothing.

    rebot-control release                # torque off now: support the arm
    rebot-control release --fold         # fold to the zero pose, then torque off
"""

from __future__ import annotations

import argparse
import datetime as _dt
import logging
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np

from .arm import Arm
from .config import Calibration

log = logging.getLogger("rebot_control")


# ---------------------------------------------------------------------------------------------------- helpers
def _deg(a) -> str:
    return np.array2string(np.degrees(np.asarray(a)), precision=1, suppress_small=True, floatmode="fixed")


def _make_arm(args) -> Arm:
    kw = {}
    if args.sim and getattr(args, "viewer", False):
        kw["viewer"] = True
    return Arm.from_config(args.arm, tool=args.tool, calibration=args.calibration, backend="sim" if args.sim else "real",
                           **kw)


def _confirm(args, what: str) -> None:
    if args.sim or args.yes:
        return
    ans = input(f"{what}\nWorkspace clear, emergency stop / power switch within reach? [y/N] ")
    if ans.strip().lower() not in ("y", "yes"):
        print("aborted")
        sys.exit(1)


def _out_path(args, default: str) -> Path:
    return Path(args.out) if getattr(args, "out", None) else Path(default)


def _save_calibration(args, update: Calibration) -> Path:
    path = _out_path(args, f"calibration/{Path(args.arm).stem}.yaml")
    base = Calibration.load(path) if path.is_file() else (Calibration.load(args.calibration) if args.calibration else Calibration())
    merged = base.merge(update)
    merged.save(path)
    print(f"saved {path}")
    return path


def _save_data(args, name: str, data: dict) -> None:
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    d = Path(getattr(args, "data_dir", None) or "calibration/data")
    d.mkdir(parents=True, exist_ok=True)
    p = d / f"{name}-{stamp}{'-sim' if args.sim else ''}.npz"
    np.savez(p, **data)
    print(f"raw data: {p}")


def _meta(args, kind: str) -> dict:
    return {f"{kind}_date": _dt.datetime.now().isoformat(timespec="seconds"), f"{kind}_tool": str(args.tool),
            f"{kind}_backend": "sim" if args.sim else "real"}


# --------------------------------------------------------------------------------------------------- commands
def cmd_state(args) -> None:
    arm = _make_arm(args)
    with arm:
        for _ in range(args.repeat if args.watch else 1):
            st = arm.state()
            p, R = arm.tcp_pose(st.q)
            print(f"q    [deg] {_deg(st.q)}")
            print(f"dq [deg/s] {_deg(st.dq)}")
            print(f"tau  [N*m] {np.array2string(st.tau, precision=2, suppress_small=True)}")
            print(f"temp   [C] {np.array2string(st.temp, precision=0)}")
            print(f"gravity model at q [N*m] {np.array2string(arm.model.gravity(st.q), precision=2, suppress_small=True)}")
            print(f"TCP ({arm.tool.name}) p = {np.round(p, 4).tolist()} m, z axis = {np.round(R[:, 2], 3).tolist()}")
            print(f"folded: {arm.is_folded()}")
            on = [arm.cfg.joints[i] for i in range(arm.n) if st.enabled[i]]
            print("torque: " + ("off" if not on else "ON, holding (left as found)" if len(on) == arm.n
                                else f"ON only on {on} (left as found)"))
            if args.watch:
                arm.sleep(0.5)
                print()


def cmd_release(args) -> None:
    arm = _make_arm(args)
    with arm:
        if not arm.enabled:
            print("torque is already off on every motor")
            return
        if args.fold:
            _confirm(args, "The arm will fold to its zero pose from where it is, then torque goes off.")
        else:
            _confirm(args, "Torque goes off NOW and the arm goes limp where it is. Hold the arm with your hand first.")
        arm.release(fold=args.fold)
        print("torque off")


def cmd_hold(args) -> None:
    arm = _make_arm(args)
    _confirm(args, "The arm will switch torque on and hold its current pose.")
    with arm:
        arm.enable()
        t_end = arm.now() + args.seconds
        print("holding; Ctrl-C to stop")
        try:
            while arm.now() < t_end:
                arm.wait(1.0)
                st = arm.state()
                print(f"q {_deg(st.q)} deg | lag {math.degrees(arm.lag()):.2f} deg | tau {np.round(st.tau, 2).tolist()} | "
                      f"temp max {st.temp.max():.0f} C | servo overruns {arm.servo.overruns}")
        except KeyboardInterrupt:
            pass


def cmd_float(args) -> None:
    arm = _make_arm(args)
    _confirm(args, "Gravity mode: the arm floats and can be moved by hand. If it drifts, the gravity model is off.")
    with arm:
        arm.enable()
        arm.gravity_mode()
        q0 = arm.q()
        print("gravity mode; Ctrl-C to stop (the arm then holds and folds back)")
        try:
            t_end = arm.now() + args.seconds
            while arm.now() < t_end:
                arm.sleep(1.0)
                q = arm.q()
                print(f"q {_deg(q)} deg | drift {_deg(q - q0)} deg")
        except KeyboardInterrupt:
            pass
        arm.hold()


def cmd_home(args) -> None:
    arm = _make_arm(args)
    _confirm(args, f"The arm will unfold to the home pose of tool {arm.tool.name!r} and fold back.")
    with arm:
        arm.enable()
        arm.move_home()
        p, R = arm.tcp_pose()
        print(f"home: q {_deg(arm.q())} deg, TCP {np.round(p, 4).tolist()} m")
        arm.wait(args.seconds)


def cmd_square(args) -> None:
    arm = _make_arm(args)
    _confirm(args, f"Cartesian square, side {args.side * 100:.0f} cm at {args.speed * 100:.0f} cm/s around the home pose.")
    with arm:
        arm.enable()
        arm.move_home()
        cart = arm.cartesian()
        cart.limits.max_speed = min(cart.limits.max_speed, args.speed)
        p0, R0 = cart.pose()
        s = args.side
        corners = [p0 + [s, 0, 0], p0 + [s, s / 2, 0], p0 + [0, s / 2, 0], p0 + [0, -s / 2, 0], p0 + [s, -s / 2, 0],
                   p0 + [s, 0, 0], p0]
        errs = []
        for c in corners:
            c = cart.limits.clip_position(np.asarray(c))
            for _ in range(int(10 / arm.dt)):
                st = cart.step(c, R0)
                arm.sleep()
                errs.append(np.linalg.norm(arm.tcp_pose()[0] - st.pos))
                if not st.ik_ok:
                    print("IK refused a step (out of reach)")
                if np.linalg.norm(st.pos - c) < 1e-4:
                    break
            arm.wait(0.3)
            print(f"corner {np.round(c, 3).tolist()}: TCP {np.round(arm.tcp_pose()[0], 4).tolist()}")
        e = np.array(errs) * 1000
        print(f"tracking error (measured TCP vs command) mean {e.mean():.1f} mm, max {e.max():.1f} mm")


def _calib_poses(args, arm):
    from .calib.poses import generate_poses

    return generate_poses(arm.model, args.poses, arm.q(), np.random.default_rng(args.seed), table_z=args.table_z)


def cmd_calib_gravity(args) -> None:
    from .calib.gravity import collect_static, fit_gravity

    if args.from_data:
        data = dict(np.load(args.from_data))
        arm = _make_arm(args)  # model only, nothing connects
    else:
        arm = _make_arm(args)
        _confirm(args, f"Gravity calibration: {args.poses} static poses (about {args.poses * 10 / 60:.0f} min).")
        with arm:
            arm.enable()
            arm.move_home()
            poses = _calib_poses(args, arm)
            print(f"{len(poses)} poses (deg):\n{np.round(np.degrees(poses), 0)}")
            data = collect_static(arm, poses, settle_s=args.settle)
            arm.move_to_joint(arm.tool.home)
        _save_data(args, "gravity", data)
    fit = fit_gravity(arm.model, data, prior_rel=args.prior_rel)
    print(fit.summary(arm.cfg.joints))
    print(fit.info)
    _save_calibration(args, Calibration(link_params=fit.link_params,
                                        meta=_meta(args, "gravity") | {"gravity_bias": np.round(fit.bias, 4).tolist(),
                                                                       "gravity_rms_after": np.round(fit.rms_after, 4).tolist()}))


def cmd_calib_payload(args) -> None:
    from .calib.gravity import collect_static, fit_payload

    if args.from_data:
        data = dict(np.load(args.from_data))
        arm = _make_arm(args)
    else:
        arm = _make_arm(args)
        _confirm(args, f"Payload calibration: {args.poses} static poses with the tool mounted.")
        with arm:
            arm.enable()
            arm.move_home()
            poses = _calib_poses(args, arm)
            data = collect_static(arm, poses, settle_s=args.settle)
            arm.move_to_joint(arm.tool.home)
        _save_data(args, "payload", data)
    fit = fit_payload(arm.model, data)
    print(fit.summary())
    tool = arm.tool.copy()
    tool.mass, tool.com = fit.mass, fit.com
    tool.extra = dict(tool.extra) | {"calibrated": {"payload": _dt.date.today().isoformat()}}
    out = _out_path(args, f"{tool.name}.yaml")
    tool.save(out)
    print(f"saved tool config {out} (use it with --tool {out})")


def cmd_calib_friction(args) -> None:
    from .calib.friction import DEFAULT_AMPLITUDE_DEG, DEFAULT_SPEEDS_DEG, collect_sweeps, fit_joint_friction, to_friction_params

    arm = _make_arm(args)
    joints = [int(j) - 1 for j in args.joints.split(",")] if args.joints else list(range(arm.n))
    speeds = tuple(math.radians(float(v)) for v in (args.speeds.split(",") if args.speeds else DEFAULT_SPEEDS_DEG))
    _confirm(args, f"Friction calibration: joints {[j + 1 for j in joints]} sweep back and forth at {np.degrees(speeds).round(0).tolist()} deg/s.")
    fits = []
    with arm:
        arm.enable()
        arm.move_home()
        center = arm.tool.home.copy() if args.center is None else np.radians([float(v) for v in args.center.split(",")])
        arm.move_to_joint(center)
        raw = {}
        for j in joints:
            print(f"{arm.cfg.joints[j]}:")
            data = collect_sweeps(arm, j, center, math.radians(DEFAULT_AMPLITUDE_DEG[j] * args.amplitude_scale), speeds)
            raw.update({f"{arm.cfg.joints[j]}_{k}": v for k, v in data.items()})
            f = fit_joint_friction(arm.model, data)
            fits.append(f)
            print(f"  fc {f.fc_pos:.3f} N*m, fv {f.fv:.3f} N*m*s/rad (fit RMS {f.rms:.3f} N*m, {f.samples} samples)")
            arm.move_to_joint(center)
        _save_data(args, "friction", raw)
    fp = to_friction_params(fits, arm.n, arm.cfg.friction)
    _save_calibration(args, Calibration(friction=fp, meta=_meta(args, "friction")))
    print("enable the friction feedforward with feedforward.friction_scale (start at 0.5) in the arm config")


def cmd_calib_tcp(args) -> None:
    from .calib.tcp import collect_pivot, solve_pivot

    if not args.from_data and args.sim:
        sys.exit("tcp pivot calibration is interactive (hand guiding): real arm only, or use --from-data")
    if args.from_data:
        d = np.load(args.from_data)
        P, R = d["positions"], d["rotations"]
        arm = _make_arm(args)
    else:
        arm = _make_arm(args)
        _confirm(args, "TCP pivot calibration: gravity mode, you guide the arm by hand.")
        with arm:
            arm.enable()
            arm.move_home()
            P, R = collect_pivot(arm, args.poses)
            arm.move_to_joint(arm.tool.home)
        _save_data(args, "tcp", {"positions": P, "rotations": R})
    fit = solve_pivot(P, R)
    print(fit.summary())
    tool = arm.tool.copy()
    print(f"configured TCP {np.round(tool.tcp_xyz, 5).tolist()} -> identified {np.round(fit.tcp_xyz, 5).tolist()} "
          f"({np.linalg.norm(fit.tcp_xyz - tool.tcp_xyz) * 1000:.1f} mm)")
    tool.tcp_xyz = fit.tcp_xyz
    tool.extra = dict(tool.extra) | {"calibrated": {"tcp": _dt.date.today().isoformat(), "tcp_rms_mm": round(fit.rms * 1000, 2)}}
    out = _out_path(args, f"{tool.name}.yaml")
    tool.save(out)
    print(f"saved tool config {out}")


def cmd_calib_gains(args) -> None:
    from .calib.gains import Limits, find_kp_max, step_response

    arm = _make_arm(args)
    joints = [int(j) - 1 for j in args.joints.split(",")] if args.joints else list(range(arm.n))
    _confirm(args, f"Gain {'check' if args.check else 'tuning'}: small steps ({args.amplitude} deg) on joints "
                   f"{[j + 1 for j in joints]}. Stop at once if a joint buzzes or squeals.")
    amp = math.radians(args.amplitude)
    results = {}
    with arm:
        arm.enable()
        arm.move_home()
        for j in joints:
            print(f"{arm.cfg.joints[j]} (J_eff = {arm.model.mass_matrix(arm.q())[j, j]:.4f} kg*m^2):")
            if args.check:
                for m in step_response(arm, j, amplitude=amp):
                    print(f"  {m.line()}")
                continue
            kp_cap = arm.cfg.gains.kp[j] * args.kp_cap
            kd_cap = arm.cfg.gains.kd[j] * args.kd_cap
            r = find_kp_max(arm, j, kp_cap=kp_cap, kd_cap=kd_cap, zeta=args.zeta, margin=args.margin,
                            limits=Limits(), amplitude=amp)
            print(f"  kp_max {r.kp_max:.1f} ({r.stop_reason}); recommended kp {r.kp:.1f}, kd {r.kd:.2f}")
            results[j] = r
    if results and args.save:
        kp, kd = arm.cfg.gains.kp.copy(), arm.cfg.gains.kd.copy()
        for j, r in results.items():
            kp[j], kd[j] = r.kp, r.kd
        _save_calibration(args, Calibration(gains={"kp": np.round(kp, 2).tolist(), "kd": np.round(kd, 3).tolist()},
                                            meta=_meta(args, "gains")))
    elif results:
        print("not saved (pass --save to write the recommended gains to the calibration file)")


# ------------------------------------------------------------------------------------------------------- main
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="rebot-control", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--arm", default="b601_rs", help="arm config name or YAML path (default b601_rs)")
    common.add_argument("--tool", default="none", help="tool config name or YAML path (default none = bare flange)")
    common.add_argument("--calibration", default=None, help="calibration YAML to load")
    common.add_argument("--sim", action="store_true", help="run in the MuJoCo simulator instead of the real arm")
    common.add_argument("--viewer", action="store_true", help="with --sim: open the MuJoCo viewer")
    common.add_argument("--yes", action="store_true", help="do not ask for confirmation before moving")
    common.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("state", parents=[common], help="print joint state, temperatures and TCP pose (torque off)")
    s.add_argument("--watch", action="store_true")
    s.add_argument("--repeat", type=int, default=20)
    s.set_defaults(func=cmd_state)
    s = sub.add_parser("release", parents=[common], help="operator's torque off, for an arm left holding")
    s.add_argument("--fold", action="store_true", help="fold to the zero pose first instead of going limp in place")
    s.set_defaults(func=cmd_release)
    s = sub.add_parser("hold", parents=[common], help="torque on, hold the current pose")
    s.add_argument("--seconds", type=float, default=10.0)
    s.set_defaults(func=cmd_hold)
    s = sub.add_parser("float", parents=[common], help="gravity mode (hand guiding)")
    s.add_argument("--seconds", type=float, default=30.0)
    s.set_defaults(func=cmd_float)
    s = sub.add_parser("home", parents=[common], help="unfold to the tool home pose and fold back")
    s.add_argument("--seconds", type=float, default=2.0)
    s.set_defaults(func=cmd_home)
    s = sub.add_parser("square", parents=[common], help="Cartesian square test around the home pose")
    s.add_argument("--side", type=float, default=0.08, help="m")
    s.add_argument("--speed", type=float, default=0.05, help="m/s")
    s.set_defaults(func=cmd_square)

    c = sub.add_parser("calib", help="calibration tools").add_subparsers(dest="calib", required=True)
    calib_common = argparse.ArgumentParser(add_help=False)
    calib_common.add_argument("--out", default=None, help="output file")
    calib_common.add_argument("--data-dir", default=None, help="where raw data is saved (default calibration/data)")
    calib_common.add_argument("--from-data", default=None, help="refit from a saved .npz instead of moving the arm")
    for name, func, help_ in (("gravity", cmd_calib_gravity, "link masses / COM from static poses"),
                              ("payload", cmd_calib_payload, "tool mass / COM from static poses")):
        s = c.add_parser(name, parents=[common, calib_common], help=help_)
        s.add_argument("--poses", type=int, default=30 if name == "gravity" else 15)
        s.add_argument("--settle", type=float, default=1.0, help="s at rest before averaging")
        s.add_argument("--seed", type=int, default=0)
        s.add_argument("--table-z", type=float, default=0.04, help="minimum height of arm and tool (m)")
        s.add_argument("--prior-rel", type=float, default=0.3, help="gravity: relative prior std of the URDF masses")
        s.set_defaults(func=func)
    s = c.add_parser("friction", parents=[common, calib_common], help="Coulomb + viscous friction from sweeps")
    s.add_argument("--joints", default=None, help="1-based, comma separated (default all)")
    s.add_argument("--speeds", default=None, help="deg/s, comma separated (default 5,10,20,40,60)")
    s.add_argument("--amplitude-scale", type=float, default=1.0, help="scale of the default sweep amplitudes")
    s.add_argument("--center", default=None, help="sweep centre pose in deg (default: tool home)")
    s.set_defaults(func=cmd_calib_friction)
    s = c.add_parser("tcp", parents=[common, calib_common], help="TCP position by pivot calibration (interactive)")
    s.add_argument("--poses", type=int, default=8)
    s.set_defaults(func=cmd_calib_tcp)
    s = c.add_parser("gains", parents=[common, calib_common], help="kp / kd step-response check and tuning")
    s.add_argument("--joints", default=None, help="1-based, comma separated (default all)")
    s.add_argument("--check", action="store_true", help="only measure the configured gains")
    s.add_argument("--amplitude", type=float, default=2.0, help="step size (deg)")
    s.add_argument("--kp-cap", type=float, default=2.0, help="kp search cap, multiple of the configured kp")
    s.add_argument("--kd-cap", type=float, default=2.0, help="kd cap, multiple of the configured kd")
    s.add_argument("--zeta", type=float, default=0.8, help="target damping ratio")
    s.add_argument("--margin", type=float, default=0.6, help="working kp = margin * kp_max")
    s.add_argument("--save", action="store_true", help="write the recommended gains to the calibration file")
    s.set_defaults(func=cmd_calib_gains)
    return p


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    def _terminate(*_):  # `kill` ends like Ctrl-C: one clean hold frame instead of a frame cut off mid-motion
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _terminate)
    t0 = time.perf_counter()
    try:
        args.func(args)
    except KeyboardInterrupt:
        print("interrupted. If torque was on, the arm now holds where it is and stays enabled: continue with another "
              "command, or end with `rebot-control release` (support the arm) / `rebot-control release --fold`.")
        sys.exit(130)
    log.debug("done in %.1f s", time.perf_counter() - t0)


if __name__ == "__main__":
    main()

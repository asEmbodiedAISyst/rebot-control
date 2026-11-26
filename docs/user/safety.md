# Safety

The library has several layers of protection. **None of them replaces a hand on the power switch.**

| Layer | Where | What it does |
| --- | --- | --- |
| Soft joint limits | `safety.soft_limits_deg`, intersected with the URDF | every joint target is clipped |
| Jump guard | `safety.max_jump_deg` (25) | a target that far from the previous one raises `CommandRejected`; the arm keeps its previous target |
| Rate limit | `safety.max_joint_speed_deg_s` (90) | joint targets move at most this fast (3 deg per 30 Hz cycle) |
| Low-pass filter | `safety.filter_tau_s` (0.1 s) | removes steps and jitter from targets |
| Torque cap | `safety.max_error_deg` (10) | the MIT position target is clamped to measured +- 10 deg, so the PD torque is at most kp x 10 deg |
| Feedforward clip | `feedforward.tau_max` | caps the total feedforward (gravity, friction, ripple) in case the model is wrong |
| Gain ramp | `gains.ramp_s` (0.5 s) | gains and feedforward rise from zero at enable: no torque step |
| Stall detection | `safety.lag_deg`, `stall_time_s`, `stall_progress_deg`, `on_stall` | a joint more than 10 deg behind its target for 1 s while moving less than 1 deg raises `SafetyStop`; the arm holds where it is (`on_stall: hold`) |
| Temperature | `safety.temp_warn_c` (115), `temp_stop_c` (125), `on_overtemp` | a warning is logged; above the stop limit torque goes off (or the arm holds, `on_overtemp: hold`) and `SafetyStop` is raised |
| Cartesian limits | tool `cartesian:` | workspace box, speed, tilt and yaw limits on TCP commands; unreachable steps are refused |
| Staged fold | `poses.fold_stages` | on a normal exit the arm folds base + wrist first, then shoulder + elbow |

## When something goes wrong: the arm holds

The rule: **torque goes off only on purpose.** An arm that loses torque in mid-air falls and hits something, so a
failure never switches torque off and never starts a motion. The arm holds where it is, stays enabled, and waits
for the next program or for the operator.

| What happens | What the arm does |
| --- | --- |
| The `with` block ends normally | folds to the zero pose in stages, then torque off |
| An exception in your code, Ctrl-C, SIGTERM (CLI) | holds the measured pose, stays enabled; the program exits |
| Stall detected (`safety.on_stall: hold`, the default) | holds the measured pose, `SafetyStop`, stays enabled |
| The servo thread dies (bus error, bug) | sends one last hold frame if it can, stays enabled |
| The program is killed, the PC hangs, the CAN cable comes off | the motors keep executing the last frame they got |
| A driver exceeds `temp_stop_c` (`safety.on_overtemp: disable`, the default) | **torque off, the arm falls**, `SafetyStop` |
| `rebot-control release`, `arm.release()`, `arm.disable_torque()` | torque off: the operator's decision |

Two settings change this: `safety.on_stall: disable` (torque off on a stall: the choice when pressing into a person
matters more than the fall) and `safety.on_overtemp: hold` (keep holding a hot motor until its own protection
trips).

### How the hold survives the program

In MIT mode a RobStride driver keeps executing the last frame it received. With its CAN timeout at 0 (parameter
0x7028; the value found on the B601-RS, checked at every connect) it does so for as long as it has power. When a
program leaves by an error, `Arm.detach()` sends one last frame, "hold the measured pose, target velocity zero,
gravity feedforward for this pose", and releases the bus without disabling anything.

```{warning}
Nobody watches a held arm. There is no stall check and no temperature guard until a program connects again.
The motors keep producing the torque the pose needs, and the only protection left is the drivers' own.
```

### Taking over

`Arm.connect()` never assumes that torque is off. It reads the joint angles with parameter reads (which do not
replace the frame a motor is executing), then sends one full-gain frame that holds exactly those angles: an
enabled motor keeps holding, a disabled one ignores it, and both answer with their state. If motors are enabled,
the `Arm` continues the hold without a jump and sets `arm.taken_over`; your program decides what to do from there
(`arm.move_home()`, `arm.fold()`, ...).

The hold is continued at the same equilibrium, not at the measured angles: the new target is placed so that the
controller asks for the torque the motors report at that moment. A held arm sits a little away from its target
(gravity model error, friction), and re-targeting to the measured angle would move it by that error at every
take-over. For the same reason a stop keeps the target of every joint that was following it and only re-targets
joints that are far from theirs.

A program that only looks (`rebot-control state`, or a `with` block that never calls `enable()` or a motion
command) leaves a held arm exactly as it found it. `state` prints `torque: ON, holding` in that case.

### Releasing

```bash
rebot-control release --fold      # fold to the zero pose from where it is, then torque off
rebot-control release             # torque off now: hold the arm with your hand first
```

Both ask for confirmation. `arm.release(fold=True)` / `arm.release()` do the same from code.

**Kill switch: cut motor power** (support the arm: it falls).

## Before running new code on the real arm

1. Run the same script with `--sim` (or `backend="sim"`) and look at it with the viewer.
2. Start slow: low `max_speed`, a small workspace box.
3. Keep the arm folded until the code has proven itself at the home pose.

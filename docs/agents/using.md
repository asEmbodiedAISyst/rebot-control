# Guide for agents that use rebot-control

Use this page when you help a person run the arm or write code against `rebot_control`.

## Rules

1. Run in simulation: `backend="sim"` in Python, `--sim` on the command line. Run on the real arm only when the user
   asks for it in the current session and is at the arm.
2. Before a real-arm run, run the same script in simulation.
3. On the real arm, do not pass `--yes`. The confirmation prompt is for the user.
4. Pass the tool that is mounted (`tool=`, `--tool`). A wrong payload gives a wrong gravity feedforward.
5. Do not change `safety.*`, `feedforward.tau_max` or the Cartesian limits to make a command pass. Report the
   refusal (`CommandRejected`, `SafetyStop`, `ik_ok == False`) to the user.
6. Do not switch torque off on a real arm (`arm.release()`, `arm.disable_torque()`, `rebot-control release` without
   `--fold`) unless the user asks. The arm falls when torque goes off.
7. Angles are radians in Python. Degrees appear only in YAML keys that end in `_deg`.

## Install

```bash
pip install -e ".[sim]"          # simulation: Linux, macOS
pip install -e ".[real,sim]"     # real arm: Linux, SocketCAN, can0 up
```

Check the installation: `rebot-control state --sim`.

## Python

```python
from rebot_control import Arm

with Arm.from_config("b601_rs", tool="pusher_ip17", backend="sim") as arm:
    arm.enable()                      # torque on, hold the measured pose
    arm.move_home()                   # unfold to the tool's home pose
    p, R = arm.tcp_pose()             # position (3,), rotation matrix (3, 3), base frame
    arm.move_to_pose(p + [0.05, 0.0, 0.0], R, speed=0.05)

    cart = arm.cartesian()            # streaming control
    for _ in range(90):
        status = cart.step(p + [0.05, 0.03, 0.0], R)
        arm.sleep()                   # one control period (30 Hz); call it once per cycle
# normal exit: fold, then torque off
```

| Task | Call |
| --- | --- |
| Load an arm | `Arm.from_config(arm, tool=, backend=, calibration=, overrides=)` |
| Joint move, blocking | `arm.move_to_joint(q, speed=)` |
| Joint target, streaming | `arm.command_joint(q)`, then `arm.sleep()` |
| Cartesian move, blocking | `arm.move_to_pose(p, R, speed=)` |
| Cartesian target, streaming | `arm.cartesian().step(p, R)`, then `arm.sleep()` |
| Hand guiding | `arm.gravity_mode()`, back with `arm.hold()` |
| Change the tool | `arm.set_tool(name_or_path)` |
| Record the servo data | `with arm.recording() as rec: ...` |

## Command line

Every command takes `--arm`, `--tool`, `--calibration`, `--sim`, `--viewer`.

| Command | What it does |
| --- | --- |
| `rebot-control state` | Prints angles, temperatures, TCP pose and torque on/off. Changes nothing. |
| `rebot-control hold` | Torque on at the current pose. |
| `rebot-control float` | Gravity mode. |
| `rebot-control home` | Unfolds to the home pose and folds back. |
| `rebot-control square` | Cartesian square test. |
| `rebot-control release --fold` | Folds an arm that was left holding, then torque off. |
| `rebot-control calib gravity\|payload\|friction\|tcp\|gains` | Calibration tools. `--from-data` refits saved data without moving. |

## What the arm does when a program ends

| End of the program | State of the arm |
| --- | --- |
| The `with` block ends normally | folded, torque off |
| Exception, Ctrl-C, stall | holds its pose, stays enabled |

A held arm stays enabled after the program exits. The next `Arm` takes the hold over at `connect()` and sets
`arm.taken_over`. Tell the user when a program leaves the arm holding.

## Where to read

| Question | Page |
| --- | --- |
| Terms, modes, the control chain | [`../user/concepts.md`](../user/concepts.md) |
| Life cycle, timing, joint and Cartesian calls | [`../user/quickstart.md`](../user/quickstart.md) |
| YAML keys, units, frames, a new tool | [`../user/configuration.md`](../user/configuration.md) |
| Cartesian limits, refused steps | [`../user/cartesian-control.md`](../user/cartesian-control.md) |
| Calibration commands and their order | [`../user/calibration.md`](../user/calibration.md) |
| Stops, holds, take-over, release | [`../user/safety.md`](../user/safety.md) |
| Error messages | [`../user/troubleshooting.md`](../user/troubleshooting.md) |
| First session on a new arm | [`../user/hardware-test-plan.md`](../user/hardware-test-plan.md) |

All pages in one file: `llms-full.txt` at the root of the documentation site.

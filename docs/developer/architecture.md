# Architecture

```
 user code / CLI (cli.py) / calibration tools (calib/)
        |  joint targets, TCP targets, mode changes             caller thread, control_hz (30 Hz)
        v
 Arm (arm.py) ----------- CartesianController (cartesian.py) -- DiffIK (ik.py)
        |  command_joint(q)
        v
 JointTargetFilter (joint_filter.py): soft limits -> jump guard -> rate limit -> low-pass
        |  servo.set_target(q, t)
        v
 ServoLoop (servo.py): interpolation, torque cap, feedforward (model.py), gain ramp,     servo thread (real) or
                       temperature guard, recording                                     stepped by Arm.sleep (sim),
        |  send_mit(q, dq, kp, kd, tau) / read()                                        servo_hz (150 Hz)
        v
 Backend (backends/): RobStrideBackend (motorbridge, SocketCAN)  |  SimBackend (MuJoCo, physics_hz 1050 Hz)
```

`RobotModel` (`model.py`) is the Pinocchio model of arm + tool. It provides FK, Jacobians, gravity, the mass matrix
and the regressor. It is shared by the servo loop (feedforward), the IK and the calibration tools. Each thread uses
its own `pin.Data` (`model.new_data()`).

## Modules

| Module | Responsibility |
| --- | --- |
| `config.py` | dataclasses for arm / tool / calibration YAML; degree -> radian conversion at load |
| `pose.py` | rotation helpers (pure numpy) |
| `model.py` | Pinocchio model, TCP frame, payload, identified link parameters |
| `ik.py` | damped least-squares IK with a null-space posture term |
| `joint_filter.py` | conditioning of joint targets at the control rate |
| `servo.py` | MIT frame generation at the servo rate; modes PASSIVE / HOLD / GRAVITY |
| `backends/base.py` | backend interface, `JointState` |
| `backends/robstride.py` | real arm |
| `backends/sim.py` | MuJoCo arm built from the same model |
| `arm.py` | facade: life cycle, modes, moves, stall check, recording |
| `cartesian.py` | Cartesian limits and the streaming controller |
| `calib/` | poses, gravity / payload, friction, torque ripple, TCP pivot, gains |
| `cli.py` | `rebot-control` command |
| `errors.py` | exception types: `CommandRejected`, `SafetyStop`, `BackendError` |

## Threads and locks

- **Real arm.** `ServoLoop._run` runs in a daemon thread. It holds `ServoLoop._lock` (an RLock) while it computes and
  sends a frame. Commands from the caller (`set_target`, `set_mode`, `set_gains`, tool swaps) take the same lock.
  `RobStrideBackend._lock` serialises every bus access (servo frames, `enable`, `disable`, `close`).
- **Simulation.** There are no threads. `Arm.sleep` calls `servo.tick` and `backend.advance(servo.period)`
  `servo_hz / control_hz` times. Simulation time is the clock (`backend.now()`).
- `ServoLoop.state` is replaced atomically each tick (a fresh `JointState`). Readers copy it.

## Life cycle

1. `connect()`: bus open. Read the joint angles without a command frame (`Backend.read_positions`), send one
   full-gain frame holding those angles, read the replies. Motors enabled: HOLD at the measured pose with full gains
   (`taken_over`). All disabled: PASSIVE (zero-gain frames, feedback only). Zero-gain frames are never sent before
   this is known.
2. `enable()`: the backend enables the motors. The servo switches to HOLD at the measured pose with the gain ramp.
3. Commands (`command_joint`, `move_*`, `cartesian()`), `gravity_mode()` / `hold()`.
4. `close()` (clean exit): fold in stages, disable, PASSIVE, stop the thread, close the bus. An arm that was found
   enabled and never commanded is left as found.
5. `detach()` (exception, Ctrl-C): HOLD at the measured pose, one last frame, stop the thread, close the bus
   **without** disabling. The motors keep executing that frame.

## Errors

| Error | Meaning |
| --- | --- |
| `CommandRejected` | one command refused; nothing changed |
| `SafetyStop` | stall (the arm holds; torque off with `on_stall: disable`) or temperature (torque off; holds with `on_overtemp: hold`) |
| `BackendError` | bus, CAN or motor problems |

Errors raised in the servo thread are stored in `servo.error` and re-raised by `Arm.check()`, which runs at the start
of every command and every `sleep()`.

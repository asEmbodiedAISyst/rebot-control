# Cartesian control

```python
cart = arm.cartesian()                  # starts at the current target pose; enables HOLD if needed
for k in range(n):
    st = cart.step(p_des, R_des)        # R_des=None keeps the current orientation
    arm.sleep()                         # one control period (30 Hz)
```

`step()` does the following, every cycle:

1. **Workspace and orientation limits** on the desired pose:
   - the position is clipped to the `x`, `y`, `z` box (base frame);
   - with `max_tilt_deg`, the tool axis (TCP z) is kept within that angle of straight down;
   - with `max_yaw_deg`, the heading of the TCP x axis is kept within that angle.
2. **Speed limits** relative to the previous *commanded* pose: `max_speed` (m/s) and `max_ang_speed` (rad/s). A far
   target is approached in a straight line at that speed. This is also how `arm.move_to_pose` makes straight lines.
3. **Lag slow-down**: when a joint lags its target by more than `safety.slow_lag_deg` (20 deg), the speed limits
   shrink linearly, down to zero at twice that lag. The arm catches up instead of running away from the command.
4. **IK** (damped least squares with a posture term, warm-started from the last solution). If the limited pose is not
   reachable within 5 mm / 3 deg, the step is **refused**: the previous pose is kept and the arm holds.
5. The joint target goes to `arm.command_joint`: soft limits, jump guard, rate limit, low-pass filter, then the servo.

The returned `CartesianStatus` holds:

| Field | Meaning |
| --- | --- |
| `pos`, `rot` | the commanded pose after limits |
| `q` | the joint target |
| `ik_ok` | False if the step was refused |
| `clipped` | True if any limit changed the desired pose |
| `lag_deg` | joint lag before the step |
| `speed_factor` | below 1 while slowing down |

## Limits

Defaults come from the tool's `cartesian:` section. Override them per controller:

```python
from rebot_control import CartesianLimits
cart = arm.cartesian(CartesianLimits(x=(0.25, 0.45), y=(-0.1, 0.1), z=(0.01, 0.15), max_speed=0.1, max_tilt_deg=20))
```

The box is a coarse bound. Which poses inside it are reachable is decided by the IK in step 4. Use
`cart.reachable(p, R)` to check a pose without moving.

To keep demonstrations and execution consistent, apply the same `CartesianLimits` to recorded data before training
on it (`limits.clip_position`, `limits.clip_orientation`). Then the robot never clips what a policy learned.

## Accuracy you can expect

- **At rest**: the steady-state joint error with gravity feedforward is a few hundredths of a degree in simulation.
  On the real arm it is set by the gravity model, so calibrate gravity.
- **While moving**: the joint low-pass filter (`filter_tau_s` = 0.1 s) delays the motion by about 0.1 s, so the
  measured TCP trails the command by about speed x 0.1 s (5 mm at 5 cm/s). For faster tracking, lower `filter_tau_s`
  (smoother input then matters more) and enable friction feedforward.

## Relative motions

`rebot_control.pose.pose_delta` and `apply_delta` express a motion in the frame of its start pose. Use them for
policies that output pose increments, or for data recorded in a different world frame.

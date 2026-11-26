# Quick start

```{warning}
Run every example in simulation first (`backend="sim"`). Before the first session on a real arm, do the
[real-arm test plan](hardware-test-plan.md) and read [Safety](safety.md).
```

## Life cycle

```python
from rebot_control import Arm

arm = Arm.from_config("b601_rs", tool="pusher_ip17", backend="real")   # or backend="sim"
with arm:              # connect(): bus open, feedback streaming; changes nothing on the motors
    arm.enable()       # torque ON, holds the measured pose; gains ramp in over 0.5 s
    arm.move_home()    # tool home pose; from the folded pose: shoulder + elbow first
    ...                # your code
# exit: fold to the zero pose in stages (base + wrist, then shoulder + elbow), torque OFF, bus closed
```

- After an exception or Ctrl-C the arm neither folds nor loses torque: it **holds where it is and stays enabled**,
  also after the program has exited. The next program takes the hold over at `connect()` (`arm.taken_over`).
  End it with `rebot-control release --fold`, or `rebot-control release` with a hand on the arm. See
  [safety.md](safety.md).
- `arm.release()` switches torque off where the arm is. Use it only when the arm is supported.

## Timing: `arm.sleep()`

Your loop runs at `control_hz` (30 Hz by default). Call `arm.sleep()` once per cycle.

- On the real arm a background thread streams MIT frames at 150 Hz, and `sleep()` keeps your loop's rate.
- In the simulator `sleep()` advances physics and the servo loop by one control period. The same script therefore
  runs in both, and in simulation it runs faster than real time unless you open the viewer (`viewer=True`).

`arm.wait(seconds)` keeps holding the current target for a while (with stall checks).

## Joint space

```python
import math
q = arm.tool.home.copy(); q[0] += math.radians(20)
arm.move_to_joint(q, speed=math.radians(15))   # smoothstep profile, stall detection
arm.command_joint(q)                          # streaming: one target per control cycle, then arm.sleep()
```

`command_joint` goes through the joint target filter:

- clip to the soft limits;
- refuse a jump of more than 25 deg (`CommandRejected`);
- rate limit to 90 deg/s;
- 0.1 s low-pass filter.

## Cartesian space

```python
p, R = arm.tcp_pose()                          # measured TCP pose (base frame)
arm.move_to_pose(p + [0.05, 0, 0], R)          # straight line at the speed limit
cart = arm.cartesian()                         # streaming controller, see cartesian-control.md
st = cart.step(p_des, R_des); arm.sleep()
```

## Hand guiding

```python
arm.gravity_mode()     # kp = 0, gravity (+ friction) feedforward: move the arm by hand
arm.hold()             # back to PD hold at the measured pose
```

## Tools

```python
arm.set_tool("stock_gripper")          # built-in name, or a path to your own YAML
arm.set_tool("my_tools/pusher.yaml")
```

## Recording

```python
with arm.recording() as rec:
    arm.move_to_joint(q)
rec["t"], rec["q"], rec["dq"], rec["tau"], rec["q_cmd"], rec["tau_ff"]   # one row per servo tick (150 Hz)
```

## Examples

`examples/` has short scripts for each feature. Each one takes `--sim` and, in simulation, `--viewer`:

```bash
cd examples
python 01_read_state.py --sim
python 04_cartesian.py --sim --viewer
```

# 快速上手

```{warning}
每个示例都先在仿真里运行（`backend="sim"`）。第一次上真机之前，先做完[真机测试计划](hardware-test-plan.md)，并阅读[安全](safety.md)。
```

## 生命周期

```python
from rebot_control import Arm

arm = Arm.from_config("b601_rs", tool="pusher_ip17", backend="real")   # or backend="sim"
with arm:              # connect(): bus open, feedback streaming; changes nothing on the motors
    arm.enable()       # torque ON, holds the measured pose; gains ramp in over 0.5 s
    arm.move_home()    # tool home pose; from the folded pose: shoulder + elbow first
    ...                # your code
# exit: fold to the zero pose in stages (base + wrist, then shoulder + elbow), torque OFF, bus closed
```

- 发生异常或按下 Ctrl-C 之后，机械臂既不折叠也不卸力：它**保持在原地，并且一直使能**，程序退出之后也是这样。下一个程序在
  `connect()` 时接管这个保持状态（`arm.taken_over`）。要结束保持，用 `rebot-control release --fold`；或者先用手扶住机械臂，再用
  `rebot-control release`。见 [safety.md](safety.md)。
- `arm.release()` 让机械臂就地卸力。只有机械臂有支撑时才用它。

## 定时：`arm.sleep()`

你的循环按 `control_hz` 运行（默认 30 Hz）。每个周期调用一次 `arm.sleep()`。

- 在真机上，后台线程以 150 Hz 连续下发 MIT 帧，`sleep()` 负责维持你的循环频率。
- 在仿真器里，`sleep()` 把物理仿真和伺服循环推进一个控制周期。所以同一个脚本在两边都能跑。仿真时如果不打开查看器（`viewer=True`），
  它跑得比实时快。

`arm.wait(seconds)` 在一段时间内保持当前目标，期间照常做堵转检测。

## 关节空间

```python
import math
q = arm.tool.home.copy(); q[0] += math.radians(20)
arm.move_to_joint(q, speed=math.radians(15))   # smoothstep profile, stall detection
arm.command_joint(q)                          # streaming: one target per control cycle, then arm.sleep()
```

`command_joint` 的目标要经过关节目标滤波器：

- 限制在软限位以内；
- 跳变超过 25 deg 时拒绝执行（`CommandRejected`）；
- 速率限制为 90 deg/s；
- 0.1 s 低通滤波。

## 笛卡尔空间

```python
p, R = arm.tcp_pose()                          # measured TCP pose (base frame)
arm.move_to_pose(p + [0.05, 0, 0], R)          # straight line at the speed limit
cart = arm.cartesian()                         # streaming controller, see cartesian-control.md
st = cart.step(p_des, R_des); arm.sleep()
```

## 手动拖动

```python
arm.gravity_mode()     # kp = 0, gravity (+ friction) feedforward: move the arm by hand
arm.hold()             # back to PD hold at the measured pose
```

## 工具

```python
arm.set_tool("stock_gripper")          # built-in name, or a path to your own YAML
arm.set_tool("my_tools/pusher.yaml")
```

## 数据记录

```python
with arm.recording() as rec:
    arm.move_to_joint(q)
rec["t"], rec["q"], rec["dq"], rec["tau"], rec["q_cmd"], rec["tau_ff"]   # one row per servo tick (150 Hz)
```

## 示例

`examples/` 里每项功能都有一个简短的脚本。每个脚本都支持 `--sim`，仿真时还可以加 `--viewer`：

```bash
cd examples
python 01_read_state.py --sim
python 04_cartesian.py --sim --viewer
```

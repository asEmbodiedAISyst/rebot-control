# 后端

后端只做一件事：以关节为单位收发 MIT 帧和反馈。控制逻辑全部在 `ServoLoop` 里，所以真机和仿真器跑的是同一份代码。
接口定义在 `backends/base.py`：

- `connect`、`close`；
- `enable`、`disable`；
- `send_mit(q, dq, kp, kd, tau)`、`send_passive()`；
- `read()`，返回一个 `JointState`；
- `now()`；
- `set_zero(joints)`，仅真机；
- `advance(dt)`，仅仿真。

## 通过 motorbridge 驱动 RobStride（`backends/robstride.py`）

这个后端使用 `motorbridge` 0.5.6：

- `Controller(channel)`；
- `add_robstride_motor(can_id, feedback_id, model)`；
- 每个电机：`ensure_mode(Mode.MIT)`、`send_mit(pos, vel, kp, kd, tau)`、`get_state()`、`disable()`、
  `clear_error()`、`set_zero_position()`、`close()`；
- 总线级：`enable_all()`、`disable_all()`、`poll_feedback_once()`、`close()`。

Seeed 的 lerobot 插件用的也是这些调用，使能顺序也一样：全部失能，设成 MIT 模式（带重试），全部使能。

B601-RS 的特殊行为：

- **反馈只来自应答。** motorbridge 只在收到命令的应答时才更新 RobStride 电机的状态。`request_feedback()` 对 RobStride
  不起作用，电机周期性的主动上报帧也不解析。所以伺服线程每个 tick 都给每个电机发一帧，卸力时发零增益帧
  （`send_passive`）。随后 `read()` 处理的是*上一个* tick 那些帧的应答（`poll_feedback_once` + `get_state`）。
  因此反馈总是滞后一个伺服周期（6.7 ms）。
- **未使能的电机仍会应答 MIT 帧**（约 3 ms 内），并且保持未使能。`state` 就是靠这一点在卸力状态下读取机械臂。
- **帧停发以后，已使能的电机会一直执行最后一帧 MIT 帧**（B601-RS 上 CAN 超时为 0）。本库靠这一点让机械臂在程序
  意外终止后继续保持，所以后端必须提供三样东西：`read_positions()`（不发命令帧读角度：用 type-17 读参数 0x7019）、
  `JointState.enabled`（反馈帧里的驱动器模式；`status_code != 0` 算作已使能）、`close(disable=False)`
  （释放总线，什么都不发）。`enable()` 只使能还没使能的电机：切换模式要求电机处于失能状态，而让一个正在保持的电机
  失能，关节就会掉下来。
- **CAN 接口在重启或重新插拔后处于 DOWN 状态。** `check_can_up()` 会提前报错，并给出修复命令。
- `MotorState` 包含 `pos`、`vel`、`torq`、`t_mos`（驱动器 MOSFET 温度）和 `t_rotor`。温度保护用的是 `t_mos`。
- 上报的力矩由相电流估算而来，带有偏置和噪声底，所以重力标定要给每个关节拟合一个偏置。
- 原装夹爪的电机（CAN id 7）没有加到总线上。装着夹爪时，它保持失能，按刚性负载处理。

待在真机上验证的事项（记录在 [hardware-notes.md](hardware-notes.md)）：

- 上报力矩相对于帧的相位（瞬时值还是滤波后的值）；
- 电机侧 CAN 超时的行为；
- 电机接受的 MIT kp / kd 范围（motorbridge 的型号表会对它们限幅）。

## MuJoCo（`backends/sim.py`）

- `build_mjspec(model, cfg)` 从 **Pinocchio 模型**生成 MuJoCo 模型：刚体位置、惯量（`fullinertia`、
  `explicitinertial`）、铰链轴（取自 q = 0 处的关节雅可比矩阵）、以弧度表示的关节范围
  （`spec.compiler.degree = False`），以及 `armature = rotor_inertia`，`damping` 和 `frictionloss` 取自 `sim:`。
  没有碰撞几何体，胶囊体只用于显示。因此运动学和动力学与控制器的模型完全一致
  （`tests/test_sim.py::test_sim_matches_model`）。
- **“真值”参数**：`truth_link_params`、`truth_tool`、`truth_friction` 和 `truth_ripple` 让仿真里的机械臂与控制器以为的不一样。
  辨识测试靠的就是它们。
- **电机仿真**：每个物理步（1050 Hz）通过 `qfrc_applied` 施加
  `tau = clip(kp (q* - q) + kd (dq* - dq) + tau_ff, +-effort)`，并保持最后一帧（零阶保持，与驱动器一致）。
  上报的力矩是最近一次 `advance()` 调用（一个伺服周期）内的平均值。这是在模拟经过滤波的电流读数，
  对摩擦辨识有影响，见 [servo-and-timing.md](servo-and-timing.md)。
- **没有建模的部分**：CAN 延迟、编码器量化、速度噪声、齿隙、热行为、接触。所以增益的上限在仿真里找不出来。
- `viewer=True` 会打开 `mujoco.viewer.launch_passive`，并把仿真节奏放慢到与真实时间一致。
- 在折叠位姿卸力时，仿真里的机械臂会略微下垂（关节 4 没有顶在限位上）。真机则靠自身结构支撑。

## 添加后端

继承 `Backend`，以关节为单位实现各个抽象方法，符号约定在后端内部处理。然后在 `arm.make_backend` 里注册。
下一个最可能加的是达妙电机（B601-DM）。它的位置-速度模式与 MIT 帧接口对不上，要用它的 MIT 模式。

# 配置

YAML 文件有三类。内置的文件在 `src/rebot_control/configs/` 下。使用时可以传内置名称（`"b601_rs"`、`"pusher_ip17"`），也可以传你自己文件的路径。

| 文件 | 内容 | 加载方式 |
| --- | --- | --- |
| 机械臂（`configs/arms/b601_rs.yaml`） | 硬件、频率、增益、前馈、安全限制、位姿、仿真器 | `ArmConfig.load` |
| 工具（`configs/tools/*.yaml`） | TCP 位姿、负载、home 位姿、笛卡尔限制 | `ToolConfig.load`、`arm.set_tool` |
| 标定（`calibration/b601_rs.yaml`，由 `rebot-control calib` 写入） | 辨识得到的连杆质量 / 质心、摩擦、力矩波动、增益 | `Calibration.load` |

```python
arm = Arm.from_config("b601_rs", tool="my_tools/pusher.yaml", calibration="calibration/b601_rs.yaml",
                      overrides={"feedforward": {"friction_scale": 0.5}})
```

`overrides` 会在解析之前深度合并到机械臂 YAML 里。做实验时用它很方便，不必复制整个文件。

## 单位和坐标系

- **YAML 里的角度用度**（键名以 `_deg` 结尾），Python 里一律用弧度。
- 长度的单位是 m，质量是 kg，力矩是 N*m。增益的单位是 N*m/rad（kp）和 N*m*s/rad（kd），与 MIT 帧一致。
- **基座坐标系**：URDF 的 `base_link`。x 朝前，y 朝左，z 朝上，原点在基座下方的桌面上。
- **关节角**就是 URDF 关节角。出厂零位下，电机原始角度 = 关节角。
- **法兰**是 URDF 的 `link6`。工具的 TCP 和负载都在法兰坐标系下表示。
- **TCP 姿态**：`tcp.rpy_deg` 采用 URDF 的约定，绕固定轴旋转，`R = Rz(yaw) Ry(pitch) Rx(roll)`。

## 机械臂文件（主要键）

| 键 | 含义 |
| --- | --- |
| `bus.channel` | SocketCAN 接口（`can0`） |
| `motors[]` | `can_id`、`feedback_id`、`model`（`rs-06` / `rs-00`）、`direction`（+1/-1）、`offset_deg`；raw = direction * joint + offset |
| `rates.control_hz` / `servo_hz` | 调用方循环的频率（30）/ MIT 帧的下发频率（150） |
| `gains.kp`、`gains.kd` | HOLD 模式下的 PD 增益；`gravity_mode_kd` 用于重力补偿模式；`ramp_s` 是使能时增益爬升的时间 |
| `feedforward.gravity`、`gravity_scale` | 重力补偿（默认开启，系数 1.0） |
| `feedforward.friction_scale` | 摩擦补偿；标定摩擦之前为 0，标定之后一般取 0.5 到 0.8 |
| `feedforward.ripple_scale` | 标定文件里力矩波动模型（随关节角周期变化）的缩放系数（默认 1；标定文件没有 `ripple` 段时不起作用） |
| `feedforward.tau_max` | 每个关节总前馈的限幅（防止模型出错） |
| `rotor_inertia` | 每个关节折算到输出端的转子惯量（kg*m^2），用于增益计算和仿真器 |
| `safety.*` | 见 [safety.md](safety.md) |
| `poses.zero_deg`、`unfold_first`、`fold_stages`、`move_speed_deg_s` | 折叠位姿，以及分阶段的展开 / 折叠 |
| `sim.*` | 只用于仿真器：物理仿真频率、仿真机械臂的关节阻尼 / 干摩擦、力矩噪声 |

## 工具文件

```yaml
name: my_pusher
mount_frame: link6
tcp:
  xyz: [-0.096, 0.0, 0.172]      # tool tip in the flange frame (m)
  rpy_deg: [0.0, -60.0, 0.0]     # TCP axes; z = approach axis (used by the tilt limit)
payload:
  mass_kg: 0.30
  com_xyz: [0.023, 0.036, 0.062]  # in the flange frame (m)
  # inertia: [ixx, iyy, izz, ixy, ixz, iyz]   # about the COM (kg*m^2); optional (point mass otherwise)
home_deg: [0, 20, 28.6, -38.8, 0, 0]          # used by arm.move_home() and as the IK posture
cartesian:                                     # default CartesianLimits for this tool (see cartesian-control.md)
  x: [0.22, 0.56]
  y: [-0.20, 0.20]
  z: [0.003, 0.20]
  max_speed: 0.25
  max_ang_speed: 2.0
  max_tilt_deg: 30.0
  max_yaw_deg: 60.0
```

添加新工具的步骤：

1. 复制一个工具文件，从 CAD 得到 TCP 和负载。
2. 在仿真里检查 home 位姿：`rebot-control home --sim --viewer --tool my_tool.yaml`。
3. 在真机上用 `calib payload` 和 `calib tcp` 修正（见 [calibration.md](calibration.md)）。

内置工具：

| 名称 | 说明 |
| --- | --- |
| `none` | 空法兰；用于上机调试和重力标定 |
| `pusher_ip17` | 打印的法兰、倾斜 30 deg 的 10 mm 杆、手机托板（用于推动实验） |
| `stock_gripper` | Seeed 夹爪，当作刚性负载；夹爪电机（CAN id 7）不驱动 |

## 标定文件

这个文件由 `rebot-control calib ...` 写入，每次的新结果都合并到已有文件里：

```yaml
meta: {gravity_date: ..., gravity_rms_after: [...], ...}
link_params: {link2: {mass: 1.58, com: [-0.118, -0.004, -0.027]}, ...}   # replaces URDF mass / COM
friction: {fc_pos: [...], fc_neg: [...], fv: [...], vs: [...], bias: [...]}
ripple: {cycles: [...], a1: [...], b1: [...], a2: [...], b2: [...]}         # torque ripple model, see calibration.md
gains: {kp: [...], kd: [...]}                                              # only with calib gains --save
```

一份标定文件只对应一台机械臂。把它和你的实验代码放在一起，不要放进包里。

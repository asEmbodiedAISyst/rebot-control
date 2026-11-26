# 架构

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

`RobotModel`（`model.py`）是机械臂加工具的 Pinocchio 模型，提供正运动学、雅可比矩阵、重力、质量矩阵和回归矩阵。
伺服循环（算前馈）、IK 和标定工具共用这一个模型。每个线程用自己的 `pin.Data`（`model.new_data()`）。

## 模块

| 模块 | 职责 |
| --- | --- |
| `config.py` | 机械臂 / 工具 / 标定 YAML 对应的 dataclass；加载时把角度从度换算成弧度 |
| `pose.py` | 旋转相关的辅助函数（纯 numpy） |
| `model.py` | Pinocchio 模型、TCP 坐标系、负载、辨识得到的连杆参数 |
| `ik.py` | 阻尼最小二乘 IK，带零空间姿态项 |
| `joint_filter.py` | 在控制频率下整形关节目标 |
| `servo.py` | 在伺服频率下生成 MIT 帧；模式有 PASSIVE / HOLD / GRAVITY |
| `backends/base.py` | 后端接口，`JointState` |
| `backends/robstride.py` | 真机 |
| `backends/sim.py` | 由同一个模型生成的 MuJoCo 机械臂 |
| `arm.py` | 门面：生命周期、模式、运动、堵转检测、数据记录 |
| `cartesian.py` | 笛卡尔限制和连续下发控制器 |
| `calib/` | 位姿生成、重力 / 负载、摩擦、力矩波动、TCP 支点标定、增益 |
| `cli.py` | `rebot-control` 命令 |
| `errors.py` | 异常类型：`CommandRejected`、`SafetyStop`、`BackendError` |

## 线程与锁

下文把伺服循环执行一次称为一个 tick。

- **真机。** `ServoLoop._run` 跑在守护线程里。计算并发送一帧的过程中，它一直持有 `ServoLoop._lock`（一把 RLock）。
  调用方的命令（`set_target`、`set_mode`、`set_gains`、更换工具）拿的是同一把锁。
  `RobStrideBackend._lock` 让所有总线访问（伺服帧、`enable`、`disable`、`close`）串行执行。
- **仿真。** 没有线程。`Arm.sleep` 调用 `servo.tick` 和 `backend.advance(servo.period)`，各
  `servo_hz / control_hz` 次。时钟就是仿真时间（`backend.now()`）。
- `ServoLoop.state` 每个 tick 整体替换一次（换成新的 `JointState`）。读的一方自己复制。

## 生命周期

1. `connect()`：打开总线。先不发命令帧，直接读出关节角（`Backend.read_positions`）；再发一帧全增益的帧，
   让电机保持在这些角度上；然后读应答。电机已使能：以全增益在实测位姿进入 HOLD（`taken_over`）。
   电机全部未使能：进入 PASSIVE（零增益帧，只取反馈）。在弄清这一点之前，绝不发零增益帧。
2. `enable()`：后端使能电机。伺服循环切到 HOLD，保持实测位姿，增益按斜坡升上来。
3. 各种命令（`command_joint`、`move_*`、`cartesian()`），以及 `gravity_mode()` / `hold()`。
4. `close()`（正常退出）：分阶段折叠，失能，切到 PASSIVE，停掉线程，关闭总线。
   如果连接时机械臂已经使能，而程序从头到尾没给过它命令，就让它保持原样。
5. `detach()`（异常、Ctrl-C）：在实测位姿进入 HOLD，发出最后一帧，停掉线程，关闭总线，但**不**失能。
   电机会一直执行这最后一帧。

## 错误

| 错误 | 含义 |
| --- | --- |
| `CommandRejected` | 某一条命令被拒绝；状态没有任何改变 |
| `SafetyStop` | 堵转（机械臂保持；`on_stall: disable` 时卸力）或过温（卸力；`on_overtemp: hold` 时保持） |
| `BackendError` | 总线、CAN 或电机故障 |

伺服线程里抛出的错误先存进 `servo.error`，再由 `Arm.check()` 重新抛出。每条命令开始时、每次 `sleep()` 时都会调用
`Arm.check()`。

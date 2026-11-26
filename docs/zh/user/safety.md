# 安全

库里有多层保护。**但哪一层都代替不了放在电源开关上的那只手。**

| 保护层 | 位置 | 作用 |
| --- | --- | --- |
| 关节软限位 | `safety.soft_limits_deg`，与 URDF 的限位取交集 | 每个关节目标都限制在范围内 |
| 跳变保护 | `safety.max_jump_deg`（25） | 目标与上一个目标相差超过这个值时抛出 `CommandRejected`；机械臂保留上一个目标 |
| 速率限制 | `safety.max_joint_speed_deg_s`（90） | 关节目标的变化速度不超过这个值（30 Hz 下每周期 3 deg） |
| 低通滤波 | `safety.filter_tau_s`（0.1 s） | 滤掉目标里的阶跃和抖动 |
| 力矩上限 | `safety.max_error_deg`（10） | MIT 位置目标限制在实测角度 +- 10 deg 以内，所以 PD 力矩最大为 kp x 10 deg |
| 前馈限幅 | `feedforward.tau_max` | 限制前馈总量（重力、摩擦、力矩波动），以防模型有误 |
| 增益爬升 | `gains.ramp_s`（0.5 s） | 使能时增益和前馈从零逐渐升起，力矩不会突变 |
| 堵转检测 | `safety.lag_deg`、`stall_time_s`、`stall_progress_deg`、`on_stall` | 某个关节落后目标超过 10 deg 并持续 1 s，这段时间内移动又不到 1 deg，就抛出 `SafetyStop`；机械臂保持在原地（`on_stall: hold`） |
| 温度 | `safety.temp_warn_c`（115）、`temp_stop_c`（125）、`on_overtemp` | 超过警告值时记录一条警告；超过停机值时卸力（设为 `on_overtemp: hold` 则保持），并抛出 `SafetyStop` |
| 笛卡尔限制 | 工具文件的 `cartesian:` | 对 TCP 指令施加工作空间范围、速度、倾角和偏航限制；不可达的步会被拒绝 |
| 分阶段折叠 | `poses.fold_stages` | 正常退出时，先折叠基座 + 腕部，再折叠肩部 + 肘部 |

## 出问题时：机械臂保持不动

规则是：**只有人主动要求时才卸力。** 机械臂在半空中卸力会掉下来砸到东西。所以出故障时，库既不卸力，也不发起任何运动。
机械臂保持在原地，一直使能，等下一个程序或操作者来处理。

| 发生的情况 | 机械臂的表现 |
| --- | --- |
| `with` 代码块正常结束 | 分阶段折叠到零位，然后卸力 |
| 你的代码抛出异常、Ctrl-C、SIGTERM（CLI） | 保持在实测位姿，一直使能；程序退出 |
| 检测到堵转（`safety.on_stall: hold`，默认值） | 保持在实测位姿，抛出 `SafetyStop`，一直使能 |
| 伺服线程挂掉（总线错误、bug） | 条件允许时再发最后一帧保持帧，一直使能 |
| 程序被杀、电脑死机、CAN 线脱落 | 电机继续执行收到的最后一帧 |
| 某个驱动器超过 `temp_stop_c`（`safety.on_overtemp: disable`，默认值） | **卸力，机械臂掉落**，抛出 `SafetyStop` |
| `rebot-control release`、`arm.release()`、`arm.disable_torque()` | 卸力：这是操作者自己的决定 |

有两个设置可以改变上面的行为：`safety.on_stall: disable`（堵转时卸力；如果压到人比机械臂掉落更要紧，就选它）和
`safety.on_overtemp: hold`（电机过热时继续保持，直到驱动器自身的保护动作）。

### 程序退出后，保持如何延续

MIT 模式下，RobStride 驱动器会一直执行收到的最后一帧。它的 CAN 超时为 0 时（参数 0x7028；B601-RS 上读到的就是这个值，
每次连接都会检查），只要有电，它就一直执行下去。程序因出错而退出时，`Arm.detach()` 发出最后一帧，内容是"保持实测位姿，
目标速度为零，带上该位姿的重力前馈"，然后释放总线，不让任何电机失能。

```{warning}
保持中的机械臂没有任何程序看护。下一个程序连接之前，没有堵转检测，也没有温度保护。
电机持续输出维持该位姿所需的力矩，剩下的保护只有驱动器自带的那些。
```

### 接管

`Arm.connect()` 从不假定机械臂已经卸力。它先用参数读取得到关节角（参数读取不会替换电机正在执行的帧），再发一帧全增益的帧，
目标正好是这些角度。已使能的电机继续保持，未使能的电机不理会这一帧，两者都会回报自己的状态。如果电机已使能，`Arm`
无跳变地延续保持，并把 `arm.taken_over` 置为真；之后做什么由你的程序决定（`arm.move_home()`、`arm.fold()` 等）。

延续保持时，沿用的是原来的平衡点，不是实测角度：新目标的位置要让控制器输出的力矩等于电机此刻回报的力矩。
保持中的机械臂与目标之间总有一点偏差（重力模型误差、摩擦）。如果把目标改到实测角度，每接管一次，机械臂就会移动这么一点。
出于同样的原因，停止时，原本跟得上目标的关节保留原目标，只有离目标很远的关节才重新设定目标。

只读取状态的程序（`rebot-control state`，或者一个从不调用 `enable()` 和运动指令的 `with` 代码块）不会改变保持中的机械臂。
这时 `state` 会打印 `torque: ON, holding`。

### 释放

```bash
rebot-control release --fold      # fold to the zero pose from where it is, then torque off
rebot-control release             # torque off now: hold the arm with your hand first
```

两条命令都会要求确认。在代码里，`arm.release(fold=True)` / `arm.release()` 的效果相同。

**急停手段：切断电机电源**（先扶住机械臂，它会掉下来）。

## 在真机上运行新代码之前

1. 先用 `--sim`（或 `backend="sim"`）运行同一个脚本，并打开查看器观察。
2. 从慢速开始：把 `max_speed` 调低，把工作空间范围设小。
3. 代码在 home 位姿验证无误之前，让机械臂保持折叠。

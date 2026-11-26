# 伺服循环与时序

## 频率

| 循环 | 频率 | 谁在跑 |
| --- | --- | --- |
| 调用方 | `control_hz` = 30 Hz | 用户代码、`Arm.sleep()`；IK 和关节目标滤波器在这一层运行 |
| 伺服 | `servo_hz` = 150 Hz | `ServoLoop.tick`：每个 tick 给每个电机发一帧 MIT 帧 |
| 驱动器 | 约 10-20 kHz（RobStride 内部） | 对保持住的那一帧执行 MIT 的 PD 控制律 |
| 仿真器 | `physics_hz` = 1050 Hz | `SimBackend.advance`，每个物理步执行一次 PD 控制律 |

把 30 Hz 的目标直接发给电机，会产生听得见的 30 Hz 台阶。所以伺服循环以 150 Hz 在目标之间插值。

## HOLD 帧

设 tick 时刻为 `t`，上一个目标为 `from`，最新目标 `to` 在 `t_set` 时刻设定，控制周期为 `Tc`：

```
a      = min(1, (t + Ts/2 - t_set) / Tc)        Ts = servo period
q_cmd  = from + a (to - from)
dq_cmd = (to - from) / Tc   while (t + Ts/2 - t_set) / Tc < 1.25, else 0
q_cmd  = q_meas + clip(q_cmd - q_meas, +-max_error)          torque cap
tau_ff = clip(gravity(q_meas) * gravity_scale + friction(dq_cmd) * friction_scale
              + ripple(q_meas) * ripple_scale, +-tau_max)
kp, kd, tau_ff  x  ramp(t)                                    after leaving PASSIVE
```

### 为什么要提前半个伺服周期

驱动器把每一帧的位置保持整整一个伺服周期（零阶保持）。如果 `q_cmd` 取的是周期*起点*处的轨迹值，
保持住的目标平均会比轨迹滞后 `Ts/2`。PD 就会多出一个与速度成正比的力矩 `kp v Ts / 2`，看上去和粘性摩擦一模一样。
在 kp = 40、速度 1 rad/s 时，它是 0.13 N*m。

在 `t + Ts/2` 处计算插值，保持住的目标就落在轨迹的中间。仿真里的摩擦辨识，是在做了这个改动，*并且*把仿真的
力矩读数改成周期平均值之后，才辨识出了真实的粘性系数（只在周期末尾采一个瞬时值，会带来同样的偏差）。

真机上的情况取决于驱动器在周期内的哪个时刻采样它上报的电流。如果辨识出的粘性摩擦随 `kp` 变化，原因就在这里。
这一项列在 [hardware-notes.md](hardware-notes.md) 里。

### 为什么速度要留宽限

在真机上，由于线程抖动，下一个目标可能在 `a` 到 1 之后才稍迟到达。如果没有 1.25 这个宽限，`dq_cmd` 会有一个 tick
掉到 0，`kd` 就会每个控制周期刹一次车：也就是 30 Hz 的波动。

## GRAVITY 帧

`kp = 0`，`kd = gravity_mode_kd`，`q_cmd = q_meas`，`dq_cmd = 0`，
`tau_ff = clip(gravity(q) * gravity_scale + friction(dq_meas) * friction_scale + ripple(q) * ripple_scale, +-tau_max)`。摩擦按实测速度计算，所以开了摩擦补偿后，手动拖动会更轻。

## 线程时序（真机）

`ServoLoop._run` 用 `time.perf_counter()` 计算每个 tick 的截止时间。某个 tick 超时后，它会重新同步，并把
`servo.overruns` 加一。不算总线，一个 tick 大约耗时 20 us（实测值；其中 Pinocchio 的重力计算占 2 us）。
真实总线上 6 次 MIT 写入加一次轮询的耗时还有待测量：在 1 Mbit/s 下，12 帧大约占 1.5 ms 的总线时间，
远小于 6.7 ms 的周期。主要风险是 Python 的 GC 停顿和 CPU 繁忙。CLI 的 `hold` 命令会打印伺服超时的次数。

## 阶跃目标

`set_target(q, t, step=True)` 在下一个 tick 直接发出 `q`，不做插值。只有增益的阶跃测试用到它。
如果在一个 tick 之前连续调用两次 `set_target`，第二次调用会从最后一帧*已发出*的帧开始插值。
所以 `step_response` 在阶跃之后先等一个周期，再开始连续下发保持目标。

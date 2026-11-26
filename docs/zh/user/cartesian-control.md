# 笛卡尔控制

```python
cart = arm.cartesian()                  # starts at the current target pose; enables HOLD if needed
for k in range(n):
    st = cart.step(p_des, R_des)        # R_des=None keeps the current orientation
    arm.sleep()                         # one control period (30 Hz)
```

每个周期，`step()` 依次做下面几件事：

1. 对期望位姿施加**工作空间和姿态限制**：
   - 位置限制在 `x`、`y`、`z` 的工作空间范围内（基座坐标系）；
   - 设置了 `max_tilt_deg` 时，工具轴（TCP 的 z 轴）与竖直向下方向的夹角不超过这个值；
   - 设置了 `max_yaw_deg` 时，TCP x 轴的朝向不超过这个角度。
2. **速度限制**，相对于上一次*下发的*位姿计算：`max_speed`（m/s）和 `max_ang_speed`（rad/s）。目标较远时，TCP
   以这个速度沿直线靠近。`arm.move_to_pose` 走直线也是靠这一步。
3. **跟随误差减速**：某个关节的跟随误差超过 `safety.slow_lag_deg`（20 deg）时，速度限制线性收紧，误差达到两倍时降到零。
   这样机械臂能追上指令，指令不会越跑越远。
4. **IK**（阻尼最小二乘，带 posture 项，以上一次的解为初值）。如果限制后的位姿在 5 mm / 3 deg 的误差内不可达，这一步会被**拒绝**：
   保留上一个位姿，机械臂保持不动。
5. 关节目标交给 `arm.command_joint`：依次经过软限位、跳变保护、速率限制、低通滤波，再进入伺服循环。

返回的 `CartesianStatus` 包含：

| 字段 | 含义 |
| --- | --- |
| `pos`、`rot` | 施加限制之后实际下发的位姿 |
| `q` | 关节目标 |
| `ik_ok` | 这一步被拒绝时为 False |
| `clipped` | 有限制改动了期望位姿时为 True |
| `lag_deg` | 这一步之前的关节跟随误差 |
| `speed_factor` | 减速期间小于 1 |

## 限制

默认值来自工具文件的 `cartesian:` 段。可以为每个控制器单独指定：

```python
from rebot_control import CartesianLimits
cart = arm.cartesian(CartesianLimits(x=(0.25, 0.45), y=(-0.1, 0.1), z=(0.01, 0.15), max_speed=0.1, max_tilt_deg=20))
```

工作空间范围只是一个粗略的边界。范围内哪些位姿可达，由第 4 步的 IK 决定。用 `cart.reachable(p, R)`
可以在不运动的情况下检查一个位姿。

为了让示教数据和实际执行一致，训练之前请对录制的数据施加同样的 `CartesianLimits`（`limits.clip_position`、
`limits.clip_orientation`）。这样策略学到的动作在机器人上就不会再被裁剪。

## 可以预期的精度

- **静止时**：有重力前馈时，仿真里的关节稳态误差是百分之几度。真机上它取决于重力模型，所以要标定重力。
- **运动时**：关节低通滤波（`filter_tau_s` = 0.1 s）让运动滞后约 0.1 s，所以实测的 TCP 落后于指令约 速度 x 0.1 s
  （5 cm/s 时为 5 mm）。想跟得更紧，可以调低 `filter_tau_s`（这时输入是否平滑就更重要），并开启摩擦前馈。

## 相对运动

`rebot_control.pose.pose_delta` 和 `apply_delta` 在起始位姿的坐标系下表示一段运动。输出位姿增量的策略可以用它们；
在另一个世界坐标系下录制的数据也可以用它们。

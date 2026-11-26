# 标定

每个标定工具都按同样的流程工作：

- 让机械臂运动（或提示你手动操作）；
- 把原始数据保存到 `calibration/data/*.npz`；
- 拟合模型；
- 把结果写入标定文件（`calibration/<arm>.yaml`）或工具 YAML。

已保存的数据文件可以用 `--from-data <file.npz>` 离线重新拟合。

推荐顺序：

1. **增益检查**：确认当前配置的增益没有问题（[gains.md](gains.md)）。
2. **重力**，尽量用空法兰。
3. **摩擦**。
4. **负载**，装上工具后做。
5. **TCP**。

每一步都要用到前面各步的结果：从第 3 步起加上 `--calibration calibration/b601_rs.yaml`。

## 重力：连杆质量与质心

```bash
rebot-control calib gravity --tool none --poses 30
```

- **位姿。** 在保守的关节范围内随机取构型，并逐个做运动学检查：相邻关节原点之间的连线、以及到 TCP 的连线，
  都必须高于 `--table-z`（默认 4 cm），并避开基座立柱；相邻两个位姿之间的关节空间直线路径也要满足同样的条件。
  这些位姿会排成一条较短的巡回路线，在机械臂运动之前先打印出来。
- **测量。** 机械臂在每个位姿保持静止（HOLD 模式，重力前馈取自当前模型），等待 `--settle` 秒稳定后，
  对电机力矩取 0.5 s 的平均值。每个位姿都从两侧各接近一次（上下各偏 3 deg），让静摩擦相互抵消。
- **模型。** 静止时，测得的力矩 = `Y(q) theta + b`。`Y` 是静态关节力矩回归矩阵（由 Pinocchio 计算），
  `theta` 是每个连杆的质量和一阶矩 `m, m*c`，`b` 是每个关节的力矩偏置。静态数据无法辨识全部参数，
  所以拟合采用以先验（URDF 或上一次的标定结果）为中心的 MAP / 岭估计。数据看不到的参数会停在先验值上，
  不会漂走。
  - `--prior-rel 0.3`（默认值）允许质量变化约 30 %。想更信任 URDF，就把它调小。
- **输出。** 标定文件中的 `link_params`（每个连杆的质量和质心）。转动惯量沿用 URDF 的值（静止时它不起作用）。
  如果装着工具，会从 link6 中扣除工具配置里的负载。
- **检验。** 比较拟合前后的残差 RMS。之后运行 `rebot-control float --calibration ...`，漂移应当变小。

## 摩擦：各关节的库仑摩擦 + 粘性摩擦

```bash
rebot-control calib friction --tool <tool> [--joints 4,5,6] [--speeds 5,10,20,40,60] [--amplitude-scale 0.5]
```

- 每次只让一个关节在 home 位姿（或 `--center`）附近往复扫描，每个速度各做一遍，速度曲线为梯形。
  其余关节保持不动。
- 匀速段满足：`tau - rnea(q, dq, 0) = fc tanh(v / vs) + fv v + (a sin q + b cos q + c)`。最后一项用来吸收
  重力模型沿扫描行程的误差，所以摩擦标定放在重力标定之前或之后都可以。
- 库仑摩擦按**对称**拟合（`fc_pos = fc_neg`）。仅凭这些数据，无法把与方向有关的部分和恒定的力矩偏置区分开。
- **输出。** 标定文件中的 `friction`。只有 `feedforward.friction_scale > 0` 时才会用到它。从 0.5 开始：
  补偿过头，机械臂在重力补偿模式下会自己慢慢爬。

## 力矩波动：随关节角周期变化

带减速器的无刷电机要转得均匀，所需的力矩会随转子角度变化，每个电周期重复一次（关节每转的周期数 =
极对数 x 减速比；RS-06 上是 126，即每 2.86 deg 一个周期）。低速时位置环压不住它，慢速的笛卡尔直线会走得
一颠一颠；速度高了，惯量会把它滤掉。

```{warning}
力矩波动前馈仍是实验性功能。自动化测试只在仿真中覆盖了它，目前也没有对应的 CLI 命令。模型的相位不对，
跟踪反而会变差。先对比同一段慢速运动在开、关前馈时的表现，确认有改善后再使用。
```

用代码采集慢速扫描数据并拟合：

```python
from rebot_control.calib.friction import collect_sweeps
from rebot_control.calib.ripple import fit_joint_ripple, to_ripple_params

data = collect_sweeps(arm, joint, center, math.radians(15), speeds=(math.radians(3), math.radians(6)))
fit = fit_joint_ripple(arm.model, data)          # periods per turn, sine / cosine terms, share explained
Calibration(ripple=to_ripple_params([fit], arm.n)).save("calib.yaml")
```

- **输出。** 标定文件中的 `ripple`：每个关节的 `cycles`、`a1`、`b1`、`a2`、`b2`。标定文件里只要有 ripple 一节，
  加载后就会生效；`feedforward.ripple_scale`（默认 1）是它的缩放系数，设为 0 即关闭。
- **它实际上是一种位置反馈。** 力矩按实测角度计算，所以幅值为 `A` 的模型相当于一个最大为 `N * A` N*m/rad
  的刚度，相位不对时是负刚度。`N * A > 0.75 * kp` 的模型，`Arm` 会拒绝使用。
- **它与关节零位绑定。** 电机重新标零后，相位就不对了，需要重新标定。
- 扫描时设 `feedforward.ripple_scale = 0`，速度要慢（3 到 6 deg/s），行程不少于 20 deg。

## 负载：工具质量与质心

```bash
rebot-control calib payload --tool my_tool.yaml --calibration calibration/b601_rs.yaml --out my_tool.yaml
```

静态数据的采集方式与重力标定相同（默认 15 个位姿），但只拟合负载的四个参数，机械臂各连杆的参数固定不动。
前提是重力已经标定过。新生成的工具 YAML 里会写入 `payload.mass_kg` 和 `payload.com_xyz`，并附一条
`calibrated:` 备注。

## TCP：支点标定

```bash
rebot-control calib tcp --tool my_tool.yaml --out my_tool.yaml --poses 8
```

交互式操作，只能在真机上做：

1. 机械臂先到 home 位姿，然后进入重力补偿模式。
2. 把工具尖端放进一个固定的凹坑。
3. 保持尖端不离开凹坑，把工具倾斜到一个新的姿态，稳住后按回车。重复若干次。

各次的法兰位姿满足 `R_i t + p_i = c`。最小二乘解给出尖端在法兰坐标系下的位置 `t` 和支点 `c`。目标是：

- 残差 RMS 小于约 1 mm；
- 姿态的变化范围至少 30 deg，并覆盖两个不同的倾斜方向。

只辨识 TCP 的位置。TCP 的姿态沿用配置值。

## 增益

见 [gains.md](gains.md)。`rebot-control calib gains --save` 把 `gains.kp` / `gains.kd` 写入标定文件，
加载标定文件时生效。

## 在代码中调用

```python
from rebot_control.calib.gravity import collect_static, fit_gravity
from rebot_control.calib.poses import generate_poses
poses = generate_poses(arm.model, 30, arm.q())
data = collect_static(arm, poses)
fit = fit_gravity(arm.model, data)
print(fit.summary(arm.cfg.joints))
```

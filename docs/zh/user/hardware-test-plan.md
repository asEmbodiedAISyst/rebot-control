# 真机测试计划

本页用于新机械臂的上机调试，也用于在一台机械臂上第一次启用本库。测试逐项进行，后一步依赖前一步，不要跳步。
每条命令都能在仿真里运行（`--sim`）：先在仿真里跑一遍，看看正常的输出是什么样。

把结果记在页末的表格里；遇到意料之外的现象，也记下来。

## 开始之前

- 使用装有 USB-CAN 适配器的 Linux 机器，`can0` 处于 UP 状态（[installation.md](installation.md)）。
  安装：`pip install -e ".[real,sim]"`。
- 机械臂已上电，折叠在零位。周围的桌面已清空（半径约 60 cm）。
- 整个过程中，一只手始终放在**电机电源开关**旁。卸力后机械臂会下落，预计要卸力时，用另一只手托住它。
- `--tool` 填**实际安装的**工具（`none`、`pusher_ip17`、`stock_gripper` 或你自己的 YAML）。
  负载填错，重力前馈就是错的。
- 凡是会让机械臂运动的命令，CLI 都会先要求确认。确认工作空间内没有障碍后再回答。

## 1. 卸力状态下读反馈：`rebot-control state`

```bash
rebot-control state --tool <tool>
rebot-control state --tool <tool> --watch      # 20 readings, 0.5 s apart
```

| 检查项 | 通过标准 |
| --- | --- |
| 6 个电机都有应答 | 没有 "no feedback from motors" 报错 |
| 折叠位姿下的角度 | 每个关节都在 0 附近约 2 deg 以内 |
| 温度 | 接近室温，6 个电机相差不大 |
| `folded: True` | 是 |

**方向检查。** 在卸力状态下用手转动各关节，同时看 `--watch` 的输出。抬起任何部位之前，先托住机械臂。
各关节的正方向：

| 关节 | 读数为正的方向 |
| --- | --- |
| 1（基座） | 从上往下看，机械臂**顺时针**转 |
| 2（肩） | 大臂从折叠位姿抬起（行程从 0 开始） |
| 3（肘） | 小臂从折叠位姿张开（行程从 0 开始） |
| 4（腕俯仰） | 在折叠位姿下，手腕向**上**翘 |
| 5（腕偏航） | 在折叠位姿下，从上往下看，手腕**逆时针**转 |
| 6（腕滚转） | 正对法兰端面看，**顺时针**转 |

方向不对，说明机械臂配置里的 `direction` 与实际不符。**所有方向都正确之前，不要上力矩。**

## 2. 在折叠位姿上力矩：`rebot-control hold`

```bash
rebot-control hold --tool <tool> --seconds 10
```

| 检查项 | 通过标准 |
| --- | --- |
| 使能 | 没有跳动；机械臂在约 0.5 s 内逐渐变硬（增益斜坡） |
| 跟随误差 | 小于 0.5 deg |
| 伺服超时 | 保持为 0，或只是缓慢增加（每秒多于约 1 次，说明电脑太慢或负载太高） |
| 声音 | 没有嗡嗡声或啸叫 |
| 退出 | 机械臂保持折叠，已卸力 |

## 2b. 程序异常退出后的保持、接管与释放

本计划后面的所有步骤都依赖这一项，所以先在折叠位姿下检查一次：在这个位姿下，即使掉落也没有危险。
背景见 [safety.md](safety.md)。

```bash
rebot-control state --tool <tool>                    # torque: off
rebot-control hold --tool <tool> --seconds 60        # while it holds: Ctrl-C  -> it folds and switches off (normal end)
rebot-control hold --tool <tool> --seconds 60 &      # while it holds: kill -9 %1  (a program that dies)
rebot-control state --tool <tool>                    # must print "torque: ON, holding (left as found)"
rebot-control release --tool <tool>                  # hand on the arm; torque off
```

| 检查项 | 通过标准 |
| --- | --- |
| 对已卸力的机械臂执行 `state` | 没有任何动作，也不变硬；显示 `torque: off` |
| `kill -9` 之后 | 机械臂保持刚性、不动；此时没有任何程序在运行 |
| 对保持中的机械臂执行 `state` | 连接和断开时都没有抖动；显示 `torque: ON, holding (left as found)`；之后机械臂仍保持刚性 |
| 对保持中的机械臂再次执行 `hold` | 日志输出 "the motors are enabled: taking over"；没有跳动 |
| `release` | 先询问确认；卸力；之后 `state` 显示 `torque: off` |
| 连接日志 | 没有关于 CAN 超时的警告 |

然后把肩和肘抬起几度（一只手托在小臂下方），再做一次 `kill -9`：机械臂必须停在原处，不能下落。

## 3. 重力补偿模式：`rebot-control float`

```bash
rebot-control float --tool <tool> --seconds 60
```

用手把机械臂从折叠位姿慢慢带出来，先动肩和肘。在几个不同的位姿松手。

| 检查项 | 通过标准 |
| --- | --- |
| 手感 | 轻，没有哪个方向会自己推出去 |
| 松手后的漂移 | 几度以内；记下哪些关节漂、往哪边漂（重力标定要修正的就是这个） |
| 退出（Ctrl-C） | 先在原位保持，再分阶段慢慢折叠回去 |

如果机械臂朝某个方向用力推，立即停下（按 Ctrl-C；或者托住机械臂后关电源）。然后检查方向（第 1 步）和
`--tool` 的负载。

## 4. 展开、home、折叠：`rebot-control home`

```bash
rebot-control home --tool <tool> --seconds 5
```

| 检查项 | 通过标准 |
| --- | --- |
| 展开 | 先肩和肘，再手腕；没有任何部位相互摩擦 |
| home 位姿 | 打印的 q 与工具的 `home_deg` 相差不超过 0.5 deg |
| TCP | 打印的 TCP 位置与直尺量出的一致，误差约 1 cm 以内（基座坐标系：x 向前，z 从桌面向上） |
| 折叠 | 先基座和手腕，再肩和肘；最后回到零位 |

## 5. 关节运动与连续下发：`examples/03_joint_moves.py`

```bash
cd examples && python 03_joint_moves.py --tool <tool>
python 06_record.py --tool <tool>        # servo log -> servo_log.npz
```

| 检查项 | 通过标准 |
| --- | --- |
| 慢速运动 | 平滑，没有台阶感或顿挫 |
| 正弦目标连续下发 | "max lag" 小于约 3 deg |
| `servo_log.npz` | `q_cmd - q` 很小（小于 1 deg）；保留这个文件 |

**堵转测试**（可选）。在慢速运动中用手挡住基座关节，约 1 s 后应出现 `SafetyStop`。
默认的 `safety.on_stall: hold` 下，机械臂停在原地保持，仍处于使能状态，用 `rebot-control release --fold` 结束保持。
如果设成 `on_stall: disable`，堵转后会卸力，**机械臂会掉下来**：用这个设置做测试之前，必须有第二个人托住小臂。

## 6. 笛卡尔运动：`rebot-control square`、`examples/04_cartesian.py`

```bash
rebot-control square --tool <tool> --side 0.08 --speed 0.05
cd examples && python 04_cartesian.py --tool <tool>
```

| 检查项 | 通过标准 |
| --- | --- |
| 正方形 | 各边是直线；拐角与打印的目标相差约 2 mm 以内 |
| 跟踪误差 | 运动中为几毫米（滤波带来的滞后，属正常现象），静止时小于 1 mm |
| 姿态 | 沿边运动时，工具轴不晃动 |
| IK 拒绝 | 这样的小幅运动中不应出现 |

## 7. MIT 增益检查：`rebot-control calib gains --check`

```bash
rebot-control calib gains --tool <tool> --check            # 2 deg steps on every joint, configured gains
```

**做阶跃时注意听声音。** 记下每个关节的超调、调节时间、残余速度和力矩波动。典型结果是超调小于 10 %、
调节时间小于 0.3 s。一旦某个关节嗡嗡响或振荡不止，立即停止。

可选：一次调一个关节，从腕部关节开始：

```bash
rebot-control calib gains --tool <tool> --joints 6 --kp-cap 2.0
```

这条命令逐步提高 kp，直到触发某项限制（振荡、嗡鸣、超调）或达到上限，然后打印 `kp_max` 和推荐增益。
不加 `--save` 就不会保存任何结果。修改增益之前先读 [gains.md](gains.md)。

## 8. 重力标定：`rebot-control calib gravity`

条件允许时用空法兰（`--tool none`）做。否则带着已安装的工具做，并确保它的负载参数正确。

```bash
rebot-control calib gravity --tool none --poses 30          # about 5-10 min
rebot-control float --tool none --calibration calibration/b601_rs.yaml   # compare with step 3
```

| 检查项 | 通过标准 |
| --- | --- |
| 位姿 | 全部与桌面保持间距；CLI 会先打印位姿列表 |
| 拟合后的残差 RMS | 明显低于拟合前，关节 2、3 通常低于 0.1 N*m |
| 辨识出的质量 | 数值合理（没有负值或离谱的值；有问题时会打印警告） |
| 带标定结果的 float | 漂移比第 3 步小 |

原始数据保存在 `calibration/data/` 下。重新拟合不需要机械臂：
`rebot-control calib gravity --from-data <file.npz> --prior-rel 0.2`。

## 9. 摩擦：`rebot-control calib friction`

先做一个腕部关节，幅度取小一些：

```bash
rebot-control calib friction --tool <tool> --joints 6 --amplitude-scale 0.5
rebot-control calib friction --tool <tool>                  # all joints, after the first one looks right
```

然后把机械臂配置复制一份，在副本里打开摩擦前馈（`feedforward.friction_scale: 0.5`），重做 float 测试和
笛卡尔正方形。

| 检查项 | 通过标准 |
| --- | --- |
| 扫描 | 匀速段运动平滑，没有堵转 |
| 拟合 | fc 和 fv 为正；RMS 远小于 fc |
| `friction_scale: 0.5` 时 | 重力补偿模式下手感更轻；走正方形时跟踪误差更小 |

## 10. 工具负载与 TCP

```bash
rebot-control calib payload --tool <tool> --calibration calibration/b601_rs.yaml --out my_tool.yaml
rebot-control calib tcp --tool my_tool.yaml --calibration calibration/b601_rs.yaml --out my_tool.yaml
```

- **负载**：在重力标定之后、装着工具时做。它会写出一份工具 YAML，里面是辨识出的质量和质心。
- **TCP**：在固定于桌面的块上做一个小凹坑（钻孔或锥坑）。在重力补偿模式下把工具尖端放进去，把工具倾斜到
  一个新的姿态，稳住后按回车。重复 8 次，各次姿态之间至少相差 30 deg。通过标准：残差 RMS 小于约 1 mm。

## 结果记录表

| 步骤 | 日期 | 工具 | 结果 | 数值 / 备注 |
| --- | --- | --- | --- | --- |
| 1 state + 方向 | | | | |
| 2 hold | | | | 伺服超时次数： |
| 2b 程序异常退出后的保持 / 接管 / 释放 | | | | |
| 3 float | | | | 各关节漂移： |
| 4 home | | | | TCP 实测值与打印值： |
| 5 关节运动 | | | | 最大跟随误差： |
| 6 笛卡尔运动 | | | | 拐角误差： |
| 7 增益检查 | | | | 各关节超调 / 调节时间： |
| 8 重力标定 | | | | RMS 拟合前 -> 拟合后： |
| 9 摩擦 | | | | 各关节 fc、fv： |
| 10 负载 / TCP | | | | 质量、质心；TCP RMS： |

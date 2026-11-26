# 给使用 rebot-control 的 agent

帮用户运行机械臂，或基于 `rebot_control` 写代码时，按本页执行。

## 规则

1. 在仿真中运行：Python 里用 `backend="sim"`，命令行加 `--sim`。只有用户在当前会话中要求、并且人在机械臂
   旁边时，才在真机上运行。
2. 上真机之前，先把同一个脚本在仿真里跑一遍。
3. 在真机上不要加 `--yes`。确认提示由用户回答。
4. 传入实际安装的工具（`tool=`、`--tool`）。负载不对，重力前馈就不对。
5. 不要为了让命令通过而修改 `safety.*`、`feedforward.tau_max` 或笛卡尔限制。命令遭到拒绝时
   （`CommandRejected`、`SafetyStop`、`ik_ok == False`），把情况报告给用户。
6. 不要在真机上卸力（`arm.release()`、`arm.disable_torque()`、不带 `--fold` 的 `rebot-control release`），
   除非用户要求。卸力后机械臂会下落。
7. Python 里角度用弧度。度只出现在以 `_deg` 结尾的 YAML 键中。

## 安装

```bash
pip install -e ".[sim]"          # simulation, any OS
pip install -e ".[real,sim]"     # real arm: Linux, SocketCAN, can0 up
```

检查安装：`rebot-control state --sim`。

## Python

```python
from rebot_control import Arm

with Arm.from_config("b601_rs", tool="pusher_ip17", backend="sim") as arm:
    arm.enable()                      # torque on, hold the measured pose
    arm.move_home()                   # unfold to the tool's home pose
    p, R = arm.tcp_pose()             # position (3,), rotation matrix (3, 3), base frame
    arm.move_to_pose(p + [0.05, 0.0, 0.0], R, speed=0.05)

    cart = arm.cartesian()            # streaming control
    for _ in range(90):
        status = cart.step(p + [0.05, 0.03, 0.0], R)
        arm.sleep()                   # one control period (30 Hz); call it once per cycle
# normal exit: fold, then torque off
```

| 任务 | 调用 |
| --- | --- |
| 加载机械臂 | `Arm.from_config(arm, tool=, backend=, calibration=, overrides=)` |
| 关节运动（阻塞） | `arm.move_to_joint(q, speed=)` |
| 关节目标（连续下发） | `arm.command_joint(q)`，然后 `arm.sleep()` |
| 笛卡尔运动（阻塞） | `arm.move_to_pose(p, R, speed=)` |
| 笛卡尔目标（连续下发） | `arm.cartesian().step(p, R)`，然后 `arm.sleep()` |
| 手动拖动 | `arm.gravity_mode()`，用 `arm.hold()` 退出 |
| 更换工具 | `arm.set_tool(name_or_path)` |
| 记录伺服数据 | `with arm.recording() as rec: ...` |

## 命令行

每条命令都接受 `--arm`、`--tool`、`--calibration`、`--sim`、`--viewer`。

| 命令 | 作用 |
| --- | --- |
| `rebot-control state` | 打印角度、温度、TCP 位姿和力矩开关状态。不改变任何状态。 |
| `rebot-control hold` | 在当前位姿上力矩。 |
| `rebot-control float` | 重力补偿模式。 |
| `rebot-control home` | 展开到 home 位姿，再折叠回去。 |
| `rebot-control square` | 笛卡尔正方形测试。 |
| `rebot-control release --fold` | 把留在保持状态的机械臂折叠好，然后卸力。 |
| `rebot-control calib gravity\|payload\|friction\|tcp\|gains` | 标定工具。加 `--from-data` 时用已保存的数据重新拟合，机械臂不运动。 |

## 程序结束时机械臂的状态

| 程序怎样结束 | 机械臂的状态 |
| --- | --- |
| `with` 块正常结束 | 已折叠，已卸力 |
| 异常、Ctrl-C、堵转 | 保持当前位姿，仍然使能 |

程序退出后，保持中的机械臂仍然使能。下一个 `Arm` 在 `connect()` 时接管这次保持，并把 `arm.taken_over`
置为真。程序让机械臂留在保持状态时，告诉用户。

## 去哪里查

| 问题 | 页面 |
| --- | --- |
| 术语、模式、控制链路 | [`../user/concepts.md`](../user/concepts.md) |
| 生命周期、时序、关节与笛卡尔调用 | [`../user/quickstart.md`](../user/quickstart.md) |
| YAML 键、单位、坐标系、新工具 | [`../user/configuration.md`](../user/configuration.md) |
| 笛卡尔限制、被拒绝的步 | [`../user/cartesian-control.md`](../user/cartesian-control.md) |
| 标定命令及其顺序 | [`../user/calibration.md`](../user/calibration.md) |
| 停止、保持、接管、释放 | [`../user/safety.md`](../user/safety.md) |
| 报错信息 | [`../user/troubleshooting.md`](../user/troubleshooting.md) |
| 新机械臂的第一次上机 | [`../user/hardware-test-plan.md`](../user/hardware-test-plan.md) |

全部页面合并成的单个文件：文档站 `zh/` 下的 `llms-full.txt`。

# rebot-control

[English](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/README.md) | 中文

面向 **Seeed reBot Arm B601-RS**（6 个 RobStride 电机，SocketCAN 通信）的控制库：笛卡尔控制、可配置的工具、自标定，
并附带一个 MuJoCo 仿真器，仿真和真机跑的是同一套控制代码。

装上任意工具，用一个小的 YAML 文件写明它的 TCP 位姿和负载，标定一遍，就可以在笛卡尔空间里直接控制工具尖端。

本项目是独立实现，与 Seeed Studio 没有隶属关系，也未获得 Seeed Studio 的认可。

> **状态：0.1.0，预发布。** 自动化测试只在仿真中运行，真机验证还在进行。这个库驱动的是真实的机械臂，
> 给真机上电之前请先读[安全说明](https://asembodiedaisyst.github.io/rebot-control/zh/user/safety.html)。

## 安装

```bash
git clone https://github.com/asEmbodiedAISyst/rebot-control.git && cd rebot-control
pip install -e ".[sim]"          # simulation only (Linux, macOS)
pip install -e ".[real,sim]"     # + motorbridge for the real arm (Linux + SocketCAN)
```

## 快速上手

```python
from rebot_control import Arm

with Arm.from_config("b601_rs", tool="pusher_ip17", backend="sim") as arm:   # backend="real" on the arm
    arm.enable()                      # torque on, hold the measured (folded) pose
    arm.move_home()                   # unfold in stages to the tool's home pose
    p, R = arm.tcp_pose()
    arm.move_to_pose(p + [0.10, 0.0, 0.0], R, speed=0.05)   # straight line, 5 cm/s
# leaving the block folds the arm back to zero and switches torque off
```

```bash
rebot-control state --sim                 # angles, temperatures, TCP pose; changes nothing
rebot-control square --sim --viewer --tool pusher_ip17   # Cartesian square test in the simulator
```

## 文档

文档站：<https://asembodiedaisyst.github.io/rebot-control/zh/>

| 读者 | 入口 |
| --- | --- |
| 使用者 | [用户指南](https://asembodiedaisyst.github.io/rebot-control/zh/user/)：安装、快速上手、工具、笛卡尔控制、标定、安全 |
| 开发者 | [开发者指南](https://asembodiedaisyst.github.io/rebot-control/zh/developer/)：架构、约定、设计决策、测试 |
| AI agent | [AGENTS.md](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/AGENTS.md)、[agent 指南](https://asembodiedaisyst.github.io/rebot-control/zh/agents/)、[llms.txt](https://asembodiedaisyst.github.io/rebot-control/zh/llms.txt)、[llms-full.txt](https://asembodiedaisyst.github.io/rebot-control/zh/llms-full.txt) |

## 许可证

代码采用 MIT 许可证，见 [LICENSE](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/LICENSE)。随包附带的 URDF 源自 Seeed Studio 的 reBot Arm B601-RS 资料，
仍按 CERN-OHL-W-2.0 授权，见 [NOTICE](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/NOTICE)。本库依赖 [motorbridge](https://pypi.org/project/motorbridge/)、
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) 和 [MuJoCo](https://mujoco.org)。

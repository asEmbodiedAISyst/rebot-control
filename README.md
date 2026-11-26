# rebot-control

English | [中文](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/README.zh-CN.md)

Cartesian control, configurable tools and self-calibration for the **Seeed reBot Arm B601-RS**
(6 RobStride motors on SocketCAN), with a MuJoCo simulator that runs the same control code.

Mount a tool, describe it in a small YAML file (TCP pose and payload), calibrate it, and command the tool tip
in Cartesian space.

This project is an independent implementation. It is not affiliated with Seeed Studio, and Seeed Studio does not
endorse it.

> **Status: 0.1.0, pre-release.** The automated tests run in simulation only, and real-arm validation is in
> progress. This library drives a real robot arm: read the [safety page](https://asembodiedaisyst.github.io/rebot-control/en/user/safety.html) before you power one.

## Install

```bash
git clone https://github.com/asEmbodiedAISyst/rebot-control.git && cd rebot-control
pip install -e ".[sim]"          # simulation only (Linux, macOS)
pip install -e ".[real,sim]"     # + motorbridge for the real arm (Linux + SocketCAN)
```

## Quick start

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

## Documentation

Website: <https://asembodiedaisyst.github.io/rebot-control/>

| Reader | Start here |
| --- | --- |
| Users | [User guide](https://asembodiedaisyst.github.io/rebot-control/en/user/): installation, quick start, tools, Cartesian control, calibration, safety |
| Developers | [Developer guide](https://asembodiedaisyst.github.io/rebot-control/en/developer/): architecture, conventions, design decisions, testing |
| AI agents | [AGENTS.md](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/AGENTS.md), [agent guides](https://asembodiedaisyst.github.io/rebot-control/en/agents/), [llms.txt](https://asembodiedaisyst.github.io/rebot-control/llms.txt), [llms-full.txt](https://asembodiedaisyst.github.io/rebot-control/llms-full.txt) |

## License

MIT, see [LICENSE](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/LICENSE). The bundled URDF is derived from Seeed Studio's reBot Arm B601-RS files and stays
under CERN-OHL-W-2.0, see [NOTICE](https://github.com/asEmbodiedAISyst/rebot-control/blob/main/NOTICE). The library uses [motorbridge](https://pypi.org/project/motorbridge/),
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) and [MuJoCo](https://mujoco.org).

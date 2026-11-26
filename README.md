# rebot-control

English | [简体中文](README.zh-CN.md)

Cartesian control, configurable tools and self-calibration for the **Seeed reBot Arm B601-RS**
(6 RobStride motors on SocketCAN), with a MuJoCo simulator that runs the same control code.

Mount a tool, describe it in a small YAML file (TCP pose and payload), calibrate it, and command the tool tip
in Cartesian space.

> **Status: 0.1.0, pre-release.** The automated tests run in simulation only, and real-arm validation is in
> progress. This library drives a real robot arm: read the [safety page](docs/user/safety.md) before you power one.

## Install

```bash
git clone https://github.com/asEmbodiedAISyst/rebot-control.git && cd rebot-control
pip install -e ".[sim]"          # simulation only (any OS)
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
| Users | [docs/user/](docs/user/index.md): installation, quick start, tools, Cartesian control, calibration, safety |
| Developers | [docs/developer/](docs/developer/index.md): architecture, conventions, design decisions, testing |
| AI agents | [AGENTS.md](AGENTS.md), [docs/agents/](docs/agents/index.md); the website publishes `llms.txt` and `llms-full.txt` |

## License

MIT, see [LICENSE](LICENSE). The bundled URDF is derived from Seeed Studio's reBot Arm B601-RS files and stays
under CERN-OHL-W-2.0, see [NOTICE](NOTICE). The library uses [motorbridge](https://pypi.org/project/motorbridge/),
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) and [MuJoCo](https://mujoco.org).

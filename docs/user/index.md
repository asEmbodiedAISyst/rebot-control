# User guide

1. [Installation](installation.md): Python packages, the CAN adapter, bringing `can0` up.
2. [Concepts](concepts.md): the control chain, modes, tools and the terms the other pages use.
3. [Quick start](quickstart.md): the life cycle of an `Arm`, in simulation and on the real arm.
4. [Real-arm test plan](hardware-test-plan.md): bring-up of a new arm, feature by feature.
5. [Configuration](configuration.md): arm YAML, tool YAML and calibration YAML, units and frames.
6. [Cartesian control](cartesian-control.md): streaming TCP targets, limits, IK failures, lag slow-down.
7. [Calibration](calibration.md): gravity, payload, friction, TCP.
8. [MIT gains (kp / kd)](gains.md): what they do, their limits, how to check and tune them.
9. [Safety](safety.md): what stops the arm, and what to do when it stops.
10. [Troubleshooting](troubleshooting.md)

```{toctree}
:hidden:

installation
concepts
quickstart
hardware-test-plan
configuration
cartesian-control
calibration
gains
safety
troubleshooting
```

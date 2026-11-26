# 用户指南

1. [安装](installation.md)：Python 包、CAN 适配器、启用 `can0`。
2. [基本概念](concepts.md)：控制链路、模式、工具，以及其他页面用到的术语。
3. [快速上手](quickstart.md)：`Arm` 的生命周期，仿真和真机都适用。
4. [真机测试计划](hardware-test-plan.md)：新机械臂的上机调试，一项功能一项功能地做。
5. [配置](configuration.md)：机械臂 YAML、工具 YAML、标定 YAML，单位和坐标系。
6. [笛卡尔控制](cartesian-control.md)：连续下发 TCP 目标、各项限制、IK 失败、跟随误差过大时的减速。
7. [标定](calibration.md)：重力、负载、摩擦、TCP。
8. [MIT 增益（kp / kd）](gains.md)：它们的作用和上限，以及如何检查和整定。
9. [安全](safety.md)：哪些情况会让机械臂停下，停下之后怎么办。
10. [故障排查](troubleshooting.md)

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

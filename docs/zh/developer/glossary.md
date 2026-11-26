# 术语表

中文文档和英文文档统一使用下面这些术语。命令、类、配置键和模式的名字（`HOLD`、`kp`、`tau_ff`、`home`）在两种语言里都保留英文。

| 中文 | English | 说明 |
| --- | --- | --- |
| 机械臂、真机 | arm, real arm | |
| 关节、关节角 | joint, joint angle | |
| 法兰 | flange | `link6` |
| 工具 | tool | |
| TCP（工具中心点） | TCP (tool centre point) | |
| 负载 | payload | 工具的质量和质心 |
| 质心 | centre of mass (COM) | |
| 基座坐标系 | base frame | |
| 位姿 | pose | 位置和姿态 |
| 折叠位姿、零位 | folded pose, zero pose | |
| home 位姿 | home pose | |
| 展开、折叠 | unfold, fold | |
| 分阶段折叠 | staged fold | |
| 后端 | backend | |
| 仿真器、仿真 | simulator, simulation | |
| 控制周期、控制频率 | control cycle, control rate | 30 Hz |
| 伺服循环、伺服频率 | servo loop, servo rate | 150 Hz |
| 伺服超时 | servo overrun | 某个 tick 的耗时超过了它的周期 |
| MIT 模式、MIT 帧 | MIT mode, MIT frame | |
| 驱动器 | driver | 电机自带的控制器 |
| 增益 | gain | `kp`、`kd` |
| 前馈 | feedforward | |
| 重力补偿 | gravity compensation | |
| 摩擦（库仑摩擦、粘性摩擦） | friction (Coulomb, viscous) | |
| 力矩波动 | torque ripple | |
| 力矩 | torque | |
| 使能、上力矩 | enable, torque on | |
| 卸力 | torque off | 机械臂变软，不再受力矩支撑 |
| 失能 | disable | 关闭力矩输出的动作 |
| 保持 | hold | |
| 重力补偿模式、手动拖动 | gravity mode, hand guiding | |
| 接管 | take over | |
| 释放 | release | |
| 目标 | target | |
| 连续下发 | streaming | 每个控制周期发一个目标 |
| 关节目标滤波器 | joint target filter | |
| 软限位 | soft limit | |
| 跳变保护 | jump guard | |
| 速率限制 | rate limit | |
| 低通滤波 | low-pass filter | |
| 力矩上限 | torque cap | |
| 跟随误差 | lag | 目标角减去实测角 |
| 堵转检测 | stall detection | |
| 工作空间范围 | workspace box | |
| 逆运动学（IK） | inverse kinematics (IK) | |
| 阻尼最小二乘 | damped least squares | |
| 零空间 | null space | |
| 雅可比矩阵 | Jacobian | |
| 标定 | calibration | |
| 辨识 | identification | |
| 回归矩阵 | regressor | |
| 先验 | prior | |
| 支点标定 | pivot calibration | |
| 转子惯量 | rotor inertia | |
| 阶跃响应 | step response | |
| 超调 | overshoot | |
| 阻尼比 | damping ratio | |
| 稳态误差 | steady-state error | |
| 上机调试 | bring-up | |

# Glossary

The English and the Chinese documentation use these terms. Names of commands, classes, config keys and modes
(`HOLD`, `kp`, `tau_ff`, `home`) stay in English in both languages.

| English | 中文 | Note |
| --- | --- | --- |
| arm, real arm | 机械臂、真机 | |
| joint, joint angle | 关节、关节角 | |
| flange | 法兰 | `link6` |
| tool | 工具 | |
| TCP (tool centre point) | TCP（工具中心点） | |
| payload | 负载 | mass and centre of mass of the tool |
| centre of mass (COM) | 质心 | |
| base frame | 基座坐标系 | |
| pose | 位姿 | position and orientation |
| folded pose, zero pose | 折叠位姿、零位 | |
| home pose | home 位姿 | |
| unfold, fold | 展开、折叠 | |
| staged fold | 分阶段折叠 | |
| backend | 后端 | |
| simulator, simulation | 仿真器、仿真 | |
| control cycle, control rate | 控制周期、控制频率 | 30 Hz |
| servo loop, servo rate | 伺服循环、伺服频率 | 150 Hz |
| servo overrun | 伺服超时 | a tick that took longer than its period |
| MIT mode, MIT frame | MIT 模式、MIT 帧 | |
| driver | 驱动器 | the motor's own controller |
| gain | 增益 | `kp`, `kd` |
| feedforward | 前馈 | |
| gravity compensation | 重力补偿 | |
| friction (Coulomb, viscous) | 摩擦（库仑摩擦、粘性摩擦） | |
| torque ripple | 力矩波动 | |
| torque | 力矩 | |
| enable, torque on | 使能、上力矩 | |
| torque off | 卸力 | the arm goes limp |
| disable | 失能 | the action that switches torque off |
| hold | 保持 | |
| gravity mode, hand guiding | 重力补偿模式、手动拖动 | |
| take over | 接管 | |
| release | 释放 | |
| target | 目标 | |
| streaming | 连续下发 | one target per control cycle |
| joint target filter | 关节目标滤波器 | |
| soft limit | 软限位 | |
| jump guard | 跳变保护 | |
| rate limit | 速率限制 | |
| low-pass filter | 低通滤波 | |
| torque cap | 力矩上限 | |
| lag | 跟随误差 | target minus measured angle |
| stall detection | 堵转检测 | |
| workspace box | 工作空间范围 | |
| inverse kinematics (IK) | 逆运动学（IK） | |
| damped least squares | 阻尼最小二乘 | |
| null space | 零空间 | |
| Jacobian | 雅可比矩阵 | |
| calibration | 标定 | |
| identification | 辨识 | |
| regressor | 回归矩阵 | |
| prior | 先验 | |
| pivot calibration | 支点标定 | |
| rotor inertia | 转子惯量 | |
| step response | 阶跃响应 | |
| overshoot | 超调 | |
| damping ratio | 阻尼比 | |
| steady-state error | 稳态误差 | |
| bring-up | 上机调试 | |

# 故障排查

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `CAN interface can0 is DOWN` | 适配器重新插拔过，或电脑重启过 | `sudo ip link set can0 type can bitrate 1000000 restart-ms 100 && sudo ip link set can0 up` |
| `CAN interface can0 not found` | 适配器没插上，或者接口不叫这个名字 | 用 `ip -br link` 查看；设置 `bus.channel` |
| `no feedback from motors with CAN id [...]` | 电机没上电、接线有问题、CAN id 不对 | 检查电源和线缆；在 Motorbridge Studio 里核对 id |
| `motorbridge is not installed` | 没装对应的 extra | `pip install -e ".[real]"` |
| `could not switch to MIT mode` | 电机处于故障状态 | 给电机断电再上电；在 Motorbridge Studio 里清除错误 |
| 折叠位姿下角度不在 0 附近 | 没设零位，或者零位设在了别处 | 在折叠位姿设零位（Motorbridge Studio），或者设置 `offset_deg` |
| 某个关节转向相反 | `direction` 设错 | 修改 `motors[].direction`，重做方向检查（测试计划第 1 步） |
| 重力补偿模式下机械臂下垂或上抬 | 重力模型不准（负载、连杆） | 把 `--tool` 改成实际安装的工具；运行 `calib gravity` / `calib payload` |
| 摩擦标定之后，重力补偿模式下机械臂缓慢漂移 | 摩擦补偿过头 | 调低 `feedforward.friction_scale` |
| 保持时有嗡嗡声或啸叫 | 该关节的 `kd` 过高（也可能是 `kp`） | 调低；用 `calib gains --check` 检查 |
| 持续的小幅振荡 | `kp` 过高，或者齿隙 / 摩擦引起的极限环 | 调低 `kp` |
| `servo overruns` 不断增加 | 电脑太慢或太忙，或者开了节能模式 | 关掉其他程序；把 CPU governor 设为 `performance` |
| `CommandRejected: ... target jumps` | IK 解翻转，或者调用方代码有 bug | 检查 IK 结果；连续下发小步长的目标 |
| 笛卡尔步被拒绝（`ik_ok` 为 False） | 目标在可达范围之外，或者关节到了限位 | 保持在可达范围内；用 `cart.reachable(p, R)` 检查 |
| `SafetyStop: ... stalled` | 碰撞、关节卡住，或者力矩不够 | 移开障碍物；检查负载和增益 |

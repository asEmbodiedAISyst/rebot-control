# 代码地图

```
src/rebot_control/
  __init__.py          公开 API 的导出
  arm.py               Arm 门面：from_config, connect/enable/close, command_joint, move_*, cartesian(),
                       gravity_mode/hold, check_stall, sleep/wait, set_tool, recording
  cartesian.py         CartesianLimits（来自工具 YAML），CartesianController.step / hold / reachable
  servo.py             ServoLoop (PASSIVE / HOLD / GRAVITY), tick(), feedforward()，线程，数据记录
  joint_filter.py      JointTargetFilter：限幅 -> 跳变保护 -> 速率限制 -> 低通滤波
  ik.py                DiffIK（DLS + 零空间姿态项），IKResult
  model.py             RobotModel (Pinocchio): fk, jacobian, gravity, mass_matrix, regressor, set_tool, set_link_params
  pose.py              旋转工具函数，倾角 / 偏航限制，位姿增量，rot6d
  config.py            ArmConfig, ToolConfig, Calibration, FrictionParams（YAML <-> dataclass，deg -> rad）
  errors.py            RebotControlError, CommandRejected, SafetyStop, BackendError
  cli.py               rebot-control: state, release, hold, float, home, square,
                       calib {gravity,payload,friction,tcp,gains}
  backends/base.py     Backend 抽象基类，JointState
  backends/robstride.py  motorbridge / SocketCAN 后端，check_can_up
  backends/sim.py      MuJoCo 后端（build_mjspec 由 Pinocchio 模型生成；truth_* 覆盖项；viewer）
  calib/poses.py       间距检查（桌面、基座立柱），随机生成无干涉的位姿巡回路线
  calib/gravity.py     collect_static, fit_gravity (MAP), fit_payload
  calib/friction.py    扫描轨迹，collect_sweeps, fit_joint_friction
  calib/tcp.py         solve_pivot, collect_pivot（交互式）
  calib/gains.py       step_response, analyze_step, find_kp_max
  configs/arms/b601_rs.yaml       硬件、增益、安全、位姿、仿真
  configs/tools/*.yaml            none, pusher_ip17, stock_gripper
  assets/b601_rs/                 只含机械臂本体的 URDF（由 tools/make_arm_urdf.py 生成）+ CERN-OHL-W 许可证
tests/test_core.py     只依赖模型的测试
tests/test_sim.py      仿真闭环测试（辨识能恢复真值）
examples/              每个功能一个脚本，都支持 --sim
tools/make_arm_urdf.py 从 Seeed 的上游文件重新生成 URDF
docs/                  英文页面在 user/、developer/、agents/；中文页面在 zh/；build.py 构建站点
```

## 一次笛卡尔 step 的调用路径

`CartesianController.step` -> `Arm.check`, `Arm.lag`, `Arm.check_stall` -> `CartesianController.limit` ->
`DiffIK.solve` -> `Arm.command_joint` -> `JointTargetFilter.__call__` -> `ServoLoop.set_target`。之后，每个
伺服周期：`ServoLoop.tick` -> `Backend.read`, `_check_temperature`, 插值 / 力矩上限 / `feedforward` ->
`Backend.send_mit`。

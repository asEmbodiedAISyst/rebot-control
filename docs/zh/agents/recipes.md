# 常见任务的步骤

## 添加机械臂配置键

1. 在 `config.py` 的 dataclass 里加一个带默认值的字段，以弧度或 SI 单位存储。
2. 在 `ArmConfig.from_dict` 里解析它（`*_deg` 要换算）。
3. 把它连同注释加到 `configs/arms/b601_rs.yaml`。
4. 在 `docs/user/configuration.md` 里写明（安全参数还要写进 `docs/user/safety.md`）。
5. 如果它改变了行为，加一个测试。

## 添加内置工具

1. 新建 `configs/tools/<name>.yaml`：TCP、负载、`home_deg`、`cartesian`。
2. 检查 home 位姿没有干涉：`is_clear(RobotModel(cfg, tool), tool.home)`。然后试运行
   `rebot-control home --sim --tool <name>`。
3. 把它加到 `docs/user/configuration.md` 的工具表里。

## 添加标定流程

1. 新建 `calib/<name>.py`，其中 `collect_*(arm, ..., progress=print)` 只调用 `Arm` 的方法，
   `fit_*(model, data)` 是只依赖已保存数据的纯函数。
2. 在 `cli.py` 里加 `calib <name>` 子命令。它保存原始数据（`_save_data`），支持 `--from-data`，
   用 `_save_calibration` 或工具 YAML 写出结果，并在运动前调用 `_confirm`。
3. 在 `tests/test_sim.py` 里扰动仿真的真值（`truth_*`），检查拟合能把它恢复出来。
4. 写文档：`docs/user/calibration.md`（怎么运行）和 `docs/developer/identification.md`（数学推导、局限）。
   如果需要真机，在测试计划里加一步。

## 添加 CLI 命令

添加 `cmd_<name>(args)`，并在 `build_parser()` 里加一个带 `parents=[common]` 的子解析器。会让机械臂运动的
命令，运动前必须调用 `_confirm(args, ...)`。在模块 docstring 和用户文档里写明这条命令。

## 添加后端

见 [`../developer/backends.md`](../developer/backends.md)。按关节单位实现 `Backend`，
并在 `arm.make_backend` 里注册。不要把控制逻辑搬进后端。

## 修改伺服或滤波行为

1. 先读 [`../developer/servo-and-timing.md`](../developer/servo-and-timing.md)。
2. 改完后运行全部测试。摩擦辨识的测试对帧时序很敏感。
3. 更新文档。如果改动需要在真机上验证，在 `docs/user/hardware-test-plan.md` 里加一步，并告诉用户。

# 测试

```bash
pip install -e ".[dev]"
pytest                    # a few seconds; the simulator tests are skipped without mujoco
ruff check src tests
```

## 覆盖范围

`tests/test_core.py` 只测模型（不用仿真器）：

- 配置的加载和往返读写，YAML 覆盖；
- 位姿辅助函数（旋转向量、rpy、rot6d、位姿增量、倾角 / 偏航限制）；
- pusher 工具 home 位姿的正运动学（TCP 位置已知），雅可比矩阵对照有限差分，回归矩阵对照重力；
- 工具负载，连杆参数的往返读写；
- IK 在正运动学生成的目标上收敛；
- 关节目标滤波器：速率限制、低通滤波、跳变保护、软限位；
- 笛卡尔限制的裁剪；
- 标定位姿不碰桌面；支点标定在合成数据上精确求解。

`tests/test_sim.py` 在 MuJoCo 里做闭环测试：

- 仿真器的偏置力等于 Pinocchio 算出的重力；
- 生命周期（使能、到 home、退出时折叠）；
- 笛卡尔运动，工作空间范围和速度限制；
- 重力补偿模式下不漂移；
- 堵转检测让机械臂保持（`on_stall: disable` 时卸力）；跳变的命令会被拒绝；
- 异常退出（异常、Ctrl-C、`on_overtemp: hold` 下的过温）之后，只有物理在跑，机械臂仍然使能且静止不动；
  同一台仿真机械臂上的第二个 `Arm` 接管保持时没有跳动；只读的程序不改变保持状态；`release` 会卸力；
- **辨识能找回被扰动的真值**：连杆质量和质心（重力）、负载质量、摩擦（fc、fv）、力矩波动；
- 阶跃响应的各项指标。

`tests/test_docs.py` 检查中文页面与英文页面是否一一对应：文件相同，每页的标题数和代码块数也相同。

## 写测试

- 用 `Arm.from_config(..., backend="sim")`，再用 `SimBackend` 的 `truth_*` 参数（作为关键字参数传给 `Arm`），
  让仿真里的机械臂与模型不一致。
- 仿真是确定性的。运动要短：1 s 的仿真时间大约耗时 50 ms。
- 测试绝不能依赖硬件。需要在真机上检查的内容，在 `docs/user/hardware-test-plan.md` 里加一步；
  如果能复用，再加一个 CLI 命令或示例。

# 给修改本仓库的 agent

## 规则

1. **用户不在场，不发真机命令。** 不要运行不带 `--sim` 的 `rebot-control`、使用 `backend="real"` 的脚本，
   或任何会打开 `can0` 的程序，除非用户在当前会话中要求，并且人在机械臂旁边。仿真随时可以运行。
2. **不要放宽安全限制。** 以下各项不得调高或移除，除非用户点名要求改这一项：
   - 跳变保护、速率限制、力矩上限（`max_error`）、前馈限幅（`tau_max`）；
   - 堵转检测、温度限值、关节软限位；
   - 笛卡尔限制和分阶段折叠。
   不要删除 `cli.py` 里的确认提示。
3. **真机路径和仿真路径保持一致。** 控制逻辑放在 `ServoLoop` / `Arm` 里，不放进后端。后端只做单位换算和
   帧的收发。
4. **单位。** 代码里用弧度和 SI 单位。度只出现在 YAML 的 `*_deg` 键中，由 `config.py` 换算。
5. **线程安全。** 伺服线程会读取的内容（`ServoLoop` 的目标、增益、模式，`RobotModel` 的惯量和坐标系），
   只在持有 `ServoLoop._lock` 时修改。每个线程使用自己的 `pin.Data`。
6. **结束前的检查。** 运行 `ruff check src tests` 和 `pytest`（需要 `mujoco`）。改了文档，还要运行
   `python docs/build.py`。
7. **注释。** 注释和 docstring 用英文写。
8. **文档。** 改动影响哪类读者，就更新哪类读者的页面；英文（`docs/`）和中文（`docs/zh/`）在同一次改动里
   一起更新。见 [`../developer/documentation.md`](../developer/documentation.md)。
9. **硬件记录。** 真机调试的原始记录放在 `docs/internal/`（已被 git 忽略）。只有用户要求时，才修改
   `docs/developer/hardware-notes.md`。

## 代码在哪里

见 [codebase-map.md](codebase-map.md)。常见任务的步骤（添加配置键、工具、标定流程、后端、CLI 命令）见
[recipes.md](recipes.md)。

## 背景

- 本库控制 Seeed reBot Arm B601-RS，用于可更换工具的实验：笛卡尔控制、可配置的 TCP、自标定。
- `docs/developer/hardware-notes.md` 列出代码所依赖的硬件事实，以及每台机械臂上有待确认的问题。
  有待确认的问题按未知处理，不要当作事实。
- Seeed 的 `reBotArm_control_py` 没有许可证。不要复制其中的代码。

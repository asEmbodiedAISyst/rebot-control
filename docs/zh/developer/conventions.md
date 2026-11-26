# 约定

## 单位

- 代码里一律用 SI 单位：rad、rad/s、m、kg、N*m、N*m/rad（kp）、N*m*s/rad（kd）、s、摄氏度。
- YAML 里的角度用度，键名以 `_deg` 结尾（`soft_limits_deg`、`home_deg`、`rpy_deg` 等）。
  换算只做一次，在 `config.py` 里。dataclass 里绝不存度。
- 打印给用户看的内容可以用度，但要标明单位。

## 坐标系

- **基座坐标系** = URDF 的 `base_link`：x 朝前，y 朝左，z 朝上，原点在基座正下方的桌面上。
  没有特别说明时，所有笛卡尔量都在这个坐标系下。
- **位姿**写成 `(p, R)`：位置 (3,) 和旋转矩阵 (3, 3)。旋转矩阵的各列是该坐标系的轴在父坐标系下的表示。
- **旋转增量**是基座坐标系下的旋转向量（`R_new = rotvec_to_matrix(dr) @ R`）。
  例外是 `pose_delta` / `apply_delta`，它们用的是起始位姿的坐标系。
- **雅可比矩阵**是 6 x n，行的顺序为 `[linear; angular]`，采用 Pinocchio 的 `LOCAL_WORLD_ALIGNED`
  （原点在 TCP，轴与基座坐标系平行）。
- **法兰** = `link6`（`ArmConfig.flange_frame`）。TCP 和负载都在法兰坐标系下给出。
  `tcp` 坐标系以 `OP_FRAME` 的形式加进 Pinocchio 模型。
- **URDF rpy**：固定轴，`R = Rz(y) Ry(p) Rx(r)`（`pose.rpy_to_matrix`）。

## 关节角与电机角

- 关节角就是 URDF 的关节角。代码里下标是 0..n-1（名字里是 `joint1..joint6`，命令行上从 1 开始数）。
- 电机原始角 = `direction * joint + offset`，只在后端内部换算。使用出厂零位（在 Motorbridge Studio 里于折叠位姿设定）时，
  每个关节都是 direction = 1、offset = 0。这一点已对照 URDF 验证。
- 电机力矩的符号跟随同一个 `direction`。

## 动力学参数

每个刚体按 Pinocchio `Inertia.toDynamicParameters()` 的顺序：
`[m, m cx, m cy, m cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`，惯量相对于关节坐标系原点。静态标定只用前四个。

## 代码

- Python >= 3.10，`from __future__ import annotations`，公开函数要写类型注解。
- 行宽 130；`ruff check src tests` 必须通过。
- 公开 API = `rebot_control/__init__.py` 导出的内容，加上 `rebot_control.calib.*` 和 `rebot_control.pose`。
- 库代码里不用 print()，`cli.py` 和标定例程的 `progress=` 回调除外。用 `logging`。
- 配置键：在 dataclass 里加带默认值的字段，在 `from_dict` 里解析，再写进 `docs/user/configuration.md` 和内置的 YAML。

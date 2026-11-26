# Conventions

## Units

- SI everywhere in code: rad, rad/s, m, kg, N*m, N*m/rad (kp), N*m*s/rad (kd), s, deg C.
- YAML angles are in degrees, in keys ending in `_deg` (`soft_limits_deg`, `home_deg`, `rpy_deg`, ...). They are
  converted once, in `config.py`. Never store degrees in a dataclass.
- User-facing prints may show degrees; label them.

## Frames

- **Base frame** = URDF `base_link`: x forward, y left, z up, origin on the table under the base. All Cartesian
  quantities are in this frame unless named otherwise.
- **Poses** are `(p, R)`: position (3,) and rotation matrix (3, 3), whose columns are the frame's axes in the parent
  frame.
- **Rotation increments** are rotation vectors in the base frame (`R_new = rotvec_to_matrix(dr) @ R`), except
  `pose_delta` / `apply_delta`, which use the start pose's frame.
- **Jacobians** are 6 x n, rows `[linear; angular]`, Pinocchio `LOCAL_WORLD_ALIGNED` (base-frame axes at the TCP
  origin).
- **Flange** = `link6` (`ArmConfig.flange_frame`). TCP and payload are expressed in it. The `tcp` frame is added to
  the Pinocchio model as an `OP_FRAME`.
- **URDF rpy**: fixed axes, `R = Rz(y) Ry(p) Rx(r)` (`pose.rpy_to_matrix`).

## Joint and motor angles

- Joint angles are URDF joint angles, indexed 0..n-1 in code (`joint1..joint6` in names, 1-based on the CLI).
- Raw motor = `direction * joint + offset`, applied only inside the backend. With the factory zero (set at the
  folded pose in Motorbridge Studio): direction = 1 and offset = 0 for every joint. This was verified against the URDF.
- The motor torque sign follows the same `direction`.

## Dynamic parameters

Per body: Pinocchio `Inertia.toDynamicParameters()` order `[m, m cx, m cy, m cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`, with
the inertia about the joint frame origin. Static calibration only uses the first four.

## Code

- Python >= 3.10, `from __future__ import annotations`, type hints on public functions.
- Line length 130; `ruff check src tests` must pass.
- Public API = what `rebot_control/__init__.py` exports, plus `rebot_control.calib.*` and `rebot_control.pose`.
- No print() in library code except `cli.py` and the `progress=` callbacks of calibration routines. Use `logging`.
- Config keys: add the field to the dataclass with a default, parse it in `from_dict`, document it in
  `docs/user/configuration.md` and the built-in YAML.

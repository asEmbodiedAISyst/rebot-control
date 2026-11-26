# Configuration

There are three kinds of YAML file. Built-in ones live in `src/rebot_control/configs/`. Pass either a built-in name
(`"b601_rs"`, `"pusher_ip17"`) or a path to your own file.

| File | What | Loaded by |
| --- | --- | --- |
| arm (`configs/arms/b601_rs.yaml`) | hardware, rates, gains, feedforward, safety limits, poses, simulator | `ArmConfig.load` |
| tool (`configs/tools/*.yaml`) | TCP pose, payload, home pose, Cartesian limits | `ToolConfig.load`, `arm.set_tool` |
| calibration (`calibration/b601_rs.yaml`, written by `rebot-control calib`) | identified link masses / COM, friction, torque ripple, gains | `Calibration.load` |

```python
arm = Arm.from_config("b601_rs", tool="my_tools/pusher.yaml", calibration="calibration/b601_rs.yaml",
                      overrides={"feedforward": {"friction_scale": 0.5}})
```

`overrides` is deep-merged into the arm YAML before it is parsed. It is handy for experiments without copying the
file.

## Units and frames

- **Angles are degrees in YAML** (keys ending in `_deg`) and radians everywhere in Python.
- Lengths are in m, masses in kg, torques in N*m. Gains are in N*m/rad (kp) and N*m*s/rad (kd), the units of the
  MIT frame.
- **Base frame**: the URDF `base_link`. x points forward, y left, z up, and the origin is on the table under the
  base.
- **Joint angles** are URDF joint angles. With the factory zero, raw motor angle = joint angle.
- The **flange** is URDF `link6`. Tool TCP and payload are expressed in it.
- **TCP rotation**: `tcp.rpy_deg` uses the URDF convention: fixed axes, `R = Rz(yaw) Ry(pitch) Rx(roll)`.

## Arm file (main keys)

| Key | Meaning |
| --- | --- |
| `bus.channel` | SocketCAN interface (`can0`) |
| `motors[]` | `can_id`, `feedback_id`, `model` (`rs-06` / `rs-00`), `direction` (+1/-1), `offset_deg`; raw = direction * joint + offset |
| `rates.control_hz` / `servo_hz` | caller loop (30) / MIT frame stream (150) |
| `gains.kp`, `gains.kd` | PD gains in HOLD mode; `gravity_mode_kd` in gravity mode; `ramp_s` gain ramp at enable |
| `feedforward.gravity`, `gravity_scale` | gravity compensation (on, 1.0) |
| `feedforward.friction_scale` | friction compensation, 0 until friction is calibrated; 0.5 to 0.8 typical after |
| `feedforward.ripple_scale` | scale of the angle-periodic torque ripple model of the calibration file (default 1; no effect without a `ripple` section) |
| `feedforward.tau_max` | clip of the total feedforward per joint (protects against a wrong model) |
| `rotor_inertia` | reflected rotor inertia per joint (kg*m^2), used in gain maths and in the simulator |
| `safety.*` | see [safety.md](safety.md) |
| `poses.zero_deg`, `unfold_first`, `fold_stages`, `move_speed_deg_s` | folded pose and the staged unfold / fold |
| `sim.*` | simulator only: physics rate, joint damping / dry friction of the simulated arm, torque noise |

## Tool file

```yaml
name: my_pusher
mount_frame: link6
tcp:
  xyz: [-0.096, 0.0, 0.172]      # tool tip in the flange frame (m)
  rpy_deg: [0.0, -60.0, 0.0]     # TCP axes; z = approach axis (used by the tilt limit)
payload:
  mass_kg: 0.30
  com_xyz: [0.023, 0.036, 0.062]  # in the flange frame (m)
  # inertia: [ixx, iyy, izz, ixy, ixz, iyz]   # about the COM (kg*m^2); optional (point mass otherwise)
home_deg: [0, 20, 28.6, -38.8, 0, 0]          # used by arm.move_home() and as the IK posture
cartesian:                                     # default CartesianLimits for this tool (see cartesian-control.md)
  x: [0.22, 0.56]
  y: [-0.20, 0.20]
  z: [0.003, 0.20]
  max_speed: 0.25
  max_ang_speed: 2.0
  max_tilt_deg: 30.0
  max_yaw_deg: 60.0
```

For a new tool:

1. Copy a tool file and get TCP and payload from CAD.
2. Check the home pose in simulation: `rebot-control home --sim --viewer --tool my_tool.yaml`.
3. On the arm, refine with `calib payload` and `calib tcp` ([calibration.md](calibration.md)).

Built-in tools:

| Name | What |
| --- | --- |
| `none` | bare flange; use it for bring-up and gravity calibration |
| `pusher_ip17` | printed flange, 10 mm rod at 30 deg, phone plate (pushing experiments) |
| `stock_gripper` | Seeed gripper as a rigid payload; the gripper motor (CAN id 7) is not driven |

## Calibration file

`rebot-control calib ...` writes it and merges each new result into the existing file:

```yaml
meta: {gravity_date: ..., gravity_rms_after: [...], ...}
link_params: {link2: {mass: 1.58, com: [-0.118, -0.004, -0.027]}, ...}   # replaces URDF mass / COM
friction: {fc_pos: [...], fc_neg: [...], fv: [...], vs: [...], bias: [...]}
ripple: {cycles: [...], a1: [...], b1: [...], a2: [...], b2: [...]}         # torque ripple model, see calibration.md
gains: {kp: [...], kd: [...]}                                              # only with calib gains --save
```

It belongs to one physical arm. Keep it next to your experiment code, not inside the package.

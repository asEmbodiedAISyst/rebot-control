# Calibration

Each calibration tool:

- moves the arm (or asks you to);
- saves the raw data under `calibration/data/*.npz`;
- fits a model;
- writes the result into the calibration file (`calibration/<arm>.yaml`) or into a tool YAML.

A saved data file can be refitted offline with `--from-data <file.npz>`.

Recommended order:

1. **Gains check**: confirm the configured gains are sane ([gains.md](gains.md)).
2. **Gravity**, bare flange if possible.
3. **Friction**.
4. **Payload**, with the tool mounted.
5. **TCP**.

Each step uses the results of the steps before it: pass `--calibration calibration/b601_rs.yaml` from step 3 on.

## Gravity: link masses and centres of mass

```bash
rebot-control calib gravity --tool none --poses 30
```

- **Poses.** Random configurations inside conservative joint ranges. Each one is checked kinematically: segments
  between joint origins and the TCP must stay above `--table-z` (default 4 cm) and away from the base column, and so
  must the straight joint-space path between consecutive poses. The poses are ordered as a short tour and printed
  before anything moves.
- **Measurement.** At each pose the arm holds still (HOLD mode, gravity feedforward from the current model), settles
  for `--settle` s, then the motor torques are averaged over 0.5 s. Every pose is approached from both sides (3 deg
  above and below) so that static friction averages out.
- **Model.** At rest, measured torque = `Y(q) theta + b`. `Y` is the static joint-torque regressor (Pinocchio),
  `theta` is the mass and first moment `m, m*c` of every link, and `b` is a per-joint torque offset. Static data
  cannot identify every parameter. The fit is therefore a MAP / ridge estimate around the prior (the URDF or the
  previous calibration). Parameters the data cannot see stay at the prior instead of drifting.
  - `--prior-rel 0.3` (the default) allows about 30 % change in mass. Lower it to trust the URDF more.
- **Output.** `link_params` (mass and COM per link) in the calibration file. The rotational inertia stays from the URDF
  (it does not matter at rest). If a tool was mounted, its configured payload is subtracted from link6.
- **Check.** Residual RMS before and after. Then `rebot-control float --calibration ...` should drift less.

## Friction: Coulomb + viscous per joint

```bash
rebot-control calib friction --tool <tool> [--joints 4,5,6] [--speeds 5,10,20,40,60] [--amplitude-scale 0.5]
```

- One joint at a time sweeps back and forth around the home pose (or `--center`), at each speed, with trapezoidal
  velocity profiles. The other joints hold.
- In the constant-velocity part: `tau - rnea(q, dq, 0) = fc tanh(v / vs) + fv v + (a sin q + b cos q + c)`. The last
  term absorbs the error of the gravity model along the sweep, so friction can be calibrated before or after
  gravity.
- Coulomb friction is fitted **symmetric** (`fc_pos = fc_neg`). A direction-dependent part cannot be told apart from
  a constant torque offset with this data.
- **Output.** `friction` in the calibration file. It is used only when `feedforward.friction_scale > 0`. Start at 0.5:
  over-compensation makes the arm creep in gravity mode.

## Torque ripple: periodic with the joint angle

Geared BLDC actuators need a torque that varies with the rotor angle to turn evenly, one period per electrical cycle
(pole pairs x gear ratio periods per turn of the joint; 126 on the RS-06, i.e. every 2.86 deg). At low speed the
position loop cannot reject it and slow Cartesian lines come out bumpy; at speed the inertia filters it out.

```{warning}
The ripple feedforward is experimental. The automated tests cover it in simulation only, and it has no CLI command
yet. A model with the wrong phase makes the tracking worse. Use it only after you have compared a slow motion with
and without it.
```

Collect slow sweeps and fit them in code:

```python
from rebot_control.calib.friction import collect_sweeps
from rebot_control.calib.ripple import fit_joint_ripple, to_ripple_params

data = collect_sweeps(arm, joint, center, math.radians(15), speeds=(math.radians(3), math.radians(6)))
fit = fit_joint_ripple(arm.model, data)          # periods per turn, sine / cosine terms, share explained
Calibration(ripple=to_ripple_params([fit], arm.n)).save("calib.yaml")
```

- **Output.** `ripple` in the calibration file: `cycles`, `a1`, `b1`, `a2`, `b2` per joint. A calibration with a
  ripple section is used as soon as it is loaded; `feedforward.ripple_scale` (default 1) scales it, 0 switches it off.
- **It is a position feedback.** The torque is evaluated at the measured angle, so a model of amplitude `A` acts
  like a stiffness of up to `N * A` N*m/rad, negative when its phase is wrong. `Arm` refuses a model with
  `N * A > 0.75 * kp`.
- **It belongs to the joint's zero.** After re-zeroing a motor the phase is wrong: calibrate again.
- Run the sweeps with `feedforward.ripple_scale = 0`, slowly (3 to 6 deg/s), over at least 20 deg.

## Payload: tool mass and COM

```bash
rebot-control calib payload --tool my_tool.yaml --calibration calibration/b601_rs.yaml --out my_tool.yaml
```

Same static data as gravity (15 poses by default), but only the four payload parameters are fitted, with the arm
links fixed. This assumes gravity is already calibrated. The new tool YAML gets `payload.mass_kg` and
`payload.com_xyz`, plus a `calibrated:` note.

## TCP: pivot calibration

```bash
rebot-control calib tcp --tool my_tool.yaml --out my_tool.yaml --poses 8
```

Interactive, real arm only:

1. The arm goes to home, then gravity mode.
2. Put the tool tip in a fixed dimple.
3. Tilt the tool to a new orientation while keeping the tip in the dimple, hold still, and press Enter. Repeat.

The flange poses satisfy `R_i t + p_i = c`. The least-squares solution gives the tip `t` in the flange frame and the
pivot point `c`. Aim for:

- residual RMS under about 1 mm;
- orientation spread of at least 30 deg, in two different tilt directions.

Only the TCP position is identified. The TCP orientation stays as configured.

## Gains

See [gains.md](gains.md). `rebot-control calib gains --save` writes `gains.kp` / `gains.kd` into the calibration
file. They are applied when the calibration file is loaded.

## In code

```python
from rebot_control.calib.gravity import collect_static, fit_gravity
from rebot_control.calib.poses import generate_poses
poses = generate_poses(arm.model, 30, arm.q())
data = collect_static(arm, poses)
fit = fit_gravity(arm.model, data)
print(fit.summary(arm.cfg.joints))
```

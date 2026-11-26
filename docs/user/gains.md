# MIT gains (kp / kd)

## What they do

In MIT mode every RobStride driver runs this law in its own fast loop:

```
tau = kp (q* - q) + kd (dq* - dq) + tau_ff
```

The library sends `q*`, `dq*`, `kp`, `kd` and `tau_ff` at 150 Hz.

- `tau_ff` carries the gravity (and friction) compensation, so `kp` only has to correct the remaining error.
- `dq*` is the velocity of the interpolated target, so `kd` damps deviations from the motion. It does not brake the
  motion itself.

Around a pose, a joint behaves like a mass-spring-damper. Its inertia `J` is the joint's diagonal entry of the mass
matrix plus the rotor inertia (`arm.model.mass_matrix(q)[j, j]`):

```
omega_n = sqrt(kp / J)              natural frequency (rad/s)
zeta    = kd / (2 sqrt(kp J))       damping ratio (0.7 - 1 is the usual target)
```

| Gain | Higher means | Too high means |
| --- | --- | --- |
| `kp` | stiffer, smaller steady-state error, faster response | ringing or a steady oscillation (limit cycle), audible tone |
| `kd` | more damping, less overshoot | buzz or squeal (velocity noise amplified), heat |

## Why there is an upper limit

The ideal model above is stable for any `kp`, `kd`. The real loop is not. Four effects set the limit:

- **Delay.** The CAN round trip, the 150 Hz frame rate and the driver's own filtering add phase lag. As `kp` grows,
  the crossover frequency rises until that phase lag eats the phase margin. The joint then rings and finally
  oscillates.
- **Encoder quantisation and velocity noise.** The motor estimates `dq` by differentiating the encoder, and `kd`
  multiplies that noise straight into torque. Too much `kd` is heard as a high-pitched buzz or squeal, and the
  driver heats up.
- **Gear backlash and friction.** These cause a small limit cycle around the target at high `kp` (the joint keeps
  hunting by a fraction of a degree). It is visible as residual velocity at rest.
- **Structure.** The links and the base flex. A high `kp` can excite their resonances.

So `kp` and `kd` have to be found on the real arm. Simulation shows only the ideal part (the simulated motor has no
delay or noise).

## Defaults

`configs/arms/b601_rs.yaml` uses Seeed's plugin defaults x 0.8.

| Joint | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- |
| kp | 40 | 120 | 120 | 40 | 40 | 40 |
| kd | 2.4 | 8 | 8 | 4 | 3.2 | 3.2 |

The model `zeta` of the wrist joints comes out above 2. Either the wrist is heavily damped, or the rotor inertia
estimate (0.005 kg*m^2) is low. A step check on the real arm tells which.

## Check: `rebot-control calib gains --check`

The command steps each joint by 2 deg and back, with the other joints holding, and prints:

| Metric | Meaning | Typical good value |
| --- | --- | --- |
| overshoot | peak past the step, in % of the step | < 10 % (zeta of about 0.6 or more) |
| rise | 10 to 90 % time | 30-150 ms |
| settle | last time outside +-5 % | < 0.3 s |
| ss | steady-state error (deg) | < 0.1 deg |
| residual vel | RMS velocity in the last 30 % (deg/s) | < 0.5 deg/s; more is ringing or a limit cycle |
| ripple | std of the joint torque at rest (N*m) | < 1 % of the joint effort limit; more is buzz |
| zeta est / model | from the overshoot, and from `kd / (2 sqrt(kp J))` | est lower than model means unmodelled lag; higher means friction |

## Tune: `rebot-control calib gains --joints <j>`

For one joint at a time:

1. Start from the configured `kp`. Set `kd` from the damping-ratio rule (`--zeta`, default 0.8), capped.
2. Do a step test. If a limit trips (overshoot > 30 %, residual velocity > 1 deg/s, ripple > 1 % of the effort
   limit, no settling in 0.6 s), stop. Otherwise multiply `kp` by 1.25 and repeat, up to `--kp-cap` (default
   2 x the configured kp; `kd` is capped at `--kd-cap` x the configured kd).
3. `kp_max` is the last `kp` that passed. The recommended working gains are `kp = 0.6 kp_max` (`--margin`) and `kd`
   from the zeta rule, checked with one more step.

Notes:

- Results are only saved with `--save`, into the calibration file.
- Stop at once (Ctrl-C) if a joint squeals. The test keeps every step small (2 deg). The torque is also capped by
  `safety.max_error` (10 deg x kp).
- Tune at the pose where you work, with the tool mounted: `J` changes with the pose and the payload.
- Prefer the defaults unless a check shows a problem. Higher gains mean stiffer contact behaviour, which
  matters in pushing experiments.
- If the arm is quiet but sluggish, raise `kp` first. If it overshoots, raise `kd`. If it buzzes, lower `kd`.

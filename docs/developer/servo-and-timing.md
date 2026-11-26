# Servo loop and timing

## Rates

| Loop | Rate | Who |
| --- | --- | --- |
| caller | `control_hz` = 30 Hz | user code, `Arm.sleep()`; IK and joint filter run here |
| servo | `servo_hz` = 150 Hz | `ServoLoop.tick`: one MIT frame per motor per tick |
| driver | about 10-20 kHz (inside the RobStride) | the MIT PD law on the held frame |
| simulator | `physics_hz` = 1050 Hz | `SimBackend.advance`, PD law at each physics step |

30 Hz targets sent directly to the motors cause audible 30 Hz steps. The servo loop interpolates between targets at
150 Hz instead.

## HOLD frame

At tick time `t`, with the previous target `from`, the newest `to` set at `t_set`, and control period `Tc`:

```
a      = min(1, (t + Ts/2 - t_set) / Tc)        Ts = servo period
q_cmd  = from + a (to - from)
dq_cmd = (to - from) / Tc   while (t + Ts/2 - t_set) / Tc < 1.25, else 0
q_cmd  = q_meas + clip(q_cmd - q_meas, +-max_error)          torque cap
tau_ff = clip(gravity(q_meas) * gravity_scale + friction(dq_cmd) * friction_scale
              + ripple(q_meas) * ripple_scale, +-tau_max)
kp, kd, tau_ff  x  ramp(t)                                    after leaving PASSIVE
```

### Why half a servo period ahead

The driver holds each frame's position for a whole servo period (zero-order hold). If `q_cmd` were the trajectory
value at the *start* of the period, the held target would lag the trajectory by `Ts/2` on average. The PD would then
add a velocity-proportional torque, `kp v Ts / 2`, which looks exactly like viscous friction. It is 0.13 N*m at
kp = 40 and 1 rad/s.

Evaluating the interpolation at `t + Ts/2` centres the held target on the trajectory. The friction identification in
simulation only recovered the true viscous coefficient after this change *and* after making the simulated torque
reading a period average (an instantaneous end-of-period sample has the same bias).

On the real arm this depends on when within the period the driver samples the current it reports. If identified
viscous friction changes with `kp`, this is the cause. It is listed in [hardware-notes.md](hardware-notes.md).

### Why the velocity grace

On the real arm the next target can arrive a little after `a` reached 1, because of thread jitter. Without the 1.25
grace, `dq_cmd` would drop to 0 for one tick, and `kd` would brake the motion once per control period: a 30 Hz
ripple.

## GRAVITY frame

`kp = 0`, `kd = gravity_mode_kd`, `q_cmd = q_meas`, `dq_cmd = 0`,
`tau_ff = clip(gravity(q) * gravity_scale + friction(dq_meas) * friction_scale + ripple(q) * ripple_scale, +-tau_max)`. Friction is evaluated at the measured velocity, so
friction compensation makes hand guiding lighter.

## Thread timing (real arm)

`ServoLoop._run` uses `time.perf_counter()` deadlines. If a tick overruns, it resynchronises and increments
`servo.overruns`. Without the bus, a tick takes about 20 us (measured; the Pinocchio gravity call is 2 us of that). The
6 MIT writes plus the poll on the real bus still have to be measured: at 1 Mbit/s, 12 frames are about 1.5 ms of bus
time, well inside the 6.7 ms period. Python GC pauses and a busy CPU are the main risks. The CLI `hold` command
prints the overrun count.

## Step targets

`set_target(q, t, step=True)` sends `q` at the next tick without interpolation. Only the gain step test uses it.
Calling `set_target` twice before a tick makes the second call interpolate from the last *sent* frame. That is why
`step_response` sleeps one cycle after the step before streaming the hold target.

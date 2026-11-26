# Identification

The maths behind `rebot_control.calib`, and where each method breaks.

## Static gravity model

Rigid-body dynamics are linear in the dynamic parameters: `tau = Y(q, dq, ddq) pi`, with 10 parameters per body
(`RobotModel.regressor`, Pinocchio `computeJointTorqueRegressor`). At rest only the gravity term remains, and it
depends only on `m` and `m c` (4 parameters per body). Those are the columns `static_columns(n)`.

**Identifiability.** For 6 bodies there are 24 static parameters, but the static regressor of this arm has rank 10
(printed as `regressor_rank`). Two reasons:

- joint 1 is vertical, so gravity never loads it, and link 1's parameters are invisible;
- for the other bodies only certain combinations matter (the "base parameters").

Ordinary least squares would put arbitrary values in the null space.

**MAP fit** (`fit_gravity`). It minimises

```
|| (Y theta + B b - tau) / s_tau ||^2 + || (theta - theta0) / s_theta ||^2 + || b / s_b ||^2
```

where:

- `theta0` is the prior (URDF or previous calibration, payload included on the flange body);
- `s_theta` is `prior_rel * m0 + prior_abs` for masses, times `lever_m` for first moments;
- `b` is the per-joint torque offset with prior `s_b`;
- `s_tau` is the torque noise.

Directions the data constrains follow the data. The others stay at the prior. The result is reported as mass + COM
per link (COM = `m c / m`; if `m` comes out non-physical, the prior is kept and a warning is logged).

**Friction at rest.** A joint at rest can carry any torque within the static friction band. Approaching every pose
from both sides and averaging cancels it to first order. The remaining scatter shows up as residual RMS.

**Payload** (`fit_payload`). The same static equations, but only the 4 parameters of the flange body change, and the
arm parameters are fixed. The problem is well conditioned if the wrist orientations vary. The COM is converted from
the joint frame to the mount frame.

## Friction

For joint j moving alone at constant velocity: `tau_j = rnea_j(q, dq, 0) + f(v) + e(q_j)`. Here `e(q_j)` is the error
of the rigid-body model along the sweep, approximated by `a sin q_j + b cos q_j + c`.

Model: `f(v) = fc tanh(v / vs) + fv v`, with `vs` = 0.02 rad/s fixed. It is fitted by linear least squares on the
constant-velocity samples: the nominal velocity is non-zero and the measured velocity is within 20 % of it. The
nominal velocity is aligned in time by `filter_tau_s`, the lag of the joint filter.

**Limits**:

- Coulomb asymmetry `fc_pos != fc_neg` is not identifiable together with `c`, so the fit is symmetric.
- Stribeck effects below about 3 deg/s are not modelled.
- Position-dependent friction (gear cogging) ends up in the residual.

The reported torque's sampling phase within the zero-order-hold period biases `fv` by up to `kp Ts / 2` (see
[servo-and-timing.md](servo-and-timing.md)). A check on the real arm: identify at two kp values and compare.

## TCP pivot

`[R_i  -I] [t; c] = -p_i` is stacked and solved by least squares. It is ill-conditioned when all `R_i` are similar,
which `solve_pivot` refuses. Residual RMS is the tip-placement error. Only position is identified, because a pivot
gives no information about the TCP orientation.

## Gains

A second-order model per joint: `omega_n = sqrt(kp / J)`, `zeta = kd / (2 sqrt(kp J))`, with
`J = M(q)_jj + rotor_inertia`. The step response metrics and the search are described in
[`../user/gains.md`](../user/gains.md). The model ignores delay, so `zeta_est` (from overshoot) below `zeta_model`
indicates unmodelled lag, and above it indicates friction.

## Ideas not implemented yet

- **Dynamic identification** (inertias, rotor inertia) from excitation trajectories, using the full regressor with
  `dq`, `ddq` from filtered encoder data. It would improve `rotor_inertia`, which the gain maths depends on.
- **Joint offset calibration** (encoder zero errors) from TCP measurements with an external tracker.
- **Payload inertia**: needs dynamic data.

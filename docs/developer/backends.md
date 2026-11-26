# Backends

A backend moves MIT frames and feedback in joint units, and nothing else. All control logic is in `ServoLoop`, so the
real arm and the simulator run the same code. The interface is `backends/base.py`:

- `connect`, `close`;
- `enable`, `disable`;
- `send_mit(q, dq, kp, kd, tau)`, `send_passive()`;
- `read()`, which returns a `JointState`;
- `now()`;
- `set_zero(joints)`, real arm only;
- `advance(dt)`, simulation only.

## RobStride over motorbridge (`backends/robstride.py`)

This backend uses `motorbridge` 0.5.6:

- `Controller(channel)`;
- `add_robstride_motor(can_id, feedback_id, model)`;
- per motor: `ensure_mode(Mode.MIT)`, `send_mit(pos, vel, kp, kd, tau)`, `get_state()`, `disable()`,
  `clear_error()`, `set_zero_position()`, `close()`;
- on the bus: `enable_all()`, `disable_all()`, `poll_feedback_once()`, `close()`.

These are the same calls the Seeed lerobot plugin makes, and the same enable sequence: disable all, set MIT mode with
retries, enable all.

Quirks of the B601-RS:

- **Feedback comes only from replies.** motorbridge updates a RobStride motor's state only from the reply to a
  command. `request_feedback()` does nothing for RobStride, and the motors' periodic active-report frames are not
  parsed. So the servo thread sends a frame to every motor every tick, with zero gains when torque is off
  (`send_passive`). `read()` then processes the replies to the *previous* tick's frames (`poll_feedback_once` +
  `get_state`). Feedback is therefore one servo period (6.7 ms) old.
- **Disabled motors still answer MIT frames** (within about 3 ms) and stay disabled. That is how `state` reads the arm
  with torque off.
- **Enabled motors keep executing their last MIT frame** when frames stop (CAN timeout 0 on the B601-RS). The library
  uses this to hold the arm through a dead program, so a backend must offer three things: `read_positions()` (angles
  without a command frame: parameter 0x7019 through type-17 reads), `JointState.enabled` (the driver's mode from the
  feedback frame; `status_code != 0` counts as enabled) and `close(disable=False)` (release the bus and send nothing).
  `enable()` switches on only the motors that are off: the mode switch needs a disabled motor, and disabling a motor
  that holds drops the joint.
- **The CAN interface comes up DOWN** after a reboot or a re-plug. `check_can_up()` fails early with the command
  to fix it.
- `MotorState` has `pos`, `vel`, `torq`, `t_mos` (driver MOSFET temperature) and `t_rotor`. The library uses
  `t_mos` for the temperature guard.
- The reported torque is estimated from the phase current. It has an offset and a noise floor, which is why gravity
  calibration fits a per-joint bias.
- The stock gripper motor (CAN id 7) is not added to the bus. With the gripper mounted, it stays disabled and is
  treated as a rigid payload.

To verify on the real arm (track in [hardware-notes.md](hardware-notes.md)):

- the phase of the reported torque relative to the frame (instantaneous or filtered);
- the motor-side CAN timeout behaviour;
- the motor's MIT kp / kd ranges (the motorbridge model tables clamp them).

## MuJoCo (`backends/sim.py`)

- `build_mjspec(model, cfg)` builds the MuJoCo model from the **Pinocchio model**: body placements, inertias
  (`fullinertia`, `explicitinertial`), hinge axes (from the joint Jacobian at q = 0), joint ranges in rad
  (`spec.compiler.degree = False`), and `armature = rotor_inertia`, `damping` and `frictionloss` from `sim:`. There is
  no collision geometry; the capsules are visual only. Kinematics and dynamics therefore match the controller's model
  exactly (`tests/test_sim.py::test_sim_matches_model`).
- **"Truth" parameters**: `truth_link_params`, `truth_tool`, `truth_friction` and `truth_ripple` make the simulated arm differ from
  what the controller believes. The identification tests rely on them.
- **Motor emulation**: every physics step (1050 Hz) applies
  `tau = clip(kp (q* - q) + kd (dq* - dq) + tau_ff, +-effort)` through `qfrc_applied`, holding the last frame (zero-order
  hold, like the driver). The reported torque is the mean over the last `advance()` call (one servo period). That
  mimics a filtered current reading and matters for friction identification, see
  [servo-and-timing.md](servo-and-timing.md).
- **Not modelled**: CAN delay, encoder quantisation, velocity noise, backlash, thermal behaviour, contact. Gain limits
  therefore cannot be found in simulation.
- `viewer=True` opens `mujoco.viewer.launch_passive` and paces the simulation to wall-clock time.
- At the folded pose with torque off, the simulated arm sags a little (joint 4 is not at a limit). The real arm rests
  on its own structure.

## Adding a backend

Subclass `Backend` and implement the abstract methods in joint units, with the sign conventions applied inside. Then
register it in `arm.make_backend`. Damiao motors (the B601-DM) would be the obvious next one. Their position-velocity
mode does not fit the MIT-frame interface, so use their MIT mode.

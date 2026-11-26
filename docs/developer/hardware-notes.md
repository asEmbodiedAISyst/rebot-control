# Hardware notes

This page lists the facts about the B601-RS that the code relies on, and the questions to answer on each arm.
Session logs from real-arm tests are not published yet.

## Facts the code relies on

- Raw motor angle = URDF joint angle with the factory zero (direction 1, offset 0 for all joints).
- CAN ids 1 to 6, feedback id 0xFD, `rs-06` x 3 + `rs-00` x 3; 1 Mbit/s SocketCAN.
- Feedback comes only from command replies. Disabled motors still reply to MIT frames.
- The default gains are Seeed's plugin defaults x 0.8. 150 Hz streaming removes the audible 30 Hz steps.
- Joint target conditioning: 25 deg jump guard, 90 deg/s rate limit, 0.1 s filter and 10 deg torque cap.
- Stall rule: 10 deg lag for 1 s with less than 1 deg of progress.
- Unfold shoulder + elbow first. Fold base + wrist first, then shoulder + elbow. Otherwise the wrist rubs the upper
  arm.
- `can0` comes up DOWN after a reboot or a re-plug.
- The drivers' CAN timeout (parameter 0x7028) is 0 on the B601-RS. The library reads it at every connect.

## Assumptions behind the hold through a dead program

The library keeps torque on after an abnormal exit, takes over an enabled arm at connect, and has an explicit
release ([`../user/safety.md`](../user/safety.md)). This behaviour rests on the assumptions below about the motors
and motorbridge. Step 2b of the [real-arm test plan](../user/hardware-test-plan.md) checks them on an arm.

| # | Assumption | If it is wrong |
| --- | --- | --- |
| 1 | An enabled motor keeps executing its last MIT frame when frames stop (CAN timeout 0) | the arm falls when a program dies |
| 2 | `MotorState.status_code` is 0 for a disabled motor and non-zero for an enabled one | an enabled arm is taken for a disabled one at connect and gets zero-gain frames: it falls |
| 3 | A parameter read (type 17) does not replace the MIT frame a motor is executing | the arm twitches or sags when a program connects |
| 4 | Freeing the motorbridge handles (`Motor.close`, `Controller.close`) sends nothing to the motors | the arm falls when a program detaches |
| 5 | A disabled motor ignores an MIT frame with non-zero gains (the probe frame at connect) | a disabled arm stiffens for one frame when a program connects |
| 6 | Per-motor `disable` / `ensure_mode` / `enable` works like `disable_all` / `enable_all` | enabling fails |

## Questions to answer on each arm

| # | Question | How |
| --- | --- | --- |
| 1 | Does the sign of every joint match the URDF? | test plan step 1 |
| 2 | Are there servo overruns at 150 Hz on the control PC? | `rebot-control hold`, 60 s |
| 3 | How long does the bus take per tick (6 writes + poll)? | log `time.perf_counter()` around `send_mit` / `read` |
| 4 | How much does each joint drift in gravity mode with the URDF model? | test plan step 3 |
| 5 | What are the bias and the noise of the torque reading at rest? | `calib gravity` output (bias, RMS) |
| 6 | When in the frame period is the reported torque sampled? (It biases friction `fv`.) | `calib friction` at kp and 2 kp, compare `fv` |
| 7 | What happens when frames stop? | test plan step 2b |
| 8 | What are the step responses with the default gains? | `calib gains --check` |
| 9 | Is the model zeta of 2 to 3 on the wrist real? | compare `zeta est` and `zeta model` |
| 10 | Is the base frame origin on the table (z = 0 at the table surface)? | TCP z at home against a ruler |
| 11 | Which MIT kp / kd ranges do the drivers accept? | `calib gains` with caps, watch for clamping |

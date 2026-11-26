# Design decisions

**Independent implementation, not a fork of Seeed's `reBotArm_control_py`.** That repository has no license, so its
code cannot be reused in an open-source package. Only facts are taken from Seeed's
material: the URDF (CERN-OHL-W, see NOTICE), the CAN ids, and the default gains.

**Python, one process, a servo thread.** 150 Hz with about 20 us of compute per tick fits comfortably in Python. A
C++ or RT-kernel stack would be faster but harder to install and change for research use. If overruns turn out to be
a problem on the real arm, the next step is a separate process for the servo loop, with shared memory for targets
and state. That needs no API change.

**MIT mode with streaming targets, not the drivers' position mode.** MIT gives direct control of stiffness and
damping, torque feedforward, and a gravity-only mode. Position mode cannot do these, and pushing experiments need
them.

**Gravity feedforward from the measured q, not the target.** It is what the motor needs to hold the arm where it
actually is. With the torque cap, a wrong target cannot create a large feedforward.

**Interpolate between targets in the servo loop.** 30 Hz steps are audible and excite the structure. The interpolation
adds one control period of delay, which the Cartesian limits and lag slow-down account for.

**Rate limit relative to the previous target, not the measured position.** Rate limiting against the measured
position caps the PD torque instead of the speed: a lagging joint would never get a larger target.

**Static gravity calibration with a prior.** It is quick (5 to 10 min), safe (slow moves, holds only), and gives what
the feedforward needs. Full dynamic identification is a later addition.

**Simulator built from the Pinocchio model.** One source of truth for kinematics and dynamics, plus explicit "truth"
overrides. The calibration tools are tested by checking that they recover parameters the controller does not know.

**Tools are data.** A tool is a YAML with TCP, payload, home pose and Cartesian limits. Swapping one needs no code
change, which is the point of the library.

**Staged fold on exit, no fold after an error.** A clean exit always leaves the arm at its resting pose. After an
unknown error, moving is the riskier option.

**Torque goes off only on purpose** (decided 2026-11-16, after the first sessions on the real arm). The earlier behaviour, torque off after an error, drops an unfolded arm onto the table and its tool.
Holding in place is the safer default for a 3 kg arm with a pusher on it: the motors execute their last frame
indefinitely (CAN timeout 0), the library sends a clean hold frame before it lets go, and the next program takes
the hold over. The price is an unsupervised, powered arm between programs, and it is the operator who ends that
state (`rebot-control release`). Over-temperature still switches torque off by default: a burning driver is
worse than a fall.

**`connect()` assumes nothing about torque.** Because a previous program may have left the motors holding, the
first frames of a new program must not be zero-gain frames. The probe frame is a full-gain hold of the angles
just read, which is a no-op for a motor that holds there and for a motor that is disabled.

**License: MIT** (decided 2026-11-09). It is short and permissive, which suits a research tool others should be
able to drop into their own code. It is compatible with the dependencies: Pinocchio (BSD-2), MuJoCo (Apache-2.0) and
motorbridge. The bundled URDF stays under CERN-OHL-W-2.0 (see NOTICE): modified versions of that file must keep the
licence and list the changes, which `tools/make_arm_urdf.py` does in the file header.

# Roadmap

## State of the project

Version 0.1 contains what is in this repository. The automated tests run in simulation. Real-arm bring-up follows
[`../user/hardware-test-plan.md`](../user/hardware-test-plan.md) and is in progress.

## Next, in rough order

1. Fix what the real-arm sessions find. Freeze the gain and safety defaults for this library.
2. Impedance / admittance mode for pushing experiments: Cartesian stiffness through
   `J^T K (x* - x)` feedforward with lower joint kp, plus contact force estimates from the motor torques with the
   identified gravity and friction.
3. Dynamic identification (inertia, rotor inertia) and joint-offset calibration.
4. A CLI command for the torque ripple calibration.
5. Optional servo process (shared memory) if Python thread timing is not good enough.
6. B601-DM (Damiao motors) backend.
7. ROS 2 wrapper (`ros2_control` hardware interface or a thin node) as a separate package.

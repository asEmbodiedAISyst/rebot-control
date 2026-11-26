# Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `CAN interface can0 is DOWN` | adapter re-plugged or rebooted | `sudo ip link set can0 type can bitrate 1000000 restart-ms 100 && sudo ip link set can0 up` |
| `CAN interface can0 not found` | adapter not plugged in, or another name | `ip -br link`; set `bus.channel` |
| `no feedback from motors with CAN id [...]` | motor power off, wiring, wrong CAN id | check power and cabling; check ids in Motorbridge Studio |
| `motorbridge is not installed` | missing extra | `pip install -e ".[real]"` |
| `could not switch to MIT mode` | motor in a fault state | power-cycle the motors; clear errors in Motorbridge Studio |
| angles not about 0 at the folded pose | zero not set, or set elsewhere | set the zero at the folded pose (Motorbridge Studio), or set `offset_deg` |
| a joint moves the wrong way | `direction` wrong | fix `motors[].direction`, re-run the sign check (test plan step 1) |
| the arm sags or pushes up in gravity mode | gravity model off (payload, links) | correct `--tool`; run `calib gravity` / `calib payload` |
| the arm creeps in gravity mode after friction calibration | friction over-compensated | lower `feedforward.friction_scale` |
| buzz or squeal while holding | `kd` too high for that joint (or `kp`) | lower it; `calib gains --check` |
| steady small oscillation | `kp` too high, or backlash / friction limit cycle | lower `kp` |
| `servo overruns` keeps growing | PC too slow or busy, power saving | close other programs; set the CPU governor to `performance` |
| `CommandRejected: ... target jumps` | IK flip or a bug in the caller | check the IK result; stream small steps |
| Cartesian step refused (`ik_ok` False) | target outside the reachable set or at a joint limit | stay inside the reachable region; check `cart.reachable(p, R)` |
| `SafetyStop: ... stalled` | collision, blocked joint, or not enough torque | remove the obstacle; check payload and gains |

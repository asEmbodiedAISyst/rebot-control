# Installation

## Python

Python 3.10 or newer.

```bash
git clone https://github.com/asEmbodiedAISyst/rebot-control.git && cd rebot-control
python -m venv .venv && source .venv/bin/activate
pip install -e ".[real,sim]"     # real arm + simulator
# pip install -e ".[sim]"        # simulator only (macOS / Windows / no arm)
```

| Extra | Pulls in | Needed for |
| --- | --- | --- |
| (none) | numpy, pin (Pinocchio), pyyaml | the model, IK and offline refits of calibration data |
| `real` | motorbridge >= 0.5.6 | the real arm (Linux, SocketCAN) |
| `sim` | mujoco >= 3.2 | `backend="sim"`, `--sim`, the tests |
| `dev` | pytest, ruff, mujoco | development |

Check the install with `rebot-control state --sim`. It prints the simulated arm's state.

## CAN adapter (real arm)

The arm talks CAN at 1 Mbit/s through a USB-CAN adapter that shows up as the SocketCAN interface `can0`.
After a reboot or a re-plug the interface is **DOWN**. The library checks this and stops with the command to run:

```bash
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -br link show can0            # should say UP
```

To bring the interface up automatically at boot, add a systemd-networkd `.network` file or a udev rule that runs the
first two commands above. A different interface name goes in `bus.channel` of the arm config.

## Motor zero and IDs

The library expects:

- CAN IDs 1 to 6 (base to wrist roll), feedback ID `0xFD`;
- motor models `rs-06` (joints 1 to 3) and `rs-00` (joints 4 to 6);
- the motor zero set at the folded pose, so that the raw motor angle equals the URDF joint angle.

This matches the factory setup used with Seeed's software. If your motors were zeroed or mounted differently, set
`direction` / `offset_deg` per motor in the arm config ([configuration.md](configuration.md)). Re-zeroing is done
with Motorbridge Studio, or with `RobStrideBackend.set_zero` with torque off and the arm held in the folded pose.
Re-zeroing overwrites the zero stored in the motor.

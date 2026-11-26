"""Record the servo loop (150 Hz: measured q / dq / tau, commanded q / dq / gains / feedforward) to a .npz file."""

import math

import numpy as np

from _common import make_arm

arm, _ = make_arm(__doc__)
with arm:
    arm.enable()
    arm.move_home()
    q = arm.tool.home.copy()
    q[0] += math.radians(15)
    with arm.recording() as rec:
        arm.move_to_joint(q)
        arm.move_to_joint(arm.tool.home)
    np.savez("servo_log.npz", **rec)
    err = np.degrees(np.abs(rec["q_cmd"] - rec["q"])).max(axis=0)
    print(f"{len(rec['t'])} samples saved to servo_log.npz; max |q_cmd - q| per joint (deg): {err.round(2)}")

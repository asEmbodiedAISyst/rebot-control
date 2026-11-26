"""Torque on, unfold to the tool's home pose, hold for a few seconds, fold back on exit."""

import numpy as np

from _common import make_arm

arm, _ = make_arm(__doc__)
with arm:
    arm.enable()  # holds the folded pose; gains ramp in over 0.5 s
    arm.move_home()  # staged: shoulder + elbow first
    print("home q (deg):", np.degrees(arm.q()).round(2), "TCP:", arm.tcp_pose()[0].round(4))
    arm.wait(3.0)
# leaving the block folds back to zero in stages and switches torque off

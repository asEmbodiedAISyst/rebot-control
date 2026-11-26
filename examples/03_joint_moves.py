"""Slow joint-space moves (smoothstep profile, stall detection) and joint streaming at the control rate."""

import math

import numpy as np

from _common import make_arm

arm, _ = make_arm(__doc__)
with arm:
    arm.enable()
    arm.move_home()
    home = arm.tool.home.copy()
    for j, amp in ((0, 20), (3, 20), (5, 30)):  # base, wrist flex, wrist roll
        q = home.copy()
        q[j] += math.radians(amp)
        arm.move_to_joint(q, speed=math.radians(15))
        arm.move_to_joint(home, speed=math.radians(15))
    # streaming: a 0.25 Hz sine on the base joint, one target per control cycle
    for k in range(int(8 / arm.dt)):
        q = home.copy()
        q[0] += math.radians(10) * math.sin(2 * math.pi * 0.25 * k * arm.dt)
        arm.command_joint(q)
        arm.sleep()
        arm.check_stall()
    print("max lag during streaming (deg):", round(math.degrees(arm.lag()), 2))
    arm.move_to_joint(home)
    print("done at", np.degrees(arm.q()).round(1))

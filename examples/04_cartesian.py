"""Cartesian control: a point-to-point straight line, then a streamed circle in the horizontal plane."""

import math

import numpy as np

from _common import make_arm

arm, _ = make_arm(__doc__)
with arm:
    arm.enable()
    arm.move_home()
    p0, R0 = arm.tcp_pose()
    arm.move_to_pose(p0 + [0.10, 0.0, 0.0], R0, speed=0.05)  # straight line, 5 cm/s
    cart = arm.cartesian()
    c, R = cart.pose()
    errs = []
    for k in range(int(8 / arm.dt)):  # circle of radius 4 cm, 8 s per turn
        a = 2 * math.pi * k * arm.dt / 8
        st = cart.step(c + 0.04 * np.array([math.cos(a) - 1, math.sin(a), 0.0]), R)
        arm.sleep()
        errs.append(np.linalg.norm(arm.tcp_pose()[0] - st.pos))
        if not st.ik_ok:
            print("unreachable step refused")
    print(f"tracking error mean {np.mean(errs) * 1000:.1f} mm, max {np.max(errs) * 1000:.1f} mm")
    arm.move_home()

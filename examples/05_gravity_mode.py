"""Gravity mode: kp = 0, gravity (+ friction) feedforward only. Move the arm by hand; it should stay where left."""

import numpy as np

from _common import make_arm

arm, _ = make_arm(__doc__)
with arm:
    arm.enable()
    arm.move_home()
    arm.gravity_mode()
    print("gravity mode for 20 s: guide the arm by hand (Ctrl-C to stop)")
    try:
        for _ in range(20):
            arm.sleep(1.0)
            print("q (deg)", np.degrees(arm.q()).round(1), "TCP", arm.tcp_pose()[0].round(3))
    except KeyboardInterrupt:
        pass
    arm.hold()  # PD hold at the current pose, then fold on exit

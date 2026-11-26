"""Read the joint state with torque off (safe: nothing moves)."""

import numpy as np

from _common import make_arm

arm, _ = make_arm(__doc__)
with arm:
    st = arm.state()
    print("q (deg)   ", np.degrees(st.q).round(2))
    print("temp (C)  ", st.temp.round(0))
    p, R = arm.tcp_pose()
    print("TCP (m)   ", p.round(4))
    print("folded    ", arm.is_folded())

"""Shared argument parsing for the examples: ``--sim`` runs in MuJoCo, otherwise the real arm on can0."""

import argparse

from rebot_control import Arm


def make_arm(description: str, tool: str = "pusher_ip17") -> tuple[Arm, argparse.Namespace]:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--sim", action="store_true", help="MuJoCo simulation instead of the real arm")
    p.add_argument("--viewer", action="store_true", help="with --sim: show the MuJoCo viewer")
    p.add_argument("--tool", default=tool)
    p.add_argument("--calibration", default=None)
    args = p.parse_args()
    kw = {"viewer": True} if args.sim and args.viewer else {}
    arm = Arm.from_config("b601_rs", tool=args.tool, calibration=args.calibration,
                          backend="sim" if args.sim else "real", **kw)
    return arm, args

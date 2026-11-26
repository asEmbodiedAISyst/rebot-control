"""rebot-control: Cartesian control, configurable tools and self-calibration for the Seeed reBot Arm (B601-RS).

Quick start (simulation; use ``backend="real"`` on the arm)::

    from rebot_control import Arm
    with Arm.from_config("b601_rs", tool="pusher_ip17", backend="sim") as arm:
        arm.enable()
        arm.move_home()
        p, R = arm.tcp_pose()
        arm.move_to_pose(p + [0.05, 0, 0], R)

See ``docs/user/`` for the guides.
"""

from .arm import Arm
from .cartesian import CartesianController, CartesianLimits, CartesianStatus
from .config import ArmConfig, Calibration, FrictionParams, RippleParams, ToolConfig
from .errors import BackendError, CommandRejected, RebotControlError, SafetyStop
from .ik import DiffIK, IKResult
from .model import RobotModel
from .servo import ServoMode

__version__ = "0.1.0"

__all__ = [
    "Arm", "ArmConfig", "BackendError", "Calibration", "CartesianController", "CartesianLimits", "CartesianStatus",
    "CommandRejected", "DiffIK", "FrictionParams", "IKResult", "RebotControlError", "RippleParams", "RobotModel",
    "SafetyStop", "ServoMode", "ToolConfig", "__version__",
]

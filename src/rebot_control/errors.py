"""Exceptions raised by the library.

* :class:`CommandRejected`: one command was refused (jump, unreachable target); the arm keeps its previous target.
* :class:`SafetyStop`: a fault (stall, over-temperature). The arm holds where it is and stays enabled, except
  with ``safety.on_stall: disable`` or an over-temperature stop with ``safety.on_overtemp: disable`` (the default),
  which switch torque off.
"""


class RebotControlError(RuntimeError):
    pass


class CommandRejected(RebotControlError):
    pass


class SafetyStop(RebotControlError):
    pass


class BackendError(RebotControlError):
    pass

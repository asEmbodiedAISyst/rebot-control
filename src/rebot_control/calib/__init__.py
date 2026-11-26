"""Self-calibration tools (also available as ``rebot-control calib ...``).

* :mod:`.poses`: clear calibration poses (table / base clearance);
* :mod:`.gravity`: link masses / COM and tool payload from static poses;
* :mod:`.friction`: Coulomb + viscous friction from constant-velocity sweeps;
* :mod:`.tcp`: TCP position by pivot calibration;
* :mod:`.gains`: MIT kp / kd step-response checks and tuning.

Each module separates data collection (``collect_*``, moves the arm) from fitting (``fit_*`` / ``solve_*``, pure
functions on saved data). See ``docs/user/calibration.md`` and ``docs/developer/identification.md``.
"""

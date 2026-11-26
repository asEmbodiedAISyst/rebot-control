# Guide for agents that change this repository

## Rules

1. **No real-arm commands without the user.** Do not run `rebot-control` without `--sim`, scripts with
   `backend="real"`, or anything that opens `can0`, unless the user asks in the current session and is at the arm.
   A simulation run is always allowed.
2. **Do not relax safety.** Do not raise or remove any of these unless the user asks for that specific change:
   - jump guard, rate limit, torque cap (`max_error`), feedforward clip (`tau_max`);
   - stall detection, temperature limits, soft joint limits;
   - Cartesian limits and the staged fold.
   Do not remove the confirmation prompt in `cli.py`.
3. **Keep the real path and the simulated path identical.** Control logic belongs in `ServoLoop` / `Arm`, not in a
   backend. A backend only converts units and moves frames.
4. **Units.** Radians and SI in code. Degrees only in YAML `*_deg` keys, converted in `config.py`.
5. **Thread safety.** Change anything the servo thread reads (`ServoLoop` targets, gains, mode, `RobotModel`
   inertias and frames) only under `ServoLoop._lock`. Each thread uses its own `pin.Data`.
6. **Checks before you finish.** `ruff check src tests` and `pytest` (needs `mujoco`). If you changed the
   documentation, also `python docs/build.py`.
7. **Comments.** Write comments and docstrings in English.
8. **Documentation.** Update the page of the reader that the change affects, in English (`docs/`) and in Chinese
   (`docs/zh/`), in the same change. See [`../developer/documentation.md`](../developer/documentation.md).
9. **Hardware notes.** Put raw notes from a real-arm session in `docs/internal/` (ignored by git). Change
   `docs/developer/hardware-notes.md` only when the user asks.

## Where things are

See [codebase-map.md](codebase-map.md). For recipes (add a config key, a tool, a calibration routine, a backend,
a CLI command), see [recipes.md](recipes.md).

## Context

- The library controls the Seeed reBot Arm B601-RS. Its purpose is experiments with interchangeable tools:
  Cartesian control, a configurable TCP, self-calibration.
- `docs/developer/hardware-notes.md` lists the hardware facts that the code relies on and the questions that are
  open on each arm. Treat an open question as an unknown, not as a fact.
- Seeed's `reBotArm_control_py` has no license. Do not copy code from it.

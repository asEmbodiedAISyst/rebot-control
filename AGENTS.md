# AGENTS.md

Instructions for coding agents that work in this repository. Read
[`docs/agents/developing.md`](docs/agents/developing.md) before you change code. The short version:

- This library drives a **real robot arm**. A change to `servo.py`, `joint_filter.py`, `arm.py`, `backends/` or the
  safety defaults in `configs/arms/*.yaml` can hurt people or hardware. Keep those changes minimal, explain them,
  and do not relax a safety limit unless the user asks for it.
- Test in simulation only (`backend="sim"`, `--sim`). Do not run commands against the real arm (`rebot-control`
  without `--sim`, `backend="real"`) unless the user asks and is at the arm.
- Before you finish: `ruff check src tests` and `pytest` must pass. After a documentation change,
  `python docs/build.py` must pass.
- Units: radians in code, degrees only in YAML keys ending in `_deg`. See `docs/developer/conventions.md`.
- Comments and docstrings are in English.
- Document what you change, in English (`docs/`) and in Chinese (`docs/zh/`): user-facing behaviour in `user/`,
  internals in `developer/`. Raw notes from real-arm sessions go to `docs/internal/` (ignored by git).

If you help a person use the library and do not change it, read [`docs/agents/using.md`](docs/agents/using.md).

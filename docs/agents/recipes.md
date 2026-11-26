# Recipes

## Add an arm config key

1. Add a field with a default to the dataclass in `config.py`. Store it in radians or SI.
2. Parse it in `ArmConfig.from_dict` (convert `*_deg`).
3. Add it, with a comment, to `configs/arms/b601_rs.yaml`.
4. Document it in `docs/user/configuration.md` (and in `docs/user/safety.md` if it is a safety parameter).
5. Add a test if it changes behaviour.

## Add a built-in tool

1. Add `configs/tools/<name>.yaml`: TCP, payload, `home_deg`, `cartesian`.
2. Check that the home pose is clear: `is_clear(RobotModel(cfg, tool), tool.home)`. Then try
   `rebot-control home --sim --tool <name>`.
3. Add it to the tools table in `docs/user/configuration.md`.

## Add a calibration routine

1. Add `calib/<name>.py` with a `collect_*(arm, ..., progress=print)` that only uses `Arm` methods, and a pure
   `fit_*(model, data)` that works on saved data.
2. Add a `calib <name>` subcommand in `cli.py`. It saves raw data (`_save_data`), supports `--from-data`, writes
   results with `_save_calibration` or a tool YAML, and asks `_confirm` before moving.
3. In `tests/test_sim.py`, perturb the simulated truth (`truth_*`) and check that the fit recovers it.
4. Document it: `docs/user/calibration.md` (how to run) and `docs/developer/identification.md` (maths, limits). Add a
   test-plan step if it needs the real arm.

## Add a CLI command

Add `cmd_<name>(args)` and a subparser in `build_parser()` with `parents=[common]`. Commands that move the arm must call
`_confirm(args, ...)` before moving. Document it in the module docstring and in the user docs.

## Add a backend

See [`../developer/backends.md`](../developer/backends.md#adding-a-backend). Implement `Backend` in joint units and
register it in `arm.make_backend`. Do not move control logic into it.

## Change servo or filter behaviour

1. Read [`../developer/servo-and-timing.md`](../developer/servo-and-timing.md) first.
2. Make the change, then run the full tests. The friction identification test is sensitive to frame timing.
3. Update the doc. If the change must be checked on the real arm, add a step to `docs/user/hardware-test-plan.md`
   and tell the user.

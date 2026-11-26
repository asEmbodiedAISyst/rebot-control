# Testing

```bash
pip install -e ".[dev]"
pytest                    # a few seconds; the simulator tests are skipped without mujoco
ruff check src tests
```

## What is covered

`tests/test_core.py` is model-only (no simulator):

- config loading and round trips, YAML overrides;
- pose helpers (rotation vectors, rpy, rot6d, pose deltas, tilt / yaw limits);
- FK of the pusher home pose (known TCP position), Jacobian against finite differences, regressor against gravity;
- tool payload, link parameter round trip;
- IK convergence on FK targets;
- joint filter: rate limit, low-pass, jump guard, soft limits;
- Cartesian limit clipping;
- calibration poses are clear of the table; pivot solve is exact on synthetic data.

`tests/test_sim.py` is closed loop in MuJoCo:

- the simulator's bias forces equal Pinocchio gravity;
- life cycle (enable, home, fold on exit);
- Cartesian moves, workspace and speed limits;
- gravity mode does not drift;
- stall detection holds the arm (or switches torque off with `on_stall: disable`); a jump command is rejected;
- abnormal exits (exception, Ctrl-C, over-temperature with `on_overtemp: hold`) leave the arm enabled and still while
  only physics runs; a second `Arm` on the same simulated arm takes the hold over without a jump, a program that
  only reads leaves it held, and `release` switches torque off;
- **identification recovers perturbed truth**: link masses and COM (gravity), payload mass, friction (fc, fv), torque ripple;
- step-response metrics.

`tests/test_docs.py` checks that the Chinese pages mirror the English pages: the same files, and the same number
of headings and code blocks on each page.

## Writing tests

- Use `Arm.from_config(..., backend="sim")` and the `truth_*` arguments of `SimBackend` (pass them as keyword
  arguments to `Arm`) to make the simulated arm differ from the model.
- Simulation is deterministic. Keep motions short: 1 s of simulated time costs about 50 ms.
- Never require hardware in tests. For real-arm checks, add a step to `docs/user/hardware-test-plan.md`, and if it
  is reusable, a CLI command or an example.

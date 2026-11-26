# Real-arm test plan

This is the bring-up of a new arm, or of this library on an arm, one feature at a time. Each step depends on the
ones before it, so do not skip ahead. Every command also runs in simulation (`--sim`): run a step there first to see
what normal output looks like.

Record the results in the table at the end, and keep a note of anything that surprises you.

## Before you start

- Use a Linux machine with the USB-CAN adapter, and `can0` UP ([installation.md](installation.md)).
  `pip install -e ".[real,sim]"`.
- The arm is folded at the zero pose and powered. The table around it is clear (about 60 cm radius).
- Keep one hand near the **motor power switch** for the whole session. Torque off makes the arm fall, so support it
  with the other hand when you expect that.
- Pass `--tool` with the tool that is **actually mounted** (`none`, `pusher_ip17`, `stock_gripper` or your YAML).
  A wrong payload makes the gravity feedforward wrong.
- Before anything that moves, the CLI asks for confirmation. Answer only when the workspace is clear.

## 1. Feedback, torque off: `rebot-control state`

```bash
rebot-control state --tool <tool>
rebot-control state --tool <tool> --watch      # 20 readings, 0.5 s apart
```

| Check | Pass |
| --- | --- |
| all 6 motors answer | no "no feedback from motors" error |
| angles at the folded pose | every joint within about 2 deg of 0 |
| temperatures | room temperature, all 6 similar |
| `folded: True` | yes |

**Sign check.** With torque off, move the joints by hand and watch `--watch`. Support the arm before you lift
anything. Positive directions:

| Joint | Positive when ... |
| --- | --- |
| 1 (base) | the arm turns **clockwise seen from above** |
| 2 (shoulder) | the upper arm lifts out of the folded pose (range starts at 0) |
| 3 (elbow) | the forearm opens out of the folded pose (range starts at 0) |
| 4 (wrist flex) | at the folded pose, the wrist tips **up** |
| 5 (wrist yaw) | at the folded pose, the wrist turns **counterclockwise seen from above** |
| 6 (wrist roll) | **clockwise** seen from in front of the flange (looking at its face) |

A wrong sign means a mismatched `direction` in the arm config. **Do not enable torque until every sign is right.**

## 2. Torque on at the folded pose: `rebot-control hold`

```bash
rebot-control hold --tool <tool> --seconds 10
```

| Check | Pass |
| --- | --- |
| enable | no jump; the arm stiffens over about 0.5 s (gain ramp) |
| lag | under 0.5 deg |
| servo overruns | stays at 0 or grows only slowly (more than about 1 per second means the PC is too slow or busy) |
| sound | no buzzing or squealing |
| exit | the arm stays folded, torque off |

## 2b. Holding through a dead program, taking over, releasing

Everything later in this plan relies on this, so check it once at the folded pose, where a fall is harmless.
Background: [safety.md](safety.md).

```bash
rebot-control state --tool <tool>                    # torque: off
rebot-control hold --tool <tool> --seconds 60        # while it holds: Ctrl-C  -> it folds and switches off (normal end)
rebot-control hold --tool <tool> --seconds 60 &      # while it holds: kill -9 %1  (a program that dies)
rebot-control state --tool <tool>                    # must print "torque: ON, holding (left as found)"
rebot-control release --tool <tool>                  # hand on the arm; torque off
```

| Check | Pass |
| --- | --- |
| `state` on a disabled arm | nothing moves, nothing stiffens; `torque: off` |
| after `kill -9` | the arm stays stiff and does not move; no program is running |
| `state` on the held arm | no twitch when it connects or leaves; `torque: ON, holding (left as found)`; the arm stays stiff afterwards |
| a second `hold` on the held arm | logs "the motors are enabled: taking over"; no jump |
| `release` | asks first; torque off; `state` then prints `torque: off` |
| connect log | no warning about a CAN timeout |

Then repeat `kill -9` once with the shoulder and elbow raised a few degrees (a hand under the forearm): the arm
must stay up.

## 3. Gravity mode: `rebot-control float`

```bash
rebot-control float --tool <tool> --seconds 60
```

Guide the arm out of the fold by hand, slowly, shoulder and elbow first. Let go at a few poses.

| Check | Pass |
| --- | --- |
| feel | light, no direction where it pushes on its own |
| drift after letting go | under a few degrees; note which joints drift and which way (this is what gravity calibration fixes) |
| exit (Ctrl-C) | holds where it is, then folds back slowly in stages |

If it pushes hard in one direction, stop it (Ctrl-C, or the power switch while supporting the arm). Check the
signs (step 1) and the `--tool` payload.

## 4. Unfold, home, fold: `rebot-control home`

```bash
rebot-control home --tool <tool> --seconds 5
```

| Check | Pass |
| --- | --- |
| unfold | shoulder and elbow first, then the wrist; nothing rubs |
| home pose | printed q matches the tool's `home_deg` within 0.5 deg |
| TCP | the printed TCP position matches a ruler measurement (base frame: x forward, z up from the table) within about 1 cm |
| fold | base and wrist first, then shoulder and elbow; ends at zero |

## 5. Joint moves and streaming: `examples/03_joint_moves.py`

```bash
cd examples && python 03_joint_moves.py --tool <tool>
python 06_record.py --tool <tool>        # servo log -> servo_log.npz
```

| Check | Pass |
| --- | --- |
| slow moves | smooth, no steps or jerks |
| streaming sine | "max lag" under about 3 deg |
| `servo_log.npz` | `q_cmd - q` small (under 1 deg); keep the file |

**Stall test** (optional). Block the base joint with your hand during a slow move. After about 1 s you should get a
`SafetyStop`. With the default `safety.on_stall: hold`, the arm holds where it is and stays enabled. End the hold
with `rebot-control release --fold`. With `on_stall: disable`, torque goes off and **the arm falls**: a second
person must support the forearm before you run the test with that setting.

## 6. Cartesian: `rebot-control square`, `examples/04_cartesian.py`

```bash
rebot-control square --tool <tool> --side 0.08 --speed 0.05
cd examples && python 04_cartesian.py --tool <tool>
```

| Check | Pass |
| --- | --- |
| square | straight edges; corners within about 2 mm of the printed targets |
| tracking error | a few mm while moving (filter lag, expected), under 1 mm at rest |
| orientation | the tool axis does not wobble along the edges |
| IK refusals | none for these small motions |

## 7. MIT gains check: `rebot-control calib gains --check`

```bash
rebot-control calib gains --tool <tool> --check            # 2 deg steps on every joint, configured gains
```

**Listen during the steps.** For each joint, note overshoot, settle time, residual velocity and ripple. Typical
results are overshoot under 10 % and settle under 0.3 s. Stop at once if a joint buzzes or rings.

Optional, one joint at a time, starting with a wrist joint:

```bash
rebot-control calib gains --tool <tool> --joints 6 --kp-cap 2.0
```

This raises kp until a limit trips (ringing, buzz, overshoot) or the cap is reached. It prints `kp_max` and
recommended gains. It saves nothing unless you pass `--save`. See [gains.md](gains.md) before changing gains.

## 8. Gravity calibration: `rebot-control calib gravity`

Run this with the bare flange (`--tool none`) if you can. Otherwise use the mounted tool with its payload set
correctly.

```bash
rebot-control calib gravity --tool none --poses 30          # about 5-10 min
rebot-control float --tool none --calibration calibration/b601_rs.yaml   # compare with step 3
```

| Check | Pass |
| --- | --- |
| poses | all clear of the table; the CLI prints the pose list first |
| residual RMS after fit | well below "before", typically under 0.1 N*m for joints 2 and 3 |
| identified masses | plausible (no negative or absurd values; warnings are printed) |
| float with calibration | less drift than in step 3 |

The raw data is saved under `calibration/data/`. A refit needs no robot:
`rebot-control calib gravity --from-data <file.npz> --prior-rel 0.2`.

## 9. Friction: `rebot-control calib friction`

Start with one wrist joint and a small amplitude:

```bash
rebot-control calib friction --tool <tool> --joints 6 --amplitude-scale 0.5
rebot-control calib friction --tool <tool>                  # all joints, after the first one looks right
```

Then enable friction feedforward in a copy of the arm config (`feedforward.friction_scale: 0.5`) and repeat the float
test and the Cartesian square.

| Check | Pass |
| --- | --- |
| sweeps | smooth constant-speed motion, no stall |
| fit | fc and fv positive; RMS small compared with fc |
| with `friction_scale: 0.5` | gravity mode feels lighter; tracking error during the square is smaller |

## 10. Tool payload and TCP

```bash
rebot-control calib payload --tool <tool> --calibration calibration/b601_rs.yaml --out my_tool.yaml
rebot-control calib tcp --tool my_tool.yaml --calibration calibration/b601_rs.yaml --out my_tool.yaml
```

- **Payload**: run it after gravity calibration with the tool mounted. It writes a tool YAML with the identified
  mass and COM.
- **TCP**: put a small dimple (a drilled hole or a cone) in a block fixed to the table. In gravity mode, place the tip
  in it, tilt the tool to a new orientation, hold still and press Enter. Do this 8 times, spreading the orientations
  by at least 30 deg. Pass: residual RMS under about 1 mm.

## Results template

| Step | Date | Tool | Result | Numbers / notes |
| --- | --- | --- | --- | --- |
| 1 state + signs | | | | |
| 2 hold | | | | overruns: |
| 2b hold through a dead program / take over / release | | | | |
| 3 float | | | | drift per joint: |
| 4 home | | | | TCP measured vs printed: |
| 5 joint moves | | | | max lag: |
| 6 Cartesian | | | | corner errors: |
| 7 gains check | | | | per joint overshoot / settle: |
| 8 gravity calib | | | | RMS before -> after: |
| 9 friction | | | | fc, fv per joint: |
| 10 payload / TCP | | | | mass, COM; TCP RMS: |

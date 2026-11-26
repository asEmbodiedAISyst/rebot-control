# Concepts

This page explains the terms that the other pages use.

## The control chain

Your code sends targets at the control rate (30 Hz). The library turns them into motor commands at the servo rate
(150 Hz).

```
your loop (30 Hz)  ->  joint target filter  ->  servo loop (150 Hz)  ->  motors (MIT mode)
 joint or TCP target    limits, smoothing        interpolation, feedforward
```

- A **joint target** is one angle per joint. A **TCP target** is a pose of the tool tip. The Cartesian controller
  converts a TCP target into a joint target with inverse kinematics (IK).
- The **joint target filter** clips, rate-limits and smooths every joint target before the arm sees it.
- The **servo loop** interpolates between targets and adds the feedforward torque.
- In **MIT mode**, each motor driver runs a PD law on the last frame it received:
  `tau = kp (q* - q) + kd (dq* - dq) + tau_ff`. See [MIT gains](gains.md).

## Backends

A backend connects the control code to an arm. There are two:

- `real`: the RobStride motors on SocketCAN. It needs Linux and the `real` extra.
- `sim`: a MuJoCo model of the same arm. It runs on any OS and needs the `sim` extra.

The control code is the same for both. A script that runs in simulation runs on the real arm when you change
`backend="sim"` to `backend="real"`.

## Modes

| Mode | Torque | What the arm does |
| --- | --- | --- |
| `PASSIVE` | off | Nothing. The arm is limp and falls if it is not supported. |
| `HOLD` | on | Follows the joint target with the PD law and the gravity feedforward. |
| `GRAVITY` | on | `kp = 0`: only gravity (and friction) feedforward. You can move the arm by hand. |

## Poses

- The **folded pose** (zero pose) is the resting pose: every joint at 0. The arm starts and ends there.
- The **home pose** belongs to a tool. It is the working pose that `arm.move_home()` goes to.
- The arm unfolds and folds **in stages**, so that the wrist does not rub the upper arm.

## Tools

A tool is a YAML file. It gives the library:

- the **TCP** (tool centre point): the pose of the tool tip in the flange frame;
- the **payload**: the mass and centre of mass of the tool, used by the gravity feedforward;
- the home pose and the default Cartesian limits.

The **flange** is the last link of the arm (`link6`), where the tool is mounted.

## Feedforward and calibration

The **feedforward** is the torque that the model predicts the arm needs: gravity, and optionally friction and torque
ripple. With a good model, the PD law only corrects a small error.

**Calibration** measures the model on your arm: link masses, tool payload, friction and the TCP position. The
results go into a calibration file that belongs to one physical arm. See [Calibration](calibration.md).

## When a program ends

- A normal end folds the arm and then switches torque off.
- After an error or Ctrl-C the arm **holds its pose and stays enabled**. The next program takes the hold over. You
  end it with `rebot-control release`.

See [Safety](safety.md) for the reasons and the risks.

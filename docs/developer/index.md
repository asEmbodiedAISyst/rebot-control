# Developer guide

- [Architecture](architecture.md): layers, threads, data flow and the life cycle of an `Arm`.
- [Conventions](conventions.md): units, frames, sign conventions, naming and config rules.
- [Backends](backends.md): the RobStride / motorbridge backend and its quirks, and the MuJoCo backend.
- [Servo loop and timing](servo-and-timing.md): interpolation, zero-order hold, feedforward, thread timing.
- [Identification](identification.md): the maths behind the calibration tools, and their known limits.
- [Testing](testing.md): what the tests cover and how to add one.
- [Design decisions](design-decisions.md): why things are the way they are.
- [Hardware notes](hardware-notes.md): facts about the arm that the code relies on, and questions to answer on
  each arm.
- [Roadmap](roadmap.md): the state of the project and what comes next.
- [Documentation](documentation.md): how the documentation is organised, written and built.
- [Glossary](glossary.md): the English and Chinese terms used in the documentation.

```{toctree}
:hidden:

architecture
conventions
backends
servo-and-timing
identification
testing
design-decisions
hardware-notes
roadmap
documentation
glossary
```

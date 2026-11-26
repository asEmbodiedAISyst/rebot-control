# rebot-control

Cartesian control, configurable tools and self-calibration for the Seeed reBot Arm B601-RS (6 RobStride motors on
SocketCAN), with a MuJoCo simulator that runs the same control code.

```{warning}
Version 0.1.0 is a pre-release. The automated tests run in simulation only, and real-arm validation is in progress.
Read [Safety](user/safety.md) before you power a real arm.
```

The documentation is split by reader:

- The [user guide](user/index.md) is for people who use the library to run an arm. It covers installation, the
  first real-arm session, tools, Cartesian control, calibration, gain tuning and safety.
- The [developer guide](developer/index.md) is for people who change the library. It covers the design, the
  conventions, hardware quirks, the identification maths, tests and design decisions.
- [AI agents](agents/index.md) has one guide for agents that use the library and one for agents that change this
  repository.
- The [API reference](apidocs/index) is generated from the docstrings.

```{toctree}
:hidden:
:maxdepth: 2

user/index
developer/index
agents/index
API reference <apidocs/index>
```

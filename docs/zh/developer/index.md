# 开发者指南

- [架构](architecture.md)：分层、线程、数据流，以及 `Arm` 的生命周期。
- [约定](conventions.md)：单位、坐标系、符号约定、命名和配置规则。
- [后端](backends.md)：RobStride / motorbridge 后端及其特殊行为，MuJoCo 后端。
- [伺服循环与时序](servo-and-timing.md)：插值、零阶保持、前馈、线程时序。
- [辨识](identification.md)：标定工具背后的数学，以及已知的局限。
- [测试](testing.md)：测试覆盖了什么，怎么加一个测试。
- [设计决策](design-decisions.md)：为什么做成现在这样。
- [硬件说明](hardware-notes.md)：代码依赖的机械臂事实，以及每台机械臂上要回答的问题。
- [路线图](roadmap.md)：项目现状和后续计划。
- [文档](documentation.md)：文档怎么组织、怎么写、怎么构建。
- [术语表](glossary.md)：文档里用到的中英文术语。

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

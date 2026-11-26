# rebot-control

rebot-control 是 Seeed reBot Arm B601-RS（6 个 RobStride 电机，SocketCAN 总线）的控制库，提供笛卡尔控制、可配置的工具和自标定。
它自带一个 MuJoCo 仿真器，仿真和真机运行同一套控制代码。

```{warning}
0.1.0 是预发布版本。自动化测试只在仿真中运行，真机验证还在进行。给真机上电之前，请先阅读[安全](user/safety.md)。
```

文档按读者分为几部分：

- [用户指南](user/index.md)写给用这个库驱动机械臂的人。内容包括安装、首次上机调试、工具、笛卡尔控制、标定、增益整定和安全。
- [开发者指南](developer/index.md)写给修改这个库的人。内容包括设计、约定、硬件上的特殊之处、辨识用到的数学、测试和设计决策。
- [AI agent](agents/index.md) 部分有两份指南：一份给使用本库的 agent，一份给修改本仓库的 agent。
- <a href="../en/apidocs/index.html">API 参考（英文）</a>由 docstring 自动生成，只有英文版。

```{toctree}
:hidden:
:maxdepth: 2

user/index
developer/index
agents/index
```

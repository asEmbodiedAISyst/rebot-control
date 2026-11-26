# 安装

## Python

需要 Python 3.10 或更新的版本。

```bash
git clone https://github.com/asEmbodiedAISyst/rebot-control.git && cd rebot-control
python -m venv .venv && source .venv/bin/activate
pip install -e ".[real,sim]"     # real arm + simulator
# pip install -e ".[sim]"        # simulator only (no arm)
```

| Extra | 安装的包 | 用途 |
| --- | --- | --- |
| （无） | numpy、pin（Pinocchio）、pyyaml | 模型、IK、离线重新拟合标定数据 |
| `real` | motorbridge >= 0.5.6 | 真机（Linux、SocketCAN） |
| `sim` | mujoco >= 3.2 | `backend="sim"`、`--sim`、测试 |
| `dev` | pytest、ruff、mujoco | 开发 |

装好后运行 `rebot-control state --sim` 检查。它会打印仿真机械臂的状态。

### 平台

| 系统 | 仿真 | 真机 |
| --- | --- | --- |
| Linux | 支持 | 支持 |
| macOS | 支持 | 不支持 |
| Windows | 未验证 | 不支持 |

真机需要 SocketCAN，只有 Linux 提供。

Windows 未经验证，也没有做过测试。PyPI 上没有 Pinocchio 的 Windows 版本，所以上面的 `pip install` 命令在 Windows 上会失败。

## CAN 适配器（真机）

机械臂通过 USB-CAN 适配器通信，速率为 1 Mbit/s。适配器在系统里是 SocketCAN 接口 `can0`。
电脑重启或适配器重新插拔之后，这个接口处于 **DOWN** 状态。库会检查这一点；接口没有启用时它会停下，并给出要执行的命令：

```bash
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -br link show can0            # should say UP
```

想在开机时自动启用接口，可以加一个 systemd-networkd 的 `.network` 文件，或者写一条 udev 规则来执行上面的前两条命令。
接口不叫 `can0` 时，把它的名字写到机械臂配置的 `bus.channel` 里。

## 电机零位和 ID

库默认以下设置：

- CAN ID 为 1 到 6（从基座到腕部滚转关节），反馈 ID 为 `0xFD`；
- 电机型号：关节 1 到 3 是 `rs-06`，关节 4 到 6 是 `rs-00`；
- 电机零位设在折叠位姿，所以电机原始角度等于 URDF 关节角。

这与 Seeed 软件使用的出厂设置一致。如果你的电机零位或安装方式不同，请在机械臂配置里为每个电机设置 `direction` /
`offset_deg`（见 [configuration.md](configuration.md)）。重新设零位可以用 Motorbridge Studio，也可以用
`RobStrideBackend.set_zero`；操作时机械臂要处于卸力状态，并用手扶在折叠位姿。重新设零位会覆盖电机里保存的零位。

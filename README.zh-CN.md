# litegrip-isaacsim

> 中文版：本文件 · 英文版：[README.md](README.md)

**LiteGrip 轻量型机械夹爪系列**的 NVIDIA Isaac Sim 仿真环境，外加一个 ROS 2 桥接节点，
通过原生 SocketCAN 把仿真开口量映射到真机夹爪。

> **状态：** 仿真模型、仿真节点和 litegrip 桥接均已就位。真机开口量由 SDK 自带的
> 标定文件推导 —— 见[真机桥接](#真机桥接)。

## 适用范围

| | |
| --- | --- |
| 产品 | LiteGrip 轻量型机械夹爪系列 |
| 仓库定位 | NVIDIA Isaac Sim 仿真环境 |
| 状态 | 模型 + 仿真节点 + 桥接已就位 |

## 目录结构

```text
.
├── gripper2/
│   ├── gripper2.urdf           # 3 个 link + 2 个移动关节，每指 0.067 m
│   ├── gripper2.usd            # 已扁平化、自包含（可直接加载）
│   ├── meshes/                 # base_link.STL + gripper_slider_link1/2.STL
│   ├── import_gripper2.py      # URDF → gripper2.usd 导入脚本
│   └── test_gripper2.py        # 不依赖 ROS 的 headless 开合自检
├── isaac_sim_gripper.py        # 仿真节点，按 /gripper/joint_traj 驱动关节
├── run_gripper.sh              # 带夹爪节点启动 Isaac Sim
├── gripper_litegrip_bridge.py  # 真机夹爪桥接（litegrip over SocketCAN）
└── tests/test_gripper_bridge.py  # 不依赖硬件的单元测试
```

## 快速开始

`gripper2/gripper2.usd` 已提交入库，克隆后无需再导入。只有 Isaac Sim 安装目录与本机相关：

```bash
export ISAAC_SIM_PATH=/path/to/isaac-sim   # 默认：/home/qql/nvidia/isaac-sim
./run_gripper.sh
```

## 在容器中开发

VS Code 用户可完全跳过本机安装：仓库自带开发容器，内含 Isaac Sim 4.5、ROS 2
Humble 桥接和 SocketCAN 工具。安装 Dev Containers 扩展，打开仓库，按 F1 选择
"Dev Containers: Reopen in Container"。首次构建会下载 NVIDIA Isaac Sim 镜像，
约 20 GB。

容器内 `ISAAC_SIM_PATH` 已预设为 `/isaac-sim`，`./run_gripper.sh` 和上面的重建
命令照常可用。GPU 配置、宿主机 ROS 组网和已知坑见
[.devcontainer/README.md](.devcontainer/README.md)。

## 驱动夹爪

`isaac_sim_gripper.py` 订阅 `/gripper/joint_traj`（`trajectory_msgs/JointTrajectory`），
并在 Isaac Sim 的时间轴上采样轨迹。

- `positions` 的顺序即节点启动日志里打印的关节名顺序；随附模型为
  `[gripper_slide_joint_left, gripper_slide_joint_right]`。
- 旋转关节取**弧度**（内部转成度）；移动关节取**米**，原样写入。
- `gripper2` 的关节约定：`q = 0` 全开，`q = +0.067` 全闭。

节点接受 USD 里出现的任何关节类型，所以换一款夹爪只需要新的 URDF/USD 加一个对应的
`USD_PATH`。

## 重建模型

仅在改动 `gripper2/gripper2.urdf` 或 `gripper2/meshes/` 之后需要：

```bash
cd "$ISAAC_SIM_PATH"
./kit/python/bin/python3 "$OLDPWD/gripper2/import_gripper2.py" \
  --/renderer/multiGpu/enabled=false
```

完全不依赖 ROS 的 headless 开合自检：

```bash
cd "$ISAAC_SIM_PATH"
./kit/python/bin/python3 "$OLDPWD/gripper2/test_gripper2.py" \
  --/renderer/multiGpu/enabled=false
```

## 真机桥接

`gripper_litegrip_bridge.py` 订阅同一个 `/gripper/joint_traj`，把关节位置映射为真机夹爪的
开口量（**毫米**），并经 `litegrip` 库驱动一颗达妙 DM4310（CAN ID `0x08`）。

`litegrip` 是自包含的纯 CAN 库：只需 Python 标准库和 Linux SocketCAN，所以本桥接
**不需要** `litearm-server`、`litearm-python`，也不需要任何在跑的 RPC 端点。只要 `can0`
处于 up 状态即可。

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

python3 gripper_litegrip_bridge.py --dry-run    # 只打印映射后的毫米值，不碰硬件
python3 gripper_litegrip_bridge.py --can can0   # 真正驱动夹爪
```

桥接按以下顺序找最新的 SDK：

1. 已安装的 `litegrip-python`（`pip install -U litegrip-python`）；
2. 仓库旁边的 `../litegrip-python/src` 源码；
3. `--lib-dir` 指定的目录（该目录下必须有 `litegrip/`）。

标定也来自这份 SDK。不带参数的 `load_calibration()` 读取 SDK 自带的
`factory_calibration.json`（实测几何，当前为 `1.170748 rad × 73.4573 mm/rad = 86.0 mm`
行程）。要使用本机实测的标定，传 `--calib PATH`（例如
`~/.litegrip/can0_calibration.json`），或设置 `LITEGRIP_CALIB`。

| | |
| --- | --- |
| 仿真侧 | 关节位置，米，`0` = 开，`0.067` = 闭 |
| 真机侧 | 开口量，毫米，`0` = 闭，`OPEN_MM` = 开 |
| 全开量 | `OPEN_MM` = 标定文件推导的行程（当前出厂文件为 `86.0` mm） |
| 映射 | `mm = CLOSED_MM + (OPEN_MM - CLOSED_MM) * (1 - q_avg / Q_FULL_M)` |

接真机之前需要知道两件事：

1. **开口量来自标定文件，不再是写死的常量。** 默认把仿真的全开姿态映射到标定行程；
   `--open-mm` 可覆盖（例如本机实测的间隙），桥接会把它夹到标定行程内。这层夹紧由桥接
   自己做：`send_mit_frame` 是底层帧，SDK 不会替它套 `clamp_to_travel`。
2. **只取轨迹的最后一个点。** 桥接不沿轨迹插值，而是用 `--speed-mm-s`（默认 `30` mm/s）
   对真机做限速。过冲不会损坏机构 —— 每个目标都被夹紧到标定行程范围内。

桥接不写任何磁盘文件，可用 `--dry-run` 只读运行。

## 测试

测试套件不依赖硬件：覆盖关节到毫米的映射、目标去重，以及轨迹回调。

```bash
python3 -m unittest discover -s tests -v
```

Isaac Sim 自检（`gripper2/test_gripper2.py`）需要 GPU 和本机 Isaac Sim 安装，因此不在 CI 内。

## 相关仓库

| 仓库 | 作用 |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | 产品文档 |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 驱动 |

## 仓库规范

本仓库遵循 NEXFORM ROBOTICS 共享的仓库规范：[`.nexform-standards.json`](.nexform-standards.json) 中的智能体操作规则、
Conventional Commits，以及每次合并到 `main` 时自动执行的 semantic-release 版本管理。

## 许可证

Copyright © 2026 NEXFORM ROBOTICS。基于 [Apache License 2.0](LICENSE) 授权。

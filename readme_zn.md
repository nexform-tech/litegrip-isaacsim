# litegrip-isaacsim

> 中文版：本文件 · 英文版：[README.md](README.md)

**LiteGrip 轻量型机械夹爪系列**的 NVIDIA Isaac Sim 仿真环境，外加一个 ROS 2 桥接节点，
通过原生 SocketCAN 把仿真开口量映射到真机夹爪。

> **状态：** 仿真模型、仿真节点和 litegrip 桥接均已就位。真机夹爪开口标定（`OPEN_MM`）
> 仍是占位值 —— 见[真机桥接](#真机桥接)。

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

| | |
| --- | --- |
| 仿真侧 | 关节位置，米，`0` = 开，`0.067` = 闭 |
| 真机侧 | 开口量，毫米，`0` = 闭，`travel_mm` = 开 |
| 映射 | `mm = CLOSED_MM + (OPEN_MM - CLOSED_MM) * (1 - q_avg / Q_FULL_M)` |

接真机之前需要知道两件事：

1. **`OPEN_MM` 是未标定的占位值。** 当 `OPEN_MM = None` 时，桥接会退回到
   `OPEN_MM_CALIB = 120.06`（由出厂标定推出：`travel_range_rad 1.605 × rad_to_mm 74.8`）
   并打印一条警告。请实测真机全开时的间隙，然后在模块头部设置 `OPEN_MM`。同一物理量的
   另一个候选值是 `134`（两指 × 67 mm），约大 10%；二者选其一，不要混用。
2. **只取轨迹的最后一个点。** 桥接不沿轨迹插值，而是用 `--speed-mm-s`（默认 `30` mm/s）
   对真机做限速。过冲不会损坏机构 —— 库会把每个目标夹紧到标定行程范围内。

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

本仓库遵循 NEXFORM ROBOTICS 共享的仓库规范：[AGENTS.md](AGENTS.md) 中的智能体操作规则、
Conventional Commits，以及每次合并到 `main` 时自动执行的 semantic-release 版本管理。

## 许可证

Copyright © 2026 NEXFORM ROBOTICS。基于 [Apache License 2.0](LICENSE) 授权。

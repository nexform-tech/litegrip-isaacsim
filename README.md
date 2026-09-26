# litegrip-isaacsim

NVIDIA Isaac Sim simulation environment for the **LiteGrip lightweight robotic
gripper series**, plus a ROS 2 bridge that mirrors the simulated opening onto the
real gripper over raw SocketCAN.

> **Status:** simulation model, simulation node and the litegrip bridge have
> landed. The real-gripper opening calibration (`OPEN_MM`) is still a placeholder
> — see [Real-hardware bridge](#real-hardware-bridge).

## Scope

| | |
| --- | --- |
| Product | LiteGrip lightweight robotic gripper series |
| Repository role | NVIDIA Isaac Sim simulation environment |
| Status | Model + simulation node + bridge landed |

## Layout

```text
.
├── gripper2/
│   ├── gripper2.urdf           # 3 links + 2 prismatic joints, 0.067 m per finger
│   ├── gripper2.usd            # flattened, self-contained (loadable as-is)
│   ├── meshes/                 # base_link.STL + gripper_slider_link1/2.STL
│   ├── import_gripper2.py      # URDF → gripper2.usd importer
│   └── test_gripper2.py        # headless open/close self-check (no ROS)
├── isaac_sim_gripper.py        # simulation node, drives joints from /gripper/joint_traj
├── run_gripper.sh              # starts Isaac Sim with the gripper node
├── gripper_litegrip_bridge.py  # real gripper bridge (litegrip over SocketCAN)
└── tests/test_gripper_bridge.py  # hardware-free unit tests
```

## Quick start

`gripper2/gripper2.usd` is committed, so a clone needs no import step. Only the
Isaac Sim installation directory is machine-specific:

```bash
export ISAAC_SIM_PATH=/path/to/isaac-sim   # default: /home/qql/nvidia/isaac-sim
./run_gripper.sh
```

## Driving the gripper

`isaac_sim_gripper.py` subscribes to `/gripper/joint_traj`
(`trajectory_msgs/JointTrajectory`) and samples the trajectory on the Isaac Sim
timeline.

- `positions` order is the joint-name order printed in the node's startup log;
  for the bundled model that is
  `[gripper_slide_joint_left, gripper_slide_joint_right]`.
- Revolute joints take **radians** (converted to degrees internally); prismatic
  joints take **metres**, written as-is.
- Joint convention for `gripper2`: `q = 0` fully open, `q = +0.067` fully closed.

The node accepts any joint type it finds in the USD, so a different gripper only
needs a new URDF/USD plus a matching `USD_PATH`.

## Rebuilding the model

Only needed after editing `gripper2/gripper2.urdf` or `gripper2/meshes/`:

```bash
cd "$ISAAC_SIM_PATH"
./kit/python/bin/python3 "$OLDPWD/gripper2/import_gripper2.py" \
  --/renderer/multiGpu/enabled=false
```

A headless open/close self-check that needs no ROS at all:

```bash
cd "$ISAAC_SIM_PATH"
./kit/python/bin/python3 "$OLDPWD/gripper2/test_gripper2.py" \
  --/renderer/multiGpu/enabled=false
```

## Real-hardware bridge

`gripper_litegrip_bridge.py` subscribes to the same `/gripper/joint_traj` and
maps the joint positions to a real gripper opening in **millimetres**, driving a
Damiao DM4310 (CAN ID `0x08`) through the `litegrip` library.

`litegrip` is a self-contained pure-CAN library: it needs only Python's standard
library plus Linux SocketCAN, so the bridge does **not** need `litearm-server`,
`litearm-python`, or a running RPC endpoint. Only `can0` has to be up.

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

python3 gripper_litegrip_bridge.py --dry-run    # print the mapped mm, touch nothing
python3 gripper_litegrip_bridge.py --can can0   # actually move the gripper
```

| | |
| --- | --- |
| Simulated | joint position, metres, `0` = open, `0.067` = closed |
| Real | opening, millimetres, `0` = closed, `travel_mm` = open |
| Mapping | `mm = CLOSED_MM + (OPEN_MM - CLOSED_MM) * (1 - q_avg / Q_FULL_M)` |

Two things to know before pointing it at hardware:

1. **`OPEN_MM` is an uncalibrated placeholder.** With `OPEN_MM = None` the bridge
   falls back to `OPEN_MM_CALIB = 120.06` (derived from the factory calibration:
   `travel_range_rad 1.605 × rad_to_mm 74.8`) and logs a warning. Measure the
   real fully-open gap, then set `OPEN_MM` in the module header. A second
   candidate for the same quantity is `134` (two fingers × 67 mm), about 10%
   larger; pick one and do not mix them.
2. **Only the last trajectory point is used.** The bridge does not interpolate
   along the trajectory; it rate-limits the real gripper with `--speed-mm-s`
   (default `30` mm/s) instead. Overshoot cannot damage the mechanism — the
   library clamps every target to the calibrated travel range.

The bridge writes nothing to disk and can be run read-only with `--dry-run`.

## Tests

The suite is hardware-free: it covers the joint-to-millimetre mapping, the
target de-duplication, and the trajectory callback.

```bash
python3 -m unittest discover -s tests -v
```

The Isaac Sim self-checks (`gripper2/test_gripper2.py`) need a GPU and a local
Isaac Sim install, so they are not part of CI.

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).

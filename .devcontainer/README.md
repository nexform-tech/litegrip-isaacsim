# Dev container for litegrip-isaacsim

This file tells VS Code users how to open the repository's dev container and
what breaks when the host setup is wrong; the simulation itself is documented
in the repository [README](../README.md).

## 1. Open the container

1. Install Docker with GPU support. On Windows that is Docker Desktop with the
   WSL 2 backend; on Linux, Docker Engine plus the NVIDIA Container Toolkit.
2. Install the Dev Containers extension in VS Code.
3. Open the repository folder, press F1, and choose
   "Dev Containers: Reopen in Container".

The first build pulls `nvcr.io/nvidia/isaac-sim:4.5.0`, about 20 GB. Pulling
and running the image records acceptance of NVIDIA's EULA, which is why the
container sets `ACCEPT_EULA=Y` and `PRIVACY_CONSENT=Y`, the same two variables
NVIDIA's own run examples use.

## 2. What is inside

| | |
| --- | --- |
| Base image | `nvcr.io/nvidia/isaac-sim:4.5.0`, standalone layout at `/isaac-sim` |
| ROS 2 | Humble bridge bundled with Isaac Sim, `exts/isaacsim.ros2.bridge/humble` |
| Preset environment | `ISAAC_SIM_PATH=/isaac-sim`, so `./run_gripper.sh` needs no `export` |
| Added OS tools | `can-utils` and `iproute2` for the SocketCAN bridge, `git`, `vim`, `less` |
| Python | `/isaac-sim/kit/python/bin/python3` is the VS Code interpreter; plain `python3` is the OS Python 3.10 and runs the unit tests |
| Caches | kit, Omniverse, pip, GLCache, and ComputeCache are named volumes, so they survive rebuilds |

The container runs as root, matching NVIDIA's documented container workflow.
On a Linux host that means files created inside the workspace are owned by
root on the host; on Windows and macOS Docker Desktop maps them to the host
user.

## 3. Run things

All commands from the workspace root, inside the container:

```bash
./run_gripper.sh                              # full simulation with the ROS 2 node
python3 -m unittest discover -s tests -v      # hardware-free unit tests
/isaac-sim/kit/python/bin/python3 gripper2/test_gripper2.py \
  --/renderer/multiGpu/enabled=false          # headless open/close self-check
```

## 4. Change the Isaac Sim version

Edit `ISAAC_SIM_VERSION` in [devcontainer.json](devcontainer.json), then run
"Dev Containers: Rebuild and Reopen in Container". Stay on a 4.x tag: the
repository's scripts are written against the 4.x standalone layout
(`setup_python_env.sh`, `setup_ros_env.sh`, `kit/python/bin/python3`) and the
ROS 2 Humble bridge. The post-create hook warns when those files are missing
from the install.

## 5. Known pitfalls

- **Do not** expect host ROS nodes to reach the bridge through Docker Desktop
  on Windows: its containers cannot use host networking, and FastDDS discovery
  cannot cross into the Windows host. Run every ROS node inside the container,
  or, on a Linux host with Docker Engine, add `"--network=host"` to `runArgs`
  in [devcontainer.json](devcontainer.json) so the container joins the host
  network directly.
- **Do not** remove `--gpus all` unless you only need the unit tests: Isaac
  Sim does not start without a working NVIDIA GPU and driver.
- **Do not** add comments to [devcontainer.json](devcontainer.json): the CI
  workflow parses it as strict JSON, and a comment breaks the check.

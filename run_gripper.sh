#!/bin/bash
# 启动夹爪仿真节点（Isaac Sim + ROS2 订阅，与机械臂并行、独立）
# Isaac Sim 安装目录：默认 /home/qql/nvidia/isaac-sim，可用 ISAAC_SIM_PATH 覆盖
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ISAAC_SIM="${ISAAC_SIM_PATH:-/home/qql/nvidia/isaac-sim}"

cd "$ISAAC_SIM" || {
  echo "找不到 Isaac Sim: $ISAAC_SIM（请 export ISAAC_SIM_PATH=/你的/isaac-sim 路径）" >&2
  exit 1
}
source ./setup_python_env.sh
source ./setup_ros_env.sh
export LD_LIBRARY_PATH="$PWD/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH"
export LD_PRELOAD=$PWD/kit/libcarb.so
export CARB_APP_PATH=$PWD/kit
export ISAAC_PATH=$PWD
export EXP_PATH=$PWD/apps
exec ./kit/python/bin/python3 "$SCRIPT_DIR/isaac_sim_gripper.py" \
  --/renderer/multiGpu/enabled=false

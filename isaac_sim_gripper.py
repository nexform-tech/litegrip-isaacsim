#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪 Isaac Sim 仿真节点：订阅 ROS2 /gripper/joint_traj 驱动夹爪关节。

与机械臂仿真完全并行、独立：本节点只加载夹爪自己的 USD，自动识别里面的
旋转(PhysicsRevoluteJoint)/直线(PhysicsPrismaticJoint)关节并驱动，
不读也不改机械臂的任何文件。

运行方式（推荐用仓库里的 run_gripper.sh，会自动设置环境变量；手动运行如下）：
  cd $ISAAC_SIM_PATH            # 你的 Isaac Sim 安装目录
  bash -c 'source ./setup_python_env.sh && source ./setup_ros_env.sh && \
    export LD_LIBRARY_PATH="$PWD/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH" && \
    export LD_PRELOAD=$PWD/kit/libcarb.so && \
    export CARB_APP_PATH=$PWD/kit ISAAC_PATH=$PWD EXP_PATH=$PWD/apps && \
    ./kit/python/bin/python3 /path/to/litegrip-isaacsim/isaac_sim_gripper.py'

轨迹约定：/gripper/joint_traj（trajectory_msgs/JointTrajectory）
  positions 顺序 = 关节名排序（默认加载的 gripper2.usd 为
    [gripper_slide_joint_left, gripper_slide_joint_right]）；
  - revolute 关节：弧度（内部转成度写入 USD）
  - prismatic 关节：米（原样写入）

当前加载模型：gripper2/gripper2.usd（q = 0 张开，q = +0.067 闭合，单指行程）。
"""
import threading
import sys
import os
import math
import time as _time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(SCRIPT_DIR, "gripper_sim_node_log.txt")
USD_PATH = os.path.join(SCRIPT_DIR, "gripper2", "gripper2.usd")


def log(msg):
    line = "[gripper] " + msg
    print(line, flush=True)
    try:
        with open(LOG, "a") as f:
            f.write(line + "\n")
    except Exception:
        pass


log("脚本开始执行")

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
    "anti_aliasing": "FXAA",
    "/renderer/multiGpu/enabled": False,
})

import omni
import omni.timeline
from pxr import UsdPhysics, Usd

log("SimulationApp 创建完成")

# ── 1. 注入 internal rclpy 路径 ──
# Isaac Sim 根目录由 run_gripper.sh 通过环境变量 ISAAC_PATH 提供
_ISAAC_PATH = os.environ.get("ISAAC_PATH", "")
BRIDGE_RCLPY = os.path.join(_ISAAC_PATH, "exts/isaacsim.ros2.bridge/humble/rclpy") if _ISAAC_PATH else ""
if os.path.isdir(BRIDGE_RCLPY):
    sys.path.insert(0, BRIDGE_RCLPY)
    log("已注入 internal rclpy 路径")

try:
    from isaacsim.core.utils import extensions
    extensions.enable_extension("isaacsim.ros2.bridge")
    log("已启用 isaacsim.ros2.bridge 扩展")
except Exception as e:
    log(f"启用 bridge 扩展失败: {type(e).__name__}: {e}")

log("空 stage 预热，等待 bridge 扩展启动完成")
for _ in range(30):
    simulation_app.update()
log("bridge 扩展预热完成")

RCLPY_OK = False
try:
    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory
    log("rclpy import OK")
    RCLPY_OK = True
except Exception as e:
    log(f"rclpy import 失败: {type(e).__name__}: {e}")


def open_usd():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    ok = ctx.open_stage(USD_PATH)
    stage = ctx.get_stage()
    for prim in stage.TraverseAll():
        if prim.HasPayload():
            prim.Load()
    log(f"打开 USD ok={ok}: {USD_PATH}")
    return stage


def setup_joints(stage):
    """找到夹爪所有可动关节，返回 [(prim, kind)]，kind = angular|linear。"""
    joints = []
    for prim in stage.TraverseAll():
        t = prim.GetTypeName()
        if t == "PhysicsRevoluteJoint":
            joints.append((prim, "angular"))
        elif t == "PhysicsPrismaticJoint":
            joints.append((prim, "linear"))
    joints.sort(key=lambda p: p[0].GetName())
    log(f"找到 {len(joints)} 个关节: {[(p.GetName(), k) for p, k in joints]}")
    return joints


class GripperSimNode(Node):
    def __init__(self, topic, joints):
        super().__init__("gripper_sim")
        self.joints = joints
        self.lock = threading.Lock()
        self.traj_q = []
        self.traj_t = []
        self.traj_start = None
        self.sub = self.create_subscription(JointTrajectory, topic, self._cb, 10)
        log(f"订阅 {topic}，关节数 {len(joints)}")

    def _cb(self, msg: JointTrajectory):
        n = len(self.joints)
        if len(msg.points) < 2:
            log("收到轨迹点数不足 2，忽略")
            return
        traj_q, traj_t = [], []
        for p in msg.points:
            if len(p.positions) < n:
                continue
            traj_q.append([float(x) for x in p.positions[:n]])
            traj_t.append(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9)
        with self.lock:
            self.traj_q = traj_q
            self.traj_t = traj_t
            self.traj_start = _time.time()
        log(f"收到轨迹 {len(traj_q)} 点，时长 {traj_t[-1]:.2f}s")

    def drive_pending(self):
        with self.lock:
            traj_q = self.traj_q
            traj_t = self.traj_t
            traj_start = self.traj_start
        if not traj_q or traj_start is None:
            return
        elapsed = _time.time() - traj_start
        q = self._sample(traj_q, traj_t, elapsed)
        if q is None:
            return
        self._write(q)

    def _sample(self, traj_q, traj_t, t):
        if t <= traj_t[0]:
            return traj_q[0]
        if t >= traj_t[-1]:
            return traj_q[-1]
        for i in range(len(traj_t) - 1):
            if traj_t[i] <= t <= traj_t[i + 1]:
                span = traj_t[i + 1] - traj_t[i]
                f = 0.0 if span == 0 else (t - traj_t[i]) / span
                return [a + (b - a) * f for a, b in zip(traj_q[i], traj_q[i + 1])]
        return traj_q[-1]

    def _write(self, q):
        for (prim, kind), val in zip(self.joints, q):
            drive = UsdPhysics.DriveAPI.Apply(prim, kind)
            drive.GetStiffnessAttr().Set(1000000.0)
            drive.GetDampingAttr().Set(10000.0)
            if kind == "angular":
                # 旋转关节：弧度 → 度
                drive.GetTargetPositionAttr().Set(float(math.degrees(val)))
            else:
                # 直线关节：单位是米，原样写入
                drive.GetTargetPositionAttr().Set(float(val))


stage = open_usd()
joints = setup_joints(stage)

log("所有 prim 就绪，准备进入订阅与主循环")

for _ in range(60):
    simulation_app.update()
log("预热完成")

node = None
spin_thread = None
if RCLPY_OK and joints:
    topic = "/gripper/joint_traj"
    rclpy.init()
    log("rclpy.init() 完成")
    node = GripperSimNode(topic, joints)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    log("spin 线程已启动")
else:
    log(f"订阅未启动: RCLPY_OK={RCLPY_OK} joints={len(joints)}")

tl = omni.timeline.get_timeline_interface()
if not tl.is_playing():
    tl.play()
    log("已启动 Play 状态")

log("进入主循环（按 Ctrl+C 退出）")
try:
    while simulation_app.is_running():
        simulation_app.update()
        if node is not None:
            node.drive_pending()
except KeyboardInterrupt:
    pass

log("脚本退出")
if node is not None:
    rclpy.shutdown()
simulation_app.close()

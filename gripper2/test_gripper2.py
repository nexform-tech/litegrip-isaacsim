#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪 URDF2 开合测试：headless 加载 gripper2.usd，驱动关节做闭合/打开，验证。

不依赖 ROS，直接对两个 prismatic 关节设 drive target，观察关节位置变化。
关节约定：q = 0 张开，q = +0.067 闭合（单指行程）。

用法：
  cd $ISAAC_SIM_PATH
  ./kit/python/bin/python3 /path/to/litegrip-isaacsim/gripper2/test_gripper2.py \
    --/renderer/multiGpu/enabled=false
"""
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True,
    "width": 1280,
    "height": 720,
})

import omni
from pxr import Usd, UsdPhysics, UsdGeom, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(SCRIPT_DIR, "gripper2.usd")

CLOSED = 0.067   # 闭合（单指行程，米）
OPEN = 0.0       # 打开


def log(msg):
    print("[test2] " + msg, flush=True)


def find_joints(stage):
    joints = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() == "PhysicsPrismaticJoint":
            joints.append(prim)
    joints.sort(key=lambda p: p.GetName())
    return joints


def apply_drive(prim, target):
    drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
    drive.GetStiffnessAttr().Set(1000000.0)
    drive.GetDampingAttr().Set(10000.0)
    drive.GetTargetPositionAttr().Set(float(target))


def read_joint_positions(joints):
    """读取每个 prismatic 关节的当前线性位置（米）。

    优先读 physics:state（PhysX 维护的 [position, velocity]），
    fallback 读 child link 相对 parent 沿 axis 的位移。
    """
    out = []
    for prim in joints:
        state_attr = prim.GetAttribute("physics:state")
        state = state_attr.Get() if state_attr.IsValid() else None
        if state is not None and len(state) >= 1:
            out.append(float(state[0]))
            continue
        out.append(None)
    return out


def read_child_x(joints, stage):
    """读每个关节 child link 的 world x 坐标（辅助验证开合方向）。"""
    out = []
    for prim in joints:
        rel = prim.GetRelationship("physics:body1")
        targets = rel.GetTargets() if rel else []
        x = None
        if targets:
            child = stage.GetPrimAtPath(targets[0])
            if child:
                xform = UsdGeom.Xformable(child)
                # world transform
                m = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                x = m.ExtractTranslation()[0]
        out.append(x)
    return out


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(USD_PATH)
    stage = ctx.get_stage()
    log(f"打开 USD: {USD_PATH}")

    joints = find_joints(stage)
    names = [j.GetName() for j in joints]
    log(f"找到 {len(joints)} 个 prismatic 关节: {names}")

    if not joints:
        log("未找到关节，退出")
        simulation_app.close()
        return 1

    # 启动 Play，物理才会驱动关节
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_playing():
        tl.play()
        log("已启动 Play 状态")

    # 先让物理场景跑起来
    for _ in range(30):
        simulation_app.update()

    def settle():
        for _ in range(90):
            simulation_app.update()

    # ── 闭合 ──
    log(f"== 闭合：target = {CLOSED} ==")
    for j in joints:
        apply_drive(j, CLOSED)
    settle()
    log(f"关节位置(state): {read_joint_positions(joints)}")
    log(f"child world x  : {read_child_x(joints, stage)}")

    # ── 打开 ──
    log(f"== 打开：target = {OPEN} ==")
    for j in joints:
        apply_drive(j, OPEN)
    settle()
    log(f"关节位置(state): {read_joint_positions(joints)}")
    log(f"child world x  : {read_child_x(joints, stage)}")

    log("测试完成")
    simulation_app.close()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())

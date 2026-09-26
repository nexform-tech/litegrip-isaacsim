#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪 URDF2 → USD 导入脚本（SolidWorks 导出版夹爪，独立可跑）。

用法（URDF/USD 路径已按脚本目录自动定位，无需手改）：
  cd $ISAAC_SIM_PATH
  ./kit/python/bin/python3 /path/to/litegrip-isaacsim/gripper2/import_gripper2.py \
    --/renderer/multiGpu/enabled=false

产出：<本目录>/gripper2.usd（已拍平、自包含、带 ArticulationRoot）
"""
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True,
    "width": 1280,
    "height": 720,
})

import omni
from pxr import Usd, UsdGeom, UsdLux, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "gripper2.urdf")
USD_OUT = os.path.join(SCRIPT_DIR, "gripper2.usd")


def log(msg):
    print("[import2] " + msg, flush=True)


def main():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    log("new_stage 完成")

    from omni.kit.commands import execute

    status, import_config = execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.create_physics_scene = True

    ok, err = execute(
        "URDFParseAndImportFile",
        urdf_path=URDF_PATH,
        import_config=import_config,
    )
    log(f"URDFParseAndImportFile ok={ok} err={err}")

    stage = ctx.get_stage()
    if stage is None:
        log("stage 为 None，导入失败")
        return 1

    # ── 灯光 ──
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(800.0)
    distant = UsdLux.DistantLight.Define(stage, "/World/Lights/DistantLight")
    distant.CreateIntensityAttr(3000.0)
    distant.AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 45.0))
    log("灯光已加")

    # ── 相机 ──
    default_prim = stage.GetDefaultPrim()
    target_path = default_prim.GetPath() if default_prim else "/World"
    try:
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(
            stage.GetPrimAtPath(target_path))
        rng = bbox.ComputeAlignedRange()
        center = Gf.Vec3d(rng.GetMidpoint())
        size = Gf.Vec3d(rng.GetSize())
        max_dim = max(size[0], size[1], size[2], 0.1)
        dist = max_dim * 3.0
        eye = center + Gf.Vec3d(dist * 0.8, -dist * 0.9, dist * 0.7)
        cam = UsdGeom.Camera.Define(stage, "/World/Camera")
        cam.CreateFocalLengthAttr(24.0)
        cam_transform = Gf.Matrix4d().SetLookAt(eye, center, Gf.Vec3d(0, 0, 1))
        q = cam_transform.ExtractRotationQuat()
        cam.AddTranslateOp().Set(eye)
        cam.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
        log(f"相机已加 eye={eye}")
    except Exception as e:
        log(f"相机计算失败（忽略）: {type(e).__name__}: {e}")

    # ── 稳定帧 ──
    log("进入稳定主循环（30 帧）")
    for _ in range(30):
        simulation_app.update()

    # ── 拍平 reference（消除 Play 时 Fabric 重载 → reopenUsd 段错误）──
    from pxr import UsdUtils
    log("开始拍平 reference...")
    UsdUtils.FlattenLayerStack(stage)
    log("拍平完成")

    stage.GetRootLayer().Export(USD_OUT)
    log(f"导出完成 -> {USD_OUT}")

    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪真机桥（litegrip 直连版，**不经 litearm-server**）。

订阅 ROS2 /gripper/joint_traj，把仿真的夹爪关节位置换算成真机开口**毫米数**，
用 litegrip 直接开 SocketCAN 驱动达妙 DM4310（CAN 0x08）。

为什么不走 litearm-server：
  - 经 server 的路线要用 litearm-python 的 RemoteDevice（RPC → litearm-server
    tcp:7447），起不了 server 时就用不了；
  - 本桥直接 `from litegrip import LiteGrip`。litegrip 是**自包含纯 CAN 库**
    （只用 stdlib + Linux SocketCAN，无 zenoh / 无 server 依赖），
    所以只要 can0 是 up 的就能跑，不需要 litearm-python、不需要 7447。

═══ 量纲与映射（★ 下面常量是占位，实测后再钉死 ★）═══
  仿真（gripper2.urdf）：2 个对称 prismatic 关节，q 单位米，
      q = 0 张开、q = +Q_FULL_M 闭合，单指行程 Q_FULL_M。
  真机（litegrip）：开口**毫米**，0 = 全合、travel_mm = 全开（标定口径 ≈ 120）。

  映射（本文件的 joints_to_mm）：
      mm = CLOSED_MM + (OPEN_MM - CLOSED_MM) * (1 - q_avg / Q_FULL_M)
      q_avg = 0        → mm = OPEN_MM   （全开）
      q_avg = Q_FULL_M → mm = CLOSED_MM （全合）

  ★ 两个候选口径，实测二选一，**别混用**：
      a) 标定口径  OPEN_MM ≈ 120.06  （= travel_range_rad 1.605 × rad_to_mm 74.8）
      b) 物理口径  OPEN_MM ≈ 134     （= 2 × 67，两指合计；CAD 每指 0.067 m）
    两者差约 10%。上机时「全开量一次间隙、全合量一次」把真实值填进来。

  安全性：litegrip 内部会把目标 clamp 到标定行程 [pos_open_rad, pos_closed_rad]，
    所以口径填偏了只会「少开一点 / 提前停」，**不会超程顶坏机构**。

⚠ 本桥只取轨迹「最后一个点」，**不按时间轴逐帧插值**。
   真机侧用 --speed-mm-s 限速平滑逼近，所以不会跳变；要严格同速需逐帧采样
  （可仿 isaac_sim_gripper.py 的 drive_pending）。

用法（需 ROS2 Humble + can0 已 up；**不需要** litearm-python / litearm-server）：
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
  python3 gripper_litegrip_bridge.py --can can0
  python3 gripper_litegrip_bridge.py --dry-run          # 只打印 mm，不碰硬件
"""
import argparse
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

DEFAULT_TOPIC = "/gripper/joint_traj"

# litegrip 包所在目录（/opt/litearm 下就是 litegrip/，装 litearm-server 时一起带的）
DEFAULT_LITEARM_LIB_DIR = "/opt/litearm"

# ── ★ 占位常量：上机实测后改这三行 ★ ────────────────────────────────────────
Q_FULL_M = 0.067     # 仿真单指满行程（米），与 gripper2.urdf 的 limit upper 一致（CAD）
OPEN_MM = None       # 真机全开时开口毫米数 ← 实测填。None = 用下面标定口径占位
CLOSED_MM = 0.0      # 真机全合时开口毫米数（litegrip 0=全合，一般不用改）
# ──────────────────────────────────────────────────────────────────────────

# OPEN_MM 未实测时的占位口径（标定推算，见文件头 a/b 两说）
OPEN_MM_CALIB = 120.06

# 每帧耗时上限保护：一次移动最多走多久（秒），避免极端参数下长时间占住线程
MAX_MOVE_S = 10.0


def joints_to_mm(q):
    """夹爪关节位置（米，可多指）→ 真机开口毫米数。

    q=0 → 全开(OPEN_MM)；q=Q_FULL_M → 全合(CLOSED_MM)。两指对称，取平均即可
    （gripper2.urdf 两个关节 limit 都是 [0, 0.067]，符号方向不影响幅值）。
    """
    if not q:
        return None
    q_avg = sum(float(x) for x in q) / len(q)
    frac_open = 1.0 - q_avg / Q_FULL_M
    frac_open = max(0.0, min(1.0, frac_open))          # 夹住超程输入
    remap = OPEN_MM if OPEN_MM is not None else OPEN_MM_CALIB
    mm = CLOSED_MM + (remap - CLOSED_MM) * frac_open
    return max(CLOSED_MM, min(remap, mm))               # 再夹到 [合, 开]


class GripperLiteGrip:
    """litegrip 后台线程封装：目标变化时才发起一次阻塞式移动。

    litegrip 的 move_at_speed / goto_rad 是**阻塞**调用，内部按 duration 持续
    下发 MIT 帧（DM 电机约 100 ms 收不到指令就失能，所以必须由它自己流式发完）。
    因此这里用一个 worker 线程承载，ROS 回调只更新目标、不阻塞。
    """

    def __init__(self, can="can0", can_id=0x08, speed_mm_s=30.0, dry_run=False):
        self._speed = float(speed_mm_s)
        self._dry = bool(dry_run)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._target = None      # 目标开口 mm；None → 不动
        self._last_cmd = None    # 上次已下发的目标（去重）
        self._g = None

        if self._dry:
            print(f"[夹爪][dry-run] 不连硬件，速度 {self._speed:.1f} mm/s", flush=True)
        else:
            from litegrip import LiteGrip  # noqa: E402  （延迟到用时再 import）
            self._g = LiteGrip(channel=can, can_id=can_id)
            self._g.connect()
            self._g.load_calibration()   # ★ 必须：config 默认值与注释自相矛盾
            self._g.enable()
            cfg = self._g.config
            print(f"[夹爪] litegrip 已使能 {can} id=0x{can_id:02X}"
                  f" 标定行程=({cfg.pos_open_rad:.3f}, {cfg.pos_closed_rad:.3f}) rad"
                  f" rad_to_mm={cfg.rad_to_mm:.1f}", flush=True)

        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    def _loop(self):
        while not self._stop.is_set():
            with self._lock:
                target = self._target
            if target is None or target == self._last_cmd:
                self._stop.wait(0.02)
                continue
            self._last_cmd = target
            try:
                self._move(target)
            except Exception as e:  # noqa: BLE001
                print(f"[夹爪] 移动失败({target:.1f} mm): {e}", flush=True)
                self._last_cmd = None   # 允许下一帧重试
                self._stop.wait(0.5)

    def _move(self, mm):
        if self._dry:
            print(f"[夹爪][dry-run] 目标 {mm:.1f} mm", flush=True)
            return
        cur = self._g.get_position()                       # mm
        dist = abs(mm - cur)
        if dist < 0.05:
            return
        dur = min(dist / max(1e-6, self._speed), MAX_MOVE_S)
        print(f"[夹爪] {cur:.1f} → {mm:.1f} mm（{self._speed:.1f} mm/s，约 {dur:.2f}s）",
              flush=True)
        self._g.move_at_speed(mm, speed_mm_s=self._speed)

    def set_target(self, mm):
        """更新目标开口（非阻塞，由 worker 线程执行）。"""
        with self._lock:
            self._target = float(mm)

    def shutdown(self):
        self._stop.set()
        if self._g is not None:
            try:
                self._g.disable()      # 失能，避免长期hold力矩
            except Exception:  # noqa: BLE001
                pass
            try:
                self._g.disconnect()
            except Exception:  # noqa: BLE001
                pass


class GripperBridge(Node):
    """订阅 /gripper/joint_traj，换算成 mm 后交给 litegrip 驱动。"""

    def __init__(self, topic, driver):
        super().__init__("gripper_litegrip_bridge")
        self._drv = driver
        self.sub = self.create_subscription(
            JointTrajectory, topic, self._cb, 10)
        self.get_logger().info(f"订阅 {topic}（litegrip 直连，无 litearm-server）")

    def _cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        q = [float(x) for x in msg.points[-1].positions]
        mm = joints_to_mm(q)
        if mm is None:
            return
        self._drv.set_target(mm)
        self.get_logger().info(
            "q=[%s] → %.1f mm" % (", ".join(f"{x:.4f}" for x in q), mm))


def main():
    p = argparse.ArgumentParser(
        description="夹爪真机桥（litegrip 直连，不经 litearm-server）")
    p.add_argument("--can", default="can0", help="CAN 接口（默认 can0）")
    p.add_argument("--can-id", type=lambda s: int(s, 0), default=0x08,
                   help="夹爪电机 CAN ID（默认 0x08）")
    p.add_argument("--topic", default=DEFAULT_TOPIC,
                   help=f"轨迹 topic（默认 {DEFAULT_TOPIC}）")
    p.add_argument("--speed-mm-s", type=float, default=30.0,
                   help="真机开口速度 mm/s（默认 30；越小越慢越安全）")
    p.add_argument("--lib-dir", default=DEFAULT_LITEARM_LIB_DIR,
                   help=f"litegrip 包所在目录（默认 {DEFAULT_LITEARM_LIB_DIR}）")
    p.add_argument("--dry-run", action="store_true",
                   help="只打印换算出的 mm，不连 CAN、不动硬件")
    args = p.parse_args()

    if not args.dry_run:
        if not os.path.isdir(os.path.join(args.lib_dir, "litegrip")):
            print(f"[错误] 在 {args.lib_dir} 下找不到 litegrip/。"
                  f"用 --lib-dir 指定，或确认已装 litearm-server 运行时。",
                  flush=True)
            return 1
        sys.path.insert(0, args.lib_dir)

    if OPEN_MM is None:
        print(f"[警告] OPEN_MM 尚未实测校准，暂用标定口径 {OPEN_MM_CALIB} mm 占位"
              f"（实测全开间隙后改文件头的 OPEN_MM）。", flush=True)

    try:
        driver = GripperLiteGrip(can=args.can, can_id=args.can_id,
                                 speed_mm_s=args.speed_mm_s, dry_run=args.dry_run)
    except Exception as e:  # noqa: BLE001
        print(f"[错误] 连接夹爪失败: {type(e).__name__}: {e}\n"
              f"      先确认: sudo ip link set {args.can} type can bitrate 1000000"
              f" && sudo ip link set {args.can} up", flush=True)
        return 1

    rclpy.init()
    node = GripperBridge(args.topic, driver)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        driver.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

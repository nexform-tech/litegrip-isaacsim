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

═══ 量纲与映射 ═══
  仿真（gripper2.urdf）：2 个对称 prismatic 关节，q 单位米，
      q = 0 张开、q = +Q_FULL_M 闭合，单指行程 Q_FULL_M。
  真机（litegrip）：开口**毫米**，0 = 全合、travel_mm = 全开（标定口径 ≈ 120）。

  映射（本文件的 joints_to_mm）：
      mm = CLOSED_MM + (OPEN_MM - CLOSED_MM) * (1 - q_avg / Q_FULL_M)
      q_avg = 0        → mm = OPEN_MM   （全开）
      q_avg = Q_FULL_M → mm = CLOSED_MM （全合）

  ★ OPEN_MM 已定为 110（2026-09-27 上机设定，口径 = litegrip 标定口径的 mm）。
    改真机全开量就改 OPEN_MM 这一个常量。原先的两个推算候选（差约 10%，仅供参考）：
      a) 标定推算 ≈ 120.06 （= travel_range_rad 1.605 × rad_to_mm 74.8）
      b) 物理/CAD  ≈ 134   （= 2 × 0.067，两指合计）
    110 不是上面任一推算值，不要拿它俩去反推。

  安全性：litegrip 内部会把目标 clamp 到标定行程 [pos_open_rad, pos_closed_rad]，
    所以口径填偏了只会「少开一点 / 提前停」，**不会超程顶坏机构**。

⚠ 只取轨迹「最后一个点」，不按时间轴逐帧插值。真机侧由常驻帧流按 --speed-mm-s
   限速平滑逼近（见 GripperLiteGrip 的 docstring），所以不会跳变；要严格同速需
   逐帧采样（可仿 isaac_sim_gripper.py 的 drive_pending）。

═══ 为什么是「常驻 MIT 帧流」而不是 move_at_speed ═══
  DM4310 使能后 ~100 ms 收不到指令就自锁「通讯丢失」故障（err=13），电机随即不
  响应后续指令。litegrip 的 move_at_speed/goto 是**阻塞定长斜坡**：流完 duration
  就返回、之后不再发帧 → 桥在两次动作之间一空闲，电机就自锁；表现为「轨迹收到
  了、真机却不动、读回 err=13」。所以本类不用 move_at_speed，改成 200 Hz 常驻
  send_mit_frame：目标变化时限速逼近，无目标时 kp=0 零力矩守住（帧不停）。
  帧流同时也是过力保护的载体（每周期都能读到新 tau）。

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
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

DEFAULT_TOPIC = "/gripper/joint_traj"

# litegrip 包所在目录（/opt/litearm 下就是 litegrip/，装 litearm-server 时一起带的）
DEFAULT_LITEARM_LIB_DIR = "/opt/litearm"

# ── ★ 占位常量：上机实测后改这三行 ★ ────────────────────────────────────────
Q_FULL_M = 0.067     # 仿真单指满行程（米），与 gripper2.urdf 的 limit upper 一致（CAD）
OPEN_MM = 110.0      # 真机全开时开口毫米数（2026-09-27 设定；litegrip 标定口径）
CLOSED_MM = 0.0      # 真机全合时开口毫米数（litegrip 0=全合，一般不用改）
# ──────────────────────────────────────────────────────────────────────────

# OPEN_MM 未实测时的占位口径（标定推算，见文件头 a/b 两说）
OPEN_MM_CALIB = 120.06

# ── 常驻控制环参数（可用命令行覆盖）──────────────────────────────────────
KP = 5.0            # 位置刚度（同 gripper_open_guard.py，小=柔）
KD = 0.5            # 速度阻尼
HZ = 200.0          # 帧流频率；★ 必须持续发：DM ~100ms 无帧即锁 err=13
TAU_ABSMAX = 3.0    # 过力保护 Nm（0 = 关）：|tau| 超阈即冻住指令不再前推
RECOVER_MIN_S = 0.5  # 故障恢复最小间隔秒数（否则 200Hz 会猛刷 enable()）


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


def mm_to_rad(cfg, mm):
    """真机开口毫米数 → 电机目标弧度（0mm=全合，张开方向 rad 减小）。"""
    return cfg.pos_closed_rad - mm / cfg.rad_to_mm


def rad_to_mm(cfg, rad):
    return (cfg.pos_closed_rad - rad) * cfg.rad_to_mm


def advance_toward(q_cmd, q_target, max_step):
    """限速逼近一步：返回 (新指令位置, 本周期 rad 位移)。

    位移绝对值不超过 max_step（限速），且到位时**精确落在** q_target 不越过
    （否则会在目标两侧来回抖）。到位后返回位移 0，可直接当速度前馈用。
    """
    step = q_target - q_cmd
    if abs(step) <= max_step:
        return q_target, 0.0
    dq = max_step if step > 0 else -max_step
    return q_cmd + dq, dq


class GripperLiteGrip:
    """litegrip 常驻 MIT 帧流封装（目标变化时限速逼近，空闲时零力矩守住）。

    为什么不用 litegrip 的 move_at_speed/goto：它们是**阻塞定长斜坡**，流完
    duration 就返回、之后不再发任何帧。DM4310 使能后 ~100 ms 收不到指令就自锁
    「通讯丢失」故障（err=13），电机随即不响应后续指令 —— 于是桥两次动作之间一
    空闲就废掉，表现为「轨迹收到了、真机不动、读回 err=13」。
    （见 litegrip/gripper.py 的 _move_at_speed_rad：循环 steps 次后 return。）

    本类改为常驻 worker 线程按 hz **不间断**下发 send_mit_frame：
      - 有目标：按 speed_mm_s 限速把指令位置推向目标（每周期最多走
        speed/rad_to_mm*dt），到位后原地守住；
      - 无目标：kp=kd=0 零力矩守住（电机使能但可自由掰动），帧照发不停。
    这样总线上永远有帧，不会自锁通讯丢失。帧流也是过力保护的载体：每周期
    poll 一帧真反馈，|tau| 超阈就把指令压回当前实际位置（卸力）并清目标。
    """

    def __init__(self, can="can0", can_id=0x08, speed_mm_s=30.0,
                 kp=KP, kd=KD, hz=HZ, tau_absmax=TAU_ABSMAX, dry_run=False):
        self._speed = float(speed_mm_s)
        self._kp = float(kp)
        self._kd = float(kd)
        self._hz = float(hz)
        self._tau_absmax = float(tau_absmax)
        self._dry = bool(dry_run)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._target = None      # 目标开口 mm；None → 零力矩守住（不往前推）
        self._last_cmd = None    # 上次已打印的目标（dry-run/日志去重）
        self._queued = None      # 已下发给控制环的目标（仅日志用）
        self._recover_at = -1e9  # 上次故障恢复尝试时刻（限频用）
        self._g = None

        if self._dry:
            print(f"[夹爪][dry-run] 不连硬件，限速 {self._speed:.1f} mm/s",
                  flush=True)
        else:
            from litegrip import LiteGrip  # noqa: E402  （延迟到用时再 import）
            self._g = LiteGrip(channel=can, can_id=can_id)
            self._g.connect()
            self._g.load_calibration()   # ★ 必须：config 默认值与注释自相矛盾
            self._g.enable()             # 内部会先 clear_fault（若已锁故障）
            cfg = self._g.config
            print(f"[夹爪] litegrip 已使能 {can} id=0x{can_id:02X}"
                  f" 标定行程=({cfg.pos_open_rad:.3f}, {cfg.pos_closed_rad:.3f}) rad"
                  f" rad_to_mm={cfg.rad_to_mm:.1f}", flush=True)

        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    # ── 控制环 ──────────────────────────────────────────────────────────
    def _loop(self):
        if self._dry:
            self._dry_loop()
            return

        cfg = self._g.config
        dt = 1.0 / self._hz
        max_step = self._speed / cfg.rad_to_mm * dt   # 每周期最大 rad 增量
        # 起步先拿一帧真反馈当指令起点（否则从缓存 0 起步会突然甩到目标）
        q_cmd = self._g.get_position_rad()
        print(f"[夹爪] 常驻帧流启动 {self._hz:.0f} Hz  kp={self._kp} kd={self._kd}"
              f"  起点 {rad_to_mm(cfg, q_cmd):.1f} mm"
              + (f"  过力阈值 {self._tau_absmax} Nm" if self._tau_absmax > 0
                 else "  过力保护已关"), flush=True)
        next_log = 0.0

        while not self._stop.is_set():
            with self._lock:
                target = self._target

            if target is None:
                # 零力矩守住：帧不能停，否则电机 ~100ms 自锁 err=13
                self._g.send_mit_frame(q_cmd, 0.0, 0.0, dq=0.0, tau=0.0)
                self._g.poll(timeout_s=0.0)
                self._sleep(dt)
                continue

            # 限速逼近目标：每周期最多走 max_step，到位后在目标原地守住
            q_target = mm_to_rad(cfg, target)
            q_cmd, dq = advance_toward(q_cmd, q_target, max_step)

            self._g.send_mit_frame(q_cmd, self._kp, self._kd,
                                   dq=dq / dt, tau=0.0)
            self._g.poll(timeout_s=0.0)
            st = self._g.get_state(wait=False)
            now = time.monotonic()

            # ① 电机故障（如 err=13 通讯丢失）：限频清故障 + 重新使能，从实时位置续上
            if st.error_code not in (0, 1):
                if now - self._recover_at >= RECOVER_MIN_S:
                    self._recover_at = now
                    print(f"[夹爪] ⚠ 电机错误码 {st.error_code}，"
                          f"尝试清故障 + 重新使能…", flush=True)
                    try:
                        self._g.enable()              # enable() 内部先 clear_fault
                        q_cmd = self._g.get_position_rad()
                    except Exception as e:  # noqa: BLE001
                        print(f"[夹爪] 恢复失败: {type(e).__name__}: {e}",
                              flush=True)
                # 故障期帧照发（零力矩），别让总线静默再把通讯丢失续上
                self._g.send_mit_frame(q_cmd, 0.0, 0.0, dq=0.0, tau=0.0)
                self._sleep(dt)
                continue

            # ② 过力：把指令压回当前实际位置（误差归零→卸力）并清目标，不再前推
            if self._tau_absmax > 0 and abs(st.torque_nm) >= self._tau_absmax:
                print(f"[夹爪] ⚠ 过力 |tau|={abs(st.torque_nm):.2f} Nm ≥ "
                      f"{self._tau_absmax:.2f}，冻在 "
                      f"{rad_to_mm(cfg, st.position_rad):.1f} mm（不再前推）",
                      flush=True)
                q_cmd = st.position_rad
                with self._lock:
                    self._target = None
                continue

            if now >= next_log:
                next_log = now + 0.5
                moving = abs(dq) > 0.0 or abs(q_target - st.position_rad) > 0.05
                if moving or target != self._queued:
                    self._queued = target
                    print(f"[夹爪] 指令={rad_to_mm(cfg, q_cmd):6.1f} mm  "
                          f"实际={rad_to_mm(cfg, st.position_rad):6.1f} mm  "
                          f"tau={st.torque_nm:+.3f} Nm", flush=True)

            self._sleep(dt)

    def _dry_loop(self):
        while not self._stop.is_set():
            with self._lock:
                t = self._target
            if t is not None and t != self._last_cmd:
                self._last_cmd = t
                print(f"[夹爪][dry-run] 目标 {t:.1f} mm", flush=True)
            self._stop.wait(0.02)

    def _sleep(self, dt):
        self._stop.wait(dt)      # 用 Event.wait 睡：shutdown 时能立刻退出

    def set_target(self, mm):
        """更新目标开口（非阻塞，由常驻帧流线程逼近）。"""
        with self._lock:
            self._target = float(mm)

    def shutdown(self):
        self._stop.set()
        self._worker.join(timeout=1.0)
        if self._g is not None:
            try:
                self._g.disable()      # 失能，避免长期 hold 力矩
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
    p.add_argument("--kp", type=float, default=KP,
                   help="位置刚度（默认 5，与 gripper_open_guard.py 同口径）")
    p.add_argument("--kd", type=float, default=KD, help="速度阻尼（默认 0.5）")
    p.add_argument("--hz", type=float, default=HZ,
                   help="常驻帧流频率（默认 200；★ 不能停，DM ~100ms 无帧锁 err=13）")
    p.add_argument("--tau-absmax", type=float, default=TAU_ABSMAX,
                   help="过力保护 Nm（默认 3.0；0 = 关）")
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
                                 speed_mm_s=args.speed_mm_s,
                                 kp=args.kp, kd=args.kd, hz=args.hz,
                                 tau_absmax=args.tau_absmax,
                                 dry_run=args.dry_run)
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

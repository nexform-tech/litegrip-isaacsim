#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gripper_litegrip_bridge 的纯逻辑单测。

跑这个测试**不需要** ROS2 / Isaac Sim / CAN 硬件：
桥脚本顶部会 `import rclpy`、`from trajectory_msgs.msg import ...`，
控制环里还会 `from litegrip import LiteGrip`，CI 容器里都没有，
所以先在 sys.modules 里塞轻量替身再 import 被测模块。

真机控制环（常驻 MIT 帧流）用替身 LiteGrip 驱动：替身只把每帧记进
`frames`，并暴露可改的 `position_rad/torque_nm/error_code` 当「电机反馈」。

运行：
  python3 -m unittest discover -s tests -v
"""
import contextlib
import io
import os
import sys
import time
import types
import unittest
from unittest import mock

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def _install_ros_stubs():
    """塞入最小 ROS2 替身，让桥脚本在无 ROS 环境下也能被 import。"""
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None

    node_mod = types.ModuleType("rclpy.node")

    class Node:  # 只需能被继承
        def __init__(self, *a, **k):
            pass

    node_mod.Node = Node
    rclpy.node = node_mod

    traj_pkg = types.ModuleType("trajectory_msgs")
    traj_msg = types.ModuleType("trajectory_msgs.msg")

    class JointTrajectory:  # 只需能被 import
        pass

    traj_msg.JointTrajectory = JointTrajectory
    traj_pkg.msg = traj_msg

    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": node_mod,
        "trajectory_msgs": traj_pkg,
        "trajectory_msgs.msg": traj_msg,
    })


class FakeLiteGrip:
    """`litegrip.LiteGrip` 的录制替身：不发真帧，只把帧记进 `.frames`。

    `position_rad` / `torque_nm` / `error_code` 是类属性，代表「电机当前反馈」，
    测试可直接改它们来模拟真机状态。标定口径取自 litegrip 出厂文件。
    """

    instances = []              # 最近构造的实例在末尾
    position_rad = 0.1140       # 标定 0 mm（全合）
    torque_nm = 0.0
    error_code = 0

    def __init__(self, channel=None, can_id=None):
        self.config = types.SimpleNamespace(
            pos_closed_rad=0.1140, pos_open_rad=-1.4910, rad_to_mm=74.80)
        self.frames = []        # (q, kp, kd, dq, tau)
        self.enable_calls = 0
        self.disabled = False
        self.disconnected = False
        FakeLiteGrip.instances.append(self)

    def connect(self):
        return True

    def load_calibration(self):
        return True

    def enable(self):
        self.enable_calls += 1
        return True

    def disable(self):
        self.disabled = True
        return True

    def disconnect(self):
        self.disconnected = True
        return True

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0):
        self.frames.append((q, kp, kd, dq, tau))
        return True

    def poll(self, timeout_s=0.0):
        return True

    def get_position_rad(self):
        return FakeLiteGrip.position_rad

    def get_state(self, wait=True):
        return types.SimpleNamespace(
            position_rad=FakeLiteGrip.position_rad,
            torque_nm=FakeLiteGrip.torque_nm,
            error_code=FakeLiteGrip.error_code)


def _install_litegrip_stub():
    """塞入 litegrip 替身，让控制环在无 SDK / 无 CAN 环境下也能跑。"""
    if "litegrip" in sys.modules:
        return
    mod = types.ModuleType("litegrip")
    mod.LiteGrip = FakeLiteGrip
    sys.modules["litegrip"] = mod


_install_ros_stubs()
_install_litegrip_stub()

import gripper_litegrip_bridge as bridge  # noqa: E402


def _active_open_mm():
    """当前生效的真机全开口径（与 joints_to_mm 的回落规则一致）。"""
    return bridge.OPEN_MM if bridge.OPEN_MM is not None else bridge.OPEN_MM_CALIB


class JointsToMmTest(unittest.TestCase):
    """joints_to_mm：仿真关节位置(米) → 真机开口(mm)。"""

    def setUp(self):
        # 出厂状态：OPEN_MM 尚未实测校准，落到标定口径占位
        self._saved_open_mm = bridge.OPEN_MM
        bridge.OPEN_MM = None

    def tearDown(self):
        bridge.OPEN_MM = self._saved_open_mm

    def test_fully_open_maps_to_open_mm(self):
        self.assertAlmostEqual(
            bridge.joints_to_mm([0.0, 0.0]), bridge.OPEN_MM_CALIB, places=3)

    def test_fully_closed_maps_to_closed_mm(self):
        q = [bridge.Q_FULL_M, bridge.Q_FULL_M]
        self.assertAlmostEqual(
            bridge.joints_to_mm(q), bridge.CLOSED_MM, places=6)

    def test_half_travel_maps_to_half_opening(self):
        half = bridge.Q_FULL_M / 2.0
        self.assertAlmostEqual(
            bridge.joints_to_mm([half, half]), bridge.OPEN_MM_CALIB / 2.0, places=3)

    def test_fingers_are_averaged_not_extremised(self):
        # 只动一指时应落在中点，而不是取较小/较大那根
        got = bridge.joints_to_mm([0.0, bridge.Q_FULL_M])
        self.assertAlmostEqual(got, bridge.OPEN_MM_CALIB / 2.0, places=3)

    def test_over_travel_is_clamped_at_both_ends(self):
        self.assertAlmostEqual(
            bridge.joints_to_mm([0.1, 0.1]), bridge.CLOSED_MM, places=6)
        self.assertAlmostEqual(
            bridge.joints_to_mm([-0.02, -0.02]), bridge.OPEN_MM_CALIB, places=3)

    def test_empty_trajectory_returns_none(self):
        self.assertIsNone(bridge.joints_to_mm([]))
        self.assertIsNone(bridge.joints_to_mm(None))

    def test_measured_open_mm_overrides_calibration(self):
        bridge.OPEN_MM = 134.0
        self.assertAlmostEqual(bridge.joints_to_mm([0.0, 0.0]), 134.0, places=3)
        self.assertAlmostEqual(
            bridge.joints_to_mm([bridge.Q_FULL_M] * 2), bridge.CLOSED_MM, places=6)

    def test_result_stays_inside_calibrated_range(self):
        for q in (-0.5, -0.02, 0.0, 0.03, bridge.Q_FULL_M, 0.2):
            got = bridge.joints_to_mm([q, q])
            self.assertGreaterEqual(got, bridge.CLOSED_MM)
            self.assertLessEqual(got, bridge.OPEN_MM_CALIB)


class MeasuredOpenMmTest(unittest.TestCase):
    """OPEN_MM 已上机实测钉死（不再是 None 占位）。"""

    def test_measured_open_mm_is_pinned(self):
        self.assertEqual(bridge.OPEN_MM, 110.0)

    def test_zero_positions_map_to_measured_open_mm(self):
        self.assertAlmostEqual(
            bridge.joints_to_mm([0.0, 0.0]), bridge.OPEN_MM, places=6)

    def test_open_mm_wins_over_calibration_fallback(self):
        self.assertAlmostEqual(
            bridge.joints_to_mm([bridge.Q_FULL_M / 2] * 2), 55.0, places=3)


class MmRadTest(unittest.TestCase):
    """mm_to_rad / rad_to_mm：真机开口(mm) 与电机弧度互转。"""

    def setUp(self):
        self.cfg = types.SimpleNamespace(
            pos_closed_rad=0.1140, pos_open_rad=-1.4910, rad_to_mm=74.80)

    def test_closed_mm_is_pos_closed_rad(self):
        self.assertAlmostEqual(
            bridge.mm_to_rad(self.cfg, 0.0), self.cfg.pos_closed_rad, places=9)

    def test_round_trip(self):
        for mm in (0.0, 12.5, 55.0, 110.0):
            self.assertAlmostEqual(
                bridge.rad_to_mm(self.cfg, bridge.mm_to_rad(self.cfg, mm)),
                mm, places=9)

    def test_opening_decreases_rad(self):
        # 张开方向 rad 减小（与标定口径一致）
        self.assertLess(bridge.mm_to_rad(self.cfg, 110.0),
                        bridge.mm_to_rad(self.cfg, 0.0))

    def test_calibrated_travel_is_about_120mm(self):
        travel = (self.cfg.pos_closed_rad - self.cfg.pos_open_rad) * self.cfg.rad_to_mm
        self.assertAlmostEqual(travel, 120.05, places=1)


class AdvanceTowardTest(unittest.TestCase):
    """advance_toward：限速逼近一步的纯数学。"""

    def test_never_exceeds_max_step(self):
        q, dq = bridge.advance_toward(0.0, 1.0, 0.01)
        self.assertAlmostEqual(q, 0.01, places=12)
        self.assertAlmostEqual(dq, 0.01, places=12)

    def test_lands_exactly_on_target_without_overshoot(self):
        q, dq = bridge.advance_toward(0.0, 0.005, 0.01)
        self.assertAlmostEqual(q, 0.005, places=12)
        self.assertAlmostEqual(dq, 0.0, places=12)

    def test_moves_both_directions(self):
        self.assertAlmostEqual(bridge.advance_toward(1.0, 0.0, 0.01)[0],
                               0.99, places=12)
        self.assertAlmostEqual(bridge.advance_toward(-1.0, 0.0, 0.01)[0],
                               -0.99, places=12)

    def test_already_at_target_is_a_noop(self):
        self.assertEqual(bridge.advance_toward(0.5, 0.5, 0.01), (0.5, 0.0))

    def test_converges_in_bounded_number_of_steps(self):
        q, target, max_step = 0.0, 0.7353, 0.002005   # ≈55 mm，30mm/s@200Hz
        steps = 0
        while abs(q - target) > 0.0 and steps < 10000:
            q, _ = bridge.advance_toward(q, target, max_step)
            steps += 1
        self.assertEqual(q, target, "必须精确收敛，不能停在目标附近抖动")
        self.assertLessEqual(steps, int(0.7353 / 0.002005) + 1)


class DryRunTest(unittest.TestCase):
    """dry-run：目标去重打印、set_target 存 float（完全不碰硬件）。"""

    def test_set_target_stores_float(self):
        drv = bridge.GripperLiteGrip(dry_run=True)
        try:
            drv.set_target(3)
            self.assertIsInstance(drv._target, float)
            self.assertEqual(drv._target, 3.0)
        finally:
            drv.shutdown()

    def test_repeated_target_is_reported_once(self):
        drv = bridge.GripperLiteGrip(dry_run=True)
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                drv.set_target(50.0)
                time.sleep(0.2)
                drv.set_target(50.0)          # 重复目标
                drv.set_target(50.0)
                time.sleep(0.2)
                drv.set_target(60.0)          # 新目标
                time.sleep(0.2)
        finally:
            drv.shutdown()
        out = buf.getvalue()
        self.assertEqual(out.count("目标 50.0 mm"), 1, out)
        self.assertEqual(out.count("目标 60.0 mm"), 1, out)


class FrameStreamTest(unittest.TestCase):
    """真机控制环（用 litegrip 替身驱动，不碰 CAN）。

    ★ 核心回归：DM4310 使能后 ~100ms 收不到指令就自锁「通讯丢失」(err=13)，
    所以帧流**任何时候都不能出现空窗** —— 这正是原版用阻塞式 move_at_speed
    的 bug：斜坡流完就返回、总线静默，电机随即不响应后续指令。
    """

    def setUp(self):
        FakeLiteGrip.instances.clear()
        FakeLiteGrip.position_rad = 0.1140     # 全合（0 mm）
        FakeLiteGrip.torque_nm = 0.0
        FakeLiteGrip.error_code = 0

    def tearDown(self):
        FakeLiteGrip.instances.clear()

    def _wait_until(self, cond, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cond():
                return True
            time.sleep(0.01)
        return cond()

    def _make(self, **kwargs):
        drv = bridge.GripperLiteGrip(**kwargs)
        self.addCleanup(drv.shutdown)
        return drv, FakeLiteGrip.instances[-1]

    def test_idle_streams_zero_torque_frames_without_gap(self):
        drv, grip = self._make(hz=200.0)
        self.assertTrue(self._wait_until(lambda: len(grip.frames) >= 10),
                        "空闲时也应发帧（零力矩守住）")
        n1 = len(grip.frames)
        time.sleep(0.1)
        n2 = len(grip.frames)
        self.assertGreater(n2, n1, "帧流停了 → 电机 ~100ms 后会自锁 err=13")
        for _q, kp, kd, _dq, _tau in grip.frames:
            self.assertEqual((kp, kd), (0.0, 0.0), "空闲时应为零力矩")

    def test_target_ramps_at_limited_speed_and_lands_exactly(self):
        drv, grip = self._make(speed_mm_s=30.0, hz=200.0)
        cfg = grip.config
        target_mm = 1.0                        # 只走 1mm，几十毫秒就到位
        q_target = bridge.mm_to_rad(cfg, target_mm)
        max_step = 30.0 / cfg.rad_to_mm / 200.0

        self.assertTrue(self._wait_until(lambda: len(grip.frames) >= 2))
        drv.set_target(target_mm)
        self.assertTrue(self._wait_until(
            lambda: abs(grip.frames[-1][0] - q_target) < 1e-12,
            0.5), "应精确停在目标上（不越过、不抖）")

        qs = [f[0] for f in grip.frames]
        for a, b in zip(qs, qs[1:]):
            self.assertLessEqual(abs(b - a), max_step + 1e-9, "每周期位移必须受限速约束")
        self.assertLess(q_target, cfg.pos_closed_rad, "张开时 rad 应减小")
        self.assertAlmostEqual(grip.frames[-1][0], q_target, places=12)
        self.assertEqual(grip.frames[-1][1], drv._kp, "目标态应用位置刚度")

    def test_overforce_freezes_command_and_drops_target(self):
        FakeLiteGrip.torque_nm = 5.0            # 超默认 3.0 Nm 阈值
        drv, grip = self._make()
        start = grip.config.pos_closed_rad
        drv.set_target(20.0)

        self.assertTrue(self._wait_until(lambda: drv._target is None),
                        "过力应清掉目标，不再往前推")
        self.assertTrue(self._wait_until(
            lambda: grip.frames[-1][1] == 0.0 and grip.frames[-1][2] == 0.0,
            0.5), "冻结后应回到零力矩")
        self.assertAlmostEqual(grip.frames[-1][0], start, places=9,
                               msg="指令位置应被压回实际位置（卸力），不再前推")

    def test_fault_triggers_throttled_recovery(self):
        FakeLiteGrip.error_code = 13            # 通讯丢失
        drv, grip = self._make(hz=200.0)
        base = grip.enable_calls                # 构造时已使能过一次
        drv.set_target(5.0)

        self.assertTrue(self._wait_until(lambda: grip.enable_calls > base),
                        "读到故障码应尝试清故障 + 重新使能")
        time.sleep(0.2)
        self.assertLessEqual(grip.enable_calls - base, 2,
                             "恢复尝试必须限频，不能 200Hz 猛刷 enable()")
        before = len(grip.frames)
        time.sleep(0.1)
        self.assertGreater(len(grip.frames), before,
                           "故障期仍要发帧，别让总线静默")

    def test_shutdown_disables_and_disconnects(self):
        drv, grip = self._make()
        drv.shutdown()
        self.assertTrue(grip.disabled, "退出应失能，避免长期保持力矩")
        self.assertTrue(grip.disconnected)


class RouterTest(unittest.TestCase):
    """GripperBridge 回调：只取轨迹最后一个点。"""

    def test_callback_uses_last_point(self):
        drv = mock.Mock()
        node = bridge.GripperBridge.__new__(bridge.GripperBridge)
        node._drv = drv
        node.get_logger = mock.Mock(return_value=mock.Mock())

        point = mock.Mock()
        point.positions = [0.0, 0.0]
        msg = mock.Mock()
        msg.points = [mock.Mock(positions=[1.0, 1.0]), point]

        bridge.GripperBridge._cb(node, msg)

        drv.set_target.assert_called_once()
        self.assertAlmostEqual(drv.set_target.call_args[0][0],
                              _active_open_mm(), places=3)

    def test_callback_ignores_empty_trajectory(self):
        drv = mock.Mock()
        node = bridge.GripperBridge.__new__(bridge.GripperBridge)
        node._drv = drv
        node.get_logger = mock.Mock(return_value=mock.Mock())

        msg = mock.Mock()
        msg.points = []
        bridge.GripperBridge._cb(node, msg)

        drv.set_target.assert_not_called()


if __name__ == "__main__":
    unittest.main()

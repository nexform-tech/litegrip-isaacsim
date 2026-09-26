#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gripper_litegrip_bridge 的纯逻辑单测。

跑这个测试**不需要** ROS2 / Isaac Sim / CAN 硬件：
被测的只有「关节位置 → 开口毫米数」的换算和「目标去重」两段纯逻辑。
桥脚本顶部会 `import rclpy`、`from trajectory_msgs.msg import ...`，
CI 容器里没有这些包，所以先在 sys.modules 里塞轻量替身再 import 被测模块。

运行：
  python3 -m unittest discover -s tests -v
"""
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


_install_ros_stubs()

import gripper_litegrip_bridge as bridge  # noqa: E402


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


class TargetDedupTest(unittest.TestCase):
    """GripperLiteGrip 只在目标变化时才发起一次移动（dry-run，不碰硬件）。

    litegrip 的移动是阻塞调用，重复下发同一目标会白占住 worker 线程，
    所以这里钉死「同值不重发、变值才重发」。
    """

    def _wait_until(self, cond, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cond():
                return True
            time.sleep(0.02)
        return cond()

    def test_same_target_is_sent_once(self):
        drv = bridge.GripperLiteGrip(dry_run=True)
        try:
            drv._move = mock.Mock()

            drv.set_target(50.0)
            self.assertTrue(self._wait_until(lambda: drv._move.call_count == 1),
                            "首个目标应在 2s 内下发一次")
            self.assertEqual(drv._move.call_args[0][0], 50.0)

            drv.set_target(50.0)          # 重复目标
            time.sleep(0.3)
            self.assertEqual(drv._move.call_count, 1, "重复目标不应再次下发")

            drv.set_target(60.0)          # 新目标
            self.assertTrue(self._wait_until(lambda: drv._move.call_count == 2),
                            "目标变化后应再下发一次")
            self.assertEqual(drv._move.call_args[0][0], 60.0)
        finally:
            drv.shutdown()

    def test_set_target_stores_float(self):
        drv = bridge.GripperLiteGrip(dry_run=True)
        try:
            drv._move = mock.Mock()
            drv.set_target(3)
            self.assertIsInstance(drv._target, float)
            self.assertEqual(drv._target, 3.0)
        finally:
            drv.shutdown()


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
                              bridge.OPEN_MM_CALIB, places=3)

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

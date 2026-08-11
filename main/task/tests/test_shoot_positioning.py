#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/task/tests/test_shoot_positioning.py — task3 射击停车定位单测 (离线, mock 检测/移动).

覆盖 main/task/task3/shoot_target.py 的 2026-08-12 初始定位逻辑:
- 距离窗口几何换算 / 判距分类 (bbox 原始宽度, 不用 clamp 距离)
- 完整入画过滤 (左缘截断不作为距离锚点)
- position_for_shooting 状态机: 优先后退 → 前进兜底 → 回最佳位
- over-shot (后退中发现更前板) 不锁到 #2/#3
- 移动失败立即停 / 预算约束 / 步长参数透传

全部走 stdlib unittest, 离线无硬件; 检测/移动用注入的 fake 函数。
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

# 路径: main/task/tests/ → repo_root
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.task.task3 import shoot_target as st  # noqa: E402


# ---------------------------------------------------------------------------
# 工具: 假检测 / 假世界 (按累计位移返回检测 + 记录移动)
# ---------------------------------------------------------------------------

def make_det(xc, wn, det_id=0, score=0.9, yc=0.0, h=0.12):
    """构造 task_feed 风格的 detection dict (bbox_norm 为 [-1,1] 中心化坐标)."""
    return {
        "cls_id": 1, "det_id": det_id, "label": "animal", "score": score,
        "bbox_norm": {"x_center": xc, "y_center": yc, "width": wn, "height": h},
    }


class FakeWorld:
    """按累计位移 offset 返回检测的假世界; move() 记录每一步位移."""

    def __init__(self, feed):
        self.offset = 0.0
        self.moves = []          # list[(distance_m, label)]
        self.feed = feed

    def move(self, distance_m, label=""):
        self.offset += distance_m
        self.moves.append((round(distance_m, 3), label))

    def detect(self):
        return list(self.feed(self.offset))

    def offsets_visited(self):
        offs = [0.0]
        for d, _ in self.moves:
            offs.append(round(offs[-1] + d, 3))
        return offs

    def net_displacement(self):
        return round(sum(d for d, _ in self.moves), 3)


# ---------------------------------------------------------------------------
# 几何: 距离 <-> bbox 宽度换算
# ---------------------------------------------------------------------------

class WidthDistanceGeometryTests(unittest.TestCase):

    def test_width_for_known_distances(self):
        self.assertAlmostEqual(st.width_norm_for_distance(1.00), 0.0571, places=3)
        self.assertAlmostEqual(st.width_norm_for_distance(0.45), 0.1269, places=3)

    def test_width_monotonic_decreasing_with_distance(self):
        for a, b in [(0.5, 0.8), (0.8, 1.5), (0.45, 2.0)]:
            self.assertGreater(st.width_norm_for_distance(a),
                               st.width_norm_for_distance(b))

    def test_distance_window_to_width_window(self):
        wn_min, wn_max = st.distance_window_to_width_window(0.45, 1.0)
        self.assertLess(wn_min, wn_max)
        self.assertAlmostEqual(wn_min, st.width_norm_for_distance(1.0), places=3)
        self.assertAlmostEqual(wn_max, st.width_norm_for_distance(0.45), places=3)

    def test_estimate_distance_is_inverse(self):
        for d in (0.45, 0.6, 0.8, 1.0):
            wn = st.width_norm_for_distance(d)
            self.assertAlmostEqual(
                st.estimate_unclamped_distance_from_bbox(wn), d, places=2)

    def test_estimate_unclamped_distance_rejects_bad_wn(self):
        self.assertIsNone(st.estimate_unclamped_distance_from_bbox(None))
        self.assertIsNone(st.estimate_unclamped_distance_from_bbox(0.0))
        self.assertIsNone(st.estimate_unclamped_distance_from_bbox(-0.1))


# ---------------------------------------------------------------------------
# 判距分类 (原始 bbox 宽度)
# ---------------------------------------------------------------------------

class ClassifyShootDistanceTests(unittest.TestCase):

    def setUp(self):
        self.wn_min, self.wn_max = st.distance_window_to_width_window()

    def test_far_ok_near(self):
        self.assertEqual(st.classify_shoot_distance(self.wn_min - 0.01,
                                                    self.wn_min, self.wn_max), "far")
        self.assertEqual(st.classify_shoot_distance((self.wn_min + self.wn_max) / 2,
                                                    self.wn_min, self.wn_max), "ok")
        self.assertEqual(st.classify_shoot_distance(self.wn_max + 0.01,
                                                    self.wn_min, self.wn_max), "near")

    def test_boundaries_inclusive(self):
        self.assertEqual(st.classify_shoot_distance(self.wn_min,
                                                    self.wn_min, self.wn_max), "ok")
        self.assertEqual(st.classify_shoot_distance(self.wn_max,
                                                    self.wn_min, self.wn_max), "ok")

    def test_invalid_inputs(self):
        for wn in (0.0, -0.1, float("nan"), float("inf"), None, "abc"):
            self.assertEqual(st.classify_shoot_distance(wn,
                                                        self.wn_min, self.wn_max), "invalid")


# ---------------------------------------------------------------------------
# 完整入画过滤 (左缘截断不作为锚点)
# ---------------------------------------------------------------------------

class CompleteLeftTargetTests(unittest.TestCase):

    def test_complete_when_left_edge_inside(self):
        self.assertTrue(st.is_complete_left_target(make_det(-0.5, 0.10)))
        self.assertTrue(st.is_complete_left_target(make_det(-0.86, 0.10)))

    def test_truncated_when_left_edge_clipped(self):
        # 左缘 xc - wn/2 < -1 + margin(0.08) = -0.92 → 截断
        self.assertFalse(st.is_complete_left_target(make_det(-0.88, 0.10)))
        self.assertFalse(st.is_complete_left_target(make_det(-0.95, 0.10)))

    def test_invalid_bbox_rejected(self):
        self.assertFalse(st.is_complete_left_target(make_det(-0.5, 0.0)))
        self.assertFalse(st.is_complete_left_target(make_det(-0.5, -0.1)))
        self.assertFalse(st.is_complete_left_target({"bbox_norm": {}}))


class FrontmostCompleteTests(unittest.TestCase):

    def test_picks_leftmost_complete(self):
        animals = [make_det(0.3, 0.09), make_det(-0.6, 0.09), make_det(-0.2, 0.09)]
        front = st.frontmost_complete(animals, 0.5)
        self.assertAlmostEqual(st.bbox_xc(front), -0.6, places=3)

    def test_excludes_truncated_leftmost(self):
        # 左缘截断的最左板不作为锚点 → 选下一个完整板
        animals = [make_det(-0.9, 0.10), make_det(-0.4, 0.09)]
        front = st.frontmost_complete(animals, 0.5)
        self.assertAlmostEqual(st.bbox_xc(front), -0.4, places=3)

    def test_excludes_low_score(self):
        animals = [make_det(-0.6, 0.09, score=0.1), make_det(-0.2, 0.09)]
        front = st.frontmost_complete(animals, 0.5)
        self.assertAlmostEqual(st.bbox_xc(front), -0.2, places=3)

    def test_none_when_no_complete(self):
        self.assertIsNone(st.frontmost_complete([], 0.5))
        self.assertIsNone(st.frontmost_complete([make_det(-0.9, 0.10)], 0.5))


class GoodPositionTests(unittest.TestCase):

    def setUp(self):
        self.wn_min, self.wn_max = st.distance_window_to_width_window()

    def test_good_when_frontmost_in_window(self):
        animals = [make_det(-0.4, (self.wn_min + self.wn_max) / 2)]
        self.assertTrue(st.is_good_shooting_position(animals, self.wn_min, self.wn_max))

    def test_not_good_when_near_far_empty(self):
        self.assertFalse(st.is_good_shooting_position([], self.wn_min, self.wn_max))
        self.assertFalse(st.is_good_shooting_position(
            [make_det(-0.4, self.wn_max + 0.05)], self.wn_min, self.wn_max))
        self.assertFalse(st.is_good_shooting_position(
            [make_det(-0.4, self.wn_min - 0.02)], self.wn_min, self.wn_max))


# ---------------------------------------------------------------------------
# position_for_shooting 状态机
# ---------------------------------------------------------------------------

class PositionForShootingTests(unittest.TestCase):

    def setUp(self):
        self.d_min = st.SHOOT_DISTANCE_MIN_M
        self.d_max = st.SHOOT_DISTANCE_MAX_M
        self.wn_min, self.wn_max = st.distance_window_to_width_window(self.d_min,
                                                                       self.d_max)
        self.good_wn = (self.wn_min + self.wn_max) / 2.0   # ~0.092
        self.near_wn = self.wn_max + 0.05                  # ~0.177 (过近)
        self.far_wn = self.wn_min - 0.02                   # ~0.037 (过远)

    def _run(self, feed, **kw):
        world = FakeWorld(feed)
        res = st.position_for_shooting(
            None, 0.5, detect_fn=world.detect, move_fn=world.move,
            settle_s=0.0, **kw)
        return res, world

    # ---- 起点已达标: 零移动 ----
    def test_already_good_no_moves(self):
        res, world = self._run(lambda off: [make_det(0.0, self.good_wn)])
        self.assertTrue(res["positioned"])
        self.assertEqual(res["reason"], "already_good")
        self.assertEqual(res["final_offset_m"], 0.0)
        self.assertEqual(world.moves, [])

    # ---- 过近: 优先后退, 首步为负 ----
    def test_too_close_backs_up_first(self):
        def feed(off):
            return [make_det(0.0, self.near_wn + 0.3 * off)]
        res, world = self._run(feed)
        self.assertTrue(res["positioned"])
        self.assertEqual(res["reason"], "backup")
        self.assertLess(res["final_offset_m"], 0.0)
        self.assertTrue(world.moves)
        self.assertLess(world.moves[0][0], 0.0)      # 首步后退 (优先后退)

    # ---- 过远: 后退方向错(early-exit) → 前进兜底 ----
    def test_too_far_backup_then_forward(self):
        def feed(off):
            return [make_det(0.0, self.far_wn + 0.2 * max(0.0, off))]
        res, world = self._run(feed)
        self.assertTrue(res["positioned"])
        self.assertEqual(res["reason"], "forward")
        self.assertGreater(res["final_offset_m"], 0.0)
        self.assertTrue(world.moves)
        self.assertLess(world.moves[0][0], 0.0)      # 仍先退一步再前进
        self.assertGreater(world.net_displacement(), 0.0)  # 净前进

    # ---- 初始视野空: 不直接退出, 有限后退搜索 ----
    def test_empty_initial_backup_search(self):
        def feed(off):
            return [make_det(-0.3, self.good_wn)] if off < 0.0 else []
        res, world = self._run(feed)
        self.assertTrue(res["positioned"])
        self.assertEqual(res["reason"], "backup")
        self.assertLess(res["final_offset_m"], 0.0)
        self.assertTrue(world.moves)

    # ---- over-shot: 后退中发现更前板, 不锁到 #2/#3 ----
    def test_overshoot_keeps_backing_to_board1(self):
        def feed(off):
            if off >= 0.0:
                return [make_det(0.30, self.near_wn, det_id=2)]   # 只见 #2 且过近
            x1 = -0.55 - (off + 0.1) * 0.3                         # #1 从左侧入画
            return [make_det(x1, self.good_wn, det_id=1),
                    make_det(0.35, self.near_wn, det_id=2)]
        res, world = self._run(feed)
        self.assertTrue(res["positioned"])
        self.assertEqual(res["reason"], "backup")
        self.assertIsNotNone(res["anchor"])
        self.assertLess(st.bbox_xc(res["anchor"]), 0.0)   # anchor 是 #1 (负 xc), 不是 #2

    # ---- 全程未达标: 回到最佳位置, positioned=False ----
    def test_never_good_returns_toward_best(self):
        def feed(off):
            return [make_det(0.0, self.near_wn)]          # 恒过近
        res, world = self._run(feed)
        self.assertFalse(res["positioned"])
        self.assertEqual(res["reason"], "no_good_position")
        self.assertLess(abs(world.net_displacement()), 0.05)  # 净位移 ≈ 0 (回原点/最佳位)

    # ---- 移动失败: 立即停止, 不累积 ----
    def test_move_failed_stops(self):
        def boom(distance_m, label):
            raise RuntimeError("boom")
        world = FakeWorld(lambda off: [make_det(0.0, self.near_wn)])
        res = st.position_for_shooting(
            None, 0.5, detect_fn=world.detect, move_fn=boom, settle_s=0.0)
        self.assertFalse(res["positioned"])
        self.assertEqual(res["reason"], "move_failed")

    # ---- 预算约束: 后退 ≤ backup_max, 前进 ≤ backup+forward ----
    def test_budget_respected(self):
        def feed(off):
            return [make_det(0.0, self.near_wn)]
        _, world = self._run(feed)
        offs = world.offsets_visited()
        self.assertGreaterEqual(min(offs), -st.POSITION_BACKUP_MAX_M - 1e-9)
        self.assertLessEqual(max(offs), st.POSITION_FORWARD_MAX_M + 1e-9)

    # ---- 自定义步长/预算参数透传 ----
    def test_custom_step_and_budget(self):
        def feed(off):
            return [make_det(0.0, self.near_wn + 0.3 * off)]
        res, world = self._run(feed, step_m=0.05, backup_max_m=0.2)
        self.assertEqual(res["reason"], "backup")
        self.assertTrue(all(abs(d) == 0.05 for d, _ in world.moves))
        self.assertGreaterEqual(min(world.offsets_visited()), -0.2 - 1e-9)


if __name__ == "__main__":
    unittest.main()

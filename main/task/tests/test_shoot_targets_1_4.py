#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/task/tests/test_shoot_targets_1_4.py — 场景测试: 目标 #1 和 #4 是害虫 (要射).

模拟 task3 射击段的完整「停车定位 + 目标选择」流程 (离线, 全部 mock):
- 场上 4 块板 L→R 一行: **板上 #1/#4 是害虫 (要射), #2/#3 是益虫 (跳过)**.
- 车在射击区停下的位置 **过近且 over-shot** (起点只看到 #2/#3/#4, 且 bbox 太大).
- 验证:
  1. `--targets "1 4"` 解析为 {1, 4};
  2. `position_for_shooting` **优先后退**把 #1 找进可射击距离窗口 (anchor = 板上 #1);
  3. 主循环目标选择: 只对 #1/#4 开火, #2/#3 跳过不射.

几何 (依赖 shoot_target 默认窗口 SHOOT_DISTANCE_MIN/MAX_M = 0.45/1.0,
wn 窗口约 [0.057, 0.127]):
  - 窗口内 bbox 宽度 GOOD_WN = 0.09;
  - 过近 bbox 宽度 NEAR_WN = 0.18 (> wn_max → near).
若现场改了窗口常量, 需同步调 GOOD_WN/NEAR_WN.

运行:
  /usr/bin/python3 -m unittest main.task.tests.test_shoot_targets_1_4 -v
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
# 假现场: 4 块板一行, #1/#4 害虫, #2/#3 益虫
# ---------------------------------------------------------------------------

def make_det(xc, wn, score=0.9):
    """task_feed 风格 detection (bbox_norm 为 [-1,1] 中心化坐标)."""
    return {
        "cls_id": 1, "det_id": 0, "label": "animal", "score": score,
        "bbox_norm": {"x_center": xc, "y_center": 0.0, "width": wn, "height": 0.12},
    }


class FakeField:
    """按车头累计位移 offset (m) 返回 cam2 视野内的目标.

    板上物理属性 (L→R): #1 害虫 / #2 益虫 / #3 益虫 / #4 害虫.
    起点 (offset>=0): 车 over-shot 且过近 → 只看到 #2/#3/#4, 全部 bbox 太大 (near).
    后退 (offset<0): #1 从左侧入画, 距离进入窗口 → 4 块全可见、宽度合适.
    """

    GOOD_WN = 0.09          # 窗口内宽度
    NEAR_WN = 0.18          # 过近宽度 (> wn_max 0.127)
    BOARD_XC = [-0.55, -0.29, -0.03, 0.23]   # 窗口距离下 4 块板 L→R 的 xc

    def detections_at(self, offset):
        if offset >= 0.0:
            return [
                make_det(0.10, self.NEAR_WN),   # #2
                make_det(0.36, self.NEAR_WN),   # #3
                make_det(0.62, self.NEAR_WN),   # #4
            ]
        x1 = -0.55 + 0.3 * (offset + 0.2)       # 后退时 #1 缓慢左移 (仍在画面内)
        return [
            make_det(x1 + 0.00, self.GOOD_WN),  # #1
            make_det(x1 + 0.26, self.GOOD_WN),  # #2
            make_det(x1 + 0.52, self.GOOD_WN),  # #3
            make_det(x1 + 0.78, self.GOOD_WN),  # #4
        ]


class FakeCar:
    """记录定位移动, 并把检测交给 FakeField."""

    def __init__(self, field):
        self.field = field
        self.offset = 0.0
        self.moves = []

    def move(self, distance_m, label=""):
        self.offset += distance_m
        self.moves.append((round(distance_m, 3), label))

    def detect(self):
        return list(self.field.detections_at(self.offset))


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------

class TargetSelectionShootingTests(unittest.TestCase):

    def test_parse_targets_1_4(self):
        """--targets "1 4" 解析为 {1, 4}."""
        self.assertEqual(st.parse_targets_arg("1 4"), {1, 4})
        self.assertEqual(st.parse_targets_arg("4 1"), {1, 4})
        self.assertEqual(st.parse_targets_arg("all"),
                         set(range(1, st.N_TOTAL_BOARDS + 1)))

    def test_positioning_backs_up_to_board1(self):
        """起点过近且 over-shot → position_for_shooting 优先后退, 锚定板上 #1."""
        field = FakeField()
        car = FakeCar(field)
        res = st.position_for_shooting(
            None, 0.5, detect_fn=car.detect, move_fn=car.move, settle_s=0.0)

        self.assertTrue(res["positioned"])
        self.assertEqual(res["reason"], "backup")          # 优先后退
        self.assertLess(res["final_offset_m"], 0.0)         # 累计后退
        self.assertIsNotNone(res["anchor"])
        self.assertLess(st.bbox_xc(res["anchor"]), -0.4)    # anchor 是 #1 (最左), 不是 #2
        self.assertTrue(all(d < 0 for d, _ in car.moves))   # 全程只后退, 未前进

    def test_sequence_shoots_only_1_and_4(self):
        """定位后按 --targets "1 4": #1/#4 开火, #2/#3 跳过."""
        field = FakeField()
        car = FakeCar(field)
        res = st.position_for_shooting(
            None, 0.5, detect_fn=car.detect, move_fn=car.move, settle_s=0.0)
        self.assertTrue(res["positioned"])
        animals = res["animals"]                            # 定位结束时的稳定帧 (L→R)

        # 用真实 BoardTracker 按 L→R 给起点 detection 赋板上号
        tracker = st.BoardTracker(n_total=st.N_TOTAL_BOARDS)
        tracker.initialize(animals)
        bound = [bn for bn in range(1, st.N_TOTAL_BOARDS + 1)
                 if tracker.get_xc(bn) is not None]
        self.assertEqual(bound, [1, 2, 3, 4])               # 4 块板都绑定

        # 模拟主循环目标选择: 只对 targets 里的板上开火
        targets_to_shoot = st.parse_targets_arg("1 4")
        shots, skips = [], []
        for shot_seq in range(1, st.N_TOTAL_BOARDS + 1):
            if shot_seq in targets_to_shoot:
                shots.append(shot_seq)
            else:
                skips.append(shot_seq)

        self.assertEqual(shots, [1, 4])                     # 射 #1、#4
        self.assertEqual(skips, [2, 3])                     # 跳过 #2、#3

    def test_boards_below_threshold_marked_not_shoot(self):
        """同一视野下, 益虫板 (#2/#3) 即使可见也不进射击集合."""
        field = FakeField()
        car = FakeCar(field)
        res = st.position_for_shooting(
            None, 0.5, detect_fn=car.detect, move_fn=car.move, settle_s=0.0)
        animals = res["animals"]
        # 4 块板都可见 → 若按物理 pest 集合 {1,4} 选择, 视野里应有 4 只
        self.assertEqual(len(animals), st.N_TOTAL_BOARDS)
        # 但射击集合只有 {1,4}, 益虫 #2/#3 不射
        targets_to_shoot = st.parse_targets_arg("1 4")
        self.assertNotIn(2, targets_to_shoot)
        self.assertNotIn(3, targets_to_shoot)


if __name__ == "__main__":
    unittest.main()

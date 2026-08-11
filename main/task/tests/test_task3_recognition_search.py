#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/task/tests/test_task3_recognition_search.py — task3 识别段「前后微调找目标」单测.

覆盖 task3_pipeline.search_front_back (及 recognize_targets 同款) 的 2026-08-12 逻辑:
- 识别不出目标时 优先后退 → 前进兜底, 找到即返回 (net_offset 记录净位移);
- 全程找不到 → 回到原点 (net=0), 不改变车位;
- 预算约束: 后退 ≤ back_max, 前进 ≤ back_max+fwd_max;
- 首步为负 (优先后退)。

全部离线, 检测/移动注入 fake 函数。
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

# 路径: main/task/tests/ → repo_root
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.task.task3.task3_pipeline import search_front_back as pipe_search  # noqa: E402
from main.task.task3.task3_pipeline import judge_and_recapture  # noqa: E402
from main.task.task3.recognize_targets import search_front_back as recog_search  # noqa: E402


def make_det(xc):
    return {
        "cls_id": 1, "det_id": 0, "label": "animal", "score": 0.9,
        "bbox_norm": {"x_center": xc, "y_center": 0.0, "width": 0.1, "height": 0.12},
    }


class FakeWorld:
    """按累计位移返回检测: feed(offset) -> det|None; move() 记录每步位移."""

    def __init__(self, feed):
        self.offset = 0.0
        self.moves = []
        self.feed = feed

    def move(self, distance_m):
        self.offset += distance_m
        self.moves.append(round(distance_m, 3))

    def detect(self):
        return self.feed(self.offset)

    def net(self):
        return round(sum(self.moves), 3)


class SearchFrontBackTests(unittest.TestCase):

    def setUp(self):
        self.back_max = 0.12
        self.fwd_max = 0.16
        self.step = 0.04

    def _run(self, feed):
        world = FakeWorld(feed)
        res = pipe_search(world.detect, world.move,
                          step_m=self.step, back_max_m=self.back_max,
                          fwd_max_m=self.fwd_max)
        return res, world

    def test_found_after_one_back_step(self):
        """目标在后退一步后出现 → 返回 det, net=-step, 首步为负."""
        world = FakeWorld(lambda off: make_det(-0.3) if off <= -0.04 else None)
        det, net = pipe_search(world.detect, world.move,
                               step_m=self.step, back_max_m=self.back_max,
                               fwd_max_m=self.fwd_max)
        self.assertIsNotNone(det)
        self.assertEqual(net, -self.step)
        self.assertEqual(world.moves, [-self.step])

    def test_found_after_going_forward(self):
        """目标只在向前时出现 → 先退到底, 再前进, 找到返回正 net."""
        world = FakeWorld(lambda off: make_det(0.3) if off >= 0.08 else None)
        det, net = pipe_search(world.detect, world.move,
                               step_m=self.step, back_max_m=self.back_max,
                               fwd_max_m=self.fwd_max)
        self.assertIsNotNone(det)
        self.assertEqual(net, 0.08)
        self.assertLess(world.moves[0], 0)          # 首步后退 (优先后退)
        self.assertGreater(sum(world.moves), 0)     # 净前进

    def test_never_found_returns_to_origin(self):
        """全程找不到 → 回到原点, (None, 0.0), 净位移 0."""
        world = FakeWorld(lambda off: None)
        det, net = pipe_search(world.detect, world.move,
                               step_m=self.step, back_max_m=self.back_max,
                               fwd_max_m=self.fwd_max)
        self.assertIsNone(det)
        self.assertEqual(net, 0.0)
        self.assertEqual(world.net(), 0.0)          # 回到原点

    def test_budget_respected(self):
        """后退累计 ≤ back_max, 前进累计 ≤ back_max+fwd_max, 且不越过 fwd_max."""
        world = FakeWorld(lambda off: None)
        pipe_search(world.detect, world.move,
                    step_m=self.step, back_max_m=self.back_max,
                    fwd_max_m=self.fwd_max)
        offset = 0.0
        min_off = 0.0
        max_off = 0.0
        for d in world.moves:
            offset += d
            min_off = min(min_off, offset)
            max_off = max(max_off, offset)
        self.assertGreaterEqual(min_off, -self.back_max - 1e-9)
        self.assertLessEqual(max_off, self.fwd_max + 1e-9)

    def test_recognize_targets_twin_matches(self):
        """recognize_targets 里的同款 search_front_back 行为一致."""
        world = FakeWorld(lambda off: make_det(-0.3) if off <= -0.04 else None)
        det, net = recog_search(world.detect, world.move,
                                step_m=self.step, back_max_m=self.back_max,
                                fwd_max_m=self.fwd_max)
        self.assertIsNotNone(det)
        self.assertEqual(net, -self.step)


class JudgeAndRecaptureTests(unittest.TestCase):
    """judge_and_recapture: LLM 判定 result=None 时前后微调重拍 (2026-08-12)."""

    def _make_record(self, number=1):
        return {"number": number, "image_path": "target_01.jpg", "xc": 0.1,
                "result": None, "species": None}

    def test_valid_first_verdict_no_recapture(self):
        record = self._make_record()
        calls = {"classify": 0, "search": 0, "recapture": 0}

        def classify(rec):
            calls["classify"] += 1
            return {"name": "aphid", "result": 0, "analysis": "pest"}

        def search():
            calls["search"] += 1
            return make_det(-0.3), -0.04

        def recapture(det):
            calls["recapture"] += 1

        judge_and_recapture(record, classify_fn=classify, search_fn=search,
                            recapture_fn=recapture, max_retries=2)
        self.assertEqual(record["result"], 0)
        self.assertEqual(record["classification"], "pest")
        self.assertEqual(calls, {"classify": 1, "search": 0, "recapture": 0})

    def test_none_then_recapture_once(self):
        record = self._make_record()
        verdicts = iter([
            {"name": "unknown", "result": None, "analysis": "unclear"},
            {"name": "ladybug", "result": 1, "analysis": "beneficial"},
        ])
        calls = {"search": 0, "recapture": 0}

        def classify(rec):
            return next(verdicts)

        def search():
            calls["search"] += 1
            return make_det(-0.3), -0.04

        def recapture(det):
            calls["recapture"] += 1
            record["image_path"] = "target_01_retry.jpg"

        judge_and_recapture(record, classify_fn=classify, search_fn=search,
                            recapture_fn=recapture, max_retries=2)
        self.assertEqual(record["result"], 1)
        self.assertEqual(record["classification"], "beneficial")
        self.assertEqual(calls, {"search": 1, "recapture": 1})

    def test_always_none_hits_retry_budget(self):
        record = self._make_record()
        calls = {"search": 0, "recapture": 0}

        def classify(rec):
            return {"name": "unknown", "result": None, "analysis": "?"}

        def search():
            calls["search"] += 1
            return make_det(-0.3), -0.04

        def recapture(det):
            calls["recapture"] += 1

        judge_and_recapture(record, classify_fn=classify, search_fn=search,
                            recapture_fn=recapture, max_retries=2)
        self.assertEqual(record["result"], None)
        self.assertEqual(record["classification"], "unknown")
        self.assertEqual(calls["search"], 2)       # 重拍 2 次后停止
        self.assertEqual(calls["recapture"], 2)

    def test_search_finds_nothing_breaks_early(self):
        record = self._make_record()
        calls = {"search": 0, "recapture": 0}

        def classify(rec):
            return {"name": "unknown", "result": None, "analysis": "?"}

        def search():
            calls["search"] += 1
            return None, 0.0                       # 重拍找不到目标

        def recapture(det):
            calls["recapture"] += 1

        judge_and_recapture(record, classify_fn=classify, search_fn=search,
                            recapture_fn=recapture, max_retries=3)
        self.assertEqual(record["result"], None)
        self.assertEqual(calls["search"], 1)       # 只搜一次就停
        self.assertEqual(calls["recapture"], 0)

    def test_search_raises_breaks_early(self):
        record = self._make_record()

        def classify(rec):
            return {"name": "unknown", "result": None, "analysis": "?"}

        def search():
            raise RuntimeError("move failed")

        def recapture(det):
            self.fail("should not recapture after move failure")

        judge_and_recapture(record, classify_fn=classify, search_fn=search,
                            recapture_fn=recapture, max_retries=3)
        self.assertEqual(record["result"], None)
        self.assertEqual(record["classification"], "unknown")


if __name__ == "__main__":
    unittest.main()

"""TargetSelector 单测"""
import unittest
from main.arm.vision import (
    BBoxNorm, Detection, TargetSelector, SelectionStrategy,
)
from main.arm.labels import Label, LABEL_GROUPS


def _det(label, cx, cy, score=0.9, w=0.1, h=0.1, tid=None):
    return Detection(
        label=label, score=score, track_id=tid, class_id=None,
        bbox_norm=BBoxNorm(x_center=cx, y_center=cy, width=w, height=h),
        bbox_pixels=None, fetched_at=0.0,
    )


class TestTargetSelectorFactory(unittest.TestCase):
    def test_for_label_with_enum(self):
        sel = TargetSelector.for_label(Label.H_DOU_JIAO)
        self.assertEqual(sel.label, "h_dou_jiao")

    def test_for_label_with_string(self):
        sel = TargetSelector.for_label("cylinder_1")
        self.assertEqual(sel.label, "cylinder_1")

    def test_for_group(self):
        sel = TargetSelector.for_group("vegetable")
        self.assertEqual(sel.group, "vegetable")
        self.assertIsNone(sel.label)

    def test_for_group_unknown_raises(self):
        with self.assertRaises(ValueError):
            TargetSelector.for_group("not_a_group")


class TestMatches(unittest.TestCase):
    def test_matches_by_label(self):
        sel = TargetSelector.for_label("h_dou_jiao")
        self.assertTrue(sel.matches(_det("h_dou_jiao", 0, 0)))
        self.assertFalse(sel.matches(_det("h_fan_qie", 0, 0)))

    def test_matches_by_group(self):
        sel = TargetSelector.for_group("vegetable")
        self.assertTrue(sel.matches(_det("h_dou_jiao", 0, 0)))
        self.assertTrue(sel.matches(_det("h_fan_qie", 0, 0)))
        self.assertFalse(sel.matches(_det("animal", 0, 0)))

    def test_matches_no_filter(self):
        sel = TargetSelector()
        self.assertTrue(sel.matches(_det("any_label", 0, 0)))


class TestApplyStrategy(unittest.TestCase):
    def test_highest_score(self):
        sel = TargetSelector(strategy="highest_score")
        candidates = [
            _det("x", 0, 0, score=0.5),
            _det("x", 0, 0, score=0.9),
            _det("x", 0, 0, score=0.7),
        ]
        pick = sel.apply_strategy(candidates)
        self.assertEqual(pick.score, 0.9)

    def test_closest_to_center(self):
        sel = TargetSelector(strategy="closest_to_center")
        candidates = [
            _det("x", 0.3, 0.0),
            _det("x", 0.05, 0.05),
            _det("x", -0.2, 0.0),
        ]
        pick = sel.apply_strategy(candidates)
        self.assertAlmostEqual(pick.bbox_norm.x_center, 0.05)

    def test_largest(self):
        sel = TargetSelector(strategy="largest")
        candidates = [
            _det("x", 0, 0, w=0.1, h=0.1),
            _det("x", 0, 0, w=0.5, h=0.5),
            _det("x", 0, 0, w=0.2, h=0.2),
        ]
        pick = sel.apply_strategy(candidates)
        self.assertAlmostEqual(pick.bbox_norm.width, 0.5)

    def test_leftmost(self):
        sel = TargetSelector(strategy="leftmost")
        candidates = [
            _det("x", 0.5, 0),
            _det("x", -0.8, 0),
            _det("x", 0.1, 0),
        ]
        pick = sel.apply_strategy(candidates)
        self.assertAlmostEqual(pick.bbox_norm.x_center, -0.8)

    def test_rightmost(self):
        sel = TargetSelector(strategy="rightmost")
        candidates = [
            _det("x", -0.5, 0),
            _det("x", 0.8, 0),
            _det("x", 0.1, 0),
        ]
        pick = sel.apply_strategy(candidates)
        self.assertAlmostEqual(pick.bbox_norm.x_center, 0.8)

    def test_empty_returns_none(self):
        sel = TargetSelector(strategy="highest_score")
        self.assertIsNone(sel.apply_strategy([]))


if __name__ == "__main__":
    unittest.main()
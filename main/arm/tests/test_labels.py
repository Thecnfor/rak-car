"""main/arm/labels.py 单测：20 项枚举 + 分组查询 + 工厂"""
import unittest
from main.arm.labels import (
    Label, LabelInfo, LABELS, LABEL_GROUPS,
    get_label_info, is_in_group,
)


class TestLabels(unittest.TestCase):
    def test_labels_count_is_twenty(self):
        self.assertEqual(len(LABELS), 20)

    def test_labels_ids_sequential_1_to_20(self):
        ids = [info.id for info in LABELS]
        self.assertEqual(ids, list(range(1, 21)))

    def test_label_enum_is_str_subclass(self):
        self.assertEqual(Label.H_DOU_JIAO, "h_dou_jiao")
        self.assertIsInstance(Label.H_DOU_JIAO, str)

    def test_get_label_info_returns_correct_entry(self):
        info = get_label_info("h_dou_jiao")
        self.assertEqual(info.id, 8)
        self.assertEqual(info.desc, "豆角")

    def test_get_label_info_unknown_raises(self):
        with self.assertRaises(ValueError) as ctx:
            get_label_info("not_a_real_label")
        self.assertIn("未知 label", str(ctx.exception))

    def test_label_groups_keys(self):
        expected_groups = {"animal", "ball", "cylinder", "cylinder_meta", "vegetable", "water"}
        self.assertEqual(set(LABEL_GROUPS.keys()), expected_groups)

    def test_vegetable_group_has_nine(self):
        self.assertEqual(len(LABEL_GROUPS["vegetable"]), 9)

    def test_water_group_has_four(self):
        self.assertEqual(len(LABEL_GROUPS["water"]), 4)

    def test_is_in_group(self):
        self.assertTrue(is_in_group("h_dou_jiao", "vegetable"))
        self.assertTrue(is_in_group("water_l1", "water"))
        self.assertFalse(is_in_group("h_dou_jiao", "ball"))
        self.assertFalse(is_in_group("not_a_label", "vegetable"))


if __name__ == "__main__":
    unittest.main()
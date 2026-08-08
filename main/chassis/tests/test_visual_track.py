"""main/chassis/tests/test_visual_track.py
通用底盘视觉追踪单测 (stdlib unittest, 离线无硬件)。

改造后：track_chassis 是 thin wrapper（一次 HTTP 调用），本文件测：
  - expand_label_set：组展开 / 单 label / list（纯函数，不变）
  - _select_target：nearest / largest / leftmost / no_match（纯函数，不变）
  - track_chassis：mock api.chassis_align → 验证参数透传 + 返回值映射
"""
import unittest
from unittest.mock import MagicMock, call

from main.chassis.loops.visual_track import (
    expand_label_set,
    track_chassis,
    _select_target,
    TrackChassisResult,
    TrackFrame,
)


# ===== expand_label_set (纯函数) =====

class TestExpandLabelSet(unittest.TestCase):
    def test_water_group_expands(self):
        labels = expand_label_set("water")
        self.assertIn("water", labels)
        self.assertIn("water_l1", labels)
        self.assertIn("water_l2", labels)
        self.assertIn("water_l3", labels)
        self.assertEqual(len(labels), 4)

    def test_cylinder_group_expands(self):
        labels = expand_label_set("cylinder")
        self.assertIn("cylinder_1", labels)
        self.assertIn("cylinder_2", labels)
        self.assertIn("cylinder_3", labels)

    def test_specific_label_is_single(self):
        labels = expand_label_set("cylinder_set")
        self.assertEqual(labels, {"cylinder_set"})

    def test_list_input(self):
        labels = expand_label_set(["water", "cylinder_1"])
        self.assertIn("water", labels)
        self.assertIn("water_l1", labels)
        self.assertIn("cylinder_1", labels)


# ===== _select_target (纯函数) =====

def _d(label="water", cx=0.0, cy=0.0, w=0.2, h=0.2, score=0.9):
    return {
        "label": label,
        "score": score,
        "bbox_norm": {"cx": cx, "cy": cy, "width": w, "height": h},
    }


class TestSelectTarget(unittest.TestCase):
    def test_nearest_to_center_prefers_closer(self):
        dets = [_d(cx=0.5), _d(cx=0.1), _d(cx=0.2, label="other")]
        chosen = _select_target(dets, {"water"}, (0.0, 0.0), "nearest_to_center")
        self.assertIsNotNone(chosen)
        self.assertAlmostEqual(chosen["bbox_norm"]["cx"], 0.1)

    def test_largest_area(self):
        dets = [_d(w=0.1, h=0.1), _d(w=0.4, h=0.4), _d(w=0.2, h=0.2, label="x")]
        chosen = _select_target(dets, {"water"}, (0.0, 0.0), "largest_area")
        self.assertEqual(chosen["bbox_norm"]["width"], 0.4)

    def test_no_match_returns_none(self):
        self.assertIsNone(_select_target([_d(label="x")], {"water"}, (0.0, 0.0)))

    def test_leftmost_prefers_min_cx(self):
        dets = [
            _d(label="ball_yellow", cx=0.3),
            _d(label="ball_blue", cx=-0.4),
            _d(label="other", cx=-0.9),
        ]
        chosen = _select_target(
            dets, {"ball_blue", "ball_yellow"}, (0.0, 0.0), "leftmost")
        self.assertEqual(chosen["label"], "ball_blue")

    def test_leftmost_tie_prefers_larger_area(self):
        dets = [
            _d(label="ball_blue", cx=-0.2, w=0.1, h=0.1),
            _d(label="ball_yellow", cx=-0.2, w=0.3, h=0.3),
        ]
        chosen = _select_target(
            dets, {"ball_blue", "ball_yellow"}, (0.0, 0.0), "leftmost")
        self.assertEqual(chosen["label"], "ball_yellow")


# ===== track_chassis thin wrapper =====

class TestTrackChassisThinWrapper(unittest.TestCase):
    """track_chassis 现在是一次 HTTP 调用的 thin wrapper。"""

    def _make_mock_api(self, response_dict):
        """创建 mock ChassisClient，chassis_align 返回 response_dict。"""
        api = MagicMock()
        api.chassis_align = MagicMock(return_value=response_dict)
        api.close = MagicMock()
        return api

    def test_arrived_maps_from_response(self):
        """runtime 返回 arrived=True → track_chassis 返回 TrackChassisResult(arrived=True)。"""
        response = {
            "arrived": True, "reason": "arrived",
            "frames": 23, "elapsed_s": 1.15,
            "final_frame": {
                "target_found": True, "label": "h_tu_dou",
                "cx": 0.01, "cy": -0.01, "cx_err": -0.01, "cy_err": 0.01,
                "area": 0.04, "score": 0.9, "vx": 0.0, "vy": 0.0,
                "age_ms": 22.0,
            },
        }
        api = self._make_mock_api(response)
        result = track_chassis("h_tu_dou", api=api, dry_run=False)
        self.assertTrue(result.arrived)
        self.assertEqual(result.reason, "arrived")
        self.assertEqual(result.frames, 23)
        self.assertAlmostEqual(result.elapsed_s, 1.15)
        self.assertIsNotNone(result.final_frame)
        self.assertEqual(result.final_frame.label, "h_tu_dou")

    def test_no_target_maps_from_response(self):
        response = {"arrived": False, "reason": "no_target",
                    "frames": 60, "elapsed_s": 3.0, "final_frame": None}
        api = self._make_mock_api(response)
        result = track_chassis("h_tu_dou", api=api)
        self.assertFalse(result.arrived)
        self.assertEqual(result.reason, "no_target")

    def test_watchdog_maps_from_response(self):
        response = {"arrived": False, "reason": "watchdog",
                    "frames": 1, "elapsed_s": 0.02, "final_frame": None}
        api = self._make_mock_api(response)
        result = track_chassis("h_tu_dou", api=api)
        self.assertFalse(result.arrived)
        self.assertEqual(result.reason, "watchdog")

    def test_timeout_maps_from_response(self):
        response = {"arrived": False, "reason": "timeout",
                    "frames": 200, "elapsed_s": 10.0, "final_frame": None}
        api = self._make_mock_api(response)
        result = track_chassis("h_tu_dou", api=api, max_seconds=10.0)
        self.assertFalse(result.arrived)
        self.assertEqual(result.reason, "timeout")

    def test_params_forwarded_to_chassis_align(self):
        """所有参数透传给 api.chassis_align()。"""
        response = {"arrived": False, "reason": "timeout",
                    "frames": 0, "elapsed_s": 0.0, "final_frame": None}
        api = self._make_mock_api(response)
        track_chassis(
            "water",
            api=api,
            setpoint_cxcy=(-0.1, 0.1),
            select_mode="leftmost",
            sign_vx=+1, sign_vy=-1, vx_only=True,
            kp=0.30, v_max=0.15, deadband=0.08, hold_frames=3,
            v_slew=0.03, max_lost_frames=30, recover_after_lost=False,
            watchdog_ms=1000.0, hz=15.0, max_seconds=5.0, dry_run=True,
        )
        call_kwargs = api.chassis_align.call_args[1]
        self.assertEqual(call_kwargs["target"], "water")
        self.assertEqual(call_kwargs["setpoint_cxcy"], [-0.1, 0.1])  # tuple → list
        self.assertEqual(call_kwargs["select_mode"], "leftmost")
        self.assertEqual(call_kwargs["sign_vx"], 1)
        self.assertEqual(call_kwargs["sign_vy"], -1)
        self.assertTrue(call_kwargs["vx_only"])
        self.assertAlmostEqual(call_kwargs["kp"], 0.30)
        self.assertAlmostEqual(call_kwargs["v_max"], 0.15)
        self.assertAlmostEqual(call_kwargs["deadband"], 0.08)
        self.assertEqual(call_kwargs["hold_frames"], 3)
        self.assertTrue(call_kwargs["dry_run"])

    def test_api_auto_connect(self):
        """api=None 时自动 ChassisClient.connect()。"""
        response = {"arrived": False, "reason": "timeout",
                    "frames": 0, "elapsed_s": 0.0, "final_frame": None}
        with unittest.mock.patch(
            "main.chassis.loops.visual_track.ChassisClient.connect"
        ) as mock_connect:
            mock_api = MagicMock()
            mock_api.chassis_align = MagicMock(return_value=response)
            mock_api.close = MagicMock()
            mock_connect.return_value = mock_api
            result = track_chassis("h_tu_dou")
            mock_connect.assert_called_once()

    def test_api_not_closed_when_passed(self):
        """调用方传入 api → 不自动 close（生命周期归调用方）。"""
        response = {"arrived": False, "reason": "timeout",
                    "frames": 0, "elapsed_s": 0.0, "final_frame": None}
        api = self._make_mock_api(response)
        track_chassis("h_tu_dou", api=api)
        api.close.assert_not_called()

    def test_own_api_closed(self):
        """api=None 自建 → 收尾 close。"""
        response = {"arrived": False, "reason": "timeout",
                    "frames": 0, "elapsed_s": 0.0, "final_frame": None}
        with unittest.mock.patch(
            "main.chassis.loops.visual_track.ChassisClient.connect"
        ) as mock_connect:
            mock_api = self._make_mock_api(response)
            mock_connect.return_value = mock_api
            track_chassis("h_tu_dou")
            mock_api.close.assert_called_once()

    def test_error_response_returns_error_result(self):
        """非 dict 响应 → TrackChassisResult(reason="error")。"""
        api = self._make_mock_api(None)  # None response
        result = track_chassis("h_tu_dou", api=api)
        self.assertFalse(result.arrived)
        self.assertEqual(result.reason, "error")

    def test_nested_result_dict(self):
        """响应含 "result" 嵌套 → 正确提取。"""
        response = {
            "ok": True,
            "result": {"arrived": True, "reason": "arrived",
                       "frames": 5, "elapsed_s": 0.3, "final_frame": None},
        }
        api = self._make_mock_api(response)
        result = track_chassis("h_tu_dou", api=api)
        self.assertTrue(result.arrived)

    def test_on_tick_alone_stays_runtime(self):
        """on_tick 单独传入（无 sense_fn）→ 仍走 runtime 下沉 + warning。"""
        response = {"arrived": False, "reason": "timeout",
                    "frames": 0, "elapsed_s": 0.0, "final_frame": None}
        api = self._make_mock_api(response)
        cb = MagicMock()
        with self.assertLogs("main.chassis.loops.visual_track", level="WARNING") as cm:
            track_chassis("h_tu_dou", api=api, on_tick=cb)
        # 走 runtime：chassis_align 被调，on_tick 未回调
        api.chassis_align.assert_called_once()
        cb.assert_not_called()
        self.assertTrue(any("on_tick ignored" in m for m in cm.output))

    def test_sense_fn_triggers_client_loop(self):
        """sense_fn 传入（如 task6 LLM-as-servo）→ 走 client 闭环。"""
        response = {"arrived": False, "reason": "timeout",
                    "frames": 0, "elapsed_s": 0.0, "final_frame": None}
        api = self._make_mock_api(response)
        calls = []
        cb = MagicMock()

        def _sense():
            calls.append(1)
            return TrackFrame(target_found=True, label="order_card",
                              cx=0.5, cy=0.5, cx_err=-0.01, cy_err=-0.01)

        result = track_chassis("order_card", api=api, sense_fn=_sense,
                               setpoint_cxcy=(0.5, 0.5),
                               deadband=0.05, hold_frames=2,
                               max_seconds=1.0, max_lost_frames=100)
        # 走 client 闭环：sense_fn 被反复调用，runtime 未参与
        api.chassis_align.assert_not_called()
        self.assertGreater(len(calls), 0)
        self.assertTrue(result.arrived)


if __name__ == "__main__":
    unittest.main()

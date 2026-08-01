"""吸嘴偏移 setpoint 单测 — find_target_* 把目标对准"吸嘴正下方"而非画面中心.

背景 (2026-08-01): 吸嘴-相机刚性绑定, 目标在吸嘴正下方时其 bbox 中心落在
(nozzle_offset_x_norm, nozzle_offset_y_norm) 而非 (0,0). 传 setpoint 即把
误差/收敛判据从 center 改为 center - setpoint.
"""
import unittest
from unittest.mock import MagicMock
from main.arm.vision import ArmVisionClient, TargetSelector, Detection, BBoxNorm
from main.arm.loops.runner import ArmRunner
from main.arm.state import ArmOrigin


def _det_dict(cx=0.0, cy=0.0, label="cylinder_3", score=0.9, height=100):
    """构造 detection dict (与 runtime 格式一致)."""
    return {
        "label": label, "score": score, "det_id": 1, "cls_id": 1,
        "bbox_norm": {"x_center": cx, "y_center": cy, "width": 0.1, "height": 0.1},
        "bbox_pixels": {"x1": 0, "y1": 0, "x2": height, "y2": height,
                        "width": height, "height": height},
    }


def _make_http_with_dets(dicts, frame_shape=None):
    http = MagicMock()
    counter = [0]
    pool = [d for d in dicts]

    def _next_state():
        counter[0] += 1
        state = {"detections": [{k: v for k, v in d.items()} for d in pool],
                 "updated_at": float(counter[0])}
        if frame_shape:
            state["frame_shape"] = frame_shape
        return {"task_state": state}

    http.get_vision_task_cache.side_effect = _next_state
    return http


class TestFindTargetPidSetpoint(unittest.TestCase):
    def test_converges_at_setpoint_not_center(self):
        """目标在 (0.1, -0.6)（吸嘴正下方），setpoint 相同 → err=0 → 收敛.

        若仍按画面中心收敛(旧行为) 则不会收敛（|0.1|>0.05）.
        """
        det = _det_dict(cx=0.1, cy=-0.6)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        result = client.find_target_pid(
            TargetSelector.for_label("cylinder_3"),
            x_mm=0.0, y_mm=-100.0,
            setpoint_x_norm=0.1, setpoint_y_norm=-0.6,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05, settle_stable_frames=1,
            timeout=2.0, max_iter=3,
        )
        self.assertTrue(result.converged)
        self.assertTrue(result.settle_stable)

    def test_moves_toward_setpoint(self):
        """目标在画面中心(0,0), setpoint=(0.1,-0.6):
        err=(-0.1, +0.6) → dx_mm = -(-0.1)*30 = +3.0, dy_mm = -(+0.6)*30 = -18.0.
        """
        det = _det_dict(cx=0.0, cy=0.0)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        result = client.find_target_pid(
            TargetSelector.for_label("cylinder_3"),
            x_mm=0.0, y_mm=-100.0,
            setpoint_x_norm=0.1, setpoint_y_norm=-0.6,
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05, settle_stable_frames=99,
            mm_per_norm_base=30.0,
            timeout=0.5, max_iter=1,
        )
        self.assertAlmostEqual(result.trace[0].x_mm, 3.0, places=1)
        self.assertAlmostEqual(result.trace[0].y_mm, -118.0, places=1)

    def test_mm_per_norm_alias_bridge(self):
        """runner 透传 mm_per_norm（legacy 参数）到 PID 路径不崩, 且被采用."""
        det = _det_dict(cx=0.1, cy=0.0)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        result = client.find_target_pid(
            TargetSelector.for_label("cylinder_3"),
            x_mm=0.0, y_mm=-100.0,
            mm_per_norm=30.0,          # 别名桥: 覆盖 mm_per_norm_base
            setpoint_x_norm=0.1,       # 触发 PID 路由的现场
            kp=1.0, ki=0.0, kd=0.0,
            settle_tol_norm=0.05, settle_stable_frames=1,
            timeout=2.0, max_iter=3,
        )
        self.assertTrue(result.converged)


class TestFindTargetLegacySetpoint(unittest.TestCase):
    def test_legacy_converges_at_setpoint(self):
        det = _det_dict(cx=0.1, cy=-0.6)
        http = _make_http_with_dets([det])
        client = ArmVisionClient(http)
        result = client.find_target_legacy(
            TargetSelector.for_label("cylinder_3"),
            x_mm=0.0, y_mm=-100.0,
            setpoint_x_norm=0.1, setpoint_y_norm=-0.6,
            settle_tol_norm=0.05, timeout=2.0, max_iter=3,
        )
        self.assertTrue(result.converged)


class TestFindTargetRoutingSetpoint(unittest.TestCase):
    def test_setpoint_routes_to_pid(self):
        """传 setpoint 无 kp/ki/kd → 走 PID: settle_stable_frames 默认 3, 需 3 帧收敛.

        legacy 1 帧即返回且 settle_stable=False; PID 3 帧 + settle_stable=True 可区分.
        """
        det = _det_dict(cx=0.1, cy=-0.6)
        http = _make_http_with_dets([det, det, det])
        client = ArmVisionClient(http)
        result = client.find_target(
            TargetSelector.for_label("cylinder_3"),
            x_mm=0.0, y_mm=-100.0,
            setpoint_x_norm=0.1, setpoint_y_norm=-0.6,
            settle_tol_norm=0.05, timeout=2.0, max_iter=5,
        )
        self.assertTrue(result.converged)
        self.assertTrue(result.settle_stable)


class TestNozzleOffsetFor(unittest.TestCase):
    """ArmOrigin.nozzle_offset_for 查表回落链 (2026-08-02 per-label)."""

    def test_label_hits_map(self):
        o = ArmOrigin(nozzle_offset_map={"ball_yellow": (0.101, -0.704)})
        self.assertEqual(o.nozzle_offset_for("ball_yellow"), (0.101, -0.704))

    def test_unknown_label_falls_back_default(self):
        o = ArmOrigin(nozzle_offset_x_norm=0.08, nozzle_offset_y_norm=-0.63,
                      nozzle_offset_map={"ball_yellow": (0.101, -0.704)})
        self.assertEqual(o.nozzle_offset_for("h_tu_dou"), (0.08, -0.63))

    def test_none_label_falls_back_default(self):
        o = ArmOrigin(nozzle_offset_x_norm=0.08, nozzle_offset_y_norm=-0.63)
        self.assertEqual(o.nozzle_offset_for(None), (0.08, -0.63))

    def test_uncalibrated_returns_none(self):
        o = ArmOrigin()
        self.assertIsNone(o.nozzle_offset_for("cylinder_3"))
        self.assertIsNone(o.nozzle_offset_for(None))

    def test_map_zero_entry_falls_back(self):
        o = ArmOrigin(nozzle_offset_x_norm=0.08, nozzle_offset_y_norm=-0.63,
                      nozzle_offset_map={"cylinder_1": (0.0, 0.0)})
        self.assertEqual(o.nozzle_offset_for("cylinder_1"), (0.08, -0.63))


class TestRealtimeSetpoint(unittest.TestCase):
    """mock WS 推流, 验证 find_target_realtime 的 setpoint 偏移."""

    class _FakeWs:
        def __init__(self, frames):
            self.frames = list(frames)

        def subscribe_task_detection(self, on_state, hz=30.0):
            for i, dets in enumerate(self.frames):
                on_state({"data": {
                    "detections": [self._d(d) for d in dets],
                    "updated_at": float(i + 1),
                }})
            return MagicMock()

        @staticmethod
        def _d(d):
            return {
                "label": d.label, "score": d.score, "track_id": d.track_id,
                "cls_id": d.class_id,
                "bbox_norm": {"x_center": d.bbox_norm.x_center,
                              "y_center": d.bbox_norm.y_center,
                              "width": d.bbox_norm.width,
                              "height": d.bbox_norm.height},
            }

    @staticmethod
    def _det(cx, cy):
        return Detection(label="cylinder_3", score=0.9, track_id=0, class_id=1,
                         bbox_norm=BBoxNorm(cx, cy, 0.1, 0.1), bbox_pixels=None,
                         fetched_at=0.0)

    def test_converges_at_setpoint_via_ws(self):
        # 目标已在 setpoint → err=0 → 不移动直接收敛
        ws = self._FakeWs([[self._det(0.1, -0.6)]])
        http = MagicMock()
        vision = ArmVisionClient(http)
        move_log = []
        result = vision.find_target_realtime(
            TargetSelector.for_label("cylinder_3"),
            x_mm=0.0, y_mm=-150.0,
            setpoint_x_norm=0.1, setpoint_y_norm=-0.6,
            settle_tol_norm=0.05, timeout=2.0,
            ws=ws, move_fn=lambda x, y: move_log.append((x, y)) or {},
        )
        self.assertTrue(result.converged)
        self.assertEqual(len(move_log), 0)

    def test_moves_toward_setpoint_via_ws(self):
        # 目标在中心, setpoint=(0.1,-0.6) → err=(-0.1,+0.6) → dx_mm=+3.0, dy_mm=-18.0
        ws = self._FakeWs([[self._det(0.0, 0.0)]])
        http = MagicMock()
        vision = ArmVisionClient(http)
        move_log = []
        vision.find_target_realtime(
            TargetSelector.for_label("cylinder_3"),
            x_mm=0.0, y_mm=-150.0,
            mm_per_norm=30.0,
            setpoint_x_norm=0.1, setpoint_y_norm=-0.6,
            settle_tol_norm=0.05, timeout=2.0,
            ws=ws, move_fn=lambda x, y: move_log.append((x, y)) or {},
        )
        self.assertEqual(len(move_log), 1)
        self.assertAlmostEqual(move_log[0][0], 3.0, places=1)
        self.assertAlmostEqual(move_log[0][1], -168.0, places=1)


class TestRunnerSetpointInjection(unittest.TestCase):
    def _runner(self, origin=None):
        client = MagicMock()
        client.origin = origin or ArmOrigin()
        finder = client._make_vision_with_move.return_value
        finder.find_target.return_value = None
        finder.find_target_realtime.return_value = None
        finder.find_target_track.return_value = None
        client.composite_run.return_value = {"ok": True}
        return ArmRunner(client), client, finder

    def test_calibrated_origin_injects_setpoint(self):
        """origin 已标定 → move_to_vision_target 自动注入 setpoint."""
        runner, client, finder = self._runner(
            ArmOrigin(nozzle_offset_x_norm=0.08, nozzle_offset_y_norm=-0.63))
        runner.move_to_vision_target(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0)
        kw = finder.find_target.call_args.kwargs
        self.assertAlmostEqual(kw["setpoint_x_norm"], 0.08)
        self.assertAlmostEqual(kw["setpoint_y_norm"], -0.63)

    def test_uncalibrated_origin_no_injection(self):
        """origin 未标定(全 0) → 不注入 setpoint, 保持旧行为."""
        runner, client, finder = self._runner()
        runner.move_to_vision_target(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0)
        kw = finder.find_target.call_args.kwargs
        self.assertNotIn("setpoint_x_norm", kw)
        self.assertNotIn("setpoint_y_norm", kw)

    def test_explicit_zero_forces_center(self):
        """显式传 (0,0) → 注入 0,0 (强制对准画面中心)."""
        runner, client, finder = self._runner(
            ArmOrigin(nozzle_offset_x_norm=0.08, nozzle_offset_y_norm=-0.63))
        runner.move_to_vision_target(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0,
            setpoint_x_norm=0.0, setpoint_y_norm=0.0)
        kw = finder.find_target.call_args.kwargs
        self.assertAlmostEqual(kw["setpoint_x_norm"], 0.0)
        self.assertAlmostEqual(kw["setpoint_y_norm"], 0.0)

    def test_realtime_injects_setpoint(self):
        runner, client, finder = self._runner(
            ArmOrigin(nozzle_offset_x_norm=0.08, nozzle_offset_y_norm=-0.63))
        runner.move_to_vision_target_realtime(
            TargetSelector.for_label("cylinder_3"), x_mm=0.0, y_mm=-100.0)
        kw = finder.find_target_realtime.call_args.kwargs
        self.assertAlmostEqual(kw["setpoint_x_norm"], 0.08)
        self.assertAlmostEqual(kw["setpoint_y_norm"], -0.63)

    def test_per_label_map_beats_global_default(self):
        """nozzle_offset_map 命中 label → 用分组值而非全局默认."""
        runner, client, finder = self._runner(ArmOrigin(
            nozzle_offset_x_norm=0.101, nozzle_offset_y_norm=-0.519,
            nozzle_offset_map={"ball_yellow": (0.101, -0.704)},
        ))
        runner.move_to_vision_target(
            TargetSelector.for_label("ball_yellow"), x_mm=0.0, y_mm=-100.0)
        kw = finder.find_target.call_args.kwargs
        self.assertAlmostEqual(kw["setpoint_x_norm"], 0.101)
        self.assertAlmostEqual(kw["setpoint_y_norm"], -0.704)

    def test_unknown_label_falls_back_to_global_default(self):
        """map 无此 label → 回落全局默认 (nozzle_offset_x/y_norm)."""
        runner, client, finder = self._runner(ArmOrigin(
            nozzle_offset_x_norm=0.101, nozzle_offset_y_norm=-0.519,
            nozzle_offset_map={"ball_yellow": (0.101, -0.704)},
        ))
        runner.move_to_vision_target(
            TargetSelector.for_label("h_tu_dou"), x_mm=0.0, y_mm=-100.0)
        kw = finder.find_target.call_args.kwargs
        self.assertAlmostEqual(kw["setpoint_x_norm"], 0.101)
        self.assertAlmostEqual(kw["setpoint_y_norm"], -0.519)

    def test_map_entry_zero_ignored_falls_back(self):
        """map 中某 label 是 (0,0) → 视为未标定, 回落全局默认."""
        runner, client, finder = self._runner(ArmOrigin(
            nozzle_offset_x_norm=0.08, nozzle_offset_y_norm=-0.63,
            nozzle_offset_map={"cylinder_1": (0.0, 0.0)},
        ))
        runner.move_to_vision_target(
            TargetSelector.for_label("cylinder_1"), x_mm=0.0, y_mm=-100.0)
        kw = finder.find_target.call_args.kwargs
        self.assertAlmostEqual(kw["setpoint_x_norm"], 0.08)
        self.assertAlmostEqual(kw["setpoint_y_norm"], -0.63)


if __name__ == "__main__":
    unittest.main()

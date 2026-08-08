"""runtime/tests/test_chassis_align.py
Server-side 底盘视觉对齐控制器单测（stdlib unittest，离线无硬件）。

覆盖：
  - _expand_label_set：组展开 / 单 label / list
  - _select_target：nearest / largest / leftmost / no_match
  - ChassisAlignController：dry_run 不下发 / deadband arrived / slew 限幅
  - no_target：lost_frames > max_lost_frames
  - watchdog：age_ms 超阈值
  - timeout：elapsed_s > max_seconds
  - vx_only 只控前后
  - recover_after_lost 反向小搜
  - HTTP 端点集成（TestClient + fake service）
"""
import time
import unittest
from unittest.mock import MagicMock, patch

from runtime.services.chassis_align import (
    ChassisAlignController,
    TrackChassisResult,
    TrackFrame,
    _expand_label_set,
    _select_target,
)


# ===== _expand_label_set tests =====

class TestExpandLabelSet(unittest.TestCase):
    def test_water_group_expands(self):
        labels = _expand_label_set("water")
        self.assertIn("water", labels)
        self.assertIn("water_l1", labels)
        self.assertEqual(len(labels), 4)

    def test_cylinder_group_expands(self):
        labels = _expand_label_set("cylinder")
        self.assertIn("cylinder_1", labels)
        self.assertIn("cylinder_2", labels)
        self.assertIn("cylinder_3", labels)

    def test_specific_label_is_single(self):
        labels = _expand_label_set("cylinder_set")
        self.assertEqual(labels, {"cylinder_set"})

    def test_list_input(self):
        labels = _expand_label_set(["water", "cylinder_2"])
        self.assertIn("water", labels)
        self.assertIn("water_l1", labels)
        self.assertIn("cylinder_2", labels)


# ===== _select_target tests =====

def _d(label="h_tu_dou", cx=0.0, cy=0.0, w=0.2, h=0.2, score=0.9):
    return {
        "label": label,
        "score": score,
        "bbox_norm": {"cx": cx, "cy": cy, "width": w, "height": h},
    }


class TestSelectTarget(unittest.TestCase):
    # 默认 _d() label 是 "h_tu_dou"（与控制器默认 target 一致），选择逻辑
    # 只关心 label 是否在匹配集内，测试集用同一 label 即可。
    def test_nearest_to_center(self):
        dets = [_d(cx=0.5), _d(cx=0.1), _d(cx=0.2, label="other")]
        chosen = _select_target(dets, {"h_tu_dou"}, (0.0, 0.0), "nearest_to_center")
        self.assertIsNotNone(chosen)
        self.assertAlmostEqual(chosen["bbox_norm"]["cx"], 0.1)

    def test_largest_area(self):
        dets = [_d(w=0.1, h=0.1), _d(w=0.4, h=0.4), _d(w=0.2, h=0.2, label="x")]
        chosen = _select_target(dets, {"h_tu_dou"}, (0.0, 0.0), "largest_area")
        self.assertEqual(chosen["bbox_norm"]["width"], 0.4)

    def test_no_match_returns_none(self):
        self.assertIsNone(_select_target([_d(label="x")], {"h_tu_dou"}, (0.0, 0.0)))

    def test_leftmost(self):
        dets = [
            _d(label="ball_yellow", cx=0.3),
            _d(label="ball_blue", cx=-0.4),
            _d(label="other", cx=-0.9),
        ]
        chosen = _select_target(dets, {"ball_blue", "ball_yellow"}, (0.0, 0.0), "leftmost")
        self.assertEqual(chosen["label"], "ball_blue")

    def test_empty_detections(self):
        self.assertIsNone(_select_target([], {"water"}, (0.0, 0.0)))


# ===== ChassisAlignController unit tests =====

class FakeService:
    """Minimal fake service for controller unit tests."""

    def __init__(self, task_state=None):
        self.car = MagicMock()
        self.car.chassis.calculate_wheel_velocities.return_value = [0.0, 0.0, 0.0, 0.0]
        self._task_state = task_state
        self.set_chassis_velocity_calls = []
        self.set_wheel_speeds_calls = []

    def get_task_state(self):
        if self._task_state is not None:
            return self._task_state
        return {"active": True, "detections": [], "updated_at": time.time()}

    def set_chassis_velocity(self, vx, vy, wz):
        self.set_chassis_velocity_calls.append((vx, vy, wz))
        return {"vx": vx, "vy": vy, "wz": wz}

    def set_wheel_speeds(self, speeds):
        self.set_wheel_speeds_calls.append(speeds)


def _make_service_with_detections(dets, updated_at=None):
    if updated_at is None:
        updated_at = time.time()
    ts = {"active": True, "detections": dets, "updated_at": updated_at}
    return FakeService(task_state=ts)


def _make_controller(service, **kwargs):
    defaults = dict(
        target="h_tu_dou",
        setpoint_cxcy=(0.0, 0.0),
        select_mode="nearest_to_center",
        sign_vx=-1, sign_vy=1, vx_only=False,
        kp=0.20, v_max=0.12, deadband=0.05, hold_frames=5,
        v_slew=0.02, max_lost_frames=60, recover_after_lost=True,
        watchdog_ms=2000.0, hz=50.0, max_seconds=3.0, dry_run=False,
    )
    defaults.update(kwargs)
    return ChassisAlignController(service, **defaults)


def _fast_run(ctrl):
    """Run ChassisAlignController with mocked time so the loop executes instantly.

    Patches time.monotonic and time.sleep to make the 50Hz control loop run
    as fast as possible without hitting max_seconds.
    """
    t = [0.0]

    def fake_monotonic():
        return t[0]

    def fake_sleep(s):
        t[0] += s

    with patch("runtime.services.chassis_align.time.monotonic", fake_monotonic), \
         patch("runtime.services.chassis_align.time.sleep", fake_sleep):
        return ctrl.run()


class TestChassisAlignController(unittest.TestCase):
    def test_dry_run_never_calls_set_velocity(self):
        """dry_run=True 时 compute 但不下发。"""
        svc = _make_service_with_detections([_d(cx=0.3, cy=0.2)])
        ctrl = _make_controller(svc, dry_run=True, hz=50, max_seconds=3.0, max_lost_frames=200)
        result = _fast_run(ctrl)
        self.assertEqual(svc.set_chassis_velocity_calls, [])
        self.assertEqual(svc.set_wheel_speeds_calls, [])
        self.assertGreater(result["frames"], 0)

    def test_deadband_arrived(self):
        """目标持续在 deadband 内 → arrived。"""
        dets = [_d(cx=0.01, cy=0.01)] * 200
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, deadband=0.05, hold_frames=5, hz=50,
                                max_seconds=3.0, max_lost_frames=200)
        result = _fast_run(ctrl)
        self.assertTrue(result["arrived"])
        self.assertEqual(result["reason"], "arrived")

    def test_slew_limits_vx(self):
        """v_slew 限幅：连续帧 |dvx| <= v_slew。"""
        dets = [_d(cx=1.0, cy=0.0)] * 200
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, kp=1.0, v_max=0.12, v_slew=0.02,
                                deadband=0.05, hold_frames=100,
                                hz=50, max_seconds=3.0, max_lost_frames=200)
        result = _fast_run(ctrl)
        calls = svc.set_chassis_velocity_calls
        # 排除 finally 的急停帧 (0.12 → 0.0 是设计行为，非 slew 违规)
        track_calls = calls[:-1]
        self.assertTrue(len(track_calls) > 1,
                        f"Expected >1 tracking call, got {len(track_calls)}")
        for i in range(1, len(track_calls)):
            dvx = abs(track_calls[i][0] - track_calls[i - 1][0])
            self.assertLessEqual(dvx, 0.02 + 1e-9,
                                 f"Frame {i}: dvx={dvx} > v_slew=0.02")

    def test_v_max_clamps(self):
        """|vx|, |vy| 不超过 v_max。"""
        dets = [_d(cx=1.0, cy=1.0)] * 200
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, kp=1.0, v_max=0.12, v_slew=0.05,
                                deadband=10.0, hold_frames=100,
                                hz=50, max_seconds=3.0, max_lost_frames=200)
        result = _fast_run(ctrl)
        for vx, vy, _ in svc.set_chassis_velocity_calls:
            self.assertLessEqual(abs(vx), 0.12 + 1e-9)
            self.assertLessEqual(abs(vy), 0.12 + 1e-9)

    def test_no_target_lost_frames(self):
        """连续丢帧 > max_lost_frames → no_target。"""
        svc = FakeService()
        svc._task_state = {"active": True, "detections": [], "updated_at": time.time()}
        ctrl = _make_controller(svc, max_lost_frames=5, hz=50, max_seconds=10.0)
        result = _fast_run(ctrl)
        self.assertFalse(result["arrived"])
        self.assertEqual(result["reason"], "no_target")

    def test_watchdog_triggers(self):
        """age_ms > watchdog_ms → watchdog。"""
        now = time.time()
        old_ts = now - 5.0  # 5 秒前
        dets = [_d(label="h_tu_dou", cx=0.3, cy=0.0)]
        svc = _make_service_with_detections(dets, updated_at=old_ts)
        ctrl = _make_controller(svc, watchdog_ms=2000.0, hz=50,
                                max_seconds=5.0, max_lost_frames=200)
        result = _fast_run(ctrl)
        self.assertFalse(result["arrived"])
        self.assertEqual(result["reason"], "watchdog")
        self.assertEqual(result["frames"], 1)

    def test_timeout(self):
        """持续大误差 + 短 max_seconds → timeout。"""
        dets = [_d(cx=1.0, cy=1.0)] * 200
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, deadband=0.05, hold_frames=100,
                                hz=50, max_seconds=0.1, max_lost_frames=200)
        result = ctrl.run()  # timeout relies on real time
        self.assertFalse(result["arrived"])
        self.assertEqual(result["reason"], "timeout")

    def test_vx_only_vy_zero(self):
        """vx_only=True → vy 始终 0。"""
        dets = [_d(cx=0.3, cy=0.5)] * 200
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, vx_only=True, kp=0.20, v_slew=0.10,
                                deadband=10.0, hold_frames=100,
                                hz=50, max_seconds=3.0, max_lost_frames=200)
        result = _fast_run(ctrl)
        for _, vy, _ in svc.set_chassis_velocity_calls:
            self.assertEqual(vy, 0.0)

    def test_recover_after_lost_reverses(self):
        """第一帧有目标（产生非零 last_vx），第二帧丢帧 → 反向小搜。"""
        dets = [_d(cx=0.3)]  # frame 1: 有目标
        call_count = [0]
        svc = _make_service_with_detections(dets)
        original = svc.get_task_state

        def _alternating():
            call_count[0] += 1
            if call_count[0] == 1:
                return original()
            return {"active": True, "detections": [], "updated_at": time.time()}

        svc.get_task_state = _alternating
        # v_slew=None 关掉限幅，露出原始 P 控制律：
        #   frame1: vx = sign_vx * kp * cx_err = -1 * 0.2 * (-0.3) = +0.06
        #   frame2 (lost): recover = -last_vx * 0.5 = -0.03
        ctrl = _make_controller(svc, kp=0.20, recover_after_lost=True,
                                v_slew=None,
                                max_lost_frames=200, hz=50, max_seconds=3.0)
        result = _fast_run(ctrl)
        calls = svc.set_chassis_velocity_calls
        self.assertTrue(len(calls) >= 2,
                        f"Expected >=2 calls, got {len(calls)}: {calls}")
        self.assertAlmostEqual(calls[0][0], 0.06, places=4)
        self.assertAlmostEqual(calls[1][0], -0.03, places=4)

    def test_no_recover_after_lost_stops(self):
        """recover_after_lost=False → 丢帧后直接停。"""
        svc = _make_service_with_detections([], updated_at=time.time())
        svc._task_state = {"active": True, "detections": [], "updated_at": time.time()}
        ctrl = _make_controller(svc, recover_after_lost=False,
                                max_lost_frames=3, hz=50, max_seconds=5.0)
        result = _fast_run(ctrl)
        self.assertFalse(result["arrived"])

    def test_final_stop_called(self):
        """finally: _set_vel(0, 0) 总是下发（即使超时 / no_target）。"""
        svc = _make_service_with_detections([])
        svc._task_state = {"active": True, "detections": [], "updated_at": time.time()}
        ctrl = _make_controller(svc, max_lost_frames=3, hz=50, max_seconds=2.0)
        result = _fast_run(ctrl)
        last_call = svc.set_chassis_velocity_calls[-1]
        self.assertEqual(last_call[0], 0.0)
        self.assertEqual(last_call[1], 0.0)

    def test_empty_labels_no_target(self):
        """label 不在任何已知组且无匹配 detections → no_target。"""
        svc = _make_service_with_detections([])
        ctrl = ChassisAlignController(svc, target="nonexistent_group",
                                      hz=50, max_seconds=3.0)
        result = _fast_run(ctrl)
        self.assertEqual(result["reason"], "no_target")
        for vx, vy, _ in svc.set_chassis_velocity_calls:
            self.assertEqual(vx, 0.0)
            self.assertEqual(vy, 0.0)

    def test_frames_counted(self):
        """frames 字段正确计数。"""
        dets = [_d(cx=0.3)] * 10
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, max_lost_frames=200, hz=10,
                                max_seconds=3.0)
        result = _fast_run(ctrl)
        self.assertGreater(result["frames"], 0)
        self.assertIsInstance(result["elapsed_s"], float)
        self.assertGreater(result["elapsed_s"], 0.0)

    def test_sign_vx_sign_vy_direction(self):
        """sign_vx=-1 / cy_err>0 → vx 为负（后退）。"""
        dets = [_d(cx=0.3, cy=0.0)] * 200
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, kp=1.0, v_slew=0.10,
                                deadband=10.0, hold_frames=100,
                                hz=50, max_seconds=3.0, max_lost_frames=200)
        result = _fast_run(ctrl)
        vx_values = [c[0] for c in svc.set_chassis_velocity_calls]
        self.assertTrue(all(v <= 0 for v in vx_values),
                        f"All vx should be <= 0 with sign_vx=-1, got {vx_values}")


# ===== Kalman 平滑（filterpy，2026-08-09）=====


class TestKalmanTracker(unittest.TestCase):
    """_KalmanTracker 封装 filterpy 常速 Kalman 的纯函数测试。"""

    def _tracker(self, **kw):
        from runtime.services.chassis_align import _KalmanTracker
        return _KalmanTracker(**kw)

    def test_first_frame_init(self):
        """首帧直接初始化, 返回原始值。"""
        tr = self._tracker()
        cx, cy = tr.update(0.3, 0.2)
        self.assertAlmostEqual(cx, 0.3)
        self.assertAlmostEqual(cy, 0.2)

    def test_step_response_lags(self):
        """阶跃响应滞后 = 平滑: 目标从 0.3 跳到 0.5, 输出应在两者之间。"""
        tr = self._tracker()
        tr.update(0.3, 0.0)          # init
        cx, _ = tr.update(0.5, 0.0)  # 阶跃
        self.assertGreater(cx, 0.3)
        self.assertLess(cx, 0.5)

    def test_converges_to_constant(self):
        """恒定测量反复喂 → 收敛到测量值。"""
        tr = self._tracker()
        cx, _ = 0.0, 0.0
        for _ in range(100):
            cx, _ = tr.update(0.3, 0.1)
        self.assertAlmostEqual(cx, 0.3, places=3)

    def test_jitter_smoothed(self):
        """抖动测量 (0.4±0.1 交替) → 输出变化量 < 原始抖动。"""
        tr = self._tracker()
        raw = [0.3, 0.5] * 20
        tr.update(raw[0], 0.0)
        out = []
        for i in range(1, len(raw)):
            cx, _ = tr.update(raw[i], 0.0)
            out.append(cx)
        max_delta_out = max(abs(out[i] - out[i - 1]) for i in range(1, len(out)))
        self.assertLess(max_delta_out, 0.2)  # < 原始 ±0.1 抖动的最大跳变


class TestKalmanController(unittest.TestCase):
    def test_disabled_by_default(self):
        """kalman=False（默认）→ 不建 tracker, 保持已验证行为。"""
        svc = _make_service_with_detections([_d(cx=0.3)] * 10)
        ctrl = _make_controller(svc)
        self.assertIsNone(ctrl._kalman)

    def test_enabled_instantiates_tracker(self):
        """kalman=True → tracker 实例化。"""
        svc = _make_service_with_detections([_d(cx=0.3)] * 10)
        ctrl = _make_controller(svc, kalman=True)
        self.assertIsNotNone(ctrl._kalman)

    def test_tracker_updated_per_found_frame(self):
        """有检测帧每帧喂 tracker（丢帧帧不喂）。"""
        dets = [_d(cx=0.3)] * 100
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, kalman=True, deadband=10.0,
                                hold_frames=100, hz=50, max_seconds=1.0,
                                max_lost_frames=200)
        real_update = ctrl._kalman.update
        calls = []

        def _spy(cx, cy):
            calls.append((cx, cy))
            return real_update(cx, cy)

        ctrl._kalman.update = _spy
        _fast_run(ctrl)
        self.assertGreater(len(calls), 0)
        self.assertEqual(calls[0][0], 0.3)

    def test_import_fail_disables_gracefully(self):
        """filterpy 未装（ImportError）→ kalman 自动禁用, 闭环照常跑。"""
        svc = _make_service_with_detections([_d(cx=0.3)] * 10)
        with patch("runtime.services.chassis_align._KalmanTracker",
                   side_effect=ImportError("no filterpy")):
            ctrl = _make_controller(svc, kalman=True)
        self.assertIsNone(ctrl._kalman)
        # 降级后闭环仍正常工作
        result = _fast_run(ctrl)
        self.assertEqual(result["reason"], "timeout")


# ===== HTTP endpoint integration test =====


class TestChassisAlignEndpoint(unittest.TestCase):
    """用 FastAPI TestClient 测 POST /v1/realtime/chassis-align 端点。"""

    def _build_app(self):
        try:
            from fastapi.testclient import TestClient
        except ModuleNotFoundError:
            self.skipTest("fastapi 未安装，跳过集成测试")
        from runtime.api.routers.realtime import build_realtime_router
        from runtime.services.chassis_align import ChassisAlignController

        # 伪造 service
        fake_svc = MagicMock()
        fake_svc.car = MagicMock()
        fake_svc._chassis_align_lock = MagicMock(
            __enter__=MagicMock(return_value=None),
            __exit__=MagicMock(return_value=False),
        )
        fake_svc.set_chassis_velocity = MagicMock()

        controller_instance = MagicMock()
        controller_instance.run.return_value = {
            "arrived": True, "reason": "arrived",
            "frames": 10, "elapsed_s": 0.5,
            "final_frame": {"target_found": True, "label": "h_tu_dou",
                            "cx": 0.01, "cy": -0.01, "cx_err": -0.01, "cy_err": 0.01},
        }

        import runtime.services.chassis_align as ca_mod
        original = ca_mod.ChassisAlignController
        ca_mod.ChassisAlignController = lambda *a, **kw: controller_instance
        try:
            router = build_realtime_router(fake_svc)
            from fastapi import FastAPI
            app = FastAPI()
            app.include_router(router)
            return TestClient(app), fake_svc, controller_instance, ca_mod, original
        finally:
            ca_mod.ChassisAlignController = original

    def test_endpoint_returns_result(self):
        client, svc, ctrl, mod, orig = self._build_app()
        try:
            resp = client.post(
                "/api/v1/realtime/chassis-align",
                json={"target": "h_tu_dou", "kp": 0.2, "v_max": 0.12},
            )
            self.assertEqual(resp.status_code, 200)
            data = resp.json()
            self.assertTrue(data.get("ok"))
            self.assertEqual(data["result"]["reason"], "arrived")
        finally:
            pass  # cleanup handled by mock context

    def test_endpoint_503_when_car_none(self):
        client, svc, ctrl, mod, orig = self._build_app()
        try:
            svc.car = None
            resp = client.post(
                "/api/v1/realtime/chassis-align",
                json={"target": "h_tu_dou"},
            )
            self.assertEqual(resp.status_code, 503)
        finally:
            pass


if __name__ == "__main__":
    unittest.main()

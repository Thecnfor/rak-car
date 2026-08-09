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
        """丢帧回拉: 连丢 2 帧才反向 (0.25 倍), 单帧闪烁只停不反向 (2026-08-09 治来回晃)."""
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
        #   frame2 (lost #1): 闪烁只停 → 0.0
        #   frame3 (lost #2): recover = -last_vx * 0.25 = -0.015
        ctrl = _make_controller(svc, kp=0.20, recover_after_lost=True,
                                v_slew=None,
                                max_lost_frames=200, hz=50, max_seconds=3.0)
        result = _fast_run(ctrl)
        calls = svc.set_chassis_velocity_calls
        self.assertTrue(len(calls) >= 3,
                        f"Expected >=3 calls, got {len(calls)}: {calls}")
        self.assertAlmostEqual(calls[0][0], 0.06, places=4)
        self.assertAlmostEqual(calls[1][0], 0.0, places=4)
        self.assertAlmostEqual(calls[2][0], -0.015, places=4)

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

    def test_stop_ok_true_normal(self):
        """正常闭环 → finally 零速成功 → stop_ok=True 且最后一条确实是零速。"""
        dets = [_d(cx=0.3)] * 10
        svc = _make_service_with_detections(dets)
        ctrl = _make_controller(svc, max_lost_frames=200, hz=50, max_seconds=3.0)
        result = _fast_run(ctrl)
        self.assertIn("stop_ok", result)
        self.assertTrue(result["stop_ok"])
        self.assertEqual(svc.set_chassis_velocity_calls[-1][:2], (0.0, 0.0))

    def test_stop_ok_false_when_velocity_write_fails(self):
        """主路径 + 兜底都失败 → finally 零速未达 → stop_ok=False (客户端可感知)."""
        dets = [_d(cx=0.3)] * 10
        svc = _make_service_with_detections(dets)

        def _boom(*a, **k):
            raise RuntimeError("serial down")

        svc.set_chassis_velocity = _boom
        svc.set_wheel_speeds = _boom
        ctrl = _make_controller(svc, max_lost_frames=200, hz=50, max_seconds=3.0)
        result = _fast_run(ctrl)
        self.assertFalse(result["stop_ok"])

    def test_control_lost_early_exit_on_write_fail(self):
        """命令路径连续失败 → 快速 control_lost 退出, 不烧满 max_seconds.

        串口/下位机掉线但视觉仍活 (task_feed 独立于 MC602) 时, 旧行为会满预算
        timeout (每球白烧 12s); 现在默认 10 帧 ≈ 0.5s 内快速失败, 任务层可立即重武装。
        """
        dets = [_d(cx=0.3)] * 100
        svc = _make_service_with_detections(dets)

        def _boom(*a, **k):
            raise RuntimeError("serial down")

        svc.set_chassis_velocity = _boom
        svc.set_wheel_speeds = _boom
        ctrl = _make_controller(svc, max_lost_frames=200, hz=50,
                                max_seconds=10.0, max_control_fail_frames=5)
        result = _fast_run(ctrl)
        self.assertEqual(result["reason"], "control_lost")
        self.assertEqual(result["frames"], 5)       # 没烧满 10s, 第 5 帧就退出
        self.assertFalse(result["stop_ok"])

    def test_control_lost_recovers_on_success(self):
        """中间成功一帧 → 连续失败计数清零, 不误触 control_lost。"""
        dets = [_d(cx=0.3)] * 100
        svc = _make_service_with_detections(dets)
        call_n = [0]
        real_set = svc.set_chassis_velocity

        def _flaky(*a, **k):
            call_n[0] += 1
            if call_n[0] <= 4:  # 前 4 帧失败, 之后恢复
                raise RuntimeError("serial busy")
            return real_set(*a, **k)

        svc.set_chassis_velocity = _flaky
        ctrl = _make_controller(svc, max_lost_frames=200, hz=50,
                                max_seconds=10.0, max_control_fail_frames=5)
        result = _fast_run(ctrl)
        # 前 4 帧失败 (streak=4 < 5), 第 5 帧成功 → streak 清零 → 不触 control_lost,
        # 最后因 deadband 收敛或 timeout 正常结束
        self.assertNotEqual(result["reason"], "control_lost")

    def test_motion_ok_false_when_commanded_but_encoders_static(self):
        """下发非零命令但编码器没动 (200 但轮不转) → motion_ok=False。"""
        dets = [_d(cx=0.5)] * 100
        svc = _make_service_with_detections(dets)
        svc.get_wheel_encoders = lambda: [0.0, 0.0, 0.0, 0.0]
        ctrl = _make_controller(svc, max_lost_frames=200, hz=50,
                                max_seconds=3.0, max_control_fail_frames=200)
        result = _fast_run(ctrl)
        self.assertFalse(result["motion_ok"])
        self.assertAlmostEqual(result["enc_delta"], 0.0, places=6)

    def test_motion_ok_true_when_wheels_moved(self):
        """编码器位移 ≥ 阈值 → motion_ok=True。"""
        dets = [_d(cx=0.5)] * 100
        svc = _make_service_with_detections(dets)
        calls = [0]

        def _read():
            calls[0] += 1
            return [5.0] * 4 if calls[0] > 1 else [0.0] * 4

        svc.get_wheel_encoders = _read
        ctrl = _make_controller(svc, max_lost_frames=200, hz=50,
                                max_seconds=3.0, max_control_fail_frames=200)
        result = _fast_run(ctrl)
        self.assertTrue(result["motion_ok"])
        self.assertGreaterEqual(result["enc_delta"], 20.0)  # 4 轮 × 5.0

    def test_motion_ok_true_when_no_command(self):
        """目标已居中 (命令 0) → 无需位移 → motion_ok=True。"""
        dets = [_d(cx=0.01, cy=0.01)] * 20
        svc = _make_service_with_detections(dets)
        svc.get_wheel_encoders = lambda: [0.0, 0.0, 0.0, 0.0]
        ctrl = _make_controller(svc, max_lost_frames=200, hz=50,
                                max_seconds=3.0, max_control_fail_frames=200)
        result = _fast_run(ctrl)
        self.assertTrue(result["motion_ok"])

    def test_decouple_xy_drives_single_axis(self):
        """decouple_xy 默认开: 每帧只驱动误差较大的单轴 (另一轴 0) → 4 轮一起平移。

        cx=0.4, cy=0.2 → |cx|>|cy| → 只动 vx, vy 恒 0; 反之只动 vy。
        """
        # x 误差更大 → vx 动, vy 0
        svc = _make_service_with_detections([_d(cx=0.4, cy=0.2)] * 50)
        ctrl = _make_controller(svc, kp=0.20, v_slew=0.10,
                                deadband=0.01, hold_frames=100,
                                hz=50, max_seconds=3.0, max_lost_frames=200)
        _fast_run(ctrl)
        x_only = [(vx, vy) for vx, vy, _ in svc.set_chassis_velocity_calls[:-1]]
        self.assertTrue(any(abs(vx) > 1e-6 for vx, _ in x_only))
        self.assertTrue(all(abs(vy) < 1e-9 for _, vy in x_only),
                        f"decouple_xy: vy 应恒 0 (x 误差更大), got {x_only[:3]}")

        # y 误差更大 → vy 动, vx 0
        svc2 = _make_service_with_detections([_d(cx=0.2, cy=0.4)] * 50)
        ctrl2 = _make_controller(svc2, kp=0.20, v_slew=0.10,
                                 deadband=0.01, hold_frames=100,
                                 hz=50, max_seconds=3.0, max_lost_frames=200)
        _fast_run(ctrl2)
        y_only = [(vx, vy) for vx, vy, _ in svc2.set_chassis_velocity_calls[:-1]]
        self.assertTrue(any(abs(vy) > 1e-6 for _, vy in y_only))
        self.assertTrue(all(abs(vx) < 1e-9 for vx, _ in y_only),
                        f"decouple_xy: vx 应恒 0 (y 误差更大), got {y_only[:3]}")

    def test_decouple_xy_false_keeps_diagonal(self):
        """decouple_xy=False → 保留对角平移 (vx, vy 同时非零)。"""
        svc = _make_service_with_detections([_d(cx=0.4, cy=0.2)] * 50)
        ctrl = _make_controller(svc, kp=0.20, v_slew=0.10,
                                decouple_xy=False,
                                deadband=0.01, hold_frames=100,
                                hz=50, max_seconds=3.0, max_lost_frames=200)
        _fast_run(ctrl)
        diag = [(vx, vy) for vx, vy, _ in svc.set_chassis_velocity_calls[:-1]]
        self.assertTrue(any(abs(vx) > 1e-6 and abs(vy) > 1e-6 for vx, vy in diag),
                        f"decouple_xy=False 应同时动两轴, got {diag[:3]}")

    def test_decouple_xy_axis_hysteresis_locks(self):
        """轴滞回: |cx|≈|cy| (每帧互换大小但都在 1.2x 内) → 轴锁定不换, 治对角来回晃。"""
        # 每帧 cx/cy 互换大小: 0.25↔0.22。无滞回会每帧 x↔y 交替。
        altern = [_d(cx=0.25, cy=0.22), _d(cx=0.22, cy=0.25)] * 50
        svc = _make_service_with_detections(altern)
        ctrl = _make_controller(svc, kp=0.20, v_slew=0.10,
                                deadband=0.01, hold_frames=100,
                                hz=50, max_seconds=3.0, max_lost_frames=200)
        _fast_run(ctrl)
        calls = [(vx, vy) for vx, vy, _ in svc.set_chassis_velocity_calls[:-1]]
        driven_x = [c for c in calls if abs(c[0]) > 1e-6]
        driven_y = [c for c in calls if abs(c[1]) > 1e-6]
        # 首次选中 x 后, 1.2x 阈值内不切 y → 只能有单轴持续驱动。
        self.assertTrue(driven_x and not driven_y,
                        f"轴滞回应锁定单轴 (x), got x={driven_x[:3]} y={driven_y[:3]}")


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
    def test_enabled_by_default(self):
        """kalman 默认开 (2026-08-09 用户决定) → tracker 实例化。"""
        svc = _make_service_with_detections([_d(cx=0.3)] * 10)
        ctrl = _make_controller(svc)
        self.assertIsNotNone(ctrl._kalman)

    def test_disabled_when_false(self):
        """显式 kalman=False → 不建 tracker, 降级原始检测。"""
        svc = _make_service_with_detections([_d(cx=0.3)] * 10)
        ctrl = _make_controller(svc, kalman=False)
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

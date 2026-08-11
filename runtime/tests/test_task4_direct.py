"""task4_direct 单元测试 —— 进程内直连新版流程 (2026-08-11)。

用 FakeCar 模拟 MyCar / SDK 直连面 (set_chassis_velocity / move_for /
arm.composite_run / arm.grasp / run_arm_servo / streamer.task_state ...),
不触任何硬件。覆盖:
  - dry_run 不构造硬件
  - 找球过滤 + 颜色映射 (_fetch_balls)
  - 左 IR 离区判定 (_ir_far)
  - 业务 mm 位姿 → SDK 米制换算 (_composite_m)
  - 全流程: creep 见球 → 底盘对齐 → 臂伺服 → 抓放 → 前进 → 预算退出
"""
import time
import unittest

from runtime.tasks.task4_direct import (
    Task4Direct,
    run,
    _composite_m,
    _fetch_balls,
    _ir_far,
    _label_to_color,
    COLOR_BLUE,
    COLOR_YELLOW,
)


class FakeStreamer:
    """task_feed 缓存: get_task_state() 返回 flat dict (active/detections/updated_at)。"""

    def __init__(self, detections):
        self._dets = detections

    def get_task_state(self):
        return {
            "active": True,
            "detections": self._dets,
            "updated_at": time.time(),
        }


def _ball_det(label="ball_blue", cx=0.0, cy=0.0, score=0.9, w=0.4, h=0.5):
    """task_state detection dict (与 runtime task_feed 输出格式一致)。"""
    return {
        "cls_id": 18 if "blue" in label else 17,
        "det_id": 0,
        "label": label,
        "score": score,
        "bbox_norm": {"x_center": cx, "y_center": cy, "width": w, "height": h},
    }


class FakeArm:
    def __init__(self):
        self.calls = []

    def composite_run(self, **kwargs):
        self.calls.append(("composite", kwargs))
        return {"ok": True}

    def grasp(self, value):
        self.calls.append(("grasp", value))


class FakeCar:
    """新版 task4 直连面 (与 runtime/services/my_car 各 mixin 同签名)。"""

    def __init__(self, detections=None, ir=None):
        self.streamer = FakeStreamer(detections or [])
        self.ir = ir or {"left": 0.4, "right": 0.4}
        self.x = 0.0
        self.moves = []          # move_for / lane_dis_offset
        self.vels = []           # set_chassis_velocity
        self.arm = FakeArm()
        self.servo_results = {"settled": True, "reason": "settled",
                              "trace_hits": 3, "end_arm": 90.0}
        self.storage_angles = []  # set_storage_angle
        self.feed_events = []     # start/stop arm_feed / task_feed
        self._stop_flag = False

    # ---- 底盘 (HardwareIO / MotionMixin / MecanumDriver) ----
    def get_odometry(self):
        return [self.x, 0.0, 0.0]

    def get_all_ir_distance(self):
        return self.ir

    def set_chassis_velocity(self, vx, vy=0.0, wz=0.0):
        self.vels.append((float(vx), float(vy), float(wz)))

    def move_for(self, offset, **kwargs):
        self.moves.append(("move_for", list(offset), kwargs))
        self.x += float(offset[0])

    def lane_dis_offset(self, speed, dis_hold, **kwargs):
        self.moves.append(("lane", float(speed), float(dis_hold), kwargs))

    def get_wheel_encoders(self):
        return [0.0, 0.0, 0.0, 0.0]

    def stop(self):
        self.moves.append(("stop", [0.0, 0.0, 0.0]))

    # ---- 臂 (StateMixin / ArmServoMixin) ----
    def get_arm_state(self):
        # P 姿态 (米制): x=-250mm y=-150mm arm=90° hand=-10°
        return {"x": -0.25, "y": -0.15, "arm_angle": 90.0, "hand_angle": -10.0}

    def run_arm_servo(self, **kw):
        return dict(self.servo_results)

    # ---- 舵机 / feed (SensorsMixin / FeedsMixin) ----
    def set_storage_angle(self, angle, speed=100):
        self.storage_angles.append((angle, speed))

    def start_task_feed(self, hz=30.0):
        self.feed_events.append(("task_feed", "start", hz))

    def stop_arm_feed(self, force=False):
        self.feed_events.append(("arm_feed", "stop", force))

    def start_arm_feed(self, hz=20.0):
        self.feed_events.append(("arm_feed", "start", hz))

    # ---- 协作取消 ----
    def _must_exit(self):
        return self._stop_flag


class TestTask4DirectFlow(unittest.TestCase):

    def test_dry_run_returns_without_hardware(self):
        result = run(dry_run=True)
        self.assertEqual(result["reason"], "dry_run")
        self.assertEqual(result["picked"], 0)

    def test_ir_far_detects_left_exit(self):
        car_near = FakeCar(ir={"left": 0.4, "right": 0.4})
        self.assertFalse(_ir_far(car_near))
        car_far = FakeCar(ir={"left": 0.9, "right": 0.4})
        self.assertTrue(_ir_far(car_far))
        car_none = FakeCar(ir={"left": "---", "right": "---"})
        self.assertFalse(_ir_far(car_none))

    def test_fetch_balls_filters_and_maps_color(self):
        car = FakeCar(detections=[
            _ball_det("ball_blue", cx=0.1, cy=-0.2, score=0.9),
            _ball_det("person", cx=0.0, cy=0.0, score=0.99),  # 非球 label → unknown
            _ball_det("ball_yellow", cx=0.3, cy=0.1, score=0.1),  # score 过低
        ])
        balls = _fetch_balls(car)
        # unknown 色不过滤 (真实语义); 低分 yellow (0.1) 被丢
        self.assertEqual(len(balls), 2)
        self.assertEqual(balls[0]["color"], COLOR_BLUE)
        self.assertAlmostEqual(balls[0]["cx_norm"], 0.1)

    def test_label_to_color(self):
        self.assertEqual(_label_to_color("ball_blue", None), COLOR_BLUE)
        self.assertEqual(_label_to_color("Ball_Yellow", None), COLOR_YELLOW)
        self.assertEqual(_label_to_color(None, 18), COLOR_BLUE)
        self.assertEqual(_label_to_color("water", None), "unknown")

    def test_composite_m_converts_mm_to_m(self):
        car = FakeCar()
        _composite_m(car, arm=90.0, x_mm=-250.0, y_mm=-150.0, hand=-10.0,
                     speed=80, timeout=30.0)
        composite_call = [c for c in car.arm.calls if c[0] == "composite"]
        self.assertEqual(len(composite_call), 1)
        kw = composite_call[0][1]
        self.assertAlmostEqual(kw["x"], -0.25)
        self.assertAlmostEqual(kw["y"], -0.15)
        self.assertEqual(kw["arm"], 90.0)
        self.assertEqual(kw["speed"], 80)

    def test_flow_picks_at_least_one_ball(self):
        car = FakeCar(detections=[_ball_det("ball_blue", cx=0.0, cy=0.0)])
        result = Task4Direct(car, max_seconds=1.5).run()

        self.assertTrue(result["ok"])
        self.assertGreaterEqual(result["picked"], 1)
        self.assertIn(result["reason"], ("time_budget", "completed", "zone_cleared"))

        # 抓放序列: grasp(True) 之后必跟 grasp(False)
        grasps = [c for c in car.arm.calls if c[0] == "grasp"]
        self.assertGreaterEqual(len(grasps), 2)
        self.assertTrue(grasps[0][1])
        self.assertFalse(grasps[-1][1])

        # creep 下发过非零底盘速度
        self.assertTrue(any(v[0] > 0 for v in car.vels))
        # 后续球 move_for 前进
        self.assertTrue(any(k == "move_for" and m[0][0] > 0 for k, *m in car.moves))
        # 开仓 75 + 关仓 98
        self.assertIn(75, [a for a, _ in car.storage_angles])
        self.assertIn(98, [a for a, _ in car.storage_angles])
        # arm_feed 任务前停、收尾恢复
        events = car.feed_events
        self.assertIn(("arm_feed", "stop", True), events)
        self.assertIn(("arm_feed", "start", 20.0), events)

    def test_flow_ends_when_left_ir_far(self):
        car = FakeCar(detections=[_ball_det("ball_yellow", cx=0.0, cy=0.0)],
                      ir={"left": 0.9, "right": 0.4})
        result = Task4Direct(car, max_seconds=10.0).run()
        self.assertEqual(result["reason"], "zone_cleared")
        self.assertGreaterEqual(result["picked"], 1)

    def test_flow_respects_cooperative_stop(self):
        car = FakeCar(detections=[_ball_det("ball_blue", cx=0.0, cy=0.0)])

        def _stop_soon():
            time.sleep(0.3)
            car._stop_flag = True

        import threading
        threading.Thread(target=_stop_soon, daemon=True).start()
        result = Task4Direct(car, max_seconds=10.0).run()
        self.assertEqual(result["reason"], "stopped")


if __name__ == "__main__":
    unittest.main()

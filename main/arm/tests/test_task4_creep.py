import unittest

from main.arm.each_task.task4.target4 import _CreepThread


def _ball_detection(label="ball_blue", score=0.8):
    """一条通过 creep 松阈值过滤的球 detection (score≥0.35 / aspect≈1 / area∈[0.03,0.90])。"""
    return {
        "label": label, "cls_id": None, "score": score,
        "bbox_norm": {"cx": 0.1, "cy": -0.2, "w": 0.3, "h": 0.3},
    }


class _FakeHttp:
    def __init__(self, odom_values=None, detections=None, raise_on_ir=False):
        # odom_values 首元素是 start() 抓的基线, 之后才是 loop 每次读数
        self.odom_values = list(odom_values) if odom_values is not None else [0.0]
        self.odom_index = 0
        self.detections = detections or []
        self.raise_on_ir = raise_on_ir
        self.posts = []

    def post(self, path, payload=None, timeout=None):
        self.posts.append((path, payload))
        return {"ok": True}

    def get_odom_state(self):
        value = self.odom_values[min(self.odom_index, len(self.odom_values) - 1)]
        self.odom_index += 1
        return {"odom_state": {"x": value}}

    def get_task_state(self):
        return {"task_state": {"active": True, "detections": self.detections}}

    def get_ir_state(self):
        if self.raise_on_ir:
            raise AssertionError("2026-08-10 删除了 IR 生命周期, creep 不应再读 IR")
        return {"ir_state": {"left": 0.3}}


class TestTask4Creep(unittest.TestCase):
    def test_stops_at_distance_budget(self):
        """里程计累计到 max_distance_m → 停, 无球 → 上层判 zone_cleared。"""
        http = _FakeHttp(odom_values=[0.0, 0.0, 0.05, 0.11], detections=[])
        creep = _CreepThread(http, speed_mps=0.05, max_distance_m=0.10, poll_hz=100.0)
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=1.0)
            self.assertIsNone(result["balls"])
            self.assertTrue(result["distance_exhausted"])
            self.assertGreaterEqual(result["distance_m"], 0.10)
        finally:
            creep.stop_and_join()

    def test_stops_on_ball(self):
        """搜索阶段见球即停, balls 返回非空。"""
        http = _FakeHttp(odom_values=[0.0], detections=[_ball_detection()])
        creep = _CreepThread(http, speed_mps=0.05, max_distance_m=999.0, poll_hz=100.0)
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=1.0)
            self.assertIsNotNone(result["balls"])
            self.assertTrue(any(b["color"] == "blue" for b in result["balls"]))
            self.assertFalse(result["distance_exhausted"])
        finally:
            creep.stop_and_join()

    def test_no_open_loop_when_odom_frozen(self):
        """里程计卡死不再按速度×时间外推 —— distance_m 必须保持 0 (2026-08-10 删开环回退)。"""
        http = _FakeHttp(odom_values=[0.0, 0.0, 0.0, 0.0, 0.0], detections=[])
        creep = _CreepThread(http, speed_mps=2.0, max_distance_m=999.0, poll_hz=100.0)
        creep.start()
        try:
            creep.wait_for_ball(timeout_s=0.2)
            self.assertEqual(creep.distance_m, 0.0)  # 大速度也不外推
        finally:
            creep.stop_and_join()

    def test_ir_not_read_anymore(self):
        """IR 生命周期已删 —— 若 creep 还读 get_ir_state 会抛 AssertionError。"""
        http = _FakeHttp(odom_values=[0.0], detections=[], raise_on_ir=True)
        creep = _CreepThread(http, speed_mps=0.05, max_distance_m=999.0, poll_hz=100.0)
        creep.start()
        try:
            creep.wait_for_ball(timeout_s=0.2)
        finally:
            creep.stop_and_join()

    def test_ball_wins_over_budget(self):
        """预算边界与球同帧出现时, 见球优先 (found_ball=True)。"""
        http = _FakeHttp(odom_values=[0.0, 0.20], detections=[_ball_detection()])
        creep = _CreepThread(http, speed_mps=0.05, max_distance_m=0.10, poll_hz=100.0)
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=1.0)
            self.assertIsNotNone(result["balls"])
        finally:
            creep.stop_and_join()


if __name__ == "__main__":
    unittest.main()

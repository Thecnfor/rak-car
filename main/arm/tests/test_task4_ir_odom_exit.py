import unittest

from main.arm.each_task.task4.target4 import _CreepThread, _Task4SearchState


class _FakeHttp:
    def __init__(self, ir_left=0.4, odom_x=0.0):
        self.ir_left = ir_left
        self.odom_x = odom_x
        self.posts = []

    def post(self, path, payload=None, timeout=None):
        self.posts.append((path, payload, timeout))
        return {"ok": True}

    def get_odom_state(self):
        return {"odom_state": {"x": self.odom_x}}

    def get_ir_state(self):
        return {"ir_state": {"left": self.ir_left}}

    def get_task_state(self):
        return {"task_state": {"active": True, "detections": []}}


class _SequenceHttp(_FakeHttp):
    def __init__(self, ir_values, odom_x):
        super().__init__()
        self.ir_values = list(ir_values)
        self.odom_values = list(odom_x)
        self.ir_index = 0
        self.odom_index = 0

    def get_ir_state(self):
        value = self.ir_values[min(self.ir_index, len(self.ir_values) - 1)]
        self.ir_index += 1
        return {"ir_state": {"left": value}}

    def get_odom_state(self):
        value = self.odom_values[min(self.odom_index, len(self.odom_values) - 1)]
        self.odom_index += 1
        return {"odom_state": {"x": value}}


class TestTask4IrOdomExit(unittest.TestCase):
    def test_far_ir_transition_starts_post_loss_creep(self):
        http = _SequenceHttp(
            ir_values=[0.5, 0.8, 0.8],
            odom_x=[0.0, 0.0, 0.16, 0.31],
        )
        state = _Task4SearchState(ir_started=True)
        creep = _CreepThread(
            http, state=state, speed_mps=0.12,
            max_distance_m=999.0, poll_hz=100.0,
        )
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=1.0)
            self.assertTrue(result["finished_by_ir_odom"])
            self.assertTrue(state.ir_lost)
            self.assertGreaterEqual(state.post_loss_distance_m, 0.30)
        finally:
            creep.stop_and_join()

    def test_one_far_sample_does_not_latch_ir_loss(self):
        state = _Task4SearchState(ir_started=True)
        self.assertFalse(state.update_ir(0.8))
        self.assertFalse(state.update_ir(0.5))
        self.assertFalse(state.ir_lost)

    def test_ir_loss_state_survives_new_creep_worker(self):
        state = _Task4SearchState(ir_started=True)
        self.assertFalse(state.update_ir(0.8))
        self.assertTrue(state.update_ir(0.8))
        self.assertTrue(state.ir_lost)
        next_creep = _CreepThread(
            _FakeHttp(ir_left=0.5), state=state,
            speed_mps=0.12, max_distance_m=999.0,
        )
        self.assertIs(next_creep.state, state)
        self.assertTrue(next_creep.state.ir_lost)

    def test_near_ir_does_not_finish_by_distance_budget(self):
        http = _FakeHttp(ir_left=0.4)
        state = _Task4SearchState(ir_started=True)
        creep = _CreepThread(
            http, state=state, speed_mps=0.12,
            max_distance_m=0.02, max_seconds_s=0.1,
            poll_hz=100.0,
        )
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=0.5)
            self.assertFalse(result["finished_by_ir_odom"])
            self.assertFalse(state.ir_lost)
        finally:
            creep.stop_and_join()

    def test_ir_loss_uses_open_loop_when_odom_frozen(self):
        http = _SequenceHttp(ir_values=[0.5, 0.8, 0.8], odom_x=[0.0])
        state = _Task4SearchState(ir_started=True)
        creep = _CreepThread(
            http, state=state, speed_mps=2.0,
            max_distance_m=999.0, max_seconds_s=1.0,
            poll_hz=100.0,
        )
        creep.start()
        try:
            result = creep.wait_for_ball(timeout_s=0.8)
            self.assertTrue(result["finished_by_ir_odom"])
            self.assertGreaterEqual(state.post_loss_distance_m, 0.30)
        finally:
            creep.stop_and_join()


if __name__ == "__main__":
    unittest.main()

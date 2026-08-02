#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""SerialEngine 离线单测（2026-08-03，无硬件依赖）。

覆盖：
  1) 引擎基础 round-trip（submit → io 线程执行 → 应答派发）
  2) 写合并 coalesce：同 key 并发提交只打最新一帧，整组共享应答
  3) 读共享 share：同 key 在飞期间并发读合并为一次物理读
  4) 优先级：URGENT 插队 READ
  5) 超时：dev 无应答 → result=None（调用方语义 ControllerNoResponseError）
  6) detach：未完成 job 全部 transport 失败，不挂死
  7) SerialWrap.get_anwser 引擎路径 + 未连接降级路径
  8) mc602 层 coalesce_key / share_key 透传（DevCmdInterface / DevListWrap）

跑法（repo 根目录）：
    RAK_CAR_SERIAL_AUTO_CONNECT=0 /usr/bin/python3 -m unittest smartcar.test.test_serial_engine -v
或直接：
    RAK_CAR_SERIAL_AUTO_CONNECT=0 /usr/bin/python3 smartcar/test/test_serial_engine.py
"""
import importlib
import os
import sys
import threading
import time
import types
import unittest
from pathlib import Path

# ---- 环境：禁止 import 时自动连串口（无硬件的开发机必须） ----
os.environ["RAK_CAR_SERIAL_AUTO_CONNECT"] = "0"
os.environ["RAK_CAR_SERIAL_ENGINE"] = "1"

REPO_ROOT = Path(__file__).resolve().parents[2]


def _bootstrap_stub_packages():
    """桩包链：smartcar/__init__ 会拖出 flask / paddle / zmq，
    开发机上不一定有。这里只给 serial_wrap / mc602_ctl2 需要的最小上下文。"""

    def stub(name, path=None):
        mod = types.ModuleType(name)
        mod.__path__ = [str(path)] if path is not None else []
        sys.modules[name] = mod
        return mod

    for name in ("smartcar", "smartcar.whalesbot",
                 "smartcar.whalesbot.vehicle",
                 "smartcar.whalesbot.vehicle.base"):
        sys.modules.pop(name, None)
    stub("smartcar", REPO_ROOT / "smartcar")
    stub("smartcar.whalesbot", REPO_ROOT / "smartcar" / "whalesbot")
    stub("smartcar.whalesbot.vehicle",
         REPO_ROOT / "smartcar" / "whalesbot" / "vehicle")
    stub("smartcar.whalesbot.vehicle.base",
         REPO_ROOT / "smartcar" / "whalesbot" / "vehicle" / "base")
    import logging
    tools = stub("smartcar.whalesbot.tools")
    tools.logger = logging.getLogger("serial-engine-test")


_bootstrap_stub_packages()

serial_wrap_mod = importlib.import_module(
    "smartcar.whalesbot.vehicle.base.serial_wrap")
mc602_mod = importlib.import_module(
    "smartcar.whalesbot.vehicle.base.mc602_ctl2")

SerialEngine = serial_wrap_mod.SerialEngine
SerialEngineJob = serial_wrap_mod.SerialEngineJob
PRIORITY_URGENT = serial_wrap_mod.PRIORITY_URGENT
PRIORITY_NORMAL = serial_wrap_mod.PRIORITY_NORMAL
PRIORITY_READ = serial_wrap_mod.PRIORITY_READ
SerialWrap = serial_wrap_mod.SerialWrap
ControllerNotReadyError = serial_wrap_mod.ControllerNotReadyError
ControllerNoResponseError = serial_wrap_mod.ControllerNoResponseError


class FakeMC602Dev:
    """模拟 CotrollerInfo：可编程应答 + 可注入延迟。"""

    def __init__(self, name="mc602"):
        self.name = name
        self.responses = []          # [(payload_or_None, delay_s), ...]
        self.sent = []               # 收到的写帧
        self.call_log = []
        self._lock = threading.Lock()

    def queue_response(self, payload, delay=0.0):
        with self._lock:
            self.responses.append((payload, delay))

    def send_cmd(self, serial_obj, cmd):
        with self._lock:
            self.sent.append(cmd)

    def get_anwser(self, serial_obj, time_out=0.2):
        with self._lock:
            if not self.responses:
                item = (None, 0.0)
            else:
                item = self.responses.pop(0)
        payload, delay = item
        self.call_log.append(payload)
        if delay:
            time.sleep(delay)
        return payload

    def ping_rx(self, serial_obj, time_out=0.05):
        with self._lock:
            self.sent.append(b"PING")
        return True



class _FakeSerial:
    """引擎只透传 serial 对象给 dev 方法；真实字节 IO 全由 FakeMC602Dev 模拟。
    reset_buffer / read / write 必须存在且无副作用（pyserial 真方法需要真 fd）。"""
    is_open = True

    def reset_buffer(self):
        pass

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def write(self, data):
        return len(data)

    def read(self, n=1):
        return b""


class _OpenSerialWrap(SerialWrap):
    """测试用：buffer 操作无副作用；构造后由测试手动置 is_open=True。"""

    def __init__(self):
        # pyserial finalize 时 close() 会查 self.fd；无真 fd 的测试件先占位
        self.fd = None
        super().__init__()

    def reset_buffer(self):
        pass


def _attach(sw, dev):
    sw.dev = dev
    sw.connect_flag = True
    try:
        sw.is_open = True
    except Exception:
        pass
    sw.engine.attach(sw, dev)




class TestSerialEngineCore(unittest.TestCase):

    def setUp(self):
        self.engine = SerialEngine()
        self.engine.start()
        self.dev = FakeMC602Dev()
        # attach 只把 dev 透传给 dev.send_cmd / dev.get_anwser；serial 用无副作用假件
        self.engine.attach(_FakeSerial(), self.dev)

    def tearDown(self):
        self.engine.shutdown()

    def _wait(self, job, timeout=2.0):
        self.assertTrue(job.event.wait(timeout), "job 未在 %.1fs 内完成" % timeout)
        return job

    def test_basic_round_trip(self):
        self.dev.queue_response(b"\x01\x02\x03")
        job = SerialEngineJob("mc602", b"\x02\x01\x10", time_out=0.2)
        self.assertTrue(self.engine.submit(job))
        self._wait(job)
        self.assertIsNone(job.error)
        self.assertEqual(job.result, b"\x01\x02\x03")
        self.assertEqual(self.engine.stats["frames"], 1)

    def test_timeout_gives_none_result(self):
        self.dev.queue_response(None)  # 无应答
        job = SerialEngineJob("mc602", b"\x02\x01\x10", time_out=0.05)
        self.assertTrue(self.engine.submit(job))
        self._wait(job)
        self.assertIsNone(job.error)
        self.assertIsNone(job.result)
        self.assertEqual(self.engine.stats["timeouts"], 1)

    def test_coalesce_only_newest_frame_sent(self):
        """忙时同 key 三连发 → 只有最新一帧落地，三个调用方都拿到最新帧应答。"""
        # 第一帧慢（占住 io 线程），给后续提交留出窗口
        self.dev.queue_response(b"FIRST", delay=0.15)
        self.dev.queue_response(b"NEWEST")

        first = SerialEngineJob("mc602", b"f0", 0.5, coalesce_key="wheels")
        self.assertTrue(self.engine.submit(first))
        time.sleep(0.05)  # 确保 first 已被 io 线程取走开始执行

        j1 = SerialEngineJob("mc602", b"w1", 0.5, coalesce_key="wheels")
        j2 = SerialEngineJob("mc602", b"w2", 0.5, coalesce_key="wheels")
        j3 = SerialEngineJob("mc602", b"w3", 0.5, coalesce_key="wheels")
        for j in (j1, j2, j3):
            self.assertTrue(self.engine.submit(j))

        for j in (first, j1, j2, j3):
            self._wait(j, timeout=3.0)

        self.assertEqual(first.result, b"FIRST")
        # 被合并的三个 job 共享最新帧应答
        self.assertEqual(j1.result, b"NEWEST")
        self.assertEqual(j2.result, b"NEWEST")
        self.assertEqual(j3.result, b"NEWEST")
        # 物理帧只有 2 个（first + newest），w1/w2 被合并掉
        self.assertEqual(self.dev.sent, [b"f0", b"w3"])
        self.assertEqual(self.engine.stats["frames"], 2)
        self.assertGreaterEqual(self.engine.stats["coalesced"], 2)

    def test_share_merges_concurrent_reads(self):
        self.dev.queue_response(b"ENC", delay=0.1)
        r1 = SerialEngineJob("mc602", b"r1", 0.5, share_key="encoder4")
        self.assertTrue(self.engine.submit(r1))
        time.sleep(0.03)  # r1 进入在飞
        r2 = SerialEngineJob("mc602", b"r2", 0.5, share_key="encoder4")
        r3 = SerialEngineJob("mc602", b"r3", 0.5, share_key="encoder4")
        self.assertTrue(self.engine.submit(r2))
        self.assertTrue(self.engine.submit(r3))

        for j in (r1, r2, r3):
            self._wait(j, timeout=3.0)
            self.assertEqual(j.result, b"ENC")
        # 三个并发读 → 一次物理帧
        self.assertEqual(self.engine.stats["frames"], 1)
        self.assertGreaterEqual(self.engine.stats["shared"], 2)

    def test_urgent_preempts_read(self):
        self.dev.queue_response(b"BUSY", delay=0.12)  # 占住 io 线程
        self.dev.queue_response(b"U")                  # URGENT 先出队 → 先拿应答
        self.dev.queue_response(b"R")                  # READ 后拿

        busy = SerialEngineJob("mc602", b"busy", 0.5)
        self.assertTrue(self.engine.submit(busy))
        time.sleep(0.03)

        read_job = SerialEngineJob("mc602", b"read", 0.5, priority=PRIORITY_READ)
        urgent = SerialEngineJob("mc602", b"stop", 0.5, priority=PRIORITY_URGENT)
        # 先交 READ，再交 URGENT —— 出队顺序必须是 URGENT 先
        self.assertTrue(self.engine.submit(read_job))
        self.assertTrue(self.engine.submit(urgent))

        self._wait(busy, timeout=3.0)
        self._wait(urgent, timeout=3.0)
        self._wait(read_job, timeout=3.0)

        self.assertEqual(list(self.dev.sent), [b"busy", b"stop", b"read"])
        self.assertEqual(urgent.result, b"U")
        self.assertEqual(read_job.result, b"R")

    def test_detach_fails_pending_jobs(self):
        self.dev.queue_response(b"BUSY", delay=0.3)
        busy = SerialEngineJob("mc602", b"busy", 0.5)
        queued = SerialEngineJob("mc602", b"q", 0.5)
        self.assertTrue(self.engine.submit(busy))
        time.sleep(0.03)
        self.assertTrue(self.engine.submit(queued))
        self.engine.detach()
        # 排队中的 job 立即失败返回
        self.assertTrue(queued.event.wait(1.0))
        self.assertEqual(queued.error, "transport")
        # detach 后不再接受新 job
        self.assertFalse(self.engine.submit(SerialEngineJob("mc602", b"x", 0.1)))

    def test_call_on_io_thread(self):
        ok, res = self.engine.call_on_io_thread(
            lambda s, d: "pong", time_out=0.2)
        self.assertTrue(ok)
        self.assertEqual(res, "pong")


class TestSerialWrapEnginePath(unittest.TestCase):
    """SerialWrap.get_anwser 的引擎路径 / 降级路径。"""

    def setUp(self):
        self.sw = _OpenSerialWrap()
        self.dev = FakeMC602Dev()

    def tearDown(self):
        self.sw.engine.shutdown()

    def test_engine_path_success(self):
        _attach(self.sw, self.dev)
        self.dev.queue_response(b"\xAA\xBB")
        res = self.sw.get_anwser(b"\x02\x01\x10", time_out=0.2)
        self.assertEqual(res, b"\xAA\xBB")

    def test_engine_path_no_response_raises(self):
        _attach(self.sw, self.dev)
        self.dev.queue_response(None)
        with self.assertRaises(ControllerNoResponseError):
            self.sw.get_anwser(b"\x02\x01\x10", time_out=0.05)

    def test_not_connected_raises(self):
        # 未连接：降级路径抛 ControllerNotReadyError
        with self.assertRaises(ControllerNotReadyError):
            self.sw.get_anwser(b"\x02\x01\x10")

    def test_mc601_sync_fallback(self):
        """mc601 设备不进引擎，走旧同步路径。"""
        dev = FakeMC602Dev(name="mc601")
        dev.queue_response(b"\x77\x68\x03\x00\x02\x67\x0A")
        _attach(self.sw, dev)
        self.assertFalse(self.sw.engine.is_attached())
        res = self.sw.get_anwser(b"\x77\x68\x04\x00\x01\xCA\x01\x0A", time_out=0.2)
        self.assertEqual(res, b"\x77\x68\x03\x00\x02\x67\x0A")

    def test_coalesce_concurrent_wheel_writes(self):
        """两个线程同 key 并发轮速下发 → 只打一帧，两者同应答。"""
        _attach(self.sw, self.dev)
        self.dev.queue_response(b"BUSY", delay=0.12)
        self.dev.queue_response(b"LATEST")

        busy = SerialEngineJob("mc602", b"busy", 0.5)
        self.sw.engine.submit(busy)
        time.sleep(0.03)

        results = {}

        def call(tag):
            try:
                results[tag] = self.sw.get_anwser(
                    b"w" + tag.encode(), time_out=0.5, coalesce_key="wheels")
            except Exception as exc:
                results[tag] = exc

        t1 = threading.Thread(target=call, args=("a",))
        t2 = threading.Thread(target=call, args=("b",))
        t1.start(); t2.start()
        t1.join(3); t2.join(3)
        busy.event.wait(3)
        self.assertEqual(results.get("a"), b"LATEST")
        self.assertEqual(results.get("b"), b"LATEST")
        # 只打了一个物理写帧（w a/w b 合并）
        self.assertEqual(self.dev.sent, [b"busy", b"wb"])


class TestMc602LayerHints(unittest.TestCase):
    """mc602_ctl2：coalesce_key / share_key 是否正确透传给 get_anwser。"""

    def setUp(self):
        self.captured = []

        class CaptureSerial:
            def get_anwser(inner_self, cmd, time_out=0.1, priority=None,
                           coalesce_key=None, share_key=None):
                self.captured.append({
                    "cmd": cmd, "priority": priority,
                    "coalesce_key": coalesce_key, "share_key": share_key,
                })
                # 回一个足够长的假应答，让 unpack 不炸
                return b"\x00" * 64

        self.fake_serial = CaptureSerial()
        self._old = mc602_mod.serial_mc602
        mc602_mod.serial_mc602 = self.fake_serial

    def tearDown(self):
        mc602_mod.serial_mc602 = self._old

    def test_motor4_write_has_coalesce_key(self):
        motor4 = mc602_mod.Motor4_2()
        motor4.ser = self.fake_serial
        motor4.set_speed([10, 20, 30, 40])
        self.assertEqual(len(self.captured), 1)
        self.assertIsNotNone(self.captured[0]["coalesce_key"])
        self.assertEqual(self.captured[0]["priority"], PRIORITY_NORMAL)

    def test_encoder4_read_has_share_key_and_read_priority(self):
        enc4 = mc602_mod.EncoderMotors4_2()
        enc4.ser = self.fake_serial
        enc4.get()
        self.assertEqual(len(self.captured), 1)
        self.assertIsNotNone(self.captured[0]["share_key"])
        self.assertEqual(self.captured[0]["priority"], PRIORITY_READ)

    def test_single_encoder_has_share_key(self):
        enc = mc602_mod.EncoderMotor_2(port_id=2)
        enc.ser = self.fake_serial
        enc.get_encoder()
        self.assertEqual(self.captured[0]["share_key"], "encoder_2")

    def test_devlist_write_coalesces(self):
        dev_list = mc602_mod.DevListWrap(
            [mc602_mod.Motor_2(p) for p in (1, 2, 3, 4)])
        for d in dev_list.dev_list:
            d.ser = self.fake_serial
        dev_list.get_all([1, 2, 3, 4], mode=2)
        self.assertEqual(len(self.captured), 1)
        self.assertIsNotNone(self.captured[0]["coalesce_key"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

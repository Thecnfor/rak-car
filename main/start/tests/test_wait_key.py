#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""MC602 板上鍵一鍵啟動 — 讀鍵鉤子 + 邊沿偵測 離線單測（無硬體，mock 一切 IO）。

註：raw→bool 映射收斂在 runtime/core/key_input.py（純函數、無依賴），
可離線 import；runtime.services.my_car 依賴 zmq/flask 等 Jetson 套件，不在此測。
"""
import os
import unittest

# 保險：若有任何 smartcar import 路徑，關掉串口自動連
os.environ.setdefault("RAK_CAR_SERIAL_AUTO_CONNECT", "0")


class TestBoardKeyPressed(unittest.TestCase):
    """raw → bool 映射。預設任一 byte 非零 = 按下（真機標定點）。"""

    # setUp（instance）而非 setUpClass：把函數存成 instance 屬性，
    # 避免 `self._f` 把 module 級函數當 bound method（多注入 self 一個參數）。
    def setUp(self):
        from runtime.core.key_input import board_key_pressed
        self._f = board_key_pressed

    def test_released(self):
        self.assertFalse(self._f((0, 0)))

    def test_any_pressed(self):
        self.assertTrue(self._f((1, 0)))
        self.assertTrue(self._f((0, 1)))
        self.assertTrue(self._f((255, 255)))

    def test_scalar_input(self):
        self.assertFalse(self._f(0))
        self.assertTrue(self._f(1))

    def test_none_is_released(self):
        self.assertFalse(self._f(None))


class TestReadKeyActionRegistered(unittest.TestCase):
    def test_car_action_registered(self):
        from runtime.core.actions import CAR_ACTIONS
        self.assertIn("read_key", CAR_ACTIONS)
        car = type("FakeCar", (), {"read_key": lambda self: True})()
        self.assertTrue(CAR_ACTIONS["read_key"](car))


class TestPressDetector(unittest.TestCase):
    """邊沿偵測 + 去抖：釋放→按下連續 confirm_samples 次才觸發；開機按住不誤觸發。"""

    def test_no_fire_when_never_pressed(self):
        from main.start.orchestrator import PressDetector
        d = PressDetector(confirm_samples=2)
        for _ in range(10):
            self.assertFalse(d.feed(False))

    def test_fire_on_press_edge(self):
        from main.start.orchestrator import PressDetector
        d = PressDetector(confirm_samples=2)
        d.feed(False)          # 穩定釋放
        d.feed(False)
        self.assertFalse(d.feed(True))    # 第 1 個按下樣本
        self.assertTrue(d.feed(True))     # 連續第 2 個 → 觸發

    def test_debounce_short_glitch(self):
        from main.start.orchestrator import PressDetector
        d = PressDetector(confirm_samples=2)
        d.feed(False)
        d.feed(False)
        d.feed(True)           # 一次雜訊
        d.feed(False)          # 又釋放 → 不觸發
        self.assertFalse(d.feed(True))    # 重新按下，streak 重計
        self.assertTrue(d.feed(True))

    def test_held_at_boot_does_not_fire(self):
        from main.start.orchestrator import PressDetector
        d = PressDetector(confirm_samples=2)
        # 開機時按鍵已被壓住：首採樣 True，之後持續 True → 永不觸發
        self.assertFalse(d.feed(True))
        for _ in range(5):
            self.assertFalse(d.feed(True))
        # 釋放後再按才觸發
        d.feed(False)
        d.feed(False)
        d.feed(True)
        self.assertTrue(d.feed(True))

    def test_confirm_1_fires_immediately(self):
        from main.start.orchestrator import PressDetector
        d = PressDetector(confirm_samples=1)
        d.feed(False)
        self.assertTrue(d.feed(True))    # 單樣本確認

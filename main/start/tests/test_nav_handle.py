#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""orchestrator._NavHandle 单测：runtime/client 双后端 start/pause/resume/stop。

runtime 后端走 HTTP 低频端点（Mock ChassisClient）；client 后端直接驱动
DoubleLoopRunner（MagicMock）。调用方只依赖统一接口，不关心后端。
"""
import unittest
from unittest.mock import MagicMock


class TestNavHandleRuntime(unittest.TestCase):
    def _handle(self):
        from main.start.orchestrator import _NavHandle
        api = MagicMock()
        api.start_lane_nav.return_value = {"ok": True, "result": {"started": True}}
        api.pause_lane_nav.return_value = {"ok": True, "result": {"paused": True}}
        api.resume_lane_nav.return_value = {"ok": True, "result": {"resumed": True}}
        api.stop_lane_nav.return_value = {"ok": True, "result": {"stopped": True}}
        return _NavHandle("runtime", api=api, start_kwargs={"hz": 50.0}), api

    def test_start_passes_kwargs_once(self):
        nav, api = self._handle()
        self.assertTrue(nav.start())
        api.start_lane_nav.assert_called_once_with(hz=50.0)
        # 幂等：第二次不重复发
        self.assertTrue(nav.start())
        api.start_lane_nav.assert_called_once()

    def test_pause_resume_stop(self):
        nav, api = self._handle()
        nav.start()
        self.assertTrue(nav.pause())
        api.pause_lane_nav.assert_called_once()
        nav.resume()
        api.resume_lane_nav.assert_called_once()
        nav.stop()
        api.stop_lane_nav.assert_called_once_with(force=True)


class TestNavHandleClient(unittest.TestCase):
    def _handle(self):
        from main.start.orchestrator import _NavHandle
        runner = MagicMock()
        thread = MagicMock()
        return _NavHandle("client", runner=runner, thread=thread), runner, thread

    def test_start_stops_thread(self):
        nav, runner, thread = self._handle()
        self.assertTrue(nav.start())
        thread.start.assert_called_once()

    def test_pause_resume_stop_drive_runner(self):
        nav, runner, thread = self._handle()
        runner.pause.return_value = True
        self.assertTrue(nav.pause())
        runner.pause.assert_called_once()
        nav.resume()
        runner.resume.assert_called_once()
        nav.stop()
        runner.stop.assert_called_once()
        thread.join.assert_called_once()


if __name__ == "__main__":
    unittest.main()

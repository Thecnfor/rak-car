#!/usr/bin/python3
# -*- coding: utf-8 -*-
import importlib
import os
import sys
import threading
import time

from runtime.hardware.controller_probe import probe_controller


os.environ.setdefault("RAK_CAR_SERIAL_AUTO_CONNECT", "0")


STATE_DISCONNECTED = "DISCONNECTED"
STATE_DEGRADED = "DEGRADED"
STATE_PROGRAM_READY = "PROGRAM_READY"
STATE_RECOVERING = "RECOVERING"


class ControllerSessionManager:
    def __init__(self):
        self._lock = threading.RLock()
        self._recover_lock = threading.Lock()
        self._heartbeat_interval = 0.5
        self._ready_ttl = 0.8
        self._failure_threshold = 2
        self._state = STATE_DEGRADED
        self._detail = "控制器状态未知"
        self._generation = 0
        self._failure_count = 0
        self._last_ok_at = 0.0
        self._last_recover_at = None
        self._last_probe = None
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
        )
        self._heartbeat_thread.start()

    def _get_loaded_serial_wrap(self):
        module = sys.modules.get("smartcar.whalesbot.vehicle.base.serial_wrap")
        if module is None:
            return None
        return getattr(module, "serial_wrap", None)

    def _load_serial_wrap(self):
        if "smartcar.whalesbot.vehicle" not in sys.modules:
            importlib.import_module("smartcar.whalesbot.vehicle")
        module = importlib.import_module("smartcar.whalesbot.vehicle.base.serial_wrap")
        return getattr(module, "serial_wrap")

    def _refresh_from_serial(self, serial_wrap):
        if serial_wrap is None:
            return
        with self._lock:
            self._generation = max(self._generation, getattr(serial_wrap, "generation", 0))
            self._last_ok_at = max(self._last_ok_at, getattr(serial_wrap, "last_ok_at", 0.0))

    def _set_probe(self, probe):
        with self._lock:
            self._last_probe = {
                "ready": probe.ready,
                "port": probe.port,
                "controller": probe.controller,
                "detail": probe.detail,
                "checked_at": time.time(),
            }

    def note_io_success(self):
        serial_wrap = self._get_loaded_serial_wrap()
        self._refresh_from_serial(serial_wrap)
        with self._lock:
            self._state = STATE_PROGRAM_READY
            self._detail = "控制器 program 模式在线"
            self._failure_count = 0
            self._last_ok_at = time.time()
            if serial_wrap is not None:
                controller_name = None
                dev = getattr(serial_wrap, "dev", None)
                if dev is not None:
                    controller_name = getattr(dev, "name", None)
                self._last_probe = {
                    "ready": True,
                    "port": getattr(serial_wrap, "port", None),
                    "controller": controller_name,
                    "detail": self._detail,
                    "checked_at": self._last_ok_at,
                }

    def note_io_failure(self, detail=None):
        with self._lock:
            self._failure_count += 1
            if self._failure_count >= self._failure_threshold:
                self._state = STATE_DEGRADED
            if detail:
                self._detail = str(detail)

    def mark_offline(self, detail=None):
        with self._lock:
            self._state = STATE_DISCONNECTED
            self._failure_count = max(self._failure_count, self._failure_threshold)
            if detail:
                self._detail = str(detail)

    def is_fast_ready(self):
        with self._lock:
            return (
                self._state == STATE_PROGRAM_READY
                and (time.time() - self._last_ok_at) <= self._ready_ttl
            )

    def get_generation(self):
        with self._lock:
            return self._generation

    def snapshot(self):
        with self._lock:
            return {
                "state": self._state,
                "detail": self._detail,
                "generation": self._generation,
                "last_ok_at": self._last_ok_at,
                "last_recover_at": self._last_recover_at,
                "failure_count": self._failure_count,
                "last_probe": self._last_probe,
            }

    def ensure_ready(self, timeout=3.0):
        if self.is_fast_ready():
            return self.snapshot()
        deadline = time.time() + float(timeout)
        last_error = "控制器未就绪"
        while time.time() < deadline:
            if self.is_fast_ready():
                return self.snapshot()
            wait_time = min(0.3, max(0.0, deadline - time.time()))
            acquired = self._recover_lock.acquire(timeout=wait_time)
            if not acquired:
                continue
            try:
                if self.is_fast_ready():
                    return self.snapshot()
                self._recover_once()
                return self.snapshot()
            except Exception as exc:
                last_error = str(exc)
            finally:
                self._recover_lock.release()
            time.sleep(0.1)
        raise RuntimeError(f"控制器恢复超时: {last_error}")

    def _recover_once(self):
        with self._lock:
            self._state = STATE_RECOVERING
            self._detail = "正在恢复控制器 program 模式"
        serial_wrap = self._get_loaded_serial_wrap()
        if serial_wrap is not None:
            try:
                with serial_wrap.lock:
                    serial_wrap._close_locked()
            except Exception:
                pass
        probe = probe_controller()
        self._set_probe(probe)
        if not probe.ready:
            self.mark_offline(probe.detail)
            raise RuntimeError(probe.detail or "控制器探测失败")
        try:
            serial_wrap = self._load_serial_wrap()
        except Exception:
            raise
        serial_wrap.sync_with_probe(probe)
        self._refresh_from_serial(serial_wrap)
        with self._lock:
            self._state = STATE_PROGRAM_READY
            self._detail = probe.detail or "控制器恢复成功"
            self._failure_count = 0
            self._last_ok_at = max(self._last_ok_at, time.time())
            self._last_recover_at = time.time()

    def _heartbeat_loop(self):
        while True:
            time.sleep(self._heartbeat_interval)
            if self._recover_lock.locked():
                continue
            serial_wrap = self._get_loaded_serial_wrap()
            if serial_wrap is None:
                continue
            try:
                if serial_wrap.ping_current(timeout=0.03):
                    self.note_io_success()
                    continue
                self.note_io_failure("控制器心跳失败")
                if self._failure_count >= self._failure_threshold:
                    self.ensure_ready(timeout=2.0)
            except Exception as exc:
                self.note_io_failure(exc)


_controller_session = ControllerSessionManager()


def get_controller_session():
    return _controller_session

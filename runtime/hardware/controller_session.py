#!/usr/bin/python3
# -*- coding: utf-8 -*-
import importlib
import os
import sys
import threading
import time

try:
    import pyudev
except ModuleNotFoundError:  # pragma: no cover
    pyudev = None

from runtime.hardware.controller_probe import list_candidate_ports, probe_controller
from runtime.hardware.controller_recover import recover_controller_with_probe


os.environ.setdefault("RAK_CAR_SERIAL_AUTO_CONNECT", "0")


STATE_DISCONNECTED = "DISCONNECTED"
STATE_NO_PORT = "NO_PORT"
STATE_UNKNOWN = "UNKNOWN"
STATE_BOOTLOADER_READY = "BOOTLOADER_READY"
STATE_PROGRAM_TRANSITION = "PROGRAM_TRANSITION"
STATE_PROGRAM_READY = "PROGRAM_READY"
STATE_RUNTIME_LOST = "RUNTIME_LOST"
STATE_RECOVERING = "RECOVERING"


class ControllerSessionManager:
    def __init__(self):
        self._lock = threading.RLock()
        self._recover_lock = threading.Lock()
        self._heartbeat_interval = 0.5
        self._usb_presence_poll_interval = 1.5
        self._not_ready_reconcile_interval = 0.8
        self._ready_ttl = 0.8
        self._failure_threshold = 2
        self._transition_window = 1.2
        self._state = STATE_UNKNOWN
        self._mode = "unknown"
        self._detail = "控制器状态未知"
        self._generation = 0
        self._failure_count = 0
        self._last_ok_at = 0.0
        self._last_recover_at = None
        self._last_probe = None
        self._last_probe_mode = None
        self._last_probe_at = None
        self._last_program_ok_at = 0.0
        self._last_bootloader_seen_at = None
        self._last_transition_at = None
        self._last_runtime_failure_at = None
        self._last_recover_reason = None
        self._recover_suppressed_until = 0.0
        self._usb_present = False
        self._last_usb_seen_at = None
        self._last_usb_event_at = None
        self._last_usb_event = None
        self._usb_monitor_error = None
        self._last_reconcile_at = None
        self._reconcile_thread = None
        self._usb_monitor_thread = threading.Thread(
            target=self._usb_monitor_loop,
            daemon=True,
        )
        self._usb_monitor_thread.start()
        self._usb_presence_thread = threading.Thread(
            target=self._usb_presence_loop,
            daemon=True,
        )
        self._usb_presence_thread.start()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
        )
        self._heartbeat_thread.start()
        self._bootstrap_usb_presence()

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
        now = time.time()
        with self._lock:
            self._last_probe = {
                "ready": probe.ready,
                "port": probe.port,
                "controller": probe.controller,
                "mode": getattr(probe, "mode", None),
                "detail": probe.detail,
                "bootloader_seen": getattr(probe, "bootloader_seen", False),
                "program_seen": getattr(probe, "program_seen", False),
                "checked_at": now,
            }
            self._last_probe_mode = getattr(probe, "mode", None)
            self._last_probe_at = now
            if getattr(probe, "bootloader_seen", False):
                self._last_bootloader_seen_at = now

    def _set_state_locked(self, state, mode=None, detail=None):
        self._state = state
        if mode is not None:
            self._mode = mode
        if detail is not None:
            self._detail = str(detail)

    def _update_usb_presence_locked(self, present, detail=None, event=None):
        now = time.time()
        previous = self._usb_present
        self._usb_present = bool(present)
        self._last_usb_event_at = now
        self._last_usb_event = event or ("present" if present else "missing")
        if present:
            self._last_usb_seen_at = now
            if not previous and self._state == STATE_NO_PORT:
                self._set_state_locked(STATE_UNKNOWN, mode="unknown", detail=detail or "检测到控制器 USB，等待探测")
        else:
            self._failure_count = max(self._failure_count, self._failure_threshold)
            self._set_state_locked(STATE_NO_PORT, mode="no_port", detail=detail or "未检测到控制器 USB，等待下位机上电")

    def _kick_reconcile_async(self, reason):
        with self._lock:
            if not self._usb_present:
                return
            if self._reconcile_thread is not None and self._reconcile_thread.is_alive():
                return
            self._reconcile_thread = threading.Thread(
                target=self._reconcile_after_event,
                args=(reason,),
                daemon=True,
            )
            self._reconcile_thread.start()

    def _reconcile_after_event(self, reason):
        with self._lock:
            self._last_recover_reason = reason
        try:
            self.ensure_ready(timeout=3.0)
        except Exception:
            pass
        finally:
            with self._lock:
                self._last_reconcile_at = time.time()

    def _bootstrap_usb_presence(self):
        usb_present = bool(list_candidate_ports())
        with self._lock:
            self._update_usb_presence_locked(
                usb_present,
                detail="启动时已检测到控制器 USB"
                if usb_present
                else "启动时未检测到控制器 USB，等待下位机上电",
                event="bootstrap",
            )
        if usb_present:
            self._kick_reconcile_async("startup_usb_present")

    def _device_matches(self, device):
        node = getattr(device, "device_node", None)
        if not node:
            return False
        base_name = os.path.basename(node)
        return base_name.startswith("ttyUSB") or base_name.startswith("ttyACM")

    def _handle_usb_event(self, action, device):
        if not self._device_matches(device):
            return
        device_node = getattr(device, "device_node", None)
        usb_present = bool(list_candidate_ports())
        with self._lock:
            self._update_usb_presence_locked(
                usb_present,
                detail="检测到 USB 事件 {} {}".format(action, device_node),
                event=action,
            )
        if usb_present:
            self._kick_reconcile_async("usb_event_{}".format(action))

    def _usb_monitor_loop(self):
        if pyudev is None:
            with self._lock:
                self._usb_monitor_error = "pyudev 不可用，退回 presence 兜底检测"
            return
        try:
            context = pyudev.Context()
            monitor = pyudev.Monitor.from_netlink(context)
            monitor.filter_by(subsystem="tty")
            monitor.start()
            while True:
                device = monitor.poll(timeout=1)
                if device is None:
                    continue
                self._handle_usb_event(device.action, device)
        except Exception as exc:  # pragma: no cover
            with self._lock:
                self._usb_monitor_error = repr(exc)

    def _usb_presence_loop(self):
        while True:
            time.sleep(self._usb_presence_poll_interval)
            usb_present = bool(list_candidate_ports())
            with self._lock:
                previous = self._usb_present
                state = self._state
                last_probe_at = self._last_probe_at or 0.0
                recovering = self._recover_lock.locked()
                self._update_usb_presence_locked(
                    usb_present,
                    detail="presence 兜底检测控制器 USB",
                    event="presence_poll",
                )
            if usb_present and not previous:
                self._kick_reconcile_async("usb_presence_detected")
                continue
            if not usb_present:
                continue
            if state in {
                STATE_PROGRAM_READY,
                STATE_NO_PORT,
            }:
                continue
            if recovering:
                continue
            if time.time() - last_probe_at >= self._not_ready_reconcile_interval:
                self._kick_reconcile_async("usb_present_not_ready")

    def _apply_probe_state_locked(self, probe, override_detail=None):
        mode = getattr(probe, "mode", None)
        detail = override_detail or getattr(probe, "detail", None)
        if mode == "program":
            self._set_state_locked(STATE_PROGRAM_READY, mode="program", detail=detail or "控制器 program 模式在线")
            self._failure_count = 0
            self._last_ok_at = max(self._last_ok_at, time.time())
            self._last_program_ok_at = self._last_ok_at
            self._recover_suppressed_until = 0.0
        elif mode == "bootloader":
            self._set_state_locked(STATE_BOOTLOADER_READY, mode="bootloader", detail=detail or "bootloader 在线，等待拉起 program")
        elif mode == "no_port":
            self._set_state_locked(STATE_NO_PORT, mode="no_port", detail=detail or "未找到控制器串口")
        else:
            self._set_state_locked(STATE_UNKNOWN, mode="unknown", detail=detail or "控制器状态未知")

    def _enter_transition_locked(self, detail):
        self._last_transition_at = time.time()
        self._recover_suppressed_until = self._last_transition_at + self._transition_window
        self._set_state_locked(
            STATE_PROGRAM_TRANSITION,
            mode="bootloader",
            detail=detail,
        )

    def note_io_success(self):
        serial_wrap = self._get_loaded_serial_wrap()
        self._refresh_from_serial(serial_wrap)
        with self._lock:
            self._set_state_locked(STATE_PROGRAM_READY, mode="program", detail="控制器 program 模式在线")
            self._failure_count = 0
            self._last_ok_at = time.time()
            self._last_program_ok_at = self._last_ok_at
            self._recover_suppressed_until = 0.0
            self._usb_present = True
            self._last_usb_seen_at = self._last_ok_at
            if serial_wrap is not None:
                controller_name = None
                dev = getattr(serial_wrap, "dev", None)
                if dev is not None:
                    controller_name = getattr(dev, "name", None)
                self._last_probe = {
                    "ready": True,
                    "port": getattr(serial_wrap, "port", None),
                    "controller": controller_name,
                    "mode": "program",
                    "detail": self._detail,
                    "bootloader_seen": False,
                    "program_seen": True,
                    "checked_at": self._last_ok_at,
                }
                self._last_probe_mode = "program"
                self._last_probe_at = self._last_ok_at

    def note_io_failure(self, detail=None):
        should_kick = False
        with self._lock:
            self._failure_count += 1
            if detail:
                self._detail = str(detail)
            if self._failure_count >= self._failure_threshold:
                self._last_runtime_failure_at = time.time()
                if self._state == STATE_PROGRAM_READY:
                    self._set_state_locked(STATE_RUNTIME_LOST, mode="unknown", detail=detail or "运行中检测到控制器掉线")
                    should_kick = self._usb_present
                elif self._state not in {STATE_NO_PORT, STATE_BOOTLOADER_READY, STATE_PROGRAM_TRANSITION}:
                    self._set_state_locked(STATE_UNKNOWN, mode="unknown", detail=detail or self._detail)
                    should_kick = self._usb_present
        if should_kick:
            self._kick_reconcile_async("io_failure")

    def mark_offline(self, detail=None, mode=None):
        with self._lock:
            self._failure_count = max(self._failure_count, self._failure_threshold)
            if mode == "no_port":
                self._set_state_locked(STATE_NO_PORT, mode="no_port", detail=detail)
            elif mode == "bootloader":
                self._set_state_locked(STATE_BOOTLOADER_READY, mode="bootloader", detail=detail)
            elif mode == "unknown":
                self._set_state_locked(STATE_UNKNOWN, mode="unknown", detail=detail)
            else:
                self._set_state_locked(STATE_DISCONNECTED, mode="unknown", detail=detail)

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
                "mode": self._mode,
                "detail": self._detail,
                "generation": self._generation,
                "last_ok_at": self._last_ok_at,
                "last_recover_at": self._last_recover_at,
                "failure_count": self._failure_count,
                "last_probe": self._last_probe,
                "last_probe_mode": self._last_probe_mode,
                "last_probe_at": self._last_probe_at,
                "last_program_ok_at": self._last_program_ok_at,
                "last_bootloader_seen_at": self._last_bootloader_seen_at,
                "last_transition_at": self._last_transition_at,
                "last_runtime_failure_at": self._last_runtime_failure_at,
                "last_recover_reason": self._last_recover_reason,
                "recover_suppressed_until": self._recover_suppressed_until,
                "recovering": self._recover_lock.locked(),
                "usb_present": self._usb_present,
                "last_usb_seen_at": self._last_usb_seen_at,
                "last_usb_event_at": self._last_usb_event_at,
                "last_usb_event": self._last_usb_event,
                "usb_monitor_error": self._usb_monitor_error,
                "last_reconcile_at": self._last_reconcile_at,
            }

    def ensure_ready(self, timeout=3.0):
        if self.is_fast_ready():
            return self.snapshot()
        deadline = time.time() + float(timeout)
        last_error = "控制器未就绪"
        while time.time() < deadline:
            if self.is_fast_ready():
                return self.snapshot()
            with self._lock:
                usb_present = self._usb_present
            if not usb_present:
                with self._lock:
                    self._update_usb_presence_locked(
                        False,
                        detail="未检测到控制器 USB，等待下位机上电",
                        event="ensure_ready_wait",
                    )
                time.sleep(min(0.2, max(0.05, deadline - time.time())))
                last_error = "未检测到控制器 USB，等待下位机上电"
                continue
            wait_time = min(0.3, max(0.0, deadline - time.time()))
            acquired = self._recover_lock.acquire(timeout=wait_time)
            if not acquired:
                continue
            try:
                if self.is_fast_ready():
                    return self.snapshot()
                self._recover_once()
                snapshot = self.snapshot()
                if snapshot.get("state") == STATE_PROGRAM_READY:
                    return snapshot
                last_error = snapshot.get("detail") or last_error
            except Exception as exc:
                last_error = str(exc)
            finally:
                self._recover_lock.release()
            time.sleep(0.1)
        raise RuntimeError(f"控制器恢复超时: {last_error}")

    def _recover_once(self):
        with self._lock:
            self._set_state_locked(STATE_RECOVERING, detail="正在恢复控制器 program 模式")
        serial_wrap = self._get_loaded_serial_wrap()
        if serial_wrap is not None:
            try:
                with serial_wrap.lock:
                    serial_wrap._close_locked()
            except Exception:
                pass
        probe = probe_controller()
        self._set_probe(probe)
        now = time.time()
        with self._lock:
            self._update_usb_presence_locked(
                getattr(probe, "mode", None) != "no_port",
                detail=getattr(probe, "detail", None),
                event="probe_result",
            )
        with self._lock:
            self._apply_probe_state_locked(probe)
        if getattr(probe, "mode", None) == "program":
            serial_wrap = self._load_serial_wrap()
            serial_wrap.sync_with_probe(probe)
            self._refresh_from_serial(serial_wrap)
            self.note_io_success()
            return
        if getattr(probe, "mode", None) == "no_port":
            self.mark_offline(probe.detail, mode="no_port")
            raise RuntimeError(probe.detail or "未找到控制器串口")
        if getattr(probe, "mode", None) == "unknown":
            self.mark_offline(probe.detail, mode="unknown")
            raise RuntimeError(probe.detail or "控制器探测失败")
        with self._lock:
            if now < self._recover_suppressed_until:
                self._enter_transition_locked("最近已触发 RUNCODE，等待 program 接管")
                return
            self._last_recover_reason = probe.detail or "bootloader 在线，尝试拉起 program"
            self._enter_transition_locked("检测到 bootloader，尝试拉起 program")
        ok, detail = recover_controller_with_probe(
            probe_result=probe,
            port_name=probe.port,
            port_supplier=probe_controller.__globals__["list_candidate_ports"],
            debug_hook=None,
        )
        with self._lock:
            self._last_recover_at = time.time()
            self._last_recover_reason = detail
        post_probe = probe_controller()
        self._set_probe(post_probe)
        with self._lock:
            self._apply_probe_state_locked(post_probe, override_detail=detail)
        if getattr(post_probe, "mode", None) == "program":
            serial_wrap = self._load_serial_wrap()
            serial_wrap.sync_with_probe(post_probe)
            self._refresh_from_serial(serial_wrap)
            self.note_io_success()
            return
        if ok:
            with self._lock:
                self._enter_transition_locked("已触发 RUNCODE，等待 program 接管")
            return
        with self._lock:
            if getattr(post_probe, "mode", None) == "bootloader":
                self._enter_transition_locked(detail or "bootloader 已响应，但 program 尚未就绪")
                return
        raise RuntimeError(detail or "控制器恢复失败")

    def _heartbeat_loop(self):
        while True:
            time.sleep(self._heartbeat_interval)
            if self._recover_lock.locked():
                continue
            with self._lock:
                if not self._usb_present or self._state != STATE_PROGRAM_READY:
                    continue
            serial_wrap = self._get_loaded_serial_wrap()
            if serial_wrap is None:
                continue
            try:
                if serial_wrap.ping_current(timeout=0.03):
                    self.note_io_success()
                    continue
                self.note_io_failure("控制器心跳失败")
            except Exception as exc:
                self.note_io_failure(exc)


_controller_session = ControllerSessionManager()


def get_controller_session():
    return _controller_session

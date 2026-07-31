#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""控制器健康巡检 Mixin（从 runtime_service.py 拆出）。

负责 MC602 控制器的在线判定 / 掉线标记 / generation 对比。依赖聚合类
CarRuntimeService 提供 `self.controller_session`、`self._ref_lock` 等属性。
"""


class ControllerWatcherMixin:
    """CarRuntimeService 的控制器健康巡检行为。"""

    def _is_controller_related_error(self, detail):
        if not detail:
            return False
        text = str(detail).lower()
        keywords = (
            "控制器",
            "下位机",
            "ttyusb",
            "serial",
            "controller",
            "program 模式",
            "program",
            "bootloader",
            "未找到控制器串口",
            "控制器恢复超时",
            "控制器探测失败",
        )
        return any(keyword in text for keyword in keywords)

    def _sync_controller_health_state(self, snapshot=None):
        snapshot = snapshot or self.controller_session.snapshot()
        self.last_controller_probe = snapshot.get("last_probe")
        if (
            snapshot.get("state") == "PROGRAM_READY"
            and self._is_controller_related_error(self.last_error)
        ):
            self.last_error = None
        return snapshot

    def _get_car_class(self):
        """返回 MyCar 类（迁移到 runtime.services.my_car）。"""
        if self._car_class is None:
            from runtime.services.my_car import MyCar
            self._car_class = MyCar
        return self._car_class

    # 2026-07-16 删 _get_task_module / _bind_task_car：任务逻辑由 main 层编排，
    # runtime 只暴露底层 car/arm action 接口。

    def _probe_controller(self):
        return self._sync_controller_health_state(self.controller_session.snapshot())

    def _is_generation_stale(self, snapshot=None):
        snapshot = snapshot or self.controller_session.snapshot()
        if self.car is None or self.controller_generation is None:
            return False
        return self.controller_generation != snapshot.get("generation")

    def _is_car_ready(self, snapshot=None):
        snapshot = snapshot or self.controller_session.snapshot()
        return (
            self.car is not None
            and snapshot.get("state") == "PROGRAM_READY"
            and not self._is_generation_stale(snapshot)
        )

    def _ensure_controller_ready(self):
        snapshot = self.controller_session.ensure_ready(
            timeout=self.action_ready_timeout
        )
        return self._sync_controller_health_state(snapshot)

    def _mark_controller_offline(self, detail=None):
        self.controller_session.mark_offline(detail, mode="unknown")
        with self._ref_lock:
            self._safe_close_locked()
        if detail:
            self.last_error = detail
        self._sync_controller_health_state()

    def _should_probe_controller(self, exc):
        import traceback as _traceback

        text = _traceback.format_exception_only(type(exc), exc)
        message = "".join(text).lower()
        keywords = (
            "input/output error",
            "device disconnected",
            "device reports readiness",
            "serial",
            "ttyusb",
            "controller",
            "控制器",
            "下位机",
            "未就绪",
            "broken pipe",
            "timed out",
            "timeout",
            "no such file",
            "resource temporarily unavailable",
            "controllernotreadyerror",
            "controllernoresponseerror",
            "controllertransporterror",
        )
        return any(keyword in message for keyword in keywords)

    def _handle_dispatch_failure(self, target, exc):
        if target == "system":
            return
        if not self._should_probe_controller(exc):
            return
        self._sync_controller_health_state()

#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""CarRuntimeService 兼容再导出层。

原 1633 行的 runtime_service.py 已按职责拆到 `car_runtime_service.py` +
4 个 mixin（controller_watcher / lifecycle_mixin / jobs_mixin / loops_mixin）。
本文件保留 `CarRuntimeService` 符号，`app.py` 及任何外部
`from runtime.services.runtime_service import CarRuntimeService` 零改动。
"""
from .car_runtime_service import CarRuntimeService

__all__ = ["CarRuntimeService"]

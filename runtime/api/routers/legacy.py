#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""旧版 /api/* 前缀路由（legacy 兼容面）。

与 /v1 同构，保留给旧调用方。前缀由 settings.get_legacy_api_prefix() 决定。
"""
try:
    from fastapi import APIRouter, Body, Query
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings

from ._helpers import (
    _build_runtime_snapshot,
    _create_job_from_payload,
    _execute_from_payload,
    _get_job,
    _submit_init_job,
    _submit_simple_system_job,
    _submit_stop_mode_job,
    health_payload,
)


def build_legacy_router(service):
    router_legacy = APIRouter(prefix=settings.get_legacy_api_prefix(), tags=["legacy"])

    @router_legacy.get("/health")
    def legacy_health(snapshot: int = Query(default=0)):
        return health_payload(service, include_snapshot=(snapshot == 1))

    @router_legacy.get("/meta")
    def legacy_meta():
        return {"ok": True, "actions": service.list_actions()}

    @router_legacy.get("/runtime")
    def legacy_runtime():
        return _build_runtime_snapshot(service)

    @router_legacy.get("/jobs")
    def legacy_jobs():
        return {"ok": True, "jobs": service.list_jobs()}

    @router_legacy.post("/execute")
    def legacy_execute(payload: dict = Body(default={})):
        return _execute_from_payload(service, payload)

    @router_legacy.post("/jobs", status_code=202)
    def legacy_create_job(payload: dict = Body(default={})):
        return _create_job_from_payload(service, payload)

    @router_legacy.get("/jobs/{job_id}")
    def legacy_job(job_id: str):
        return _get_job(service, job_id)

    @router_legacy.post("/system/init", status_code=202)
    def legacy_init(payload: dict = Body(default={})):
        return _submit_init_job(service, payload)

    @router_legacy.post("/system/stop-mode", status_code=202)
    def legacy_stop_mode(payload: dict = Body(default={})):
        return _submit_stop_mode_job(service, payload)

    @router_legacy.post("/system/reset-stop", status_code=202)
    def legacy_reset_stop():
        return _submit_simple_system_job(service, "reset_stop_flag")

    @router_legacy.post("/system/close", status_code=202)
    def legacy_close():
        return _submit_simple_system_job(service, "close")

    @router_legacy.post("/system/emergency-stop")
    def legacy_emergency_stop():
        return {"ok": True, "stopped": service.emergency_stop()}

    return router_legacy

#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""系统信息 / 急停路由：/v1/health、/v1/runtime、/v1/actions、/v1/config、
/v1/infer/*、/v1/estop*。
"""
try:
    from fastapi import APIRouter, Body, Query
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings

from ._helpers import _build_runtime_snapshot, health_payload


def build_system_router(service):
    router_v1 = APIRouter(prefix=settings.get_api_v1_prefix(), tags=["runtime"])

    @router_v1.get("/health")
    def v1_health(snapshot: int = Query(default=0)):
        return health_payload(service, include_snapshot=(snapshot == 1))

    @router_v1.get("/runtime")
    def v1_runtime():
        return _build_runtime_snapshot(service)

    @router_v1.get("/actions")
    def v1_actions():
        return {"ok": True, "actions": service.list_actions()}

    @router_v1.get("/config")
    def v1_config():
        return {"ok": True, "config": settings.get_runtime_settings()}

    @router_v1.get("/infer/state")
    def v1_infer_state():
        return {"ok": True, "infer": service.get_infer_state()}

    @router_v1.post("/infer/drop-oldest")
    def v1_infer_drop_oldest(payload: dict = Body(default={})):
        """2026-08-01：触发后端按 LRU 卸载非 eager 模型（OOM 主动缓解）。

        payload.timeout_s：单端口超时（默认取 settings.get_infer_health_timeout）。
        """
        return service.infer_drop_oldest(timeout_s=payload.get("timeout_s"))

    @router_v1.post("/estop")
    def v1_estop(payload: dict = Body(default={})):
        # 软件急停：直达 service.emergency_stop()，不进 job_queue、不抢 car_lock，
        # 因此能在 worker 跑长动作时立刻抢占（详见 runtime_service.emergency_stop）。
        return {"ok": True, "stopped": service.emergency_stop()}

    @router_v1.post("/estop/clear")
    def v1_estop_clear(payload: dict = Body(default={})):
        # 解除急停，同样走无锁直达路径。急停后须调用本端点才能恢复运动。
        return {"ok": True, "stopped": not service.reset_stop_flag()}

    return router_v1

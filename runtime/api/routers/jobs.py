#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务队列 / 控制路由：/v1/jobs*、/v1/execute、/v1/control/*。

全部委托给 service 的 job_queue（car/arm/system 双 worker），辅助函数在
_helpers 里。
"""
try:
    from fastapi import APIRouter, Body
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings

from ._helpers import (
    _create_job_from_payload,
    _execute_from_payload,
    _get_job,
    _submit_init_job,
    _submit_simple_system_job,
    _submit_stop_mode_job,
)


def build_jobs_router(service):
    router_v1 = APIRouter(prefix=settings.get_api_v1_prefix(), tags=["runtime"])

    @router_v1.get("/jobs")
    def v1_jobs():
        return {"ok": True, "jobs": service.list_jobs()}

    @router_v1.post("/jobs", status_code=202)
    def v1_create_job(payload: dict = Body(default={})):
        return _create_job_from_payload(service, payload)

    @router_v1.post("/execute")
    def v1_execute(payload: dict = Body(default={})):
        return _execute_from_payload(service, payload)

    @router_v1.get("/jobs/{job_id}")
    def v1_job(job_id: str):
        return _get_job(service, job_id)

    @router_v1.post("/jobs/{job_id}/stop")
    def v1_job_stop(job_id: str):
        """D.6 协作取消：触发 job 的 _stop_event + 车端 _stop_flag。

        SDK 在下个 PID 循环检测到 _stop_flag 后协作退出，job 状态置 failed
        （参考 emergency_stop 模式）。立即返回，不等 SDK 完成。
        """
        cancelled = bool(service.cancel_job(job_id))
        return {"ok": True, "cancelled": cancelled, "job_id": job_id}

    @router_v1.post("/control/init", status_code=202)
    def v1_init(payload: dict = Body(default={})):
        return _submit_init_job(service, payload)

    @router_v1.post("/control/stop-mode", status_code=202)
    def v1_stop_mode(payload: dict = Body(default={})):
        return _submit_stop_mode_job(service, payload)

    @router_v1.post("/control/reset-stop", status_code=202)
    def v1_reset_stop():
        return _submit_simple_system_job(service, "reset_stop_flag")

    @router_v1.post("/control/close", status_code=202)
    def v1_close():
        return _submit_simple_system_job(service, "close")

    @router_v1.post("/control/emergency-stop")
    def v1_emergency_stop():
        return {"ok": True, "stopped": service.emergency_stop()}

    return router_v1

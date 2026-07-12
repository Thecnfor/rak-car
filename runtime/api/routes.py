#!/usr/bin/python3
# -*- coding: utf-8 -*-
try:
    from fastapi import APIRouter, Body, HTTPException, Query, WebSocket, WebSocketDisconnect
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings


def get_public_links():
    api_base = settings.get_public_api_base()
    v1 = settings.get_api_v1_prefix()
    legacy = settings.get_legacy_api_prefix()
    ws_base = api_base.replace("http://", "ws://").replace("https://", "wss://")
    return {
        "api_base": api_base,
        "docs": f"{api_base}/docs",
        "health_v1": f"{api_base}{v1}/health",
        "jobs_v1": f"{api_base}{v1}/jobs",
        "ws_v1": f"{ws_base}{v1}/ws",
        "health_legacy": f"{api_base}{legacy}/health",
        "streamer": settings.get_public_stream_base(),
    }


def health_payload(service, include_snapshot=False):
    state = service.get_state()
    snapshot = None
    if include_snapshot:
        try:
            snapshot = service.get_runtime_snapshot()
        except Exception as exc:  # pragma: no cover
            snapshot = {"error": str(exc)}
    return {
        "ok": True,
        "state": state,
        "snapshot": snapshot,
        "links": get_public_links(),
    }


def _build_runtime_snapshot(service):
    snapshot = service.get_runtime_snapshot()
    if snapshot is None:
        raise HTTPException(status_code=409, detail="小车尚未初始化")
    return {"ok": True, "runtime": snapshot}


def _create_job_from_payload(service, payload):
    target = payload.get("target", "task")
    name = payload.get("name")
    args = payload.get("args", [])
    kwargs = payload.get("kwargs", {})
    if not name:
        raise HTTPException(status_code=400, detail="缺少 name")
    try:
        job = service.submit_job(target=target, name=name, args=args, kwargs=kwargs)
    except KeyError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "job": job}


def _execute_from_payload(service, payload):
    target = payload.get("target", "task")
    name = payload.get("name")
    args = payload.get("args", [])
    kwargs = payload.get("kwargs", {})
    timeout = payload.get("timeout")
    if not name:
        raise HTTPException(status_code=400, detail="缺少 name")
    try:
        job = service.submit_job_and_wait(
            target=target,
            name=name,
            args=args,
            kwargs=kwargs,
            timeout=timeout,
        )
    except KeyError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except TimeoutError as exc:
        raise HTTPException(status_code=504, detail=str(exc)) from exc
    return {"ok": job["status"] == "succeeded", "job": job}


def _get_job(service, job_id):
    job = service.get_job(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail="任务不存在")
    return {"ok": True, "job": job}


def _submit_init_job(service, payload):
    job = service.submit_job(
        target="system",
        name="init",
        kwargs={
            "reset_arm": payload.get("reset_arm", False),
            "force": payload.get("force", False),
            "reset_position": payload.get(
                "reset_position",
                settings.get_reset_position_on_init(),
            ),
        },
    )
    return {"ok": True, "job": job}


def _submit_stop_mode_job(service, payload):
    job = service.submit_job(
        target="system",
        name="set_stop_mode",
        kwargs={"enabled": payload.get("enabled", False)},
    )
    return {"ok": True, "job": job}


def _submit_simple_system_job(service, name):
    job = service.submit_job(target="system", name=name)
    return {"ok": True, "job": job}


async def _handle_websocket_message(service, payload):
    op = payload.get("op", "execute")
    if op == "ping":
        return {"ok": True, "op": "pong"}
    if op == "health":
        include_snapshot = bool(payload.get("snapshot"))
        return {
            "ok": True,
            "op": "health",
            "data": health_payload(service, include_snapshot=include_snapshot),
        }
    if op == "runtime":
        return {"ok": True, "op": "runtime", "data": _build_runtime_snapshot(service)}
    if op == "actions":
        return {"ok": True, "op": "actions", "data": {"actions": service.list_actions()}}
    if op == "config":
        return {"ok": True, "op": "config", "data": {"config": settings.get_runtime_settings()}}
    if op == "create_job":
        return {"ok": True, "op": "create_job", "data": _create_job_from_payload(service, payload)}
    if op == "get_job":
        job_id = payload.get("job_id")
        if not job_id:
            raise HTTPException(status_code=400, detail="缺少 job_id")
        return {"ok": True, "op": "get_job", "data": _get_job(service, job_id)}
    if op == "execute":
        return {"ok": True, "op": "execute", "data": _execute_from_payload(service, payload)}
    if op == "init":
        return {"ok": True, "op": "init", "data": _submit_init_job(service, payload)}
    if op == "stop_mode":
        return {
            "ok": True,
            "op": "stop_mode",
            "data": _submit_stop_mode_job(service, payload),
        }
    if op == "reset_stop":
        return {
            "ok": True,
            "op": "reset_stop",
            "data": _submit_simple_system_job(service, "reset_stop_flag"),
        }
    if op == "close":
        return {"ok": True, "op": "close", "data": _submit_simple_system_job(service, "close")}
    if op == "emergency_stop":
        return {"ok": True, "op": "emergency_stop", "data": {"stopped": service.emergency_stop()}}
    raise HTTPException(status_code=400, detail=f"不支持的 op: {op}")


def create_runtime_router(service):
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

    @router_v1.websocket("/ws")
    async def v1_ws(websocket: WebSocket):
        await websocket.accept()
        await websocket.send_json(
            {
                "ok": True,
                "op": "welcome",
                "links": get_public_links(),
                "usage": {
                    "execute": {"op": "execute", "target": "car|arm|task|system", "name": "动作名"},
                    "create_job": {"op": "create_job", "target": "task", "name": "任务名"},
                    "health": {"op": "health", "snapshot": 0},
                },
            }
        )
        while True:
            try:
                payload = await websocket.receive_json()
            except WebSocketDisconnect:
                break
            except Exception as exc:
                await websocket.send_json(
                    {"ok": False, "op": "invalid_json", "error": str(exc)}
                )
                continue
            request_id = payload.get("request_id")
            try:
                result = await _handle_websocket_message(service, payload)
            except HTTPException as exc:
                result = {"ok": False, "op": payload.get("op"), "error": exc.detail}
            except Exception as exc:  # pragma: no cover
                result = {"ok": False, "op": payload.get("op"), "error": str(exc)}
            if request_id is not None:
                result["request_id"] = request_id
            await websocket.send_json(result)

    return router_v1


def create_legacy_router(service):
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

#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""路由聚合入口：create_runtime_router / create_legacy_router。

取代原 `api/routes.py` 的 1735 行单文件。各资源子路由独立成模块，
这里只负责 include_router 组合，保证注册顺序与公开 endpoint 不变：
- 无前缀：/stream/*、/video_feed/*、/keypress
- /v1/*：system / vision / realtime / jobs / ws
- /api/*（legacy）：health / meta / runtime / jobs / execute / system/*
"""
try:
    from fastapi import APIRouter
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings

from .routers import jobs, keypress, legacy, realtime, stream, system, vision, ws
from .routers._helpers import get_public_links

__all__ = ["create_legacy_router", "create_runtime_router", "get_public_links"]


def create_runtime_router(service, camera_stream_service):
    router = APIRouter(tags=["runtime"])

    # 无前缀资源：摄像头推流 + 按键转发
    router.include_router(stream.build_stream_router(camera_stream_service))
    router.include_router(keypress.build_keypress_router(camera_stream_service))

    # /v1 前缀资源
    router.include_router(system.build_system_router(service))
    router.include_router(vision.build_vision_router(service, camera_stream_service))
    router.include_router(realtime.build_realtime_router(service))
    router.include_router(jobs.build_jobs_router(service))
    router.include_router(ws.build_ws_router(service, camera_stream_service))

    return router


def create_legacy_router(service):
    return legacy.build_legacy_router(service)

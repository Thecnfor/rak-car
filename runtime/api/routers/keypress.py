#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""键盘按键转发路由：/keypress。

按键透传给 camera_stream_service（历史遗留：HRI 网页按键远程转发）。
"""
try:
    from fastapi import APIRouter, Body, HTTPException
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc


def build_keypress_router(camera_stream_service):
    router = APIRouter(tags=["runtime"])

    @router.post("/keypress")
    def keypress(payload: dict = Body(default={})):
        key = payload.get("key")
        if key is None:
            raise HTTPException(status_code=400, detail="缺少 key")
        return {"ok": True, "received": camera_stream_service.set_key(key)}

    return router

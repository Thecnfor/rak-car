#!/usr/bin/python3
# -*- coding: utf-8 -*-
try:
    from fastapi import FastAPI
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.api.routes import create_legacy_router, create_runtime_router, get_public_links
from runtime.core import settings
from runtime.services.camera_stream_service import CameraStreamService
from runtime.services.runtime_service import CarRuntimeService


service = CarRuntimeService()
camera_stream_service = CameraStreamService(service)
service.set_stream_service(camera_stream_service)
_startup_ran = False


def create_app():
    app = FastAPI(
        title="rak-car runtime api",
        description="Jetson 小车常驻式调试服务",
        version="1.0.0",
    )

    @app.on_event("startup")
    def startup_event():
        global _startup_ran
        if _startup_ran:
            return
        _startup_ran = True
        service.start_background_services()
        camera_stream_service.start()
        if settings.get_auto_init_on_start():
            service.start_auto_init()

    @app.on_event("shutdown")
    def shutdown_event():
        camera_stream_service.stop()
        service.close()

    @app.get("/")
    def index():
        return {
            "service": "rak-car-runtime",
            "status": "ok",
            "links": get_public_links(),
            "config_hint": "/v1/config",
        }

    app.include_router(create_runtime_router(service, camera_stream_service))
    app.include_router(create_legacy_router(service))
    return app


app = create_app()

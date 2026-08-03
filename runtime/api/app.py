#!/usr/bin/python3
# -*- coding: utf-8 -*-
try:
    from fastapi import FastAPI
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc


from runtime.api.router_registry import create_legacy_router, create_runtime_router, get_public_links
from runtime.core import settings
from runtime.services.camera_stream_service import CameraStreamService
from runtime.services.runtime_service import CarRuntimeService


service = CarRuntimeService()
camera_stream_service = CameraStreamService(service)
service.set_stream_service(camera_stream_service)
_startup_ran = False


class NoStoreMiddleware:
    """2026-08-03：全部 HTTP 响应加 Cache-Control: no-store（纯 ASGI,零拷贝）。

    状态缓存端点（lane/arm/task/ir/odom state、health、jobs）都是实时数据,
    任何中间代理/客户端缓存都会把外环喂过期帧。
    不用 BaseHTTPMiddleware —— 它会在长连接 MJPEG 上引入缓冲层与断连悬挂问题。
    """

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        async def send_with_no_store(message):
            if message["type"] == "http.response.start":
                headers = list(message.get("headers", []))
                has_cc = any(
                    name.decode("latin-1", "ignore").lower() == "cache-control"
                    for name, _ in headers
                )
                if not has_cc:
                    headers.append((b"cache-control", b"no-store"))
                message = {**message, "headers": headers}
            await send(message)

        await self.app(scope, receive, send_with_no_store)


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
        service.shutdown()

    @app.get("/")
    def index():
        return {
            "service": "rak-car-runtime",
            "status": "ok",
            "links": get_public_links(),
            "config_hint": "/v1/config",
        }

    app.include_router(create_runtime_router(service, camera_stream_service))
    app.add_middleware(NoStoreMiddleware)
    app.include_router(create_legacy_router(service))
    # 工程化控制台静态站：StaticFiles 必须直接 app.mount（老 starlette 的
    # include_router 不携带 Mount，见 routers/web_console.py 模块 docstring）
    from .routers.web_console import mount_web_console
    mount_web_console(app)
    return app


app = create_app()

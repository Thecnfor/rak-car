#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""/console/ 工程化控制台静态站。

前端源码在仓库根 `web/`（Vite 多页：monitor 监控台 + teach 示教器），
开发机 `cd web && npm run build` 后产物落到 `runtime/static_web/` 并提交；
车端（Jetson 无 node）由本路由直接挂出 —— 与 /v1 API、/video_feed 同源同端口，
前端一律用根路径绝对地址访问。

缺失 `static_web/`（未构建）时返回指引页而不是 404。
"""
from pathlib import Path

try:
    from fastapi import APIRouter
    from fastapi.responses import HTMLResponse
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "runtime/requirements.txt"
    ) from exc

_STATIC_DIR = Path(__file__).resolve().parent.parent.parent / "static_web"

_MISSING_PAGE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="UTF-8"><title>console 未构建</title>
<style>body{background:#0f1419;color:#e0e0e0;font-family:monospace;padding:40px;}
code{background:#1c2430;padding:2px 8px;border-radius:4px;}</style></head>
<body><h2>runtime/static_web/ 不存在 —— 前端尚未构建</h2>
<p>在开发机（需要 node）执行：</p>
<pre><code>cd web &amp;&amp; npm install &amp;&amp; npm run build</code></pre>
<p>产物会写入 <code>runtime/static_web/</code>，提交后车端即可访问 /console/。</p>
<p>旧版监控页仍可用：<a href="/stream/" style="color:#00d4ff">/stream/</a></p>
</body></html>"""


def build_web_console_router():
    router = APIRouter(tags=["console"])

    if _STATIC_DIR.is_dir():
        # 延迟 import：只有构建产物存在时才引入 starlette 静态文件依赖
        try:
            from starlette.staticfiles import StaticFiles
        except ModuleNotFoundError as exc:  # pragma: no cover
            raise RuntimeError("缺少 starlette 依赖（随 fastapi 安装）") from exc
        # html=True：目录路径自动解析到 index.html（/console/、/console/monitor/）
        router.mount(
            "/console",
            StaticFiles(directory=str(_STATIC_DIR), html=True),
            name="web_console",
        )

        @router.get("/console", include_in_schema=False)
        def console_no_slash():
            from fastapi.responses import RedirectResponse
            return RedirectResponse(url="/console/")
    else:
        @router.get("/console/{rest:path}")
        def console_missing(rest: str):  # noqa: ARG001
            return HTMLResponse(_MISSING_PAGE, status_code=200)

    return router

#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys

# 兼容 `python runtime/server.py` 这种直接脚本启动方式。
if __package__ in {None, ""}:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

try:
    import uvicorn
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 uvicorn 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings


def main():
    uvicorn.run(
        "runtime.api.app:app",
        host=settings.get_bind_host(),
        port=settings.get_bind_port(),
        reload=False,
        log_level="info",
        # 2026-08-03：关 access log。50Hz realtime 轮询下每请求一行 INFO,
        # 磁盘写 + 格式化是纯开销；排障走 pm2 logs 的应用日志。
        access_log=False,
    )


if __name__ == "__main__":
    main()

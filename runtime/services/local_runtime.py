"""进程内 runtime 生命周期入口。

本地 mission 模式复用与 HTTP 服务相同的 CarRuntimeService，避免重复创建
MyCar、worker、feed 和控制器恢复线程。
"""
from __future__ import annotations

import threading
from typing import Optional, Tuple

from .camera_stream_service import CameraStreamService
from .runtime_service import CarRuntimeService


_lock = threading.RLock()
_service: Optional[CarRuntimeService] = None
_stream: Optional[CameraStreamService] = None
_started = False


def get_local_runtime() -> Tuple[CarRuntimeService, CameraStreamService]:
    """返回进程内唯一 runtime service 与 camera stream service。"""
    global _service, _stream
    with _lock:
        if _service is None:
            _service = CarRuntimeService()
            _stream = CameraStreamService(_service)
            _service.set_stream_service(_stream)
        return _service, _stream  # type: ignore[return-value]


def start_local_runtime() -> CarRuntimeService:
    """启动后台服务并请求 auto-init；重复调用安全。"""
    global _started
    service, stream = get_local_runtime()
    with _lock:
        if not _started:
            service.start_background_services()
            stream.start()
            service.start_auto_init()
            _started = True
    return service


def shutdown_local_runtime() -> None:
    """关闭本地 runtime，供 mission finally 或进程退出调用。"""
    global _service, _stream, _started
    with _lock:
        service, stream = _service, _stream
        _service = None
        _stream = None
        _started = False
    if stream is not None:
        stream.stop()
    if service is not None:
        service.shutdown()

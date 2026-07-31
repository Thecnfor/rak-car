#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""摄像头推流 / 帧 / 截图路由：/stream/*、/video_feed/*。

全部依赖 camera_stream_service，无 service（car）依赖。
"""
import time

try:
    from fastapi import APIRouter, Body, Header, HTTPException, Query, Request
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

try:
    from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc


def build_stream_router(camera_stream_service):
    router = APIRouter(tags=["runtime"])

    @router.get("/stream/info")
    def stream_info(request: Request):
        return camera_stream_service.get_stream_info(str(request.base_url).rstrip("/"))

    @router.get("/stream")
    @router.get("/stream/")
    def stream_index():
        return HTMLResponse(camera_stream_service.render_page())

    @router.get("/video_feed/{cam_id}")
    async def video_feed(cam_id: str):
        # 走 async 生成器：MJPEG 长连接不占 threadpool，不阻塞 event loop
        # 上的其他 async 端点（realtime_lane_state / vision / health）。
        return StreamingResponse(
            camera_stream_service.stream_frames_async(cam_id),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @router.get("/stream/health")
    def stream_health():
        return camera_stream_service.get_status()

    @router.get("/stream/frame/{cam_id}.jpg")
    def stream_frame(
        cam_id: str,
        download: int = Query(default=0),
        if_none_match=Header(default=None),
    ):
        """返回单帧 JPEG + ETag。客户端带 If-None-Match 时若帧未变返回 304。

        ETag 用 `streamer._jpeg_cache[cam_id]` 的 source_updated_at——编码器
        每帧刷新一次。`Cache-Control: public, max-age=1` 允许浏览器/CDN 缓存
        1s，把高频轮询的请求量再砍一档。
        """
        filename = f"{camera_stream_service.normalize_cam_id(cam_id)}.jpg"
        updated_at, _bytes = camera_stream_service.get_jpeg_meta(cam_id)
        # updated_at 可能为 None（编码器尚未跑出第一帧）——回退到 wall-clock 秒
        etag = f'W/"{int(updated_at * 1000) if updated_at else int(time.time())}"'
        headers = {
            "Cache-Control": "public, max-age=1",
            "ETag": etag,
        }
        if download == 1:
            headers["Content-Disposition"] = f'attachment; filename="{filename}"'
        if if_none_match and if_none_match == etag:
            # 客户端帧未变：返回 304 不带 body，省一次 JPEG 传输
            return Response(status_code=304, headers=headers)
        return Response(
            content=camera_stream_service.encode_jpeg_bytes(cam_id),
            media_type="image/jpeg",
            headers=headers,
        )

    @router.get("/stream/clear")
    def stream_clear(cam_id: str = Query(default=None)):
        camera_stream_service.clear_frame(cam_id)
        return {"ok": True, "cam_id": cam_id or "all"}

    @router.post("/stream/capture")
    def stream_capture(payload: dict = Body(default={})):
        cam_id = payload.get("cam_id", "cam1")
        prefix = payload.get("prefix", "capture")
        subdir = payload.get("subdir")
        try:
            capture = camera_stream_service.save_capture(
                cam_id=cam_id,
                prefix=prefix,
                subdir=subdir,
            )
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        normalized = capture["cam_id"]
        capture["download_url"] = f"/stream/captures/{normalized}/{capture['filename']}"
        if subdir:
            capture["download_url"] += f"?subdir={subdir}"
        capture["frame_url"] = f"/stream/frame/{normalized}.jpg"
        return {"ok": True, "capture": capture}

    @router.post("/stream/capture/{cam_id}/download")
    def stream_capture_download(cam_id: str, payload: dict = Body(default={})):
        prefix = payload.get("prefix", "capture")
        subdir = payload.get("subdir")
        try:
            capture = camera_stream_service.save_capture(
                cam_id=cam_id,
                prefix=prefix,
                subdir=subdir,
            )
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        file_path = camera_stream_service.get_saved_capture_path(
            cam_id=capture["cam_id"],
            filename=capture["filename"],
            subdir=subdir,
        )
        return FileResponse(
            path=str(file_path),
            media_type="image/jpeg",
            filename=capture["download_name"],
        )

    @router.get("/stream/captures/{cam_id}/{filename}")
    def stream_capture_file(
        cam_id: str,
        filename: str,
        subdir: str = Query(default=None),
        download: int = Query(default=1),
    ):
        try:
            file_path = camera_stream_service.get_saved_capture_path(
                cam_id=cam_id,
                filename=filename,
                subdir=subdir,
            )
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return FileResponse(
            path=str(file_path),
            media_type="image/jpeg",
            filename=filename if download == 1 else None,
        )

    return router

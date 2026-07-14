#!/usr/bin/python3
# -*- coding: utf-8 -*-
try:
    from fastapi import APIRouter, Body, Header, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
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

from runtime.core import settings
import time

try:
    import cv2  # 仅 /v1/vision/lane/preview.jpg 用，单独 try 避免污染启动路径
    _HAS_CV2 = True
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    _HAS_CV2 = False


def get_public_links():
    """返回带链接的 dict。**模块级缓存**——env vars 不变就不重算。

    健康检查 1Hz 轮询时省下 30+ f-string 拼接；env vars 通过 settings 模块
    的 cache-busting 接口（如果有）显式触发失效。
    """
    global _PUBLIC_LINKS_CACHE
    cached = _PUBLIC_LINKS_CACHE
    if cached is not None:
        return cached
    api_base = settings.get_public_api_base()
    v1 = settings.get_api_v1_prefix()
    legacy = settings.get_legacy_api_prefix()
    ws_base = api_base.replace("http://", "ws://").replace("https://", "wss://")
    cached = {
        "api_base": api_base,
        "docs": f"{api_base}/docs",
        "health_v1": f"{api_base}{v1}/health",
        "jobs_v1": f"{api_base}{v1}/jobs",
        "ws_v1": f"{ws_base}{v1}/ws",
        "health_legacy": f"{api_base}{legacy}/health",
        "infer_state": f"{api_base}{v1}/infer/state",
        "streamer": settings.get_public_stream_base(),
        "stream_info": f"{api_base}/stream/info",
        "stream_health": f"{api_base}/stream/health",
        "stream_cam1_frame": f"{api_base}/stream/frame/cam1.jpg",
        "stream_cam2_frame": f"{api_base}/stream/frame/cam2.jpg",
        "vision_models": f"{api_base}{v1}/vision/models",
        "vision_lane": f"{api_base}{v1}/vision/lane",
        "vision_lane_state": f"{api_base}{v1}/vision/lane/state",
        "vision_task": f"{api_base}{v1}/vision/task",
        "vision_ocr": f"{api_base}{v1}/vision/ocr",
        "realtime_wheels_speeds": f"{api_base}{v1}/realtime/wheels/speeds",
        "realtime_wheels_encoders": f"{api_base}{v1}/realtime/wheels/encoders",
        "realtime_chassis_velocity": f"{api_base}{v1}/realtime/chassis-velocity",
        "realtime_motor_speed": f"{api_base}{v1}/realtime/motor/speed",
        "realtime_encoder": f"{api_base}{v1}/realtime/encoder",
        "realtime_stepper_rad": f"{api_base}{v1}/realtime/stepper/rad",
        "realtime_bus_servo_angle": f"{api_base}{v1}/realtime/bus-servo/angle",
        "realtime_analog": f"{api_base}{v1}/realtime/analog",
        "realtime_analog2": f"{api_base}{v1}/realtime/analog2",
        "realtime_lane_state": f"{api_base}{v1}/realtime/lane/state",
    }
    _PUBLIC_LINKS_CACHE = cached
    return cached


# 模块级缓存：env vars 不变就不重算。要 invalidate 直接赋 None。
_PUBLIC_LINKS_CACHE = None


def _execute_sync(service, target, name, args=None, kwargs=None, timeout=None):
    try:
        job = service.submit_job_and_wait(
            target=target,
            name=name,
            args=args or [],
            kwargs=kwargs or {},
            timeout=timeout,
        )
    except KeyError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except TimeoutError as exc:
        raise HTTPException(status_code=504, detail=str(exc)) from exc
    if job["status"] != "succeeded":
        detail = job["error"] or "动作执行失败"
        status_code = 500
        if "推理服务未就绪" in detail:
            status_code = 503
        raise HTTPException(status_code=status_code, detail=detail)
    return job["result"]


def _frame_shape(camera_stream_service, cam_id):
    frame = camera_stream_service.get_frame(cam_id)
    if frame is None:
        return None
    return list(frame.shape)


def _bbox_to_pixels(det, image_shape):
    if not image_shape:
        return None
    img_h, img_w = image_shape[0], image_shape[1]
    x_c, y_c, width, height = det[4], det[5], det[6], det[7]
    center_x = int((x_c + 1) / 2 * img_w)
    center_y = int((y_c + 1) / 2 * img_h)
    box_w = int(width * img_w / 2)
    box_h = int(height * img_h / 2)
    x1 = int(center_x - box_w / 2)
    y1 = int(center_y - box_h / 2)
    x2 = int(center_x + box_w / 2)
    y2 = int(center_y + box_h / 2)
    return {
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "width": box_w,
        "height": box_h,
    }


def _format_detection(det, index, image_shape=None):
    return {
        "index": index,
        "class_id": det[0],
        "track_id": det[1],
        "label": det[2],
        "score": det[3],
        "bbox_norm": {
            "x_center": det[4],
            "y_center": det[5],
            "width": det[6],
            "height": det[7],
        },
        "bbox_pixels": _bbox_to_pixels(det, image_shape),
    }


def _vision_models_payload():
    return {
        "models": [
            {
                "name": "lane",
                "enabled": True,
                "camera": "cam1",
                "camera_alias": "front",
                "return_schema": {"error": "float", "angle": "float"},
                "preview_frame_url": "/stream/frame/cam1.jpg",
            },
            {
                "name": "task",
                "enabled": True,
                "camera": "cam2",
                "camera_alias": "side",
                "return_schema": {"detections": "list"},
                "preview_frame_url": "/stream/frame/cam2.jpg",
            },
            {
                "name": "ocr",
                "enabled": True,
                "camera": "cam2",
                "camera_alias": "side",
                "return_schema": {"text": "string|null", "matched_detection": "object|null"},
                "preview_frame_url": "/stream/frame/cam1.jpg",
            },
            {
                "name": "front",
                "enabled": False,
                "reason": "当前业务未使用，MyCar 未接入 front_det",
            },
        ]
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
    if op == "infer_state":
        return {"ok": True, "op": "infer_state", "data": {"infer": service.get_infer_state()}}
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

    # === 实时硬件直达 op（car_lock 同步路径，不进 job_queue） ===
    if op == "realtime/wheel_speeds":
        speeds = payload.get("speeds")
        if not isinstance(speeds, list) or len(speeds) != 4:
            raise HTTPException(status_code=400, detail="speeds 必须是长度为 4 的数组")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"result": service.set_wheel_speeds([float(s) for s in speeds])},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/chassis_velocity":
        # 外环最常用：(vx, vy, wz) 直接下发，内部 IK 反算 4 轮速、绕开 set_velocity 里程计耦合
        try:
            vx = float(payload.get("vx", 0.0))
            vy = float(payload.get("vy", 0.0))
            wz = float(payload.get("wz", 0.0))
        except (TypeError, ValueError):
            raise HTTPException(status_code=400, detail="vx/vy/wz 必须是数字")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"result": service.set_chassis_velocity(vx, vy, wz)},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/wheel_encoders":
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"encoders": service.get_wheel_encoders()},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/motor_speed":
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {
                    "result": service.set_single_motor(
                        int(port),
                        float(payload.get("speed", 0)),
                        reverse=int(payload.get("reverse", 1)),
                    )
                },
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/encoder":
        port = payload.get("port")
        reverse = payload.get("reverse", 1)
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"encoder": service.get_encoder(int(port), reverse=int(reverse))},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/stepper_rad":
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {
                    "result": service.set_stepper_rad(
                        int(port),
                        float(payload.get("rad", 0.0)),
                        time=float(payload.get("time", 0.5)),
                        reverse=int(payload.get("reverse", 1)),
                        perimeter=float(payload.get("perimeter", 0.008)),
                    )
                },
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/bus_servo_angle":
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {
                    "result": service.set_bus_servo(
                        int(port),
                        float(payload.get("angle", 0)),
                        speed=int(payload.get("speed", 100)),
                    )
                },
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/bus_servo_read":
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"angle": service.read_bus_servo(int(port))},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/analog":
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"value": service.read_analog(int(port))},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/analog2":
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"value": service.read_analog2(int(port))},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    if op == "realtime/lane_state":
        # 外环最常用：读 streamer 缓存的 lane_state。
        # 数据来源是 lane_feed 守护线程（runtime 启动后默认 20Hz）通过
        # `car.streamer.set_lane_state(...)` 持续刷新的内存缓存。
        # 不走 job_queue、不打 ZMQ、不抢 car_lock——只取 meta_lock（极快），
        # 50Hz+ 外环轮询安全，不会和 lane_feed 守护线程或 MJPEG 推流抢锁。
        try:
            return {
                "ok": True,
                "op": op,
                "data": {"lane_state": service.get_lane_state()},
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    raise HTTPException(status_code=400, detail=f"不支持的 op: {op}")


def create_runtime_router(service, camera_stream_service):
    router = APIRouter(tags=["runtime"])
    router_v1 = APIRouter(prefix=settings.get_api_v1_prefix(), tags=["runtime"])

    @router.get("/stream/info")
    def stream_info(request: Request):
        return camera_stream_service.get_stream_info(str(request.base_url).rstrip("/"))

    @router.get("/stream")
    @router.get("/stream/")
    def stream_index():
        return HTMLResponse(camera_stream_service.render_page())

    @router.get("/video_feed/{cam_id}")
    def video_feed(cam_id: str):
        return StreamingResponse(
            camera_stream_service.stream_frames(cam_id),
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

    @router.post("/keypress")
    def keypress(payload: dict = Body(default={})):
        key = payload.get("key")
        if key is None:
            raise HTTPException(status_code=400, detail="缺少 key")
        return {"ok": True, "received": camera_stream_service.set_key(key)}

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

    @router_v1.get("/infer/state")
    def v1_infer_state():
        return {"ok": True, "infer": service.get_infer_state()}

    @router_v1.get("/vision/models")
    def v1_vision_models():
        return {"ok": True, **_vision_models_payload()}

    @router_v1.post("/vision/lane")
    def v1_vision_lane(payload: dict = Body(default={})):
        timeout = payload.get("timeout", 20)
        result = _execute_sync(
            service,
            target="car",
            name="get_lane_results",
            timeout=timeout,
        )
        return {
            "ok": True,
            "model": "lane",
            "camera": "cam1",
            "frame_url": "/stream/frame/cam1.jpg",
            "preview_url": "/stream/",
            "state_url": "/v1/vision/lane/state",
            "result": {
                "error": result[0],
                "angle": result[1],
            },
            "frame_shape": _frame_shape(camera_stream_service, "cam1"),
        }

    @router_v1.get("/vision/lane/state")
    def v1_vision_lane_state():
        return {"ok": True, **camera_stream_service.get_lane_state()}

    # lane_feed 守护线程：runtime init 默认自动启动（详见 runtime_service._create_car_locked），
    # 这里不暴露 start/stop 端点 —— 比赛阶段 lane_state 必须持续更新。

    @router_v1.get("/vision/lane/preview.jpg")
    def v1_vision_lane_preview_jpg(cam_id: str = Query(default="cam1")):
        """cam1 帧 + 车道误差 overlay 一次性 JPEG。

        设计目的：lane_feed 守护线程已经不再 cv2.putText 主帧流，
        但调试时仍然想看到 "d_e:... d_a:..." 字样。这个端点：
          1) 读 streamer.frames[cam_id] 缓存（不读摄像头，不抢 _capture_loop）
          2) 读 lane_state 拿 (error_y, error_angle)
          3) 调 cv2.putText × 2 一次性画字（白底+绿字，和之前调试期一致）
          4) imencode JPEG 返回
        端点不抢摄像头、不污染 cam1 主帧流，前端 <img> 默认走
        /video_feed/cam1 拿干净流；想看叠加就切到本端点。
        """
        if not _HAS_CV2:
            raise HTTPException(status_code=503, detail="cv2 不可用，无法生成 overlay")
        normalized = camera_stream_service.normalize_cam_id(cam_id)
        frame = camera_stream_service.get_frame(normalized)
        if frame is None:
            raise HTTPException(status_code=409, detail="摄像头当前没有可用的帧")
        # 拷贝再画字，避免污染缓存
        try:
            drawn = frame.copy()
        except Exception:
            drawn = frame
        state = camera_stream_service.get_lane_state() or {}
        ey = state.get("error_y")
        ea = state.get("error_angle")
        if ey is not None and ea is not None:
            try:
                label = f"d_e:{float(ey):7.5f}  d_a:{float(ea):7.5f}"
                # 与 get_lane_results 旧实现一致：白底厚 + 绿字薄（双重 putText 防锯齿）
                cv2.putText(drawn, label, (20, 40),
                            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
                cv2.putText(drawn, label, (20, 40),
                            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1, cv2.LINE_AA)
            except cv2.error:
                # cv2 putText 偶发在 copy 后的非连续数组上失败，忽略 overlay 即可
                drawn = frame
        try:
            ret, buf = cv2.imencode(
                ".jpg", drawn,
                [int(cv2.IMWRITE_JPEG_QUALITY), 80],
            )
        except cv2.error:
            raise HTTPException(status_code=500, detail="overlay JPEG 编码失败")
        if not ret:
            raise HTTPException(status_code=500, detail="overlay JPEG 编码失败")
        return Response(content=buf.tobytes(), media_type="image/jpeg")

    @router_v1.post("/vision/task")
    def v1_vision_task(payload: dict = Body(default={})):
        timeout = payload.get("timeout", 20)
        sort_pos = payload.get("sort_pos", [0, 0])
        limit_x = payload.get("limit_x", 1)
        limit_y = payload.get("limit_y", 1)
        result = _execute_sync(
            service,
            target="car",
            name="get_detection_results",
            kwargs={
                "sort_pos": sort_pos,
                "limit_x": limit_x,
                "limit_y": limit_y,
            },
            timeout=timeout,
        )
        image_shape = _frame_shape(camera_stream_service, "cam2")
        detections = [
            _format_detection(det, index=index, image_shape=image_shape)
            for index, det in enumerate(result)
        ]
        return {
            "ok": True,
            "model": "task",
            "camera": "cam2",
            "frame_url": "/stream/frame/cam2.jpg",
            "preview_url": "/stream/",
            "filters": {
                "sort_pos": sort_pos,
                "limit_x": limit_x,
                "limit_y": limit_y,
            },
            "count": len(detections),
            "detections": detections,
            "frame_shape": image_shape,
        }

    @router_v1.post("/vision/ocr")
    def v1_vision_ocr(payload: dict = Body(default={})):
        timeout = payload.get("timeout", 20)
        label = payload.get("label")
        sort_pos = payload.get("sort_pos", [0, 0])
        limit_x = payload.get("limit_x", 1)
        limit_y = payload.get("limit_y", 1)
        detections = _execute_sync(
            service,
            target="car",
            name="get_detection_results",
            kwargs={
                "sort_pos": sort_pos,
                "limit_x": limit_x,
                "limit_y": limit_y,
            },
            timeout=timeout,
        )
        image_shape = _frame_shape(camera_stream_service, "cam2")
        formatted_detections = [
            _format_detection(det, index=index, image_shape=image_shape)
            for index, det in enumerate(detections)
        ]
        matched_index = None
        matched_det = None
        for index, det in enumerate(detections):
            det_label = det[2]
            if label is None and det_label in {"order", "name"}:
                matched_index = index
                matched_det = det
                break
            if label is not None and det_label == label:
                matched_index = index
                matched_det = det
                break
        if matched_det is None:
            return {
                "ok": True,
                "model": "ocr",
                "camera": "cam2",
                "frame_url": "/stream/frame/cam1.jpg",
                "preview_url": "/stream/",
                "label": label,
                "text": None,
                "matched_detection": None,
                "detections": formatted_detections,
                "message": "当前画面未找到匹配的 OCR 检测框",
            }
        text = _execute_sync(
            service,
            target="car",
            name="get_det_ocr",
            args=[matched_det],
            kwargs={"label": label, "time_out": timeout},
            timeout=timeout,
        )
        return {
            "ok": True,
            "model": "ocr",
            "camera": "cam2",
            "frame_url": "/stream/frame/cam1.jpg",
            "source_frame_url": "/stream/frame/cam2.jpg",
            "preview_url": "/stream/",
            "label": label,
            "text": text,
            "matched_detection": formatted_detections[matched_index],
            "detections": formatted_detections,
        }

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

    # === 实时硬件直达（car_lock 同步路径，不进 job_queue） ===

    @router_v1.post("/realtime/wheels/speeds")
    def v1_realtime_wheel_speeds(payload: dict = Body(default={})):
        speeds = payload.get("speeds")
        if not isinstance(speeds, list) or len(speeds) != 4:
            raise HTTPException(status_code=400, detail="speeds 必须是长度为 4 的数组")
        try:
            return {
                "ok": True,
                "result": service.set_wheel_speeds([float(s) for s in speeds]),
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.get("/realtime/wheels/encoders")
    def v1_realtime_wheel_encoders():
        try:
            return {"ok": True, "encoders": service.get_wheel_encoders()}
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.get("/realtime/lane/state")
    def v1_realtime_lane_state():
        """外环 50Hz 轮询 lane 误差。读 lane_feed 守护线程缓存的 lane_state。

        比 `get_lane_results` action 路径轻得多——不走 job_queue、不打 ZMQ、
        不持 car_lock。响应字段见 `camera_stream_service.get_lane_state`。
        """
        try:
            return {"ok": True, "lane_state": service.get_lane_state()}
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router_v1.post("/realtime/chassis-velocity")
    def v1_realtime_chassis_velocity(payload: dict = Body(default={})):
        """(vx, vy, wz) 直发，绕开 set_velocity 里程计耦合。供外环 50Hz 用。"""
        try:
            vx = float(payload.get("vx", 0.0))
            vy = float(payload.get("vy", 0.0))
            wz = float(payload.get("wz", 0.0))
        except (TypeError, ValueError):
            raise HTTPException(status_code=400, detail="vx/vy/wz 必须是数字")
        try:
            return {
                "ok": True,
                "result": service.set_chassis_velocity(vx, vy, wz),
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.post("/realtime/motor/speed")
    def v1_realtime_motor_speed(payload: dict = Body(default={})):
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "result": service.set_single_motor(
                    int(port),
                    float(payload.get("speed", 0)),
                    reverse=int(payload.get("reverse", 1)),
                ),
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.get("/realtime/encoder")
    def v1_realtime_encoder(
        port: int = Query(...),
        reverse: int = Query(default=1),
    ):
        try:
            return {
                "ok": True,
                "encoder": service.get_encoder(port, reverse=reverse),
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.post("/realtime/stepper/rad")
    def v1_realtime_stepper_rad(payload: dict = Body(default={})):
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "result": service.set_stepper_rad(
                    int(port),
                    float(payload.get("rad", 0.0)),
                    time=float(payload.get("time", 0.5)),
                    reverse=int(payload.get("reverse", 1)),
                    perimeter=float(payload.get("perimeter", 0.008)),
                ),
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.post("/realtime/bus-servo/angle")
    def v1_realtime_bus_servo_angle(payload: dict = Body(default={})):
        port = payload.get("port")
        if port is None:
            raise HTTPException(status_code=400, detail="缺少 port")
        try:
            return {
                "ok": True,
                "result": service.set_bus_servo(
                    int(port),
                    float(payload.get("angle", 0)),
                    speed=int(payload.get("speed", 100)),
                ),
            }
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.get("/realtime/bus-servo/angle")
    def v1_realtime_bus_servo_read(port: int = Query(...)):
        try:
            return {"ok": True, "angle": service.read_bus_servo(port)}
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.get("/realtime/analog")
    def v1_realtime_analog(port: int = Query(...)):
        try:
            return {"ok": True, "value": service.read_analog(port)}
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.get("/realtime/analog2")
    def v1_realtime_analog2(port: int = Query(...)):
        try:
            return {"ok": True, "value": service.read_analog2(port)}
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @router_v1.websocket("/ws")
    async def v1_ws(websocket: WebSocket):
        import asyncio as _asyncio

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
                    "subscribe_lane": {"op": "subscribe_lane", "note": "持续 push lane_state"},
                    "unsubscribe_lane": {"op": "unsubscribe_lane"},
                    "realtime_chassis_velocity": {"op": "realtime/chassis_velocity", "vx": 0.0, "vy": 0.0, "wz": 0.0},
                    "realtime_wheel_speeds": {"op": "realtime/wheel_speeds", "speeds": [0,0,0,0]},
                },
            }
        )

        # ---- lane_state push 后台任务 ----
        # 外环订阅：服务端在 updated_at 变化时主动 push 一次完整 lane_state。
        # 订阅存在则后台 task 一直在跑；disconnect / unsubscribe 时 cancel。
        lane_push_task = None
        lane_subscribed = False
        lane_push_hz = 20.0  # 默认 20Hz 轮询 lane_state，更新才推

        async def _lane_push_loop():
            last_updated_at = None
            interval = 1.0 / max(float(lane_push_hz), 1.0)
            while True:
                try:
                    state = camera_stream_service.get_lane_state()
                except Exception:
                    state = None
                updated_at = state.get("updated_at") if state else None
                if state and updated_at is not None and updated_at != last_updated_at:
                    last_updated_at = updated_at
                    try:
                        await websocket.send_json(
                            {"ok": True, "op": "lane_state", "data": state}
                        )
                    except Exception:
                        # 连接已断
                        return
                await _asyncio.sleep(interval)

        async def _start_lane_push():
            nonlocal lane_push_task, lane_subscribed
            if lane_subscribed and lane_push_task is not None and not lane_push_task.done():
                return False
            lane_subscribed = True
            lane_push_task = _asyncio.create_task(_lane_push_loop())
            return True

        async def _stop_lane_push():
            nonlocal lane_push_task, lane_subscribed
            lane_subscribed = False
            if lane_push_task is not None and not lane_push_task.done():
                lane_push_task.cancel()
                try:
                    await lane_push_task
                except (_asyncio.CancelledError, Exception):
                    pass
            lane_push_task = None

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

            op = payload.get("op")

            # ---- 订阅控制（不走 _handle_websocket_message，避免被解释成通用 op）----
            if op == "subscribe_lane":
                started = await _start_lane_push()
                await websocket.send_json(
                    {
                        "ok": True,
                        "op": "subscribe_lane",
                        "subscribed": started,
                        "hz": lane_push_hz,
                    }
                )
                continue
            if op == "unsubscribe_lane":
                await _stop_lane_push()
                await websocket.send_json(
                    {"ok": True, "op": "unsubscribe_lane", "subscribed": False}
                )
                continue

            request_id = payload.get("request_id")
            try:
                result = await _handle_websocket_message(service, payload)
            except HTTPException as exc:
                result = {"ok": False, "op": op, "error": exc.detail}
            except Exception as exc:  # pragma: no cover
                result = {"ok": False, "op": op, "error": str(exc)}
            if request_id is not None:
                result["request_id"] = request_id
            await websocket.send_json(result)

        # ---- disconnect 清理 ----
        await _stop_lane_push()

    router.include_router(router_v1)
    return router


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

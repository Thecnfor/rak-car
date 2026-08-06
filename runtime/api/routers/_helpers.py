#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""routes.py 拆分出的共享辅助层：payload 构建、execute helper、WS dispatch。

原来与路由定义挤在同一个 1735 行文件里；拆出后各 router 模块只 import
自己需要的 helper，职责单一。
"""
from fastapi import HTTPException

from runtime.core import settings


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
        # 2026-07-31: 新增 ir / odom realtime cache 端点。
        "realtime_ir_state": f"{api_base}{v1}/realtime/ir/state",
        "realtime_odom_state": f"{api_base}{v1}/realtime/odom/state",
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
    # 2026-07-16: target 允许 "car" / "arm" / "system"（system 用于 reset_stop_flag 等系统动作）
    target = payload.get("target")
    if target not in ("car", "arm", "system"):
        raise HTTPException(status_code=400, detail="target 必须是 'car' / 'arm' / 'system'")
    name = payload.get("name")
    args = payload.get("args", [])
    kwargs = dict(payload.get("kwargs", {}) or {})
    kwargs.pop("timeout", None)
    if not name:
        raise HTTPException(status_code=400, detail="缺少 name")
    try:
        job = service.submit_job(target=target, name=name, args=args, kwargs=kwargs)
    except KeyError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "job": job}


def _execute_from_payload(service, payload):
    # 2026-07-16: target 允许 "car" / "arm" / "system"
    target = payload.get("target")
    if target not in ("car", "arm", "system"):
        raise HTTPException(status_code=400, detail="target 必须是 'car' / 'arm' / 'system'")
    name = payload.get("name")
    args = payload.get("args", [])
    kwargs = dict(payload.get("kwargs", {}) or {})
    # 2026-07-16：timeout 由 runtime 自己用（submit_job_and_wait），不让它透传给 SDK action
    kwargs.pop("timeout", None)
    timeout = payload.get("timeout")
    # D 改造：默认异步（立即返回 job_id，状态查 /v1/jobs/{id}）。
    # 旧同步调用方传 sync=True 拿原语义（submit_job_and_wait，阻塞到完成）。
    sync = bool(payload.get("sync", False))
    if not name:
        raise HTTPException(status_code=400, detail="缺少 name")
    if sync:
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
    # 异步：立即返回 job dict（status=queued），不阻塞。
    try:
        job = service.submit_job(
            target=target, name=name, args=args, kwargs=kwargs
        )
    except KeyError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "async": True, "job": job}


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


# ===== WS dispatch table =====
# 每个 op 一个独立小函数（语义单一、好测、好维护），统一签名
# `(service, payload) -> data_dict` 或 `-> None`（特殊：ping/pong）。
# 出错由函数体抛 HTTPException（保持原 25 路 if 链的对外行为不变）。

def _ws_op_ping(_service, _payload):
    return {"ok": True, "op": "pong"}


def _ws_op_health(service, payload):
    return {"ok": True, "op": "health", "data": health_payload(service, include_snapshot=bool(payload.get("snapshot")))}


def _ws_op_runtime(service, _payload):
    return {"ok": True, "op": "runtime", "data": _build_runtime_snapshot(service)}


def _ws_op_actions(service, _payload):
    return {"ok": True, "op": "actions", "data": {"actions": service.list_actions()}}


def _ws_op_config(_service, _payload):
    return {"ok": True, "op": "config", "data": {"config": settings.get_runtime_settings()}}


def _ws_op_infer_state(service, payload):
    return {"ok": True, "op": "infer_state", "data": {"infer": service.get_infer_state()}}


def _ws_op_infer_drop_oldest(service, payload):
    return {"ok": True, "op": "infer_drop_oldest", "data": service.infer_drop_oldest(timeout_s=payload.get("timeout_s"))}


def _ws_op_create_job(service, payload):
    return {"ok": True, "op": "create_job", "data": _create_job_from_payload(service, payload)}


def _ws_op_get_job(service, payload):
    job_id = payload.get("job_id")
    if not job_id:
        raise HTTPException(status_code=400, detail="缺少 job_id")
    return {"ok": True, "op": "get_job", "data": _get_job(service, job_id)}


def _ws_op_execute(service, payload):
    return {"ok": True, "op": "execute", "data": _execute_from_payload(service, payload)}


def _ws_op_init(service, payload):
    return {"ok": True, "op": "init", "data": _submit_init_job(service, payload)}


def _ws_op_stop_mode(service, payload):
    return {"ok": True, "op": "stop_mode", "data": _submit_stop_mode_job(service, payload)}


def _ws_op_reset_stop(service, _payload):
    return {"ok": True, "op": "reset_stop", "data": _submit_simple_system_job(service, "reset_stop_flag")}


def _ws_op_close(service, _payload):
    return {"ok": True, "op": "close", "data": _submit_simple_system_job(service, "close")}


def _ws_op_emergency_stop(service, _payload):
    return {"ok": True, "op": "emergency_stop", "data": {"stopped": service.emergency_stop()}}


# === 实时硬件直达 op（car_lock 同步路径，不进 job_queue） ===

def _ws_op_realtime_wheel_speeds(service, payload):
    speeds = payload.get("speeds")
    if not isinstance(speeds, list) or len(speeds) != 4:
        raise HTTPException(status_code=400, detail="speeds 必须是长度为 4 的数组")
    try:
        result = service.set_wheel_speeds([float(s) for s in speeds])
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/wheel_speeds", "data": {"result": result}}


def _ws_op_realtime_chassis_velocity(service, payload):
    """外环最常用：(vx, vy, wz) 直接下发，内部 IK 反算 4 轮速、绕开 set_velocity 里程计耦合。"""
    try:
        vx = float(payload.get("vx", 0.0))
        vy = float(payload.get("vy", 0.0))
        wz = float(payload.get("wz", 0.0))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="vx/vy/wz 必须是数字")
    try:
        result = service.set_chassis_velocity(vx, vy, wz)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/chassis_velocity", "data": {"result": result}}


def _ws_op_realtime_wheel_encoders(service, _payload):
    try:
        encoders = service.get_wheel_encoders()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/wheel_encoders", "data": {"encoders": encoders}}


def _ws_op_realtime_motor_speed(service, payload):
    port = payload.get("port")
    if port is None:
        raise HTTPException(status_code=400, detail="缺少 port")
    try:
        result = service.set_single_motor(
            int(port),
            float(payload.get("speed", 0)),
            reverse=int(payload.get("reverse", 1)),
        )
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/motor_speed", "data": {"result": result}}


def _ws_op_realtime_encoder(service, payload):
    port = payload.get("port")
    if port is None:
        raise HTTPException(status_code=400, detail="缺少 port")
    reverse = payload.get("reverse", 1)
    try:
        encoder = service.get_encoder(int(port), reverse=int(reverse))
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/encoder", "data": {"encoder": encoder}}


def _ws_op_realtime_stepper_rad(service, payload):
    port = payload.get("port")
    if port is None:
        raise HTTPException(status_code=400, detail="缺少 port")
    try:
        result = service.set_stepper_rad(
            int(port),
            float(payload.get("rad", 0.0)),
            time=float(payload.get("time", 0.5)),
            reverse=int(payload.get("reverse", 1)),
            perimeter=float(payload.get("perimeter", 0.008)),
        )
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/stepper_rad", "data": {"result": result}}


def _ws_op_realtime_bus_servo_angle(service, payload):
    port = payload.get("port")
    if port is None:
        raise HTTPException(status_code=400, detail="缺少 port")
    try:
        result = service.set_bus_servo(
            int(port),
            float(payload.get("angle", 0)),
            speed=int(payload.get("speed", 100)),
        )
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/bus_servo_angle", "data": {"result": result}}


def _ws_op_realtime_bus_servo_read(service, payload):
    port = payload.get("port")
    if port is None:
        raise HTTPException(status_code=400, detail="缺少 port")
    try:
        angle = service.read_bus_servo(int(port))
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/bus_servo_read", "data": {"angle": angle}}


def _ws_op_realtime_analog(service, payload):
    port = payload.get("port")
    if port is None:
        raise HTTPException(status_code=400, detail="缺少 port")
    try:
        value = service.read_analog(int(port))
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/analog", "data": {"value": value}}


def _ws_op_realtime_analog2(service, payload):
    port = payload.get("port")
    if port is None:
        raise HTTPException(status_code=400, detail="缺少 port")
    try:
        value = service.read_analog2(int(port))
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/analog2", "data": {"value": value}}


def _ws_op_realtime_lane_state(service, _payload):
    """外环最常用：读 streamer 缓存的 lane_state。

    数据来源是 lane_feed 守护线程（runtime 启动后默认 50Hz，2026-07-16 上调）通过
    `car.streamer.set_lane_state(...)` 持续刷新的内存缓存。
    不走 job_queue、不打 ZMQ、不抢 car_lock——只取 meta_lock（极快），
    50Hz+ 外环轮询安全，不会和 lane_feed 守护线程或 MJPEG 推流抢锁。
    """
    try:
        lane_state = service.get_lane_state()
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/lane_state", "data": {"lane_state": lane_state}}


def _ws_op_realtime_arm_state(service, _payload):
    """WS op=realtime/arm_state: 读 streamer 缓存的 arm_state 一次。

    数据来源是 arm_feed 守护线程(runtime 启动后默认 20Hz)通过
    `car.streamer.set_arm_state(...)` 持续刷新的内存缓存。
    不走 job_queue、不打 ZMQ、不抢 car_lock——只取 meta_lock(极快)。
    """
    try:
        arm_state = service.get_arm_state()
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/arm_state", "data": {"arm_state": arm_state}}


def _ws_op_realtime_task_state(service, _payload):
    """WS op=realtime/task_state: 读 streamer 缓存的 task_state 一次（侧摄目标检测）。

    数据来源是 task_feed 守护线程(runtime 启动后默认 10Hz)通过
    `car.streamer.set_task_state(...)` 持续刷新的内存缓存。
    不走 job_queue、不打 ZMQ、不抢 car_lock——只取 meta_lock(极快)。
    让业务层"边走边看"侧摄目标成为可能（之前 /v1/vision/task 是 sync 5-15s 阻塞）。
    """
    try:
        task_state = service.get_task_state()
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/task_state", "data": {"task_state": task_state}}


def _ws_op_realtime_ir_state(service, _payload):
    """WS op=realtime/ir_state: 读 streamer 缓存的 ir_state 一次（左右 IR 距离）。"""
    try:
        ir_state = service.get_ir_state()
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/ir_state", "data": {"ir_state": ir_state}}


def _ws_op_realtime_odom_state(service, _payload):
    """WS op=realtime/odom_state: 读 streamer 缓存的 odom_state 一次（底盘里程计）。"""
    try:
        odom_state = service.get_odom_state()
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "op": "realtime/odom_state", "data": {"odom_state": odom_state}}


# op string → handler。统一 `(service, payload) -> dict`。
_WS_OP_DISPATCH = {
    "ping": _ws_op_ping,
    "health": _ws_op_health,
    "runtime": _ws_op_runtime,
    "actions": _ws_op_actions,
    "config": _ws_op_config,
    "infer_state": _ws_op_infer_state,
    "infer_drop_oldest": _ws_op_infer_drop_oldest,
    "create_job": _ws_op_create_job,
    "get_job": _ws_op_get_job,
    "execute": _ws_op_execute,
    "init": _ws_op_init,
    "stop_mode": _ws_op_stop_mode,
    "reset_stop": _ws_op_reset_stop,
    "close": _ws_op_close,
    "emergency_stop": _ws_op_emergency_stop,
    "realtime/wheel_speeds": _ws_op_realtime_wheel_speeds,
    "realtime/chassis_velocity": _ws_op_realtime_chassis_velocity,
    "realtime/wheel_encoders": _ws_op_realtime_wheel_encoders,
    "realtime/motor_speed": _ws_op_realtime_motor_speed,
    "realtime/encoder": _ws_op_realtime_encoder,
    "realtime/stepper_rad": _ws_op_realtime_stepper_rad,
    "realtime/bus_servo_angle": _ws_op_realtime_bus_servo_angle,
    "realtime/bus_servo_read": _ws_op_realtime_bus_servo_read,
    "realtime/analog": _ws_op_realtime_analog,
    "realtime/analog2": _ws_op_realtime_analog2,
    "realtime/lane_state": _ws_op_realtime_lane_state,
    "realtime/arm_state": _ws_op_realtime_arm_state,
    "realtime/task_state": _ws_op_realtime_task_state,
    "realtime/ir_state": _ws_op_realtime_ir_state,
    "realtime/odom_state": _ws_op_realtime_odom_state,
}

# 2026-08-06 无创优化：把同步 realtime / job handler 丢到 threadpool，
# 不再在 asyncio event loop 内同步执行。realtime/wheel_speeds 等会走
# SerialEngine round-trip（最坏 time_out+2.0s），串口慢时整条 loop 推迟，
# lane/arm/ir/odom push 全部跟着抖。to_thread 后 loop 几乎不被阻塞，
# 客户端的 WS 收帧保持节奏。
# 接口零变化：handler 签名 (service, payload) -> dict，response 包络不变。
# 内部 dispatch 表、op 集合、_handle_websocket_message 的调用方都保持原状。
async def _handle_websocket_message(service, payload):
    op = payload.get("op", "execute")
    handler = _WS_OP_DISPATCH.get(op)
    if handler is None:
        raise HTTPException(status_code=400, detail=f"不支持的 op: {op}")
    import asyncio as _asyncio
    # 2026-08-07 修复: to_thread 是 Python 3.9+ 才有的 API, Jetson /usr/bin/python3
    # 是 3.8 → AttributeError, 所有 WS realtime op 返回 ok=False, 外环 wheel_speeds
    # 静默失败, 车不走。get_event_loop().run_in_executor() 3.6+ 通用, 语义等价。
    loop = _asyncio.get_event_loop()
    return await loop.run_in_executor(None, handler, service, payload)

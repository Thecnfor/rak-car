#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""WebSocket 路由：/v1/ws。

通用 op 走 `_handle_websocket_message`（dispatch 表在 _helpers）；订阅类 op
（subscribe_*/unsubscribe_*）在连接作用域内维护各自的 push 后台 task，
disconnect 时全部 cancel。
"""
try:
    from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings

from ._helpers import _handle_websocket_message, get_public_links


def build_ws_router(service, camera_stream_service):
    router_v1 = APIRouter(prefix=settings.get_api_v1_prefix(), tags=["runtime"])

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
                    "subscribe_arm_state": {"op": "subscribe_arm_state", "note": "持续 push arm_state (y/x 位置)"},
                    "unsubscribe_arm_state": {"op": "unsubscribe_arm_state"},
                    "subscribe_task_detection": {"op": "subscribe_task_detection", "note": "持续 push 侧摄 task_state (边走边看)"},
                    "unsubscribe_task_detection": {"op": "unsubscribe_task_detection"},
                    "subscribe_ir": {"op": "subscribe_ir", "note": "持续 push 左右 IR 距离 (50Hz)"},
                    "unsubscribe_ir": {"op": "unsubscribe_ir"},
                    "subscribe_odom": {"op": "subscribe_odom", "note": "持续 push 底盘里程计 (50Hz)"},
                    "unsubscribe_odom": {"op": "unsubscribe_odom"},
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
        lane_push_hz = 50.0  # 默认 50Hz 轮询 lane_state（2026-07-16 上调），更新才推
        # ---- arm_state push 后台任务(同 lane 模式)----
        arm_push_task = None
        arm_subscribed = False
        arm_push_hz = 20.0

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

        # ---- arm_state push loop ----
        async def _arm_push_loop():
            last_updated_at = None
            interval = 1.0 / max(float(arm_push_hz), 1.0)
            while True:
                try:
                    state = service.get_arm_state()
                except Exception:
                    state = None
                updated_at = state.get("updated_at") if state else None
                if state and updated_at is not None and updated_at != last_updated_at:
                    last_updated_at = updated_at
                    try:
                        await websocket.send_json(
                            {"ok": True, "op": "arm_state", "data": state}
                        )
                    except Exception:
                        return
                await _asyncio.sleep(interval)

        async def _start_arm_push():
            nonlocal arm_push_task, arm_subscribed
            if arm_subscribed and arm_push_task is not None and not arm_push_task.done():
                return False
            arm_subscribed = True
            arm_push_task = _asyncio.create_task(_arm_push_loop())
            return True

        async def _stop_arm_push():
            nonlocal arm_push_task, arm_subscribed
            arm_subscribed = False
            if arm_push_task is not None and not arm_push_task.done():
                arm_push_task.cancel()
                try:
                    await arm_push_task
                except (_asyncio.CancelledError, Exception):
                    pass
            arm_push_task = None

        # ---- task_state push loop（边走边看侧摄目标）----
        task_push_task = None
        task_subscribed = False
        task_push_hz = 30.0  # 2026-07-31: 匹配 task_feed 默认 30Hz（my_car.py:1461），原 10Hz 浪费 3 倍数据

        async def _task_push_loop():
            last_updated_at = None
            interval = 1.0 / max(float(task_push_hz), 1.0)
            while True:
                try:
                    state = service.get_task_state()
                except Exception:
                    state = None
                updated_at = state.get("updated_at") if state else None
                if state and updated_at is not None and updated_at != last_updated_at:
                    last_updated_at = updated_at
                    try:
                        await websocket.send_json(
                            {"ok": True, "op": "task_state", "data": state}
                        )
                    except Exception:
                        return
                await _asyncio.sleep(interval)

        async def _start_task_push():
            nonlocal task_push_task, task_subscribed
            if task_subscribed and task_push_task is not None and not task_push_task.done():
                return False
            task_subscribed = True
            task_push_task = _asyncio.create_task(_task_push_loop())
            return True

        async def _stop_task_push():
            nonlocal task_push_task, task_subscribed
            task_subscribed = False
            if task_push_task is not None and not task_push_task.done():
                task_push_task.cancel()
                try:
                    await task_push_task
                except (_asyncio.CancelledError, Exception):
                    pass
            task_push_task = None

        # ---- 2026-07-31：ir_state push loop（外环/触发判定专用）----
        ir_push_task = None
        ir_subscribed = False
        ir_push_hz = 50.0  # 与 ir_feed 默认刷新频率同档

        async def _ir_push_loop():
            last_updated_at = None
            interval = 1.0 / max(float(ir_push_hz), 1.0)
            while True:
                try:
                    state = service.get_ir_state()
                except Exception:
                    state = None
                updated_at = state.get("updated_at") if state else None
                if state and updated_at is not None and updated_at != last_updated_at:
                    last_updated_at = updated_at
                    try:
                        await websocket.send_json(
                            {"ok": True, "op": "ir_state", "data": state}
                        )
                    except Exception:
                        return
                await _asyncio.sleep(interval)

        async def _start_ir_push():
            nonlocal ir_push_task, ir_subscribed
            if ir_subscribed and ir_push_task is not None and not ir_push_task.done():
                return False
            ir_subscribed = True
            ir_push_task = _asyncio.create_task(_ir_push_loop())
            return True

        async def _stop_ir_push():
            nonlocal ir_push_task, ir_subscribed
            ir_subscribed = False
            if ir_push_task is not None and not ir_push_task.done():
                ir_push_task.cancel()
                try:
                    await ir_push_task
                except (_asyncio.CancelledError, Exception):
                    pass
            ir_push_task = None

        # ---- 2026-07-31：odom_state push loop ----
        odom_push_task = None
        odom_subscribed = False
        odom_push_hz = 50.0

        async def _odom_push_loop():
            last_updated_at = None
            interval = 1.0 / max(float(odom_push_hz), 1.0)
            while True:
                try:
                    state = service.get_odom_state()
                except Exception:
                    state = None
                updated_at = state.get("updated_at") if state else None
                if state and updated_at is not None and updated_at != last_updated_at:
                    last_updated_at = updated_at
                    try:
                        await websocket.send_json(
                            {"ok": True, "op": "odom_state", "data": state}
                        )
                    except Exception:
                        return
                await _asyncio.sleep(interval)

        async def _start_odom_push():
            nonlocal odom_push_task, odom_subscribed
            if odom_subscribed and odom_push_task is not None and not odom_push_task.done():
                return False
            odom_subscribed = True
            odom_push_task = _asyncio.create_task(_odom_push_loop())
            return True

        async def _stop_odom_push():
            nonlocal odom_push_task, odom_subscribed
            odom_subscribed = False
            if odom_push_task is not None and not odom_push_task.done():
                odom_push_task.cancel()
                try:
                    await odom_push_task
                except (_asyncio.CancelledError, Exception):
                    pass
            odom_push_task = None

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
            if op == "subscribe_arm_state":
                started = await _start_arm_push()
                await websocket.send_json(
                    {
                        "ok": True,
                        "op": "subscribe_arm_state",
                        "subscribed": started,
                        "hz": arm_push_hz,
                    }
                )
                continue
            if op == "unsubscribe_arm_state":
                await _stop_arm_push()
                await websocket.send_json(
                    {"ok": True, "op": "unsubscribe_arm_state", "subscribed": False}
                )
                continue
            if op == "subscribe_task_detection":
                started = await _start_task_push()
                await websocket.send_json(
                    {
                        "ok": True,
                        "op": "subscribe_task_detection",
                        "subscribed": started,
                        "hz": task_push_hz,
                    }
                )
                continue
            if op == "unsubscribe_task_detection":
                await _stop_task_push()
                await websocket.send_json(
                    {"ok": True, "op": "unsubscribe_task_detection", "subscribed": False}
                )
                continue
            # 2026-07-31：IR / odom 订阅控制（同 task_detection 模式）
            if op == "subscribe_ir":
                started = await _start_ir_push()
                await websocket.send_json(
                    {
                        "ok": True,
                        "op": "subscribe_ir",
                        "subscribed": started,
                        "hz": ir_push_hz,
                    }
                )
                continue
            if op == "unsubscribe_ir":
                await _stop_ir_push()
                await websocket.send_json(
                    {"ok": True, "op": "unsubscribe_ir", "subscribed": False}
                )
                continue
            if op == "subscribe_odom":
                started = await _start_odom_push()
                await websocket.send_json(
                    {
                        "ok": True,
                        "op": "subscribe_odom",
                        "subscribed": started,
                        "hz": odom_push_hz,
                    }
                )
                continue
            if op == "unsubscribe_odom":
                await _stop_odom_push()
                await websocket.send_json(
                    {"ok": True, "op": "unsubscribe_odom", "subscribed": False}
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
        await _stop_arm_push()
        await _stop_task_push()
        await _stop_ir_push()
        await _stop_odom_push()

    return router_v1

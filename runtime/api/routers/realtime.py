#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""实时硬件直达路由：/v1/realtime/*。

都是 car_lock 同步路径、不进 job_queue。缓存型端点（lane/arm/ir/odom/task
state）只取 meta_lock，供 50Hz 外环轮询。
"""
try:
    from fastapi import APIRouter, Body, HTTPException, Query
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 FastAPI 依赖，请先执行: /usr/bin/python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/runtime/requirements.txt"
    ) from exc

from runtime.core import settings


def build_realtime_router(service):
    router_v1 = APIRouter(prefix=settings.get_api_v1_prefix(), tags=["runtime"])

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

    @router_v1.get("/realtime/arm/state")
    def v1_realtime_arm_state():
        """调试/UI 用:读 arm_feed 守护线程缓存的机械臂 y/x 位置。

        与 /v1/realtime/lane/state 完全同构:不走 job_queue、不打 ZMQ、不持 car_lock,
        只取 meta_lock(极快),20Hz+ 轮询安全。响应字段见 `camera_stream_service.get_arm_state`。
        """
        try:
            return {"ok": True, "arm_state": service.get_arm_state()}
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router_v1.get("/realtime/ir/state")
    def v1_realtime_ir_state():
        """外环/触发判定专用：读 ir_feed 守护线程缓存的左右 IR 距离。

        与 /v1/realtime/lane/state 完全同构：不走 job_queue、不打 ZMQ、不抢 car_lock，
        只取 meta_lock（极快）。ir_feed 默认 50Hz 刷新。

        返回字段见 `camera_stream_service.get_ir_state`：
          `{active, mode, left, right, updated_at}`。
        """
        try:
            return {"ok": True, "ir_state": service.get_ir_state()}
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router_v1.get("/realtime/odom/state")
    def v1_realtime_odom_state():
        """外环/触发判定专用：读 odom_feed 守护线程缓存的底盘里程计。

        与 /v1/realtime/lane/state 完全同构：不走 job_queue、不打 ZMQ、不抢 car_lock，
        只取 meta_lock（极快）。odom_feed 默认 50Hz 刷新。

        返回字段见 `camera_stream_service.get_odom_state`：
          `{active, mode, x, y, theta, distance, updated_at}`。
        """
        try:
            return {"ok": True, "odom_state": service.get_odom_state()}
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router_v1.get("/realtime/vision/task")
    def v1_realtime_task_state():
        """边走边看侧摄目标:读 task_feed 守护线程缓存的最新一次目标检测结果。

        与 /v1/realtime/lane/state 完全同构:不走 job_queue、不打 ZMQ、不持 car_lock,
        只取 meta_lock(极快)。task_feed 默认 10Hz 刷新。

        之前 /v1/vision/task 是 sync POST（5-15s 阻塞）,"边走边看"做不到。
        现在业务层可以 50Hz 轮询本端点 + 同步下发轮速,实现实时闭环。
        """
        try:
            return {"ok": True, "task_state": service.get_task_state()}
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

    @router_v1.post("/realtime/arm-velocity")
    def v1_realtime_arm_velocity(payload: dict = Body(default={})):
        """arm (x_vel, y_vel) 直发 — 绕开 arm_queue, 供视觉伺服速度模式。

        与 /realtime/chassis-velocity 同构: 走 _realtime_gate 免 car_lock, 不进 job_queue。
        单位 m/s (与 arm_base.x_speed / y_speed 一致)。
        x_vel / y_vel 缺省或 null 表示该轴不动; 传 0.0 显式停。
        ⚠️ 只发速度不做位置闭环; 调用方负责收敛 (检测丢失时发 0 停), 否则 x 可能直行撞墙。
        """
        try:
            x_vel = payload.get("x_vel")
            y_vel = payload.get("y_vel")
            if x_vel is not None:
                x_vel = float(x_vel)
            if y_vel is not None:
                y_vel = float(y_vel)
        except (TypeError, ValueError):
            raise HTTPException(status_code=400, detail="x_vel/y_vel 必须是数字")
        try:
            return {
                "ok": True,
                "result": service.set_arm_velocity(x_vel, y_vel),
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

    return router_v1

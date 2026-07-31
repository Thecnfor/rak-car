#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/_helpers.py

任务层共用的 motion helpers: 把原来散在 auto_seeding.py / water_tower_task.py
以及 task6 (get_order.py) 里的私有函数统一抽到这里, 避免 3 份重复实现和
"import 不存在的 main.task.auto_seeding_safe" 死代码。

业务层只走 /v1/execute HTTP client (= RuntimeApiClient), 跟运行时端保持薄封装。
所有函数失败语义 = raise RuntimeError, 调用方在 task 顶层 try/except 包。

约定:
  - X/Y 单位 mm, 角度单位 °, 距离单位 m
  - 同步等待 (sync=True), 超时由调用方传 timeout 参数控制
  - 不持 car_lock, 也不感知 chassis outer loop —— 仅发动作指令
"""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient

logger = logging.getLogger("task.helpers")


# ============================================================
# runtime 就绪 / 推理就绪
# ============================================================

def _ensure_runtime(client: RuntimeApiClient, timeout_s: float = 10.0) -> None:
    """等 runtime 进入 PROGRAM_READY 状态。

    失败时 raise, 提示运维 pm2 restart rak-car-api.
    """
    if not client.wait_until_ready(timeout=timeout_s):
        raise RuntimeError("runtime not ready, check pm2 logs rak-car-api")


def _wait_infer_ready(client: RuntimeApiClient, timeout_s: float = 30.0) -> None:
    """探 /v1/health 等 task 模型 ready。

    失败时 raise, 提示运维 pm2 restart rak-car-api.
    """
    deadline = time.time() + timeout_s
    last: Any = None
    while time.time() < deadline:
        try:
            h = client.get_health(snapshot=True)
        except Exception as exc:
            last = f"health call failed: {exc}"
            time.sleep(1.0)
            continue
        last = h
        # health response shape: {ok, state: {infer_service: {models: [...]}}}
        state = h.get("state") or {}
        infer = state.get("infer_service") or {}
        models = infer.get("models") or []
        task = next((m for m in models if m.get("name") == "task"), None)
        if task and task.get("ready") and task.get("response"):
            logger.info("task inference backend ready (port=%s)", task.get("port"))
            return
        time.sleep(1.0)
    raise RuntimeError(
        f"task inference backend not ready within {timeout_s}s; "
        f"on Jetson run: pm2 restart rak-car-api. last={last}"
    )


# ============================================================
# 机械臂: X / Y / 大臂角度 / 末端角度
# ============================================================

def _move_x(
    client: RuntimeApiClient,
    x_mm: float,
    v_max_mms: float = 80.0,
    out_time: float = 15.0,
    timeout: float = 30.0,
) -> None:
    """move_x_position PID 闭环 (X 编码器反馈准确到位)。

    v_max_mms 收紧 PID output_limits → 避免第一帧全速 jerk.
    """
    job = client.execute(
        "arm", "move_x_position",
        args=[x_mm / 1000.0],
        kwargs={"v_max_mms": v_max_mms, "out_time": out_time},
        sync=True, timeout=timeout + 5,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(
            "arm move_x({:.0f}) failed: status={} error={}".format(
                x_mm, job.get("status"), job.get("error")
            )
        )


def _move_y(client: RuntimeApiClient, y_mm: float, timeout: float = 25.0) -> None:
    """move_y_position PID 闭环 (y 步进电机, 不动舵机)。"""
    job = client.execute(
        "arm", "move_y_position",
        args=[y_mm / 1000.0],
        sync=True, timeout=timeout,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(
            "arm move_y({:.0f}) failed: status={} error={}".format(
                y_mm, job.get("status"), job.get("error")
            )
        )


def _set_arm_angle(
    client: RuntimeApiClient,
    angle_deg: float,
    speed: int = 80,
    timeout: float = 20.0,
    retries: int = 2,
) -> None:
    """set_arm_angle with retry. Runtime 在繁忙操作后偶尔响应慢, 8s timeout 可恢复。"""
    last = None
    for attempt in range(1, retries + 1):
        try:
            job = client.execute(
                "arm", "set_arm_angle",
                args=[angle_deg, speed],
                sync=True, timeout=timeout + 5,
            )
            if job.get("status") == "succeeded" and not job.get("error"):
                return
            last = "status={} error={}".format(job.get("status"), job.get("error"))
        except Exception as exc:
            last = "{}: {}".format(type(exc).__name__, exc)[:200]
        logger.warning(
            "set_arm_angle(%.0f) attempt %d/%d failed: %s",
            angle_deg, attempt, retries, last,
        )
        if attempt < retries:
            time.sleep(1.0)
    raise RuntimeError(
        "set_arm_angle({}) failed after {} retries: {}".format(angle_deg, retries, last)
    )


def _set_hand_angle(
    client: RuntimeApiClient,
    angle_deg: float,
    speed: int = 80,
    timeout: float = 10.0,
    retries: int = 2,
) -> None:
    """set_hand_angle with retry. 同 _set_arm_angle 重试策略。"""
    last = None
    for attempt in range(1, retries + 1):
        try:
            job = client.execute(
                "arm", "set_hand_angle",
                args=[angle_deg, speed],
                sync=True, timeout=timeout + 5,
            )
            if job.get("status") == "succeeded" and not job.get("error"):
                return
            last = "status={} error={}".format(job.get("status"), job.get("error"))
        except Exception as exc:
            last = "{}: {}".format(type(exc).__name__, exc)[:200]
        logger.warning(
            "set_hand_angle(%.0f) attempt %d/%d failed: %s",
            angle_deg, attempt, retries, last,
        )
        if attempt < retries:
            time.sleep(1.0)
    raise RuntimeError(
        "set_hand_angle({}) failed after {} retries: {}".format(angle_deg, retries, last)
    )


# ============================================================
# 吸盘 (真空取放)
# ============================================================

def _grasp(client: RuntimeApiClient, on: bool, timeout: float = 10.0) -> None:
    """grasp(true) 开阀吸气关阀保真空, grasp(false) 放气释放。"""
    job = client.execute(
        "arm", "grasp", args=[bool(on)],
        sync=True, timeout=timeout,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(
            "grasp({}) failed: status={} error={}".format(
                on, job.get("status"), job.get("error")
            )
        )


# ============================================================
# 底盘 move_for
# ============================================================

def _chassis_move_for(
    client: RuntimeApiClient,
    dx_m: float,
    dy_m: float = 0.0,
    dtheta_rad: float = 0.0,
    timeout: float = 30.0,
) -> None:
    """底盘相对位移 (move_for): dx>0 前进 / dy>0 左移 / dtheta>0 逆时针。"""
    job = client.execute(
        "car",
        "move_for",
        args=[[dx_m, dy_m, dtheta_rad]],
        timeout=timeout,
        sync=True,
    )
    if job.get("status") != "succeeded" or job.get("error"):
        raise RuntimeError(
            "chassis move_for failed: status={} error={}".format(
                job.get("status"), job.get("error")
            )
        )


# ============================================================
# 带位置校验的 X 移动 (task6 用, 防 PID 超调撞杆)
# ============================================================

def _move_x_checked(
    client: RuntimeApiClient,
    x_mm: float,
    v_max_mms: float = 40.0,
    tolerance_mm: float = 3.0,
    timeout: float = 30.0,
) -> None:
    """X 移动 + 完成后读回实际位置校验, 偏差 > tolerance 时低速 (30mm/s) 再纠一次。

    防 PID 在方向切换时 (扫动向右→复位向左) 超调导致撞杆。
    """
    _move_x(client, x_mm, v_max_mms=v_max_mms, timeout=timeout)

    actual = _read_x_mm(client)
    if actual is None:
        logger.warning("_move_x_checked: cannot read back X position, assume ok")
        return

    err = actual - x_mm
    logger.info("_move_x_checked(%.0f): actual=%.0f err=%.1f mm", x_mm, actual, err)

    if abs(err) > tolerance_mm:
        logger.warning(
            "X overshoot! actual=%.0f target=%.0f err=%.1f mm — correcting at 30 mm/s",
            actual, x_mm, err,
        )
        _move_x(client, x_mm, v_max_mms=30.0, timeout=timeout)
        time.sleep(0.3)
        actual2 = _read_x_mm(client)
        if actual2 is not None:
            err2 = actual2 - x_mm
            logger.info("_move_x_checked retry: actual=%.0f err=%.1f mm", actual2, err2)
            if abs(err2) > tolerance_mm:
                raise RuntimeError(
                    "X position correction failed: target=%.0f actual=%.0f err=%.1f mm"
                    % (x_mm, actual2, err2)
                )


def _read_x_mm(client: RuntimeApiClient) -> Optional[float]:
    """读取当前 X 轴实际位置 (mm)。arm_feed daemon 缓存 (20Hz)。"""
    try:
        resp = client.get("/v1/realtime/arm/state", timeout=3)
        return (resp.get("arm_state") or {}).get("x_mm", None)
    except Exception:
        return None
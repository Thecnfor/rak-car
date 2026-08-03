#!/usr/bin/python
# -*- coding: utf-8 -*-
"""守护线程 FeedsMixin：lane / arm / task / ir / odom 五个缓存刷新线程。

从 my_car.py 拆出（原 1304-2202 行）。每个 feed 一个独立后台线程，只刷
streamer 的 *_state 缓存，不下发任何控制指令。用户决策：拆为 FeedsMixin
继承——MyCar 仍持有线程，但循环主体独立成模块，可单独阅读/维护。

统一模式：
  - start_*_feed(hz) 幂等启动（already_running fast-path / 清旧建新）
  - stop_*_feed(force=False) 默认 no-op，force=True 才真停（消费者生命周期
    绝不该杀生产者守护线程）
  - restart_*_feed(force)  force 才 stop+start，否则只 start
  - 每个 _*_loop 内层 backoff + 外层 while True 兜底，守护线程绝不自然死亡
  - _*_feed_health 心跳字段，供 runtime_service 的 feed watchdog 巡检
"""
import threading
import time

from smartcar import logger
from smartcar.paddlebaidu.infer_cs import ClintInterface


def _new_health():
    """守护线程心跳模板（lane / arm / task / ir / odom 共用）。

    字段被 runtime 的 feed watchdog 巡检：alive=False 表示显式 stop；
    last_iter_at / last_ok_at 距今超阈值视为"卡死"要 restart。
    """
    return {
        "alive": True,
        "started_at": time.time(),
        "last_iter_at": 0.0,
        "last_ok_at": 0.0,
        "iter_count": 0,
        "ok_count": 0,
        "err_count": 0,
        "last_err": None,
        "last_err_at": 0.0,
    }


def _stop_feed(self, attr_base, force):
    """通用 feed 停止骨架：force=False 一律 no-op（消费者生命周期不该杀生产者）。

    attr_base: "lane_feed" / "arm_feed" / "task_feed" / "ir_feed" / "odom_feed"。
    stop 的 5 份重复实现收敛成这一个；各 feed 的 start 因 loop 差异保留显式。
    """
    lock = getattr(self, "_{}_lock".format(attr_base), None)
    if lock is None:
        return {"stopped": True, "reason": "never_started"}
    if not force:
        logger.warning("stop_{} called without force=True → NOOP".format(attr_base))
        return {"stopped": False, "reason": "noop_without_force"}
    with lock:
        stop_event = getattr(self, "_{}_stop".format(attr_base), None)
        thread = getattr(self, "_{}_thread".format(attr_base), None)
    if stop_event is None and thread is None:
        return {"stopped": True, "reason": "never_started"}
    if stop_event is not None:
        stop_event.set()
    joined = True
    if thread is not None and thread.is_alive():
        thread.join(timeout=5.0)
        joined = not thread.is_alive()
    with lock:
        if joined:
            setattr(self, "_{}_thread".format(attr_base), None)
            setattr(self, "_{}_stop".format(attr_base), None)
    return {"stopped": True, "joined": joined}


class FeedsMixin:
    """MyCar 的守护线程缓存刷新行为。"""

    def _lane_feed_inner_loop(
        self, stop_event, period, get_frame, norm_id, feed_infer, set_state, health
    ):
        """lane_feed 单次"健康循环"。外层 while True + try/except 包住,任何崩溃都不允许守护线程死亡。

        2026-08-03 异步流水线改造:旧版本 "取帧→同步 get_infer→写状态" 三步串行,
        单次推理慢时整条链路停摆,实测 lane_feed 长期只跑到 ~1.66Hz 而非目标 20-50Hz。

        新版本 "取最新帧(丢弃过时)→非阻塞 send→非阻塞 poll 拿旧结果→有结果写状态":
          - 抓帧永远跟得上 period(50ms / 20ms)
          - 推理在飞时只用最新的帧(快路径下推理 8ms 也只丢 1 帧)
          - 推理慢(如 GC / 偶发卡顿)也不会把采样率拉到 500ms+
          - 维持 backoff 健康统计 + outer while 兜底不变
        """
        import zmq
        backoff = 1
        max_backoff = 20  # 20 * period = 1s @ period=50ms
        inflight_inflight = False  # 当前是否有未返回的推理请求
        inflight_started_at = 0.0
        inflight_timeout_s = max(0.5, period * 10)  # 推理超时 = 10 个 period
        last_send_at = 0.0
        socket_lock = getattr(feed_infer, "_socket_lock", None)
        while not stop_event.is_set():
            health["last_iter_at"] = time.time()
            health["iter_count"] += 1
            try:
                # ---- 1) 非阻塞 poll:拿上次 inflight 推理的应答 ----
                if inflight_inflight:
                    sock = getattr(feed_infer, "client", None)
                    if sock is not None:
                        recv_done = False
                        recv_result = None
                        if socket_lock is not None:
                            with socket_lock:
                                try:
                                    result_bytes = sock.recv(zmq.NOBLOCK)
                                    recv_done = True
                                except zmq.error.Again:
                                    recv_done = False
                        else:
                            try:
                                result_bytes = sock.recv(zmq.NOBLOCK)
                                recv_done = True
                            except zmq.error.Again:
                                recv_done = False
                        if recv_done:
                            try:
                                import json as _json
                                recv_result = _json.loads(result_bytes)
                            except Exception:
                                recv_result = result_bytes
                            inflight_inflight = False
                            if isinstance(recv_result, (list, tuple)) and len(recv_result) >= 2:
                                error_y = float(recv_result[0])
                                error_angle = float(recv_result[1])
                                backoff = 1
                                health["ok_count"] += 1
                                health["last_ok_at"] = time.time()
                                health["last_err"] = None
                                if set_state is not None:
                                    set_state(
                                        active=True,
                                        mode="external_feed",
                                        error_y=error_y,
                                        error_angle=error_angle,
                                        forward_speed=None,
                                        lateral_speed=None,
                                        angular_speed=None,
                                        distance=self.get_distance(),
                                    )
                        else:
                            # 没应答,看是否超时
                            if time.time() - inflight_started_at > inflight_timeout_s:
                                inflight_inflight = False
                                health["err_count"] += 1
                                health["last_err"] = "lane_infer_timeout(>{}s)".format(inflight_timeout_s)
                                health["last_err_at"] = time.time()
                                if health["err_count"] <= 5 or health["err_count"] % 20 == 0:
                                    logger.warning("lane_feed: infer timeout, drop inflight")

                # ---- 2) 取最新帧(同步但只读共享 buffer,极快) ----
                img = get_frame(norm_id) if get_frame is not None else None
                if img is None:
                    if stop_event.wait(period):
                        return
                    continue

                # ---- 3) 没有 inflight 时 → 非阻塞 send 一次 ----
                if not inflight_inflight:
                    sock = getattr(feed_infer, "client", None)
                    if sock is not None:
                        send_done = False
                        if socket_lock is not None:
                            with socket_lock:
                                try:
                                    sock.send(img, flags=zmq.NOBLOCK)
                                    send_done = True
                                except zmq.error.Again:
                                    send_done = False
                        else:
                            try:
                                sock.send(img, flags=zmq.NOBLOCK)
                                send_done = True
                            except zmq.error.Again:
                                send_done = False
                        if send_done:
                            inflight_inflight = True
                            inflight_started_at = time.time()
                            last_send_at = inflight_started_at
                        # send 失败:本帧丢,下次再试
                    else:
                        # 兜底:用 ClintInterface.get_infer(同步) — fallback
                        result = feed_infer.get_infer(img)
                        if isinstance(result, (list, tuple)) and len(result) >= 2:
                            health["ok_count"] += 1
                            health["last_ok_at"] = time.time()
                            if set_state is not None:
                                set_state(
                                    active=True, mode="external_feed",
                                    error_y=float(result[0]),
                                    error_angle=float(result[1]),
                                    forward_speed=None, lateral_speed=None,
                                    angular_speed=None,
                                    distance=self.get_distance(),
                                )
                # 否则:本帧被丢(已有 in-flight,等下次再送) — 防止堆积
            except Exception as exc:
                wait_s = min(period * backoff, period * max_backoff)
                health["err_count"] += 1
                health["last_err"] = str(exc)
                health["last_err_at"] = time.time()
                if health["err_count"] <= 5 or health["err_count"] % 20 == 0:
                    logger.warning(
                        "lane feed transient err (errs=%d, backoff=%d, wait=%.2fs): %s",
                        health["err_count"], backoff, wait_s, exc,
                    )
                backoff = min(backoff * 2, max_backoff)
                inflight_inflight = False
                if set_state is not None:
                    try:
                        set_state(
                            active=True,
                            mode="external_feed_stale",
                            error_y=0.0,
                            error_angle=0.0,
                            forward_speed=None,
                            lateral_speed=None,
                            angular_speed=None,
                            distance=self.get_distance(),
                            last_error=str(exc),
                        )
                    except Exception:
                        pass
                if stop_event.wait(wait_s):
                    return
                continue
            # 节奏 sleep:取帧与 inflight poll 都很轻,period 控制频率
            if stop_event.wait(period):
                return

    def start_lane_feed(self, hz: float = 50.0):
        """
        启动 lane 误差缓存守护线程（只刷 lane_state，不下发轮速）。

        给客户端外环用：外环不在车上、但要稳定拿到 (error_y, error_angle)，
        由本方法在车端起一个守护线程持续跑 get_lane_results(),
        并把结果写到 streamer.set_lane_state(...)。

        重要：不会调用任何 set_velocity / set_wheel_speeds,
        不会与客户端外环的轮速下发抢锁。

        2026-08-01 鲁棒性改造:
          - 守护线程拥有**独立**的 ClintInterface 实例,不再复用 self.crusie。
            业务路径 (get_lane_results) 和守护线程共用 self.crusie 时,REQ socket
            串行化互相 block,一次慢调用会让 lane_feed 抖到 EAGAIN。
          - 守护线程不再因 self._stop_flag=True 退出 (急停只是"硬件协作停", 不
            应该关闭 lane_feed),只响应 stop_event。
          - 推理失败时仍写 active=True + last_error, 前端能看到 "守护线程还活着,
            只是推理抖了"。
          - 加 _lane_feed_health 心跳字段, runtime_service 的 watchdog 拿这个判断。
        """
        with self._lane_feed_lock:
            # 2026-08-01：修复幂等检查。
            # 旧检查: _lane_feed_thread is not None AND is_alive()
            #   - 守护线程**自然死**（while 退出 / 未捕获异常）时 _lane_feed_thread 引用
            #     仍存在但 is_alive=False → 检查 True and False = False → start 不 return
            #     → 重复创建**第二个**守护线程！然后 2x feed 同时打 ZMQ 5001 → 雪崩。
            # 新检查: 只要 is_alive() 就返回（thread is None 时它也不是 alive 的）。
            try:
                alive_now = (
                    self._lane_feed_thread is not None
                    and self._lane_feed_thread.is_alive()
                )
            except Exception:
                alive_now = False
            if alive_now:
                return {"started": False, "reason": "already_running", "hz": hz}
            self._lane_feed_stop = threading.Event()
            stop_event = self._lane_feed_stop
            period = 1.0 / max(float(hz), 1.0)

            # 守护线程独立 ClintInterface,与业务 self.crusie 隔开。
            # 即便业务路径短时卡 socket 也不影响 lane_feed。
            try:
                feed_infer = ClintInterface("lane")
            except Exception as exc:
                logger.warning("lane_feed 创建独立 ClintInterface 失败: %s", exc)
                feed_infer = self.crusie  # fallback

            def _feed_loop():
                streamer = getattr(self, "streamer", None)
                set_state = getattr(streamer, "set_lane_state", None) if streamer else None
                get_frame = getattr(streamer, "get_frame", None) if streamer else None
                norm_id = (
                    streamer.normalize_cam_id("cam1")
                    if (streamer is not None and hasattr(streamer, "normalize_cam_id"))
                    else "cam1"
                )

                # 心跳字段:watchdog 用它判断"守护线程是否还在喂数据"
                health = _new_health()
                self._lane_feed_health = health

                # backoff 状态:连续异常不退出,只拉长间隔
                backoff = 1
                max_backoff = 20  # 20 * period = 1s @ period=50ms
                # 2026-08-01:不再因 _stop_flag 退出 — 急停只应该停硬件,守护线程
                # 应当保持跑(后续可被 stop_event 或 watchdog 收尾)。
                # 2026-08-01:最外层加 while True + try/except 包住整个健康循环,
                # 任何"未捕获的致命异常"(set_state 抛 / self 引用被置 None / streamer 被替换)
                # 都只退出内层、外层立即重建 health + 继续跑。守护线程**绝不**自然死亡。
                outer_crash = 0
                while not stop_event.is_set():
                    try:
                        self._lane_feed_inner_loop(
                            stop_event=stop_event,
                            period=period,
                            get_frame=get_frame,
                            norm_id=norm_id,
                            feed_infer=feed_infer,
                            set_state=set_state,
                            health=health,
                        )
                    except Exception as exc:
                        outer_crash += 1
                        health["err_count"] += 1
                        health["last_err"] = "outer_crash#{}: {}".format(outer_crash, exc)
                        health["last_err_at"] = time.time()
                        if outer_crash <= 5 or outer_crash % 20 == 0:
                            logger.warning(
                                "lane_feed outer_loop crashed (crash#%d), re-enter in %.2fs: %s",
                                outer_crash, min(period * 20, 1.0), exc,
                            )
                        # 重建 health 里的 iter 标记,让 watchdog 看到 "活着"
                        health["last_iter_at"] = time.time()
                        stop_event.wait(min(period * 20, 1.0))
                        continue
                # 正常退出路径(只有显式 stop_lane_feed 才走这里)
                health["alive"] = False
                if set_state is not None:
                    try:
                        set_state(
                            active=False,
                            mode="idle",
                            forward_speed=None,
                            lateral_speed=None,
                            angular_speed=None,
                            distance=self.get_distance(),
                        )
                    except Exception:
                        pass

            self._lane_feed_thread = threading.Thread(target=_feed_loop, name="lane-feed")
            self._lane_feed_thread.daemon = True
            self._lane_feed_thread.start()
            # 把独立 infer client 引用挂在实例上,debug 时可观察
            self._lane_feed_infer = feed_infer
            return {"started": True, "hz": hz}

    def stop_lane_feed(self, force: bool = False):
        """
        停止 lane 误差缓存守护线程，并把 lane_state 复位成 idle。

        设计原则: **消费者脚本的生命周期绝不应该杀掉生产者守护线程。**
        默认 force=False → no-op, 只有显式 force=True(运维 / runtime 关闭路径) 才真的停。
        """
        return _stop_feed(self, "lane_feed", force)

    def restart_lane_feed(self, hz: float = 50.0, force: bool = False):
        """force=True 才 stop+start；否则只是 start(幂等,不丢状态)。"""
        if force:
            self.stop_lane_feed(force=True)
        return self.start_lane_feed(hz=hz)

    # === arm 位置推送守护线程（实时 y/x,给 WS subscribe_arm_state 订阅） ===
    def start_arm_feed(self, hz: float = 20.0):
        """启动机械臂 y/x 位置守护线程,持续刷新 streamer.arm_state。

        与 start_lane_feed 模式一致:
        - 不抢 car_lock(arm 位置用 arm.y_get_position / x_get_position,SDK 内部读编码器无锁)
        - 不会与 move_y / reset_y 等动作互斥(读 vs 写,SDK 编码器读是独立的)
        - 订阅一次即可一直 push,disconnect / unsubscribe 时 cancel
        """
        if not hasattr(self, "_arm_feed_lock"):
            self._arm_feed_lock = threading.Lock()
            self._arm_feed_thread = None
            self._arm_feed_stop = None
        with self._arm_feed_lock:
            try:
                alive_now = (
                    self._arm_feed_thread is not None
                    and self._arm_feed_thread.is_alive()
                )
            except Exception:
                alive_now = False
            if alive_now:
                return {"started": False, "reason": "already_running", "hz": hz}
            self._arm_feed_stop = threading.Event()
            stop_event = self._arm_feed_stop
            period = 1.0 / max(float(hz), 1.0)

            def _arm_feed_loop():
                streamer = getattr(self, "streamer", None)
                set_state = getattr(streamer, "set_arm_state", None) if streamer else None
                # 2026-08-01：心跳字段, runtime_service 的 feed watchdog 巡检用
                health = _new_health()
                self._arm_feed_health = health
                # 2026-08-01：外层绝不死 while True + try/except
                outer_crash = 0
                while not stop_event.is_set():
                    try:
                        health["last_iter_at"] = time.time()
                        health["iter_count"] += 1
                        try:
                            y_m = self.arm.y_get_position() if self.arm is not None else None
                            x_m = self.arm.x_get_position() if self.arm is not None else None
                            y_mm = (y_m * 1000.0) if y_m is not None else None
                            x_mm = (x_m * 1000.0) if x_m is not None else None
                            ref = getattr(self.arm, "_y_ref_encoder_at_zero", None)
                            health["ok_count"] += 1
                            health["last_ok_at"] = time.time()
                            health["last_err"] = None
                            if set_state is not None:
                                set_state(
                                    active=True,
                                    mode="arm_feed",
                                    y_m=y_m, x_m=x_m,
                                    y_mm=y_mm, x_mm=x_mm,
                                    ref_encoder=ref,
                                )
                        except Exception as exc:
                            health["err_count"] += 1
                            health["last_err"] = str(exc)
                            health["last_err_at"] = time.time()
                            if health["err_count"] <= 5 or health["err_count"] % 20 == 0:
                                logger.warning("arm feed transient err (errs=%d): %s", health["err_count"], exc)
                    except Exception as exc:
                        outer_crash += 1
                        health["err_count"] += 1
                        health["last_err"] = "outer_crash#{}: {}".format(outer_crash, exc)
                        health["last_err_at"] = time.time()
                        if outer_crash <= 5 or outer_crash % 20 == 0:
                            logger.warning("arm_feed outer_loop crashed#%d: %s", outer_crash, exc)
                        health["last_iter_at"] = time.time()
                        stop_event.wait(min(period * 20, 1.0))
                        continue
                    if stop_event.wait(period):
                        break
                health["alive"] = False
                if set_state is not None:
                    try:
                        set_state(active=False, mode="idle")
                    except Exception:
                        pass

            self._arm_feed_thread = threading.Thread(target=_arm_feed_loop, name="arm-feed")
            self._arm_feed_thread.daemon = True
            self._arm_feed_thread.start()
            return {"started": True, "hz": hz}

    def stop_arm_feed(self, force: bool = False):
        return _stop_feed(self, "arm_feed", force)

    def restart_arm_feed(self, hz: float = 20.0, force: bool = False):
        """force=True 才 stop+start；否则只是 start(幂等)。"""
        if force:
            self.stop_arm_feed(force=True)
        return self.start_arm_feed(hz=hz)

    # === 侧摄目标检测推送守护线程（实时 task 检测结果,给 WS subscribe_task_detection 订阅） ===
    def _task_feed_inner_loop(
        self, stop_event, period, get_stream_frame, get_fallback, feed_infer, set_state, health,
    ):
        """task_feed 单次"健康循环",外层 while True + try/except 包住,守护线程绝不死。"""
        backoff = 1
        max_backoff = 20  # 20 * period = 1s @ period~33ms
        while not stop_event.is_set():
            health["last_iter_at"] = time.time()
            health["iter_count"] += 1
            try:
                img = None
                if get_stream_frame is not None:
                    try:
                        img = get_stream_frame("cam2")
                    except Exception:
                        img = None
                if img is None and get_fallback is not None:
                    img = get_fallback()
                if img is None:
                    stop_event.wait(period)
                    continue
                if feed_infer is None:
                    stop_event.wait(period)
                    continue
                raw = feed_infer.get_infer(img)
                backoff = 1
                health["ok_count"] += 1
                health["last_ok_at"] = time.time()
                health["last_err"] = None
                if set_state is not None:
                    set_state(
                        active=True,
                        mode="task_feed",
                        frame_shape=list(img.shape) if img is not None else None,
                        detections=[
                            {
                                "cls_id": int(d[0]) if len(d) > 0 else None,
                                "det_id": int(d[1]) if len(d) > 1 else None,
                                "label": str(d[2]) if len(d) > 2 else "",
                                "score": float(d[3]) if len(d) > 3 else 0.0,
                                "bbox_norm": {
                                    "x_center": float(d[4]) if len(d) > 4 else 0.0,
                                    "y_center": float(d[5]) if len(d) > 5 else 0.0,
                                    "width":    float(d[6]) if len(d) > 6 else 0.0,
                                    "height":   float(d[7]) if len(d) > 7 else 0.0,
                                }
                            }
                            for d in (raw or []) if len(d) >= 8
                        ],
                        count=len(raw or []),
                    )
            except Exception as exc:
                wait_s = min(period * backoff, period * max_backoff)
                health["err_count"] += 1
                health["last_err"] = str(exc)
                health["last_err_at"] = time.time()
                if health["err_count"] <= 5 or health["err_count"] % 20 == 0:
                    logger.warning(
                        "task feed transient err (errs=%d, backoff=%d, wait=%.2fs): %s",
                        health["err_count"], backoff, wait_s, exc,
                    )
                backoff = min(backoff * 2, max_backoff)
                if set_state is not None:
                    try:
                        set_state(
                            active=True,
                            mode="task_feed_stale",
                            detections=[],
                            count=0,
                            last_error=str(exc),
                        )
                    except Exception:
                        pass
                if stop_event.wait(wait_s):
                    return
                continue
            if stop_event.wait(period):
                return

    def start_task_feed(self, hz: float = 30.0):
        """启动侧摄目标检测守护线程,持续刷新 streamer.task_state。

        与 start_lane_feed / start_arm_feed 模式一致:
          - 不抢 car_lock(读摄像头 + ZMQ 推理是独立 IO)
          - 默认 30Hz（task 模型 ~30-50ms/次,30Hz 是上限,适合机械臂动态捕捉目标）
          - 不会下发任何控制指令,只刷新 task_state 缓存
          - 订阅一次即可一直 push,disconnect / unsubscribe 时 cancel

        2026-07-16 改造:
          - 数据源改走 streamer.get_frame("cam2") (与 lane_feed 同模式),避免直接读摄像头
            与 camera_stream_service._capture_loop 抢 GIL + frame_lock
          - 默认 hz 10→30,适配机械臂实时动态捕捉目标场景
          - 增加 fallback: 如果 streamer 取不到帧(摄像头掉线),fallback 到 cap_side.read()
            (Camera.read 是同步阻塞等 camera.update 线程写 self.frame)

        2026-08-01 鲁棒性改造:
          - 守护线程拥有**独立** ClintInterface,不再复用 self.task_det。
            arm_runner / get_detection_results 等业务路径会瞬时抢 socket。
          - 守护线程不再因 self._stop_flag 退出,只响应 stop_event。
          - 推理失败时写 active=True + last_error。
          - 加 _task_feed_health 心跳字段。

        业务场景:"边走边看"侧摄目标 — 不必每帧调 sync /v1/vision/task（5-15s 阻塞）,
        直接读 /v1/realtime/vision/task 缓存或订阅 WS subscribe_task_detection。
        """
        if not hasattr(self, "_task_feed_lock"):
            self._task_feed_lock = threading.Lock()
            self._task_feed_thread = None
            self._task_feed_stop = None
        with self._task_feed_lock:
            try:
                alive_now = (
                    self._task_feed_thread is not None
                    and self._task_feed_thread.is_alive()
                )
            except Exception:
                alive_now = False
            if alive_now:
                return {"started": False, "reason": "already_running", "hz": hz}
            self._task_feed_stop = threading.Event()
            stop_event = self._task_feed_stop
            period = 1.0 / max(float(hz), 1.0)

            # 守护线程独立 ClintInterface,与业务 self.task_det 隔开
            try:
                feed_infer = ClintInterface("task")
            except Exception as exc:
                logger.warning("task_feed 创建独立 ClintInterface 失败: %s", exc)
                feed_infer = self.task_det if hasattr(self, "task_det") else None

            def _task_feed_loop():
                streamer = getattr(self, "streamer", None)
                set_state = getattr(streamer, "set_task_state", None) if streamer else None
                # 2026-07-16: 优先走 streamer.get_frame (camera_stream_service 维护的 cache),
                # fallback 到 cap_side.read() (Camera.read 同步阻塞等 update 线程写 self.frame)
                get_stream_frame = getattr(streamer, "get_frame", None) if streamer else None
                get_fallback = getattr(self.cap_side, "read", None) if hasattr(self, "cap_side") else None

                # 心跳
                health = _new_health()
                self._task_feed_health = health

                backoff = 1
                max_backoff = 20  # 20 * period = 1s @ period~33ms
                # 2026-08-01:最外层 while True + try/except 包住,守护线程绝不死
                outer_crash = 0
                while not stop_event.is_set():
                    try:
                        self._task_feed_inner_loop(
                            stop_event=stop_event,
                            period=period,
                            get_stream_frame=get_stream_frame,
                            get_fallback=get_fallback,
                            feed_infer=feed_infer,
                            set_state=set_state,
                            health=health,
                        )
                    except Exception as exc:
                        outer_crash += 1
                        health["err_count"] += 1
                        health["last_err"] = "outer_crash#{}: {}".format(outer_crash, exc)
                        health["last_err_at"] = time.time()
                        if outer_crash <= 5 or outer_crash % 20 == 0:
                            logger.warning(
                                "task_feed outer_loop crashed (crash#%d), re-enter in %.2fs: %s",
                                outer_crash, min(period * 20, 1.0), exc,
                            )
                        health["last_iter_at"] = time.time()
                        stop_event.wait(min(period * 20, 1.0))
                        continue
                # 显式 stop 路径
                health["alive"] = False
                if set_state is not None:
                    try:
                        set_state(active=False, mode="idle", detections=[], count=0)
                    except Exception:
                        pass

            self._task_feed_thread = threading.Thread(target=_task_feed_loop, name="task-feed")
            self._task_feed_thread.daemon = True
            self._task_feed_thread.start()
            self._task_feed_infer = feed_infer
            return {"started": True, "hz": hz}

    def stop_task_feed(self, force: bool = False):
        """停止 task_feed 守护线程,并把 task_state 复位成 idle。

        默认 force=False → no-op；只有显式 force=True(运维/关闭) 才真停。
        """
        return _stop_feed(self, "task_feed", force)

    def restart_task_feed(self, hz: float = 30.0, force: bool = False):
        """force=True 才 stop+start；否则只是 start(幂等)。"""
        if force:
            self.stop_task_feed(force=True)
        return self.start_task_feed(hz=hz)

    # === 2026-07-31：左右 IR 距离缓存守护线程（实时 IR,给 /realtime/ir/state 轮询 / WS 订阅） ===
    def start_ir_feed(self, hz: float = 50.0):
        """启动左右 IR 距离守护线程,持续刷新 streamer.ir_state。

        与 start_lane_feed / start_arm_feed / start_task_feed 模式完全一致:
          - 默认 50Hz（与 lane_feed 同档,env RAK_CAR_IR_FEED_HZ 可覆盖）
          - 不抢 car_lock：读 IR 是 MC602 字节往返(内部 SDK 锁),
            与 move_y / move_x 不互斥,但慢(~10-30ms/次),适合后台串行喂缓存
          - 不会下发任何控制指令,只刷新 ir_state 缓存
          - 订阅一次即可一直 push,disconnect / unsubscribe 时 cancel

        业务场景: main/chassis/tasks/read_ir.py、orchestrator._wait_until_triggered
        不再每次都进 job_queue + car_lock + MC602 字节往返,直接读缓存。
        """
        # 2026-07-31 修：init 后被 force-init 重建过 car 引用的 race —— car 实例对象
        # 是新的,_ir_feed_hz 属性可能不存在,用 getattr 防御
        cur_hz = getattr(self, "_ir_feed_hz", None)
        cur_thread = getattr(self, "_ir_feed_thread", None)
        cur_lock = getattr(self, "_ir_feed_lock", None)
        cur_stop = getattr(self, "_ir_feed_stop", None)
        if cur_lock is None:
            self._ir_feed_lock = threading.Lock()
            self._ir_feed_thread = None
            self._ir_feed_stop = None
            self._ir_feed_hz = None
        else:
            self._ir_feed_lock = cur_lock
            self._ir_feed_thread = cur_thread
            self._ir_feed_stop = cur_stop
            self._ir_feed_hz = cur_hz
        with self._ir_feed_lock:
            # 同一 hz 已经在跑，直接 fast-path（即使别的并发 caller 刚 restart 完）。
            cur_thread = self._ir_feed_thread
            cur_hz = self._ir_feed_hz
            if (
                cur_thread is not None
                and cur_thread.is_alive()
                and cur_hz is not None
                and float(cur_hz) == float(hz)
            ):
                return {"started": False, "reason": "already_running", "hz": hz}
            # 否则无论 alive 与否，都走"清旧建新"——也覆盖了"join 失败但活进程失去响应"边界
            if self._ir_feed_thread is not None:
                if self._ir_feed_stop is not None:
                    self._ir_feed_stop.set()
                if self._ir_feed_thread.is_alive():
                    self._ir_feed_thread.join(timeout=2.0)
            self._ir_feed_thread = None
            self._ir_feed_stop = None
            self._ir_feed_stop = threading.Event()
            stop_event = self._ir_feed_stop
            self._ir_feed_hz = float(hz)
            period = 1.0 / max(float(hz), 1.0)

            def _ir_feed_loop():
                streamer = getattr(self, "streamer", None)
                set_state = getattr(streamer, "set_ir_state", None) if streamer else None
                # 2026-08-01：心跳字段, feed watchdog 巡检用
                health = _new_health()
                self._ir_feed_health = health
                # 与 lane_feed / arm_feed 同模式：连续异常不退出,
                # backoff 封顶 1s,正常一次立刻复位 period
                backoff = 1
                max_backoff = 20  # 20 * period = 1s @ period=50ms
                # 2026-08-01：外层 while True + try/except,守护线程绝不死
                outer_crash = 0
                while not stop_event.is_set():
                    try:
                        health["last_iter_at"] = time.time()
                        health["iter_count"] += 1
                        try:
                            irs = self.get_all_ir_distance() if hasattr(self, "get_all_ir_distance") else {}
                            left = irs.get("left") if isinstance(irs, dict) else None
                            right = irs.get("right") if isinstance(irs, dict) else None
                            backoff = 1
                            health["ok_count"] += 1
                            health["last_ok_at"] = time.time()
                            health["last_err"] = None
                            if set_state is not None:
                                set_state(
                                    active=True,
                                    mode="ir_feed",
                                    left=float(left) if left is not None else None,
                                    right=float(right) if right is not None else None,
                                )
                        except Exception as exc:
                            health["err_count"] += 1
                            health["last_err"] = str(exc)
                            health["last_err_at"] = time.time()
                            wait_s = min(period * backoff, period * max_backoff)
                            if health["err_count"] <= 5 or health["err_count"] % 20 == 0:
                                logger.warning(
                                    "ir feed transient err (errs=%d, backoff=%d, wait=%.2fs): %s",
                                    health["err_count"], backoff, wait_s, exc,
                                )
                            backoff = min(backoff * 2, max_backoff)
                            if stop_event.wait(wait_s):
                                break
                            continue
                        if stop_event.wait(period):
                            break
                    except Exception as exc:
                        outer_crash += 1
                        health["err_count"] += 1
                        health["last_err"] = "outer_crash#{}: {}".format(outer_crash, exc)
                        health["last_err_at"] = time.time()
                        if outer_crash <= 5 or outer_crash % 20 == 0:
                            logger.warning("ir_feed outer_loop crashed#%d: %s", outer_crash, exc)
                        health["last_iter_at"] = time.time()
                        stop_event.wait(min(period * 20, 1.0))
                        continue
                health["alive"] = False
                if set_state is not None:
                    try:
                        set_state(active=False, mode="idle", left=None, right=None)
                    except Exception:
                        pass

            self._ir_feed_thread = threading.Thread(target=_ir_feed_loop, name="ir-feed")
            self._ir_feed_thread.daemon = True
            self._ir_feed_thread.start()
            return {"started": True, "hz": hz}

    def stop_ir_feed(self, force: bool = False):
        """停止 ir_feed 守护线程,并把 ir_state 复位成 idle。

        默认 force=False → no-op；显式 force=True(运维/关闭) 才真停。
        """
        return _stop_feed(self, "ir_feed", force)

    def restart_ir_feed(self, hz: float = 50.0, force: bool = False):
        """force=True 才 stop+start；否则只是 start(幂等)。"""
        if force:
            self.stop_ir_feed(force=True)
        return self.start_ir_feed(hz=hz)

    # === 2026-07-31：底盘里程计缓存守护线程（实时 odom,给 /realtime/odom/state 轮询 / WS 订阅） ===
    def start_odom_feed(self, hz: float = 50.0):
        """启动底盘里程计守护线程,持续刷新 streamer.odom_state。

        与 start_ir_feed 同模式:
          - 默认 50Hz（env RAK_CAR_ODOM_FEED_HZ 可覆盖）
          - 读 get_odometry / get_distance 都只是内存读（带 _ref_lock 短锁）,
            比 IR 廉价很多,但走原路径每次要抢 _ref_lock + job_queue 也慢
          - 不下发任何控制指令,只刷新 odom_state 缓存

        业务场景: main/chassis/api.py.get_odometry、orchestrator 的 distance 轮询,
        不再每次都进 job_queue,主路径延迟接近 0。
        """
        # 2026-07-31 修：init 后被 force-init 重建过 car 引用的 race —— car 实例对象
        # 是新的,_odom_feed_hz 属性可能不存在,用 getattr 防御
        cur_hz = getattr(self, "_odom_feed_hz", None)
        cur_thread = getattr(self, "_odom_feed_thread", None)
        cur_lock = getattr(self, "_odom_feed_lock", None)
        cur_stop = getattr(self, "_odom_feed_stop", None)
        if cur_lock is None:
            self._odom_feed_lock = threading.Lock()
            self._odom_feed_thread = None
            self._odom_feed_stop = None
            self._odom_feed_hz = None
        else:
            self._odom_feed_lock = cur_lock
            self._odom_feed_thread = cur_thread
            self._odom_feed_stop = cur_stop
            self._odom_feed_hz = cur_hz
        with self._odom_feed_lock:
            cur_thread = self._odom_feed_thread
            cur_hz = self._odom_feed_hz
            if (
                cur_thread is not None
                and cur_thread.is_alive()
                and cur_hz is not None
                and float(cur_hz) == float(hz)
            ):
                return {"started": False, "reason": "already_running", "hz": hz}
            if self._odom_feed_thread is not None:
                if self._odom_feed_stop is not None:
                    self._odom_feed_stop.set()
                if self._odom_feed_thread.is_alive():
                    self._odom_feed_thread.join(timeout=2.0)
            self._odom_feed_thread = None
            self._odom_feed_stop = None
            self._odom_feed_stop = threading.Event()
            stop_event = self._odom_feed_stop
            self._odom_feed_hz = float(hz)
            period = 1.0 / max(float(hz), 1.0)

            def _odom_feed_loop():
                streamer = getattr(self, "streamer", None)
                set_state = getattr(streamer, "set_odom_state", None) if streamer else None
                # 2026-08-01：心跳字段
                health = _new_health()
                self._odom_feed_health = health
                # 2026-08-01：外层 while True + try/except,绝不死
                outer_crash = 0
                while not stop_event.is_set():
                    try:
                        health["last_iter_at"] = time.time()
                        health["iter_count"] += 1
                        try:
                            pos = self.get_odometry() if hasattr(self, "get_odometry") else None
                            x = float(pos[0]) if pos is not None and len(pos) > 0 else None
                            y = float(pos[1]) if pos is not None and len(pos) > 1 else None
                            theta = float(pos[2]) if pos is not None and len(pos) > 2 else None
                            distance = (
                                float(self.get_distance()) if hasattr(self, "get_distance") else None
                            )
                            health["ok_count"] += 1
                            health["last_ok_at"] = time.time()
                            health["last_err"] = None
                            if set_state is not None:
                                set_state(
                                    active=True,
                                    mode="odom_feed",
                                    x=x, y=y, theta=theta, distance=distance,
                                )
                        except Exception as exc:
                            health["err_count"] += 1
                            health["last_err"] = str(exc)
                            health["last_err_at"] = time.time()
                            if health["err_count"] <= 5 or health["err_count"] % 20 == 0:
                                logger.warning("odom feed transient err (errs=%d): %s", health["err_count"], exc)
                            if stop_event.wait(period):
                                break
                            continue
                        if stop_event.wait(period):
                            break
                    except Exception as exc:
                        outer_crash += 1
                        health["err_count"] += 1
                        health["last_err"] = "outer_crash#{}: {}".format(outer_crash, exc)
                        health["last_err_at"] = time.time()
                        if outer_crash <= 5 or outer_crash % 20 == 0:
                            logger.warning("odom_feed outer_loop crashed#%d: %s", outer_crash, exc)
                        health["last_iter_at"] = time.time()
                        stop_event.wait(min(period * 20, 1.0))
                        continue
                health["alive"] = False
                if set_state is not None:
                    try:
                        set_state(active=False, mode="idle",
                                  x=None, y=None, theta=None, distance=None)
                    except Exception:
                        pass

            self._odom_feed_thread = threading.Thread(target=_odom_feed_loop, name="odom-feed")
            self._odom_feed_thread.daemon = True
            self._odom_feed_thread.start()
            return {"started": True, "hz": hz}

    def stop_odom_feed(self, force: bool = False):
        """停止 odom_feed 守护线程,并把 odom_state 复位成 idle。

        默认 force=False → no-op；显式 force=True(运维/关闭) 才真停。
        """
        return _stop_feed(self, "odom_feed", force)

    def restart_odom_feed(self, hz: float = 50.0, force: bool = False):
        """force=True 才 stop+start；否则只是 start(幂等)。"""
        if force:
            self.stop_odom_feed(force=True)
        return self.start_odom_feed(hz=hz)

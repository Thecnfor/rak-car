#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务队列 Mixin（从 runtime_service.py 拆出）。

负责双 worker 队列（arm_queue / car_queue）、job 生命周期、action dispatch、
协作取消、等待。依赖聚合类提供 `self.job_lock`、`self.jobs`、
`self.job_stop_events`、`self.arm_queue`、`self.car_queue`、`self._ref_lock`、
`self._realtime_gate`、`self.car` 等属性，以及 `ControllerWatcherMixin` /
`LifecycleMixin` 的方法。
"""
import logging
import threading
import time
import traceback
import uuid

from runtime.core import settings
from runtime.core.actions import ARM_ACTIONS, CAR_ACTIONS

from ._common import _debug_emit, normalize_value

logger = logging.getLogger(__name__)


class JobsMixin:
    """CarRuntimeService 的任务队列行为。"""

    def _trim_jobs(self, keep=None):
        if keep is None:
            keep = settings.JOB_HISTORY_LIMIT
        if len(self.jobs) <= keep:
            return
        removable_ids = [
            job_id
            for job_id, job in self.jobs.items()
            if job["status"] in {"succeeded", "failed"}
        ]
        while len(self.jobs) > keep and removable_ids:
            rid = removable_ids.pop(0)
            self.jobs.pop(rid, None)
            # 同步清理 D.6 的 stop_event，避免 Event 对象泄漏
            self.job_stop_events.pop(rid, None)

    def _set_job(self, job_id, **updates):
        with self.job_lock:
            self.jobs[job_id].update(updates)

    def submit_job(self, target, name, args=None, kwargs=None):
        args = args or []
        kwargs = kwargs or {}
        target = str(target)
        name = str(name)
        valid_actions = self.list_actions()
        if target not in valid_actions:
            raise KeyError(f"不支持的 target: {target}")
        if name not in valid_actions[target]:
            raise KeyError(f"不支持的动作: {target}.{name}")
        job_id = uuid.uuid4().hex[:12]
        job = {
            "id": job_id,
            "target": target,
            "name": name,
            "args": normalize_value(args),
            "kwargs": normalize_value(kwargs),
            "status": "queued",
            "submitted_at": time.time(),
            "started_at": None,
            "finished_at": None,
            "result": None,
            "error": None,
            # 协作退出事件存在 self.job_stop_events[job_id]，不进 job dict（避免 JSON 序列化出错）。
        }
        with self.job_lock:
            self.jobs[job_id] = job
            self.job_stop_events[job_id] = threading.Event()
            self._trim_jobs()
        # arm → arm_queue；car / task / system → car_queue。两条队列物理隔离。
        target_queue = self.arm_queue if target == "arm" else self.car_queue
        target_queue.put(job_id)
        return job

    def wait_job(self, job_id, timeout=None, poll_interval=None):
        timeout = (
            settings.DEFAULT_JOB_WAIT_TIMEOUT if timeout is None else float(timeout)
        )
        poll_interval = (
            settings.DEFAULT_POLL_INTERVAL
            if poll_interval is None
            else float(poll_interval)
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            job = self.get_job(job_id)
            if job is None:
                raise KeyError(f"任务不存在: {job_id}")
            if job["status"] in {"succeeded", "failed"}:
                return job
            time.sleep(poll_interval)
        raise TimeoutError(f"等待任务超时: {job_id}")

    def submit_job_and_wait(self, target, name, args=None, kwargs=None, timeout=None):
        """D 改造：保留旧 API 给同步调用方（main.arm 等）。

        D 路径默认异步后，调用方要等结果用 sync=True 走这个方法（见
        `main/api_client.execute(sync=True)`）；新代码请直接用 `submit_job` +
        轮询 `get_job`。
        """
        job = self.submit_job(target=target, name=name, args=args, kwargs=kwargs)
        return self.wait_job(job["id"], timeout=timeout)

    def cancel_job(self, job_id):
        """D.6 协作退出：set job 的 stop_event，并触发 SDK _hardware_stop + _stop_flag。

        立即返回 True/False，不阻塞。worker 的 SDK 循环会在下个 y/x_stop_check
        检测到 _hardware_stop / _stop_flag → 协作退出。Feed 守护线程看到
        _stop_flag 也会退出（取消正在跑的视觉/推理轮询）。

        与 /v1/control/emergency-stop 的区别：
          - cancel_job    → 同时置 _hardware_stop + _stop_flag (硬件 + 上位机都停)
          - emergency_stop → 仅置 _hardware_stop (只停硬件，上位机仍对外提供数据)
        """
        with self.job_lock:
            stop_event = self.job_stop_events.get(job_id)
            job_exists = job_id in self.jobs
        if not job_exists:
            return False
        if stop_event is not None:
            stop_event.set()
        # 同时触发车端 _hardware_stop + _stop_flag，让硬件协作退出、feed 守护线程退出
        try:
            with self._realtime_gate:
                car = self.car
            if car is not None:
                setattr(car, "_hardware_stop", True)
                setattr(car, "_stop_flag", True)
        except Exception:
            pass
        return True

    def list_jobs(self):
        with self.job_lock:
            return list(self.jobs.values())

    def get_job(self, job_id):
        with self.job_lock:
            return self.jobs.get(job_id)

    def _dispatch_car(self, car, name, args, kwargs):
        return CAR_ACTIONS[name](car, *args, **kwargs)

    def _dispatch_arm(self, car, name, args, kwargs):
        return ARM_ACTIONS[name](car.arm, *args, **kwargs)

    def _dispatch_system(self, name, _args, kwargs):
        if name == "init":
            self.ensure_initialized(
                reset_arm=kwargs.get("reset_arm", False),
                force=kwargs.get("force", False),
                reset_position=kwargs.get(
                    "reset_position",
                    settings.get_reset_position_on_init(),
                ),
            )
            return self.get_state()
        if name == "close":
            self.close()
            return {"closed": True}
        if name == "set_stop_mode":
            return {
                "stop_after_action": self.set_stop_mode(
                    kwargs.get("enabled", False)
                )
            }
        if name == "reset_stop_flag":
            return {"cleared": self.reset_stop_flag()}
        if name == "emergency_stop":
            return {"stopped": self.emergency_stop()}
        raise KeyError(f"不支持的系统动作: {name}")

    def _dispatch_target_locked(self, car, target, name, args, kwargs):
        """方法名沿用旧 `_locked` 后缀只是历史命名，**实际不持任何锁**。

        持锁只在 `_dispatch` 入口处瞬时取 `car` 引用（A.2 改造），动作执行期间
        完全不持 runtime 锁。硬件层字节串行靠 SDK 的 `serial_mc602.lock`。
        """
        if car is not None:
            car.STOP_PARAM = self.stop_after_action
        if target == "car":
            return self._dispatch_car(car, name, args, kwargs)
        if target == "arm":
            return self._dispatch_arm(car, name, args, kwargs)
        raise KeyError(f"不支持的 target: {target}")

    def _dispatch(self, target, name, args, kwargs):
        if target == "system":
            return self._dispatch_system(name, args, kwargs)
        self._wait_until_ready(reset_position=False)
        # 入口处瞬时拿 car 引用（_ref_lock），之后整个动作期间不持任何 runtime 锁。
        # 目的：让 lane 外环的 set_wheel_speeds 50Hz 调用不再被 arm 长动作（1-3s PID 闭环）挡住。
        with self._ref_lock:
            car = self.car
        if car is None:
            raise RuntimeError("car 未初始化")
        try:
            return self._dispatch_target_locked(car, target, name, args, kwargs)
        except Exception as exc:
            if not self._should_probe_controller(exc):
                raise
            snapshot = self._recover_controller_runtime(exc)
            raise RuntimeError(
                "下位机掉线，已转入后台自愈: {}".format(
                    (snapshot.get("last_probe") or {}).get("detail")
                    or snapshot.get("detail")
                )
            ) from exc

    def _worker_loop(self, target_filter):
        """target_filter: "arm" 或 "car"（system 走 car worker）。

        入口处瞬时取 self.car 引用（_ref_lock），之后整个动作期间不持任何
        runtime 锁（_dispatch_target_locked 已 A.2 重构）。
        """
        target_queue = self.arm_queue if target_filter == "arm" else self.car_queue
        while True:
            job_id = target_queue.get()
            job = self.get_job(job_id)
            if job is None:
                target_queue.task_done()
                continue
            self.current_job_id = job_id
            self._set_job(
                job_id,
                status="running",
                started_at=time.time(),
                error=None,
            )
            #region debug-point runtime-init-queue-worker
            _debug_emit(
                "H2",
                "runtime_service._worker_loop",
                "worker 开始执行任务",
                {
                    "job_id": job_id,
                    "target": job["target"],
                    "name": job["name"],
                    "queued_size": target_queue.qsize(),
                },
            )
            #endregion debug-point runtime-init-queue-worker
            try:
                result = self._dispatch(
                    job["target"],
                    job["name"],
                    job["args"],
                    job["kwargs"],
                )
                self._set_job(
                    job_id,
                    status="succeeded",
                    result=normalize_value(result),
                    finished_at=time.time(),
                )
                #region debug-point runtime-init-queue-worker
                _debug_emit(
                    "H2",
                    "runtime_service._worker_loop",
                    "worker 任务成功",
                    {
                        "job_id": job_id,
                        "target": job["target"],
                        "name": job["name"],
                    },
                )
                #endregion debug-point runtime-init-queue-worker
            except Exception as exc:
                self._handle_dispatch_failure(job["target"], exc)
                self._set_job(
                    job_id,
                    status="failed",
                    error=traceback.format_exc(),
                    finished_at=time.time(),
                )
                #region debug-point runtime-init-queue-worker
                _debug_emit(
                    "H2",
                    "runtime_service._worker_loop",
                    "worker 任务失败",
                    {
                        "job_id": job_id,
                        "target": job["target"],
                        "name": job["name"],
                        "exc_type": type(exc).__name__,
                        "exc_repr": repr(exc),
                    },
                )
                #endregion debug-point runtime-init-queue-worker
            finally:
                self.current_job_id = None
                target_queue.task_done()

"""无硬件 fake runtime：记录动作包并提供最小可预测状态模型。"""
from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass(frozen=True)
class ActionEvent:
    sequence: int
    timestamp: float
    target: str
    action: str
    args: tuple = ()
    kwargs: dict = field(default_factory=dict)
    queue: Optional[str] = None
    job_id: Optional[str] = None
    phase: str = "called"
    state: dict = field(default_factory=dict)


class ActionRecorder:
    def __init__(self):
        self._lock = threading.Lock()
        self.events: List[ActionEvent] = []
        self._sequence = 0

    def record(self, target, action, *args, queue=None, job_id=None,
               phase="called", state=None, **kwargs):
        with self._lock:
            self._sequence += 1
            event = ActionEvent(
                self._sequence, time.monotonic(), target, action,
                tuple(args), dict(kwargs), queue, job_id, phase,
                dict(state or {}),
            )
            self.events.append(event)
            return event

    def clear(self):
        with self._lock:
            self.events.clear()
            self._sequence = 0

    def matching(self, *, target=None, action=None, phase=None):
        return [event for event in self.events
                if (target is None or event.target == target)
                and (action is None or event.action == action)
                and (phase is None or event.phase == phase)]

    def names(self):
        return [(event.target, event.action) for event in self.events]

    def to_dicts(self):
        return [event.__dict__.copy() for event in self.events]


@dataclass
class _Job:
    id: str
    target: str
    name: str
    args: list
    kwargs: dict
    status: str = "queued"
    result: Any = None
    error: Optional[str] = None
    cancel: threading.Event = field(default_factory=threading.Event)
    done: threading.Event = field(default_factory=threading.Event)

    def payload(self):
        return {"job_id": self.id, "status": self.status,
                "result": self.result, "error": self.error}


class FakeCarRuntimeService:
    """CarRuntimeService 的离线替身；只模拟 client 可观察的语义。"""

    def __init__(self, *, recorder=None, action_delay=0.0, initial_state=None):
        self.recorder = recorder or ActionRecorder()
        self.is_fake = True
        self.action_delay = float(action_delay)
        self._jobs: Dict[str, _Job] = {}
        self._lock = threading.RLock()
        self._stop_flag = False
        self._initialized = True
        self.state = {
            "odom": {"x": 0.0, "y": 0.0, "theta": 0.0, "distance": 0.0},
            "arm": {"x_mm": 0.0, "y_mm": 0.0, "arm_angle": 0.0,
                    "hand_angle": 0.0, "grasped": False},
            "wheels": [0.0, 0.0, 0.0, 0.0],
            "feeds": {"lane": False, "arm": False},
        }
        if initial_state:
            self._merge(self.state, initial_state)

    @staticmethod
    def _merge(dst, src):
        for key, value in src.items():
            if isinstance(value, dict) and isinstance(dst.get(key), dict):
                FakeCarRuntimeService._merge(dst[key], value)
            else:
                dst[key] = value

    def _snapshot(self):
        with self._lock:
            return {key: (dict(value) if isinstance(value, dict) else list(value)
                          if isinstance(value, list) else value)
                    for key, value in self.state.items()}

    def close(self):
        self._initialized = False

    def get_state(self):
        return {"initialized": self._initialized, "stop_flag": self._stop_flag}

    def list_actions(self):
        return {"car": ["move_for", "set_chassis_velocity", "set_wheel_speeds",
                         "get_odometry", "start_lane_feed", "stop_lane_feed",
                         "start_arm_feed", "stop_arm_feed"],
                "arm": ["composite_run", "move_x_position", "move_y_position",
                        "grasp", "set_arm_pose"]}

    def get_runtime_snapshot(self):
        return {"fake": True, "state": self._snapshot(),
                "jobs": [job.payload() for job in self._jobs.values()]}

    def submit_job(self, target, name, args=None, kwargs=None):
        job = _Job(uuid.uuid4().hex, target, name, list(args or []), dict(kwargs or {}))
        with self._lock:
            self._jobs[job.id] = job
        self.recorder.record(target, name, *job.args, queue=target, job_id=job.id,
                             phase="queued", **job.kwargs)
        threading.Thread(target=self._run_job, args=(job,), daemon=True).start()
        return job.payload()

    def submit_job_and_wait(self, target, name, args=None, kwargs=None, timeout=None):
        payload = self.submit_job(target, name, args, kwargs)
        return self.wait_job(payload["job_id"], timeout=timeout)

    def _run_job(self, job):
        with self._lock:
            if job.cancel.is_set():
                job.status = "cancelled"
                job.done.set()
                return
            job.status = "running"
        self.recorder.record(job.target, job.name, *job.args, queue=job.target,
                             job_id=job.id, phase="started", **job.kwargs)
        try:
            if self.action_delay:
                if job.cancel.wait(self.action_delay):
                    raise RuntimeError("cancelled")
            job.result = self._invoke(job.target, job.name, job.args, job.kwargs,
                                      job.id, job.cancel)
            job.status = "succeeded"
            phase = "completed"
        except RuntimeError as exc:
            job.status = "cancelled" if job.cancel.is_set() else "failed"
            job.error = str(exc)
            phase = job.status
        except Exception as exc:
            job.status = "failed"
            job.error = str(exc)
            phase = "failed"
        self.recorder.record(job.target, job.name, *job.args, queue=job.target,
                             job_id=job.id, phase=phase, **job.kwargs)
        job.done.set()

    def wait_job(self, job_id, timeout=None, poll_interval=None):
        job = self._jobs[job_id]
        if not job.done.wait(timeout):
            raise TimeoutError("fake job timeout: {}".format(job_id))
        return job.payload()

    def get_job(self, job_id):
        job = self._jobs.get(job_id)
        return job.payload() if job else None

    def list_jobs(self):
        return [job.payload() for job in self._jobs.values()]

    def cancel_job(self, job_id):
        job = self._jobs[job_id]
        job.cancel.set()
        return True

    def _invoke(self, target, name, args, kwargs, job_id, cancel):
        if self._stop_flag and name not in {"reset_stop_flag"}:
            raise RuntimeError("emergency stop active")
        if target == "car":
            return self._car_action(name, args, kwargs, job_id, cancel)
        if target == "arm":
            return self._arm_action(name, args, kwargs, job_id, cancel)
        return {"ok": True}

    def _car_action(self, name, args, kw, job_id, cancel):
        self.recorder.record("car", name, *args, queue="car", job_id=job_id,
                             phase="physical", **kw)
        if name == "move_for":
            offset = args[0] if args else kw.get("offset", [0, 0, 0])
            self.state["odom"]["x"] += float(offset[0])
            self.state["odom"]["y"] += float(offset[1])
            self.state["odom"]["theta"] += float(offset[2])
            self.state["odom"]["distance"] += abs(float(offset[0])) + abs(float(offset[1]))
        elif name == "set_wheel_speeds":
            self.state["wheels"] = list(args[0] if args else kw.get("speeds", []))
        elif name == "set_chassis_velocity":
            self.state["wheels"] = [float(kw.get("vx", args[0] if args else 0.0))] * 4
        elif name == "start_lane_feed":
            self.state["feeds"]["lane"] = True
        elif name == "stop_lane_feed":
            self.state["feeds"]["lane"] = False
        elif name == "start_arm_feed":
            self.state["feeds"]["arm"] = True
        elif name == "stop_arm_feed":
            self.state["feeds"]["arm"] = False
        return {"ok": True, "state": self._snapshot()}

    def _arm_action(self, name, args, kw, job_id, cancel):
        self.recorder.record("arm", name, *args, queue="arm", job_id=job_id,
                             phase="physical", **kw)
        arm = self.state["arm"]
        if name == "composite_run":
            for key, state_key in (("x_mm", "x_mm"), ("y_mm", "y_mm"),
                                   ("arm", "arm_angle"), ("hand", "hand_angle")):
                if key in kw and kw[key] is not None:
                    arm[state_key] = float(kw[key])
        elif name == "move_x_position":
            arm["x_mm"] += float(args[0] if args else kw.get("distance", 0.0))
        elif name == "move_y_position":
            arm["y_mm"] += float(args[0] if args else kw.get("distance", 0.0))
        elif name == "grasp":
            arm["grasped"] = bool(args[0] if args else kw.get("enable", False))
        return {"ok": True, "state": self._snapshot()}

    def set_wheel_speeds(self, speeds):
        return self._car_action("set_wheel_speeds", [list(speeds)], {}, None, threading.Event())

    def set_chassis_velocity(self, vx, vy, wz, duration=None):
        return self._car_action("set_chassis_velocity", [], {"vx": vx, "vy": vy, "wz": wz}, None, threading.Event())

    def set_arm_velocity(self, x_vel=None, y_vel=None, arm_angle=None, hand_angle=None):
        self.recorder.record("realtime", "set_arm_velocity", queue=None,
                             phase="physical", x_vel=x_vel, y_vel=y_vel,
                             arm_angle=arm_angle, hand_angle=hand_angle)
        return {"ok": True}

    def emergency_stop(self):
        self._stop_flag = True
        self.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
        self.set_arm_velocity(0.0, 0.0)
        return True

    def reset_stop_flag(self):
        self._stop_flag = False
        return True

    def set_stop_mode(self, enabled):
        return bool(enabled)

    def get_wheel_encoders(self): return [0.0] * 4
    def get_lane_state(self): return {"active": False, "detections": []}
    def get_arm_state(self): return dict(self.state["arm"])
    def get_task_state(self): return {"active": False, "detections": []}
    def get_ir_state(self): return {"active": False, "left": None, "right": None}
    def get_odom_state(self): return {"active": True, **self.state["odom"]}
    def get_odometry_sync(self, show_info=False): return [self.state["odom"][k] for k in ("x", "y", "theta")]
    def get_distance_sync(self): return self.state["odom"]["distance"]
    def get_ir_distance_sync(self, side="left"): return None
    def get_all_ir_distance_sync(self): return {"left": None, "right": None}

    def start_lane_feed(self, hz=50.0): self.state["feeds"]["lane"] = True; return {"started": True}
    def stop_lane_feed(self, force=False): self.state["feeds"]["lane"] = False; return {"stopped": True}
    def start_arm_feed(self, hz=20.0): self.state["feeds"]["arm"] = True; return {"started": True}
    def stop_arm_feed(self, force=False): self.state["feeds"]["arm"] = False; return {"stopped": True}

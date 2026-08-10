"""无硬件 fake runtime：记录动作包并提供关节级运动学仿真。

与 `runtime/services/fake_robot.py` 的关系
------------------------------------------
本模块的 `FakeCarRuntimeService` 是 `LocalRuntimeClient(transport="fake")`
的进程内替身。所有“物理动作”（CAR_ACTIONS / ARM_ACTIONS）不再直接改 state
字典，而是路由到 `FakeRobotSim`：

- 机械臂动作被解释成带速度轮廓的关节轨迹（5 段梯形，移植自
  `main/arm/trajectory.py`），每个动作的物理事件里附带 `motion` 元信息，
  并按 `phase="physical_sample"` 逐帧记录 (t, 关节值, 末端 FK)——这就是
  “关节如何从 A 平滑走到 B”的动作展示；
- 底盘 `move_for` 推进 odom；对齐动作通过 `create_aligner` 输出可观察的
  `set_chassis_velocity` 命令包。

纯标准库约束
------------
本模块绝不 import `runtime/core/actions.py`（它会经 `task1_runner` 拖入
`smartcar.whalesbot.tools.get_yaml` 等硬件/配置依赖）。动作清单以字面量维护在
`FAKE_CAR_ACTIONS` / `FAKE_ARM_ACTIONS`，与 `runtime/core/actions.py` 的
注册表保持一致。
"""
from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .fake_robot import FakeRobotSim, forward_kinematics, create_aligner, POST_RESET_Y_MM

# ---------------------------------------------------------------------------
# 动作清单（字面量镜像 runtime/core/actions.py 的注册表，禁止 import 其模块）
# ---------------------------------------------------------------------------

FAKE_CAR_ACTIONS = [
    "beep", "stop", "reset_position", "set_storage", "set_storage_angle",
    "shooting", "set_shoot_state", "move_for", "move_time", "move_distance",
    "move_to_position", "set_chassis_velocity", "lane_time", "lane_dis",
    "lane_dis_offset", "start_lane_feed", "stop_lane_feed", "restart_lane_feed",
    "start_arm_feed", "stop_arm_feed", "restart_arm_feed",
    "start_ir_feed", "stop_ir_feed", "restart_ir_feed",
    "start_odom_feed", "stop_odom_feed", "restart_odom_feed",
    "move_to_detection_target", "adjust_arm_position",
    "get_detection_results", "get_lane_results", "get_odometry", "get_distance",
    "get_ocr", "get_det_ocr", "get_bluetooth_pad", "get_battery_voltage",
    "get_ir_distance", "get_all_ir_distance", "set_light_color", "show_text",
    "set_pwm_servo_angle", "set_digital_output", "get_arm_state",
    "run_arm_servo", "read_key", "run_task1", "run_task2", "run_task4",
]

FAKE_ARM_ACTIONS = [
    "reset_position", "reset_y", "reset_x", "reset_all", "composite_pick",
    "composite_release", "composite_go_home", "composite_run",
    "composite_run_reset", "set_arm_pose", "set_hand_angle", "set_arm_angle",
    "move_x_position", "move_y_position", "goto_position", "go_for",
    "x_speed", "y_speed", "grasp", "x_get_position", "y_get_position",
]

# arm_angle / hand_angle 的字符串方向常量（mirror arm_base 的 pose 语义）
_ARM_DIR = {"LEFT": -90.0, "MID": 0.0, "RIGHT": 90.0}
_HAND_DIR = {"UP": -90.0, "MID": 0.0, "DOWN": 10.0}


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

    def record(self, target_, action, *args, queue=None, job_id=None,
               phase="called", state=None, **kwargs):
        """记录一条动作事件。

        首个形参命名 `target_` 而非 `target` —— 某些物理动作（如 arm
        `move_y`）的实参 kwargs 里就有一个叫 `target` 的字段（y 位置目标，
        米）。若 record 形参也叫 `target`，`record(..., **kwargs)` 会撞名
        （got multiple values for argument 'target'），动作包里就丢了那个值。
        改名为 target_ 后 `target=0.1` 会干净地落入 kwargs 保留进事件。
        """
        with self._lock:
            self._sequence += 1
            event = ActionEvent(
                self._sequence, time.monotonic(), target_, action,
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
    """CarRuntimeService 的离线替身；所有物理动作路由到 FakeRobotSim。

    保留 client 可观察语义：
    - queued -> running -> succeeded/failed/cancelled 生命周期；
    - `emergency_stop` 置 stop_flag、四轮清零并记录零速命令包；
    - arm/car 动作都进 recorder，机械臂动作额外记录 trajectory 采样。
    """

    def __init__(self, *, recorder=None, action_delay=0.0, initial_state=None):
        self.recorder = recorder or ActionRecorder()
        self.is_fake = True
        self.action_delay = float(action_delay)
        self._jobs: Dict[str, _Job] = {}
        self._lock = threading.RLock()
        self._ref_lock = threading.RLock()   # LocalRuntimeClient._current_car() 依赖
        self.car = None
        self._stop_flag = False
        self._initialized = True
        self._last_chassis_cmd_t = None  # 底盘速度命令墙钟积分基准
        self._last_arm_vel_t = None      # 机械臂滑台速度命令墙钟积分基准
        self.sim = FakeRobotSim()
        self.state = {
            "odom": dict(self.sim.odom),
            "arm": self.sim.arm_state_mm(),
            "wheels": list(self.sim.wheels),
            "feeds": {"lane": True, "arm": True, "ir": False, "odom": True},
        }
        self.state["arm"]["grasped"] = self.sim.grasped
        self.state["arm"]["side"] = self.sim.side
        if initial_state:
            self._merge(self.state, initial_state)
            self._apply_state_to_sim()
        self._sync_state()

    # ---------------- 内部：state <-> sim 同步 ----------------

    @staticmethod
    def _merge(dst, src):
        for key, value in src.items():
            if isinstance(value, dict) and isinstance(dst.get(key), dict):
                FakeCarRuntimeService._merge(dst[key], value)
            else:
                dst[key] = value

    def _apply_state_to_sim(self):
        arm = self.state.get("arm") or {}
        if isinstance(arm, dict):
            for key in ("x_mm", "y_mm", "arm_angle", "hand_angle"):
                if key in arm and arm[key] is not None:
                    self.sim.joints[key].set(float(arm[key]))
            if "grasped" in arm:
                self.sim.grasped = bool(arm["grasped"])
            if "side" in arm:
                self.sim.side = str(arm["side"])
        odom = self.state.get("odom")
        if isinstance(odom, dict):
            for key in ("x", "y", "theta", "distance"):
                if key in odom:
                    self.sim.odom[key] = float(odom[key])
        feeds = self.state.get("feeds")
        if isinstance(feeds, dict):
            self.sim.feeds.update(feeds)

    def _sync_state(self):
        arm = self.sim.arm_state_mm()
        arm["grasped"] = self.sim.grasped
        arm["side"] = self.sim.side
        self.state["arm"] = arm
        self.state["odom"] = dict(self.sim.odom)
        self.state["wheels"] = list(self.sim.wheels)
        self.sim.feeds.update(self.state["feeds"])

    def _snapshot(self):
        with self._lock:
            return {key: (dict(value) if isinstance(value, dict) else list(value)
                          if isinstance(value, list) else value)
                    for key, value in self.state.items()}

    # ---------------- 采样记录（动作展示核心） ----------------

    def _emit_motion(self, action: str, job_id, motion, n: int = 48):
        """把一个 MotionResult 按最多 n 个采样点逐帧记录为 physical_sample 事件。

        `motion.samples` 是 MotionResult 的字段（已由 plan.samples() 预采样，
        默认 60 点）；这里按 n 降采样并逐帧补 FK，得到“关节如何平滑走到目标”
        的可视化数据。每个事件携带 (t, joints, ee)。
        """
        samples = motion.samples
        if n is not None and len(samples) > n:
            step = max(1, len(samples) // n)
            samples = samples[::step]
        for t, joints in samples:
            ee = forward_kinematics(
                joints.get("x_mm", self.sim.joints["x_mm"].value),
                joints.get("y_mm", self.sim.joints["y_mm"].value),
                joints.get("arm_angle", self.sim.joints["arm_angle"].value),
                joints.get("hand_angle", self.sim.joints["hand_angle"].value),
                l1=self.sim.l1, l2=self.sim.l2,
            )
            self.recorder.record("arm", action, queue="arm", job_id=job_id,
                                 phase="physical_sample",
                                 state={"t": t, "joints": dict(joints), "ee": ee})

    @staticmethod
    def _resolve_angle(value, kind: str):
        if value is None:
            return None
        if isinstance(value, str):
            table = _ARM_DIR if kind == "arm" else _HAND_DIR
            return table.get(value, 0.0)
        return float(value)

    # ---------------- 生命周期 / 任务 ----------------

    def close(self):
        self._initialized = False

    def get_state(self):
        return {"initialized": self._initialized, "stop_flag": self._stop_flag}

    def list_actions(self):
        return {"car": list(FAKE_CAR_ACTIONS), "arm": list(FAKE_ARM_ACTIONS)}

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
        if self._stop_flag and name not in {"reset_stop_flag", "stop"}:
            raise RuntimeError("emergency stop active")
        if target == "car":
            return self._car_action(name, args, kwargs, job_id, cancel)
        if target == "arm":
            return self._arm_action(name, args, kwargs, job_id, cancel)
        return {"ok": True}

    # ---------------- 底盘 / car 动作 ----------------

    def _car_action(self, name, args, kw, job_id, cancel):
        self.recorder.record("car", name, *args, queue="car", job_id=job_id,
                             phase="physical", **kw)
        sim = self.sim
        if name == "move_for":
            offset = args[0] if args else kw.get("offset", [0, 0, 0])
            sim.move_for(float(offset[0]), float(offset[1]), float(offset[2]))
        elif name == "set_wheel_speeds":
            sim.set_wheels(args[0] if args else kw.get("speeds", []))
        elif name == "set_chassis_velocity":
            sim.set_chassis_velocity(
                float(kw.get("vx", args[0] if args else 0.0)),
                float(kw.get("vy", 0.0)), float(kw.get("wz", 0.0)))
        elif name == "move_distance":
            distance = float(args[0] if args else kw.get("distance", 0.0))
            sim.move_for(distance, 0.0, 0.0)
        elif name in ("start_lane_feed", "restart_lane_feed"):
            self.state["feeds"]["lane"] = True
        elif name == "stop_lane_feed":
            self.state["feeds"]["lane"] = False
        elif name in ("start_arm_feed", "restart_arm_feed"):
            self.state["feeds"]["arm"] = True
        elif name == "stop_arm_feed":
            self.state["feeds"]["arm"] = False
        elif name in ("start_ir_feed", "restart_ir_feed"):
            self.state["feeds"]["ir"] = True
        elif name == "stop_ir_feed":
            self.state["feeds"]["ir"] = False
        elif name in ("start_odom_feed", "restart_odom_feed"):
            self.state["feeds"]["odom"] = True
        elif name == "stop_odom_feed":
            self.state["feeds"]["odom"] = False
        elif name == "reset_position":
            sim.reset_position()
        elif name == "stop":
            self._stop_flag = True
            sim.emergency_stop()
        # ---- 只读 / 信息类，直接返回结果（不落入通用 ok 字典）----
        elif name == "get_odometry":
            self._sync_state()
            return [sim.odom["x"], sim.odom["y"], sim.odom["theta"]]
        elif name == "get_distance":
            return sim.odom["distance"]
        elif name == "get_arm_state":
            return self.get_arm_state()
        elif name == "get_detection_results":
            return self._filter_detections(args, kw)
        elif name == "get_lane_results":
            return {"ok": True, "detections": list(sim.lane_detections)}
        elif name in ("get_ocr", "get_det_ocr"):
            return dict(sim.ocr_result)
        elif name == "get_ir_distance":
            side = args[0] if args else kw.get("side", "left")
            return sim.ir_distances.get(side)
        elif name == "get_all_ir_distance":
            return {"left": sim.ir_distances["left"], "right": sim.ir_distances["right"]}
        elif name == "get_battery_voltage":
            return 24.0
        elif name == "get_bluetooth_pad":
            return {}
        elif name == "read_key":
            return {"key": None}
        elif name == "run_arm_servo":
            return self._run_arm_servo(args, kw)
        elif name == "replay_arm_trajectory":
            return self._replay_arm_trajectory(args, kw)
        self._sync_state()
        return {"ok": True, "state": self._snapshot()}

    def _filter_detections(self, args, kw):
        detections = [dict(d) for d in self.sim.task_detections]
        sort_pos = kw.get("sort_pos")
        if sort_pos:
            sx, sy = float(sort_pos[0]), float(sort_pos[1])
            detections.sort(
                key=lambda d: ((float(d.get("cx", d.get("x", 0.0))) - sx) ** 2
                               + (float(d.get("cy", d.get("y", 0.0))) - sy) ** 2))
        limit_x = kw.get("limit_x")
        if limit_x is not None:
            detections = [d for d in detections
                          if abs(float(d.get("cx", d.get("x", 0.0)))) <= float(limit_x)]
        limit_y = kw.get("limit_y")
        if limit_y is not None:
            detections = [d for d in detections
                          if abs(float(d.get("cy", d.get("y", 0.0)))) <= float(limit_y)]
        return detections

    def _run_arm_servo(self, args, kw):
        """机械臂视觉伺服（确定性版本）：读 task_detections，比例对齐几帧。

        无目标 → {"ok": False, "reason": "no-target"}；有目标 → 输出可观察的
        set_chassis_velocity 对齐命令包并返回 settled。
        """
        label = kw.get("label")
        detections = self.sim.task_detections
        if label is not None:
            detections = [d for d in detections if d.get("label") == label]
        if not detections:
            return {"ok": False, "reason": "no-target"}
        aligner = create_aligner("proportional", axis="x")
        packets = []
        err = float(detections[0].get("cx", 0.0))
        for _ in range(3):
            packets.append(aligner.step(err))
        self.recorder.record("car", "set_chassis_velocity", queue="car",
                             phase="physical_sample", state={"packets": packets})
        return {"ok": True, "reason": "settled", "trace_hits": len(detections),
                "settled": True, "end_arm": self.sim.arm_state_mm()}

    def _replay_arm_trajectory(self, args, kw):
        """进程内连续轨迹回放（fake 版）：一次收 route，本地规划 + 逐帧喂。"""
        import time

        from main.arm.planning.joint_trajectory import (
            JointPose, plan_joint_trajectory,
        )

        route = kw.get("route") or (args[0] if args else None)
        if not route:
            raise ValueError("replay_arm_trajectory 需要 route (姿态列表)")
        poses = [JointPose.from_mapping(p) for p in route]
        if len(poses) < 2:
            raise ValueError("route 至少需要 2 个姿态（首尾 goal）")
        traj = plan_joint_trajectory(
            poses, max_speed_scale=float(kw.get("max_speed_scale", 1.0)))
        hz = max(float(kw.get("hz", 30.0)), 1.0)
        tick = 1.0 / hz
        T = traj.total_time

        # 1. 绝对定位到轨迹起点
        s = poses[0]
        self.sim.composite_move({"x_mm": s.x_mm, "y_mm": s.y_mm,
                                 "arm_angle": s.arm_deg,
                                 "hand_angle": s.hand_deg})
        # 2. 连续回放（按节拍喂速度，set_arm_velocity 按墙钟积分）
        prev = traj.sample(0.0)
        n = max(1, int(math.ceil(T / tick)))
        for i in range(1, n + 1):
            t = min(T, i * tick)
            pose = traj.sample(t)
            vx = (pose.x_mm - prev.x_mm) / tick
            vy = (pose.y_mm - prev.y_mm) / tick
            prev = pose
            self.set_arm_velocity(x_vel=vx / 1000.0, y_vel=vy / 1000.0,
                                  arm_angle=pose.arm_deg,
                                  hand_angle=pose.hand_deg)
            time.sleep(tick)
        # 3. 结束停速 + 末姿态
        end = traj.sample(T)
        self.set_arm_velocity(x_vel=0.0, y_vel=0.0,
                              arm_angle=end.arm_deg, hand_angle=end.hand_deg)
        self._sync_state()
        return {"ok": True, "T": T, "end_pose": end.to_dict(),
                "end_arm_state": self.sim.arm_state_mm()}

    # ---------------- 机械臂动作（全部路由到 FakeRobotSim） ----------------

    def _arm_action(self, name, args, kw, job_id, cancel):
        self.recorder.record("arm", name, *args, queue="arm", job_id=job_id,
                             phase="physical", **kw)
        handler = getattr(self, "_arm_" + name, None)
        if handler is None:
            raise KeyError("fake runtime 不支持的 arm 动作: {}".format(name))
        result = handler(args, kw, job_id)
        self._sync_state()
        return result

    def _arm_reset_position(self, args, kw, job_id):
        m1 = self.sim.composite_move({"arm_angle": 90.0, "hand_angle": -90.0})
        self._emit_motion("reset_position", job_id, m1)
        m2 = self.sim.reset_y()
        self._emit_motion("reset_position", job_id, m2)
        return {"ok": True, "steps": {"arm": 90.0, "hand": -90.0,
                                      "y": POST_RESET_Y_MM / 1000.0}}

    def _arm_reset_y(self, args, kw, job_id):
        motion = self.sim.reset_y()
        self._emit_motion("reset_y", job_id, motion)
        return {"ok": True, "steps": {"y": POST_RESET_Y_MM / 1000.0}}

    def _arm_reset_x(self, args, kw, job_id):
        direction = args[0] if args else kw.get("direction", "right")
        motion = self.sim.reset_x(direction)
        self._emit_motion("reset_x", job_id, motion)
        return {"ok": True, "steps": {"x": 0.0}}

    def _arm_reset_all(self, args, kw, job_id):
        arm = self._resolve_angle(kw.get("arm_angle", 90.0), "arm")
        hand = self._resolve_angle(kw.get("hand_angle", -90.0), "hand")
        m1 = self.sim.reset_x(kw.get("x_direction", "right"))
        self._emit_motion("reset_all", job_id, m1)
        m2 = self.sim.composite_move({"arm_angle": arm, "hand_angle": hand})
        self._emit_motion("reset_all", job_id, m2)
        m3 = self.sim.reset_y()
        self._emit_motion("reset_all", job_id, m3)
        return {"ok": True, "steps": {"x": 0.0, "arm": arm, "hand": hand,
                                      "y": POST_RESET_Y_MM / 1000.0}}

    def _arm_composite_pick(self, args, kw, job_id):
        arm = self._resolve_angle(kw.get("arm_angle", 90.0), "arm")
        x = float(kw.get("x", 0.0))
        y = float(kw.get("y", 0.0))
        hand = self._resolve_angle(kw.get("hand", 0.0), "hand")
        m1 = self.sim.composite_move(
            {"arm_angle": arm, "x_mm": x * 1000.0, "y_mm": y * 1000.0})
        self._emit_motion("composite_pick", job_id, m1)
        m2 = self.sim.move_joint("hand_angle", hand)
        self._emit_motion("composite_pick", job_id, m2)
        self.sim.grasped = True
        return {"ok": True, "steps": {"arm": arm, "position": [x, y],
                                      "hand": hand, "grasp": True}}

    def _arm_composite_release(self, args, kw, job_id):
        drop_x = float(kw.get("drop_x", 0.0))
        drop_y = float(kw.get("drop_y", 0.03))
        hand = self._resolve_angle(kw.get("hand", 0.0), "hand")
        m1 = self.sim.move_joint("hand_angle", hand)
        self._emit_motion("composite_release", job_id, m1)
        m2 = self.sim.composite_move({"x_mm": drop_x * 1000.0, "y_mm": drop_y * 1000.0})
        self._emit_motion("composite_release", job_id, m2)
        self.sim.grasped = False
        return {"ok": True, "steps": {"hand": hand, "position": [drop_x, drop_y],
                                      "grasp": False}}

    def _arm_composite_go_home(self, args, kw, job_id):
        arm = self._resolve_angle(kw.get("arm", 0.0), "arm")
        hand = self._resolve_angle(kw.get("hand", -90.0), "hand")
        m1 = self.sim.composite_move({"arm_angle": arm, "x_mm": 0.0, "y_mm": 0.0})
        self._emit_motion("composite_go_home", job_id, m1)
        m2 = self.sim.move_joint("hand_angle", hand)
        self._emit_motion("composite_go_home", job_id, m2)
        return {"ok": True, "steps": {"arm": arm, "position": [0.0, 0.0],
                                      "hand": hand}}

    def _arm_composite_run(self, args, kw, job_id):
        targets = {}
        if kw.get("arm") is not None:
            targets["arm_angle"] = self._resolve_angle(kw["arm"], "arm")
        if kw.get("x") is not None:
            targets["x_mm"] = float(kw["x"]) * 1000.0
        if kw.get("y") is not None:
            targets["y_mm"] = float(kw["y"]) * 1000.0
        if kw.get("hand") is not None:
            targets["hand_angle"] = self._resolve_angle(kw["hand"], "hand")
        if targets:
            v_lim = {}
            if kw.get("x_v_max_mms") is not None:
                v_lim["x_mm"] = float(kw["x_v_max_mms"])
            motion = self.sim.composite_move(targets, v_max=v_lim or None)
            self._emit_motion("composite_run", job_id, motion)
        return {"ok": True,
                "steps": {"arm": kw.get("arm"), "x": kw.get("x"),
                          "y": kw.get("y"), "hand": kw.get("hand")}}

    def _arm_composite_run_reset(self, args, kw, job_id):
        arm = self._resolve_angle(kw.get("arm_angle", 90.0), "arm")
        hand = self._resolve_angle(kw.get("hand_angle", -90.0), "hand")
        m1 = self.sim.composite_move({"arm_angle": arm, "hand_angle": hand})
        self._emit_motion("composite_run_reset", job_id, m1)
        if kw.get("reset_x", True):
            m2 = self.sim.reset_x(kw.get("x_direction", "right"))
            self._emit_motion("composite_run_reset", job_id, m2)
        m3 = self.sim.reset_y()
        self._emit_motion("composite_run_reset", job_id, m3)
        return {"ok": True, "steps": {"arm": arm, "hand": hand, "x": 0.0,
                                      "y": POST_RESET_Y_MM / 1000.0}}

    def _arm_set_arm_pose(self, args, kw, job_id):
        targets = {}
        if kw.get("x") is not None:
            targets["x_mm"] = float(kw["x"]) * 1000.0
        if kw.get("y") is not None:
            targets["y_mm"] = float(kw["y"]) * 1000.0
        if targets:
            motion = self.sim.composite_move(targets)
            self._emit_motion("set_arm_pose", job_id, motion)
        arm = self._resolve_angle(kw.get("arm"), "arm")
        hand = self._resolve_angle(kw.get("hand"), "hand")
        if arm is not None:
            motion = self.sim.move_joint("arm_angle", arm)
            self._emit_motion("set_arm_pose", job_id, motion)
        elif hand is not None:
            motion = self.sim.move_joint("hand_angle", hand)
            self._emit_motion("set_arm_pose", job_id, motion)
        return {"ok": True, "steps": {"x": kw.get("x"), "y": kw.get("y"),
                                      "arm": arm, "hand": hand}}

    def _arm_set_hand_angle(self, args, kw, job_id):
        hand = self._resolve_angle(args[0] if args else kw.get("hand"), "hand")
        motion = self.sim.move_joint("hand_angle", hand)
        self._emit_motion("set_hand_angle", job_id, motion)
        return {"ok": True, "angle": hand}

    def _arm_set_arm_angle(self, args, kw, job_id):
        arm = self._resolve_angle(args[0] if args else kw.get("arm"), "arm")
        motion = self.sim.move_joint("arm_angle", arm)
        self._emit_motion("set_arm_angle", job_id, motion)
        return {"ok": True, "angle": arm}

    def _arm_move_x_position(self, args, kw, job_id):
        target_m = float(args[0] if args else kw.get("target", 0.0))
        v_max_mms = kw.get("v_max_mms")
        motion = self.sim.move_joint("x_mm", target_m * 1000.0,
                                     v_max=float(v_max_mms) if v_max_mms is not None else None)
        self._emit_motion("move_x_position", job_id, motion)
        return {"ok": True, "target": target_m}

    def _arm_move_y_position(self, args, kw, job_id):
        target_m = float(args[0] if args else kw.get("target", 0.0))
        motion = self.sim.move_joint("y_mm", target_m * 1000.0)
        self._emit_motion("move_y_position", job_id, motion)
        return {"ok": True, "target": target_m}

    def _arm_goto_position(self, args, kw, job_id):
        x = kw.get("x")
        y = kw.get("y")
        targets = {}
        if x is not None:
            targets["x_mm"] = float(x) * 1000.0
        if y is not None:
            targets["y_mm"] = float(y) * 1000.0
        if targets:
            motion = self.sim.composite_move(targets)
            self._emit_motion("goto_position", job_id, motion)
        return {"ok": True, "x": x, "y": y}

    def _arm_go_for(self, args, kw, job_id):
        x_off = float(args[0] if args else kw.get("x_offset", 0.0))
        y_off = float(args[1] if len(args) > 1 else kw.get("y_offset", 0.0))
        targets = {}
        if x_off:
            targets["x_mm"] = self.sim.joints["x_mm"].value + x_off * 1000.0
        if y_off:
            targets["y_mm"] = self.sim.joints["y_mm"].value + y_off * 1000.0
        if targets:
            motion = self.sim.composite_move(targets)
            self._emit_motion("go_for", job_id, motion)
        return {"ok": True, "x_offset": x_off, "y_offset": y_off}

    def _arm_x_speed(self, args, kw, job_id):
        velocity = float(args[0] if args else kw.get("velocity", 0.0))
        self.sim.velocity_x(velocity)
        return {"ok": True, "velocity": velocity}

    def _arm_y_speed(self, args, kw, job_id):
        velocity = float(args[0] if args else kw.get("velocity", 0.0))
        self.sim.velocity_y(velocity)
        return {"ok": True, "velocity": velocity}

    def _arm_grasp(self, args, kw, job_id):
        value = bool(args[0] if args else kw.get("enable", kw.get("value", False)))
        self.sim.grasped = value
        return {"ok": True, "grasped": value}

    def _arm_x_get_position(self, args, kw, job_id):
        return self.sim.joints["x_mm"].value / 1000.0

    def _arm_y_get_position(self, args, kw, job_id):
        return self.sim.joints["y_mm"].value / 1000.0

    # ---------------- realtime（不创建 job） ----------------

    def chassis_align(self, **kw):
        """确定性底盘视觉对齐（`track_chassis` 的 runtime 端）。

        对应生产 `POST /v1/realtime/chassis-align`（ChassisAlignController）。
        读 task_detections：目标可见 → 比例对齐到 setpoint 并返回 arrived；
        无目标 → `{"arrived": False, "reason": "no_target"}`。不创建 job，
        对齐命令包以 physical_sample 记录到 recorder。
        """
        target = kw.get("target")
        if isinstance(target, (list, tuple, set)):
            targets = set(target)
        elif isinstance(target, str):
            targets = {target}
        else:
            targets = set()
        detections = [d for d in self.sim.task_detections
                      if not targets or d.get("label") in targets]
        if not detections:
            return {"ok": True, "result": {
                "arrived": False, "reason": "no_target", "final_frame": None,
                "frames": 0, "elapsed_s": 0.0,
                "stop_ok": True, "motion_ok": True, "enc_delta": None}}
        setpoint = kw.get("setpoint_cxcy") or [0.0, 0.0]
        sx, sy = float(setpoint[0]), float(setpoint[1])
        detections.sort(key=lambda d: ((float(d.get("cx", 0.0)) - sx) ** 2
                                       + (float(d.get("cy", 0.0)) - sy) ** 2))
        best = detections[0]
        cx, cy = float(best.get("cx", 0.0)), float(best.get("cy", 0.0))
        deadband = float(kw.get("deadband", 0.05))
        aligner = create_aligner("proportional", axis="x")
        packets = []
        for _ in range(3):
            packets.append(aligner.step(cx - sx))
        self.recorder.record("car", "chassis_align", queue=None,
                             phase="physical_sample",
                             state={"packets": packets, "final_frame": {
                                 "target_found": True, "label": best.get("label"),
                                 "cx": cx, "cy": cy,
                                 "score": best.get("score", 0.0),
                                 "cx_err": cx - sx, "cy_err": cy - sy}})
        return {"ok": True, "result": {
            "arrived": abs(cx - sx) <= deadband,
            "reason": "settled",
            "final_frame": {"target_found": True, "label": best.get("label"),
                            "cx": cx, "cy": cy,
                            "score": best.get("score", 0.0),
                            "cx_err": cx - sx, "cy_err": cy - sy},
            "frames": 3, "elapsed_s": 0.15,
            "stop_ok": True, "motion_ok": True, "enc_delta": None}}

    def set_wheel_speeds(self, speeds):
        speeds = list(speeds)
        vx, vy, wz = self._wheels_to_odom_vel(speeds)
        self._integrate_odom(vx, vy, wz)
        return self._car_action("set_wheel_speeds", [speeds], {},
                                None, threading.Event())

    def set_chassis_velocity(self, vx, vy, wz, duration=None):
        self._integrate_odom(vx, vy, wz)
        return self._car_action("set_chassis_velocity", [],
                                {"vx": vx, "vy": vy, "wz": wz}, None,
                                threading.Event())

    @staticmethod
    def _wheels_to_odom_vel(speeds):
        """麦克纳姆轮速 → 底盘速度（SDK forward_kinematics 的标量版，免 numpy）。

        轮序 [FL, FR, RL, RR]，SDK 正解矩阵（mecanum.py 225-247）:
          vx = (w0 - w1 - w2 + w3) / 4
          vy = (w0 + w1 - w2 - w3) / (4·tan(roller))
          wz = (w0 + w1 + w2 + w3) / (4·wheel_constant)
        直线前进时四轮 [v, -v, -v, v] → vx=v, vy=0。roller≈45° 时 tan≈1、
        wheel_constant 只是 wz 的比例，fake 只关心方向与相对量级，取近似即可。
        """
        w0, w1, w2, w3 = (float(s) for s in (list(speeds) + [0.0, 0.0, 0.0, 0.0])[:4])
        return (w0 - w1 - w2 + w3) / 4.0, (w0 + w1 - w2 - w3) / 4.0, \
               (w0 + w1 + w2 + w3) / 4.0

    def _integrate_odom(self, vx, vy, wz):
        """按真实墙钟积分底盘 odom（速度命令模式）。

        真车上 20/50Hz 的 lane-follow / creep 每帧发一次轮速，轮子转 → 编码器
        odom 累加。fake 若不积分，`move_along_lane(distance_m)` 的 distance_stop
        读 lane_state.distance 永远到不了目标 → 每次跑满 max_seconds 兜底
        （task6 的 approach 并发就因此 TimeoutError）。这里按两次命令间真实
        间隔 dt（上限 1.0s 防长动作后跳变）积分，使距离模式正确收敛且速度
        可测；`move_for` 仍直接加位移，语义不变。
        """
        now = time.monotonic()
        dt = (now - self._last_chassis_cmd_t) if self._last_chassis_cmd_t else 0.0
        self._last_chassis_cmd_t = now
        dt = min(max(dt, 0.0), 1.0)
        if dt <= 0.0:
            return
        vx = float(vx or 0.0)
        vy = float(vy or 0.0)
        wz = float(wz or 0.0)
        self.sim.odom["x"] += vx * dt
        self.sim.odom["y"] += vy * dt
        self.sim.odom["theta"] += wz * dt
        self.sim.odom["distance"] += (abs(vx) + abs(vy)) * dt

    def set_arm_velocity(self, x_vel=None, y_vel=None, arm_angle=None, hand_angle=None):
        self.recorder.record("realtime", "set_arm_velocity", queue=None,
                             phase="physical", x_vel=x_vel, y_vel=y_vel,
                             arm_angle=arm_angle, hand_angle=hand_angle)
        # 滑台速度按真实墙钟积分（镜像底盘 _integrate_odom；连续轨迹回放依赖）
        now = time.monotonic()
        dt = (now - self._last_arm_vel_t) if self._last_arm_vel_t else 0.0
        self._last_arm_vel_t = now
        dt = min(max(dt, 0.0), 1.0)
        if x_vel is not None:
            self.sim.velocity_x(float(x_vel))
        if y_vel is not None:
            self.sim.velocity_y(float(y_vel))
        if arm_angle is not None:
            self.sim.joints["arm_angle"].set(float(arm_angle))
        if hand_angle is not None:
            self.sim.joints["hand_angle"].set(float(hand_angle))
        if dt > 0.0:
            self.sim.advance(dt)
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

    # ---------------- feeds 守护线程（fake 只翻标志位） ----------------

    def get_wheel_encoders(self):
        return [0.0] * 4

    def get_lane_state(self):
        return {"active": bool(self.state["feeds"].get("lane")),
                "detections": list(self.sim.lane_detections),
                "distance": self.sim.odom["distance"]}

    def get_arm_state(self):
        snap = self.sim.posture_snapshot()
        active = bool(self.state["feeds"].get("arm"))
        snap["active"] = active
        snap["mode"] = "arm_feed" if active else "idle"
        return snap

    def get_task_state(self):
        return {"active": True, "detections": list(self.sim.task_detections)}

    def get_ir_state(self):
        return {"active": bool(self.state["feeds"].get("ir")),
                "left": self.sim.ir_distances["left"],
                "right": self.sim.ir_distances["right"]}

    def get_odom_state(self):
        return {"active": bool(self.state["feeds"].get("odom")),
                **dict(self.sim.odom)}

    def get_odometry_sync(self, show_info=False):
        return [self.sim.odom[k] for k in ("x", "y", "theta")]

    def get_distance_sync(self):
        return self.sim.odom["distance"]

    def get_ir_distance_sync(self, side="left"):
        return self.sim.ir_distances.get(side)

    def get_all_ir_distance_sync(self):
        return {"left": self.sim.ir_distances["left"],
                "right": self.sim.ir_distances["right"]}

    def start_lane_feed(self, hz=50.0):
        self.state["feeds"]["lane"] = True
        self._sync_state()
        return {"started": True}

    def stop_lane_feed(self, force=False):
        self.state["feeds"]["lane"] = False
        self._sync_state()
        return {"stopped": True}

    def start_arm_feed(self, hz=20.0):
        self.state["feeds"]["arm"] = True
        self._sync_state()
        return {"started": True}

    def stop_arm_feed(self, force=False):
        self.state["feeds"]["arm"] = False
        self._sync_state()
        return {"stopped": True}

    # ---------------- fixture 注入（供任务 harness / 测试使用） ----------------

    def set_task_detections(self, detections):
        """注入 task 视觉检测结果（label/cx/cy/x/y/w/h/conf 列表）。"""
        self.sim.task_detections = [dict(d) for d in detections]

    def set_lane_detections(self, detections):
        self.sim.lane_detections = [dict(d) for d in detections]

    def set_ir_distances(self, left=None, right=None):
        self.sim.ir_distances["left"] = left
        self.sim.ir_distances["right"] = right

    def set_ocr_result(self, ok=False, text="", order=None):
        self.sim.ocr_result = {"ok": bool(ok), "text": str(text),
                               "order": list(order or [])}

    def advance(self, dt):
        """推进仿真时钟（速度模式的关节积分）。"""
        self.sim.advance(dt)
        self._sync_state()


# ==================== 进程内单例 ====================

_fake_service = None
_fake_lock = threading.RLock()


def get_fake_runtime(action_delay=0.0, **kwargs) -> "FakeCarRuntimeService":
    """进程内唯一 fake runtime service（单例）。

    业务层 `create_runtime_client(transport="fake")` 与 `ArmClient.connect()`
    各自都会新造 client，但都走 `get_fake_runtime()` 拿同一个 service ——
    这样任务对任一 client 的 fixture 注入（detections / IR / ocr / odom）
    在任务内部自建的 client 上同样可见，fake 与生产 local 的共享语义一致。
    """
    global _fake_service
    with _fake_lock:
        if _fake_service is None:
            _fake_service = FakeCarRuntimeService(action_delay=action_delay, **kwargs)
        return _fake_service


def reset_fake_runtime():
    """重置单例（测试隔离）。返回被丢弃的服务，无则返回 None。"""
    global _fake_service
    with _fake_lock:
        svc = _fake_service
        _fake_service = None
    return svc

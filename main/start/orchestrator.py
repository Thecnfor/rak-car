#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/start/orchestrator.py

run.py 的实现后端（#1 重构）：
  - 后台 A：DoubleLoopRunner 50Hz 巡线外环（pause/resume 控制）
  - 后台 B：20Hz 里程计累计
  - 主线程：顺序遍历 DEFAULT_WAYPOINTS，等待「IR + 里程计」双触发 → 暂停巡线
    → 调 task.run() → 恢复巡线 → 终点处 break。

所有 main.start 之外的脚本都不应该 import 本文件 —— 只服务于 run.py。
"""
from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

# 让 main.start.orchestrator 可被仓库根目录的 run.py 直接 import
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.api_client import RuntimeApiClient
from main.chassis import LANE_FOLLOW
from main.chassis.api import ChassisClient
from main.chassis.loops.closed_loop import DoubleLoopRunner
from main.chassis.tasks.read_dis import read_dis
from main.chassis.tasks.read_ir import read_ir

logger = logging.getLogger("main.start.orchestrator")


@dataclass
class Waypoint:
    """一个任务点位.

    Attributes:
        name:           人类可读名字,会出现在日志 (位置参数,保持向后兼容).
        task_id:        任务编号 (1..7), 用于 TASK_RUNNERS[id] 查表. None 表示纯导航/finish.
        task_module:    (兼容旧字段,实际不再用 —— orchestrator 走 TASK_RUNNERS).
        ir_threshold_m: IR 接近阈值 (None 表示不参与 IR 判断).
        ir_side:        IR 哪一侧触发: "left" / "right" / "any" (任一侧) /
                            "both" (两侧都触发, 用于窄通道) (默认 "right").
        ir_threshold_left_m:  左侧 IR 单独阈值 (None 走 ir_threshold_m).
                              用于 ir_side="both" 时左右阈值不同.
        ir_threshold_right_m: 右侧 IR 单独阈值 (None 走 ir_threshold_m).
        dis_at_least_m: 累计里程计 ≥ 该值才算"到了这个点" (None 表示不参与).
        trigger_op:     "AND" (默认,严格防误触) / "OR".
        pause_before_s: 触发后、调 task 前的停顿.
        pause_after_s:  任务跑完、恢复巡线前的停顿.
        settle_before_pause_s: trigger 满足后、pause 外环前的等待 (旧版, 仅 sleep).
            这段时间 lane 外环继续跑,把车从弯道偏航 / 偏横向状态拉直.
            新代码优先用 settle_forward_s (走 move_along_lane 沿车道直行).
        settle_forward_s: trigger 满足后, 沿中心车道线直行 N 秒 (走 move_along_lane,
            vy=0 + ω 锁对齐, 不偏航). 替代 settle_before_pause_s 的功能, 但走
            chassis 控制器而不是 time.sleep 让外环自由跑.
            默认 0.0 = 不直行 (旧行为).
        settle_forward_speed_mps: 直行速度 (m/s), 正值. 默认 0.15.
        back_off_m:     settle 完成后沿车道线向后移动的距离 (m).
            >0 时, 走 move_along_lane(vx=-back_off_speed_mps, distance_m=back_off_m)
            沿车道后退 (不偏). 替代原 move_for(物理后退, 弯道会偏).
            默认 0.0 = 不后退.
        back_off_speed_mps: 后退速度 (m/s, 正值). 默认 0.10.
        back_off_delay_s: settle 跑完和 back_off 后退之间的停顿 (s).
            给底盘减速 + 车物理稳定的时间, 避免前进→后退直接切换冲击底盘.
            默认 0.0 = 不停顿.
        is_finish:      True = 这是终点 (里程计达到即整个流程结束).
    """
    name: str
    task_id: Optional[int] = None
    task_module: Optional[str] = None
    ir_threshold_m: Optional[float] = None
    ir_side: str = "right"
    ir_threshold_left_m: Optional[float] = None
    ir_threshold_right_m: Optional[float] = None
    dis_at_least_m: Optional[float] = None
    trigger_op: str = "AND"
    pause_before_s: float = 0.0
    pause_after_s: float = 0.0
    settle_before_pause_s: float = 0.0
    settle_forward_s: float = 0.0
    settle_forward_speed_mps: float = 0.15
    back_off_m: float = 0.0
    back_off_speed_mps: float = 0.10
    back_off_delay_s: float = 0.0
    is_finish: bool = False


# 默认 8 任务点位 + 1 终点. 换场地改 task_config.yml 的 waypoints 段 (业务代码不动).
# 保留 DEFAULT_WAYPOINTS 作为 fallback —— 启动时优先从 yaml 加载.
DEFAULT_WAYPOINTS: List[Waypoint] = [
    Waypoint("task1_seeding",     task_id=1,
             ir_threshold_m=0.70, ir_side="right",
             dis_at_least_m=0.90, trigger_op="AND"),
    Waypoint("task3_pest_scout", task_id=3,
             ir_threshold_m=2.00, ir_side="left",
             dis_at_least_m=2.33, trigger_op="AND"),
    Waypoint("task2_water_tower",  task_id=2,
             ir_threshold_m=0.70, ir_side="left",
             dis_at_least_m=4.5, trigger_op="OR"),
    Waypoint("task4_harvest",     task_id=4,
             ir_threshold_m=0.70, ir_side="left",
             dis_at_least_m=None, trigger_op="AND"),
    Waypoint("task5_sort",        task_id=5,
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=11.0, trigger_op="AND"),
    Waypoint("task6_get_order",   task_id=6,
             ir_threshold_m=0.50, ir_side="left",
             dis_at_least_m=13.0, trigger_op="AND"),
    Waypoint("task7_deliver",     task_id=7,
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=14.5, trigger_op="AND"),
    Waypoint("task3_shoot",       task_id=8,
             # 2026-08-06 拆分: task3 识别 + task3_shoot 射击, 中间放 task2.
             # 实际位置由用户在 task_config.yml 里调 (跟在 task2 之后).
             ir_threshold_m=None, dis_at_least_m=15.5, trigger_op="AND"),
    # 终点: 里程计达到 16.5m → 整个流程结束
    Waypoint("cruise_done",       ir_threshold_m=None,
             dis_at_least_m=16.5, is_finish=True),
]


class PressDetector:
    """按鍵邊沿偵測 + 去抖（純邏輯，無 IO，可離線單測）。

    ``feed(pressed)`` 每採樣呼叫一次；回傳 True 表示「確認了一次按下事件」。
    - 邊沿：僅「釋放→按下」後開始累計；開機時按鍵被壓住不會誤觸發（首採樣只記錄）。
    - 去抖：連續 ``confirm_samples`` 個按下樣本才判定觸發。
    - 觸發後須等釋放才重新武裝（下一次按下再觸發）。
    """

    def __init__(self, confirm_samples: int = 2) -> None:
        self.confirm = max(1, int(confirm_samples))
        self.prev: Optional[bool] = None
        self.streak = 0
        self.armed = False   # 已在一次按壓中（已觸發，或開機即按住）

    def is_armed(self) -> bool:
        return self.armed

    def feed(self, pressed: bool) -> bool:
        pressed = bool(pressed)
        if self.prev is None:                # 開機首採樣：只記錄，不觸發
            self.prev = pressed
            self.armed = pressed             # 開機即按住 → 先視為按壓中
            self.streak = 1 if pressed else 0
            return False
        rising = (not self.prev) and pressed
        self.prev = pressed
        if not pressed:                      # 釋放：歸零並重新武裝
            self.streak = 0
            self.armed = False
            return False
        if self.armed:                       # 已在按壓中（或已觸發過）→ 不再觸發
            return False
        self.streak += 1
        if self.streak >= self.confirm:
            self.armed = True
            return True
        return False


class Orchestrator:
    """巡线导航 + 任务点位调度器。"""

    def __init__(self,
                 waypoints: Optional[List[Waypoint]] = None,
                 lane_hz: float = 50.0,
                 ir_interval_s: float = 0.02,
                 config_path: Optional[str] = None):
        """config_path: 自定义 task_config.yml 路径, None 走默认 (根目录 task_config.yml)."""
        if waypoints is not None:
            self.waypoints = waypoints
        else:
            # 优先从 yaml 加载; 失败 fallback DEFAULT_WAYPOINTS
            self.waypoints = self._load_waypoints_from_yaml(config_path) or DEFAULT_WAYPOINTS
            if self.waypoints is DEFAULT_WAYPOINTS:
                logger.warning("using DEFAULT_WAYPOINTS fallback (%d waypoints)",
                               len(self.waypoints))
        self.lane_hz = lane_hz
        self.ir_interval_s = ir_interval_s
        self._ball_counts: Dict[str, int] = {}
        # task1→识别区 / task2→射击区 途中后台摆臂的完成句柄 (task3 用).
        self._pending_arm_pose: Optional[Dict[str, Any]] = None

    @staticmethod
    def _load_waypoints_from_yaml(config_path: Optional[str]) -> Optional[List[Waypoint]]:
        """从 yaml 加载 waypoints, 失败返 None."""
        try:
            from main.task._config import load_waypoints
            wp_dicts = load_waypoints()
        except (FileNotFoundError, KeyError, ValueError) as exc:
            logger.warning("yaml load_waypoints failed, fallback DEFAULT_WAYPOINTS: %s", exc)
            return None
        out = []
        for w in wp_dicts:
            out.append(Waypoint(
                name=w.get("name", ""),
                task_id=w.get("task_id"),
                task_module=w.get("task_module"),  # 保留旧字段, 不参与 _run_task
                ir_threshold_m=w.get("ir_threshold_m"),
                ir_threshold_left_m=w.get("ir_threshold_left_m"),
                ir_threshold_right_m=w.get("ir_threshold_right_m"),
                ir_side=w.get("ir_side", "right"),
                dis_at_least_m=w.get("dis_at_least_m"),
                trigger_op=w.get("trigger_op", "AND"),
                settle_before_pause_s=w.get("settle_before_pause_s", 0.0),
                settle_forward_s=w.get("settle_forward_s", 0.0),
                settle_forward_speed_mps=w.get("settle_forward_speed_mps", 0.15),
                back_off_m=w.get("back_off_m", 0.0),
                back_off_speed_mps=w.get("back_off_speed_mps", 0.10),
                back_off_delay_s=w.get("back_off_delay_s", 0.0),
                is_finish=w.get("is_finish", False),
            ))
        wp_summary = ", ".join(
            f"{w.name}(side={w.ir_side},thr={w.ir_threshold_m},dis={w.dis_at_least_m},"
            f"op={w.trigger_op})"
            for w in out
        )
        logger.info("loaded %d waypoints from task_config.yml: %s", len(out), wp_summary)
        return out

    def _init_mission(self, start_lane: bool = True) -> Dict[str, Any]:
        """建好整套任務機制並回傳 state（初始化/巡線/IR/里程計/TUI/下位機屏幕）。

        由 _run_mission()（全流程/單任務）與 wait_key_then_run()（--wait-key 一鍵啟動）共用。
        start_lane=False 時 lane runner 線程**不啟動**：等待階段車子不許動，
        由 wait_key_then_run 在按鍵按下瞬間才啟動（保證按下即開始、無預移動）。
        """
        client = RuntimeApiClient()
        if not client.wait_until_ready(timeout=10.0):
            raise RuntimeError("runtime not ready (pm2 logs rak-car-api)")

        # 任务启动前清零里程计：每次 run.py 从零点起算，避免沿用上次任务的累计距离。
        # 旧 car_start_2026.py 的 init() 会清零；orchestrator 接管后必须显式补上，
        # 否则 run.py 连跑两次，第二次的 dis 起点 = 第一次的终点，所有 dis 阈值/终点全乱。
        try:
            client.execute("car", "reset_position", sync=True, timeout=10.0)
            logger.info("odometry reset: mission starts from distance 0")
        except Exception as exc:
            logger.warning("reset_position failed, dis baseline may be stale: %s", exc)

        api = ChassisClient.connect()
        try:
            api.start_lane_feed(hz=self.lane_hz)
        except Exception as exc:
            logger.warning("start_lane_feed failed: %s", exc)

        # 确保 ir_feed 就绪（#seed-overshoot）：首次 read_ir 走 fast-path，
        # 避免 fallback 到 sync execute(timeout=2s) 导致 0.6m 盲区。
        _ir_deadline = time.time() + 5.0
        while time.time() < _ir_deadline:
            ir_state = api.get_ir_state()
            if ir_state.active:
                logger.info("ir_feed active (age_ms=%s)", ir_state.age_ms)
                break
            time.sleep(0.2)
        else:
            logger.warning("ir_feed not active after 5s, IR reads may use slow fallback")

        # 用 LANE_FOLLOW profile 装配 DoubleLoopRunner（#1）
        # —— 不再自己 new CurvatureAdaptiveOuterLoop + WheelSmoother。
        profile = LANE_FOLLOW
        # 弯道阶梯转弯常开（巡线段随时可能遇弯）：CurveDetector 识别 →
        # StaircaseTurn θ 闭环 45→90→120°，lane 回正后交还 outer。
        # crossroad_turn（task_config.yml 顶层声明）：第几个弯出口紧接着十字路口，
        # 那个弯换加固转弯（里程碑窗口出口+触发冷却），其余弯走原版逻辑。
        from main.chassis.controllers.odom_turn import CurveDetector, StaircaseTurn
        from main.task._config import (load_crossroad_turn, load_post_task1,
                                       load_post_task6)
        runner = DoubleLoopRunner(
            api=api,
            outer=profile.build_outer(),
            hz=self.lane_hz,
            watchdog_ms=profile.watchdog_ms,
            lost_line_ms=profile.lost_line_ms,
            smoother=profile.build_smoother(),
            turn=StaircaseTurn(),
            detector=CurveDetector(),
            crossroad_turn=load_crossroad_turn(),
        )
        # task1 结束后: 清零里程 → 切断视觉 → 直行 → 里程计 θ 转 → 恢复视觉.
        # None = task_config.yml 未配 / enabled=false → 保持现状 (只清零里程).
        post_task1 = load_post_task1()
        # task6 结束后同款盲转段 (读订单后掉头 120° 去 task7 投放), 见 post_task6.
        post_task6 = load_post_task6()

        # 后台 A：DoubleLoopRunner 50Hz 巡线（#1：用 runner.pause/resume 控制暂停）
        # max_seconds=inf：常驻，由 runner.stop() 终止
        runner_thread = threading.Thread(
            target=runner.run,
            kwargs={"max_seconds": math.inf},
            daemon=True, name="lane",
        )
        # start_lane=False（一鍵啟動等待階段）→ 只建不啟，按鍵按下才 .start()
        if start_lane:
            runner_thread.start()

        # 后台 B：里程计（全程累计，写共享 buffer）
        dis_buf = [0.0]
        dis_epoch = [0]   # 里程计清零信号: 递增即让 read_dis 重新起算基线
        threading.Thread(target=read_dis,
                         kwargs={"api": api, "hz": 20.0,
                                 "on_tick": lambda v: dis_buf.__setitem__(0, v),
                                 "reset_epoch": dis_epoch},
                         daemon=True, name="distance").start()

        # 后台 C：TUI 状态栏（200ms 刷新）
        tui_running = threading.Event()
        tui_running.set()
        tui_buf: List[Dict[str, Any]] = [{"wp": "", "dis": 0.0,
                                          "ir_left": None, "ir_right": None,
                                          "state": "init"}]
        threading.Thread(target=self._tui_loop,
                         args=(tui_buf, tui_running),
                         daemon=True, name="tui").start()

        # 后台 D：下位机 led_show 屏幕 UI（250ms 刷新，带帧率限制）
        display_running = threading.Event()
        display_running.set()
        from main.start.display_ui import Mc602Display
        display_ui = Mc602Display(client, layout="20x5")
        threading.Thread(target=self._display_ui_loop,
                         args=(display_ui, tui_buf, display_running),
                         daemon=True, name="display").start()

        return {
            "client": client, "api": api, "runner": runner,
            "runner_thread": runner_thread,
            "dis_buf": dis_buf, "dis_epoch": dis_epoch,
            "tui_buf": tui_buf, "tui_running": tui_running,
            "display_ui": display_ui, "display_running": display_running,
            "post_task1": post_task1, "post_task6": post_task6,
        }

    def _walk_waypoints(self, state: Dict[str, Any],
                        waypoints: List[Waypoint]) -> List[str]:
        """按 waypoints 列表顺序导航并执行任务（機制由 _init_mission 建立）。

        返回 completed 列表。结束/异常时清理 runner / 线程 / feeds / 下位机屏幕。
        """
        api = state["api"]
        runner = state["runner"]
        runner_thread = state["runner_thread"]
        dis_buf = state["dis_buf"]
        dis_epoch = state["dis_epoch"]
        tui_buf = state["tui_buf"]
        tui_running = state["tui_running"]
        display_running = state["display_running"]
        display_ui = state["display_ui"]
        post_task1 = state["post_task1"]
        post_task6 = state["post_task6"]
        client = state["client"]

        completed: List[str] = []
        task4_task5_handoff = None
        self._pending_arm_pose = None   # 每次 mission 重置, 防上次中断残留
        try:
            for waypoint_index, wp in enumerate(waypoints):
                logger.info("=== navigating to %s ===", wp.name)
                self._wait_until_triggered(wp, api, dis_buf, tui_buf,
                                           interval_s=self.ir_interval_s)
                # 触发后立即 pause 后台 lane 巡线 runner (settle_forward/back_off
                # 要新建自己的 DoubleLoopRunner, 跟后台 A 的 runner 共享 lane_feed
                # 但各自独立发轮速, 必须先 pause 否则两个 runner 互相打架 "原地抽搐".
                self._pause_lane(runner, api)
                # settle_forward_s: 沿中心车道线直行 N 秒 (走 move_along_lane,
                # vy=0 + ω 锁对齐, 不偏航). 用户 2026-08-06 规定: task2 出弯后
                # 沿车道直行 1.5s 拉直车身/车头, 不再让外环自由跑 (会偏).
                # 默认 0 = 不直行.
                if wp.settle_forward_s > 0:
                    logger.info("[settle_forward] %s 沿车道直行 %.2fs @ %.2fm/s",
                                wp.name, wp.settle_forward_s,
                                wp.settle_forward_speed_mps)
                    try:
                        from main.chassis import move_along_lane
                        move_along_lane(
                            vx=float(wp.settle_forward_speed_mps),
                            max_seconds=float(wp.settle_forward_s),
                        )
                    except Exception as exc:
                        logger.warning("[settle_forward] %s 失败 (%s), 继续",
                                       wp.name, exc)
                elif wp.settle_before_pause_s > 0:
                    # 旧版: time.sleep 让外环继续跑 (可能偏航, 向后兼容).
                    logger.info("[settle] %s 外环多跑 %.2fs 拉直车身/车头",
                                wp.name, wp.settle_before_pause_s)
                    time.sleep(wp.settle_before_pause_s)
                # back_off_m: settle 完成后底盘后退 (m).
                # 用户 2026-08-06 实测决定: 跟 detect_retry_step_m 一样走 move_for
                # (SDK 字节流 4 轮等速倒), 不是 move_along_lane. 因为后退距离短 (0.2m),
                # 出弯后位置已知, 不需要走视觉对齐的 move_along_lane.
                if wp.back_off_m > 0:
                    # back_off_delay_s: settle 跑完和后退之间的停顿.
                    if wp.back_off_delay_s > 0:
                        logger.info("[back_off] %s settle→后退停顿 %.2fs",
                                    wp.name, wp.back_off_delay_s)
                        time.sleep(wp.back_off_delay_s)
                    logger.info("[back_off] %s 底盘后退 %.2f m", wp.name, wp.back_off_m)
                    try:
                        # 仿 detect_retry 走 move_for (跟 _chassis_move_for 一致):
                        # sync=False + wait_job, 走 car_queue, SDK SerialEngine 调度.
                        job = client.execute_car_action(
                            "move_for", [-wp.back_off_m, 0.0, 0.0],
                            timeout=10.0, sync=False,
                        )
                        jid = job.get("id") if isinstance(job, dict) else None
                        if jid:
                            client.wait_job(jid, timeout=10.0)
                    except Exception as exc:
                        logger.warning("[back_off] %s 失败 (%s), 继续",
                                       wp.name, exc)
                if wp.is_finish:
                    logger.info("finish waypoint reached (dis=%.2fm), mission done",
                                dis_buf[0])
                    tui_buf[0] = {"wp": wp.name, "dis": dis_buf[0],
                                  "ir_left": None, "ir_right": None,
                                  "state": "done"}
                    completed.append(wp.name)
                    break
                # 注意: _pause_lane 已经在 trigger 满足后立即调过 (settle_forward/
                # back_off 之前), 这里不再重复 pause (幂等但冗余).
                time.sleep(wp.pause_before_s)
                # task3_pest_scout 触发时清零里程计 distance, 识别段从 0 起算,
                # 跑满 odom_stop_m (0.66m) 即任务完成. read_dis 有单调保护,
                # dis_buf 保持上次读数, 不污染后续 waypoint 的 dis 阈值.
                if wp.task_id == 3:
                    try:
                        client.execute("car", "reset_position", sync=True, timeout=10.0)
                        logger.info("odometry reset at task3 trigger: distance from 0")
                    except Exception as exc:
                        logger.warning("odom reset at task3 trigger failed: %s", exc)
                defer_task5_handoff = False
                if wp.task_module:
                    tui_buf[0] = {"wp": wp.name, "dis": dis_buf[0],
                                  "ir_left": None, "ir_right": None,
                                  "state": "task"}
                    extra_kwargs: Dict[str, Any] = {}
                    defer_task5_handoff = self._should_defer_task4_handoff(
                        waypoints, waypoint_index,
                    )
                    if defer_task5_handoff:
                        # 仅当当前选中的 waypoint 后面紧跟 task5 时，
                        # 才把 task4 收尾交给后台 handoff；单任务 --task 4
                        # 必须走 task4 自己的 finally，避免引入 task5 线程池。
                        extra_kwargs["defer_task5_handoff"] = True
                    if wp.task_id == 5 and task4_task5_handoff is not None:
                        handoff_ok = self._wait_task4_task5_handoff(task4_task5_handoff)
                        if not handoff_ok:
                            logger.warning("[task4→task5] handoff failed; task5 will retry Phase 1 pose")
                        else:
                            extra_kwargs["phase1_pose_ready"] = True
                        task4_task5_handoff = None
                    if wp.task_id == 5:
                        extra_kwargs["prev_ball_counts"] = dict(self._ball_counts)
                    # 2026-08-09 用户: task3 识别/射击区, 车已停在任务点,
                    # 先确认途中摆臂完成 (来不及就停下调整).
                    self._wait_pending_arm_pose(wp.task_id)
                    task_result = self._run_task(client, wp, **extra_kwargs)
                    if not task_result.get("ok"):
                        logger.warning("task %s did not succeed, continuing to next waypoint", wp.name)
                    # task4 结束后提取球色统计, 供 task5 使用
                    if wp.task_id == 4 and task_result.get("ok"):
                        detail = task_result.get("detail", {})
                        history = []
                        if isinstance(detail, dict):
                            history = detail.get("history", []) or []
                        counts = {"blue": 0, "yellow": 0}
                        for entry in history:
                            if (isinstance(entry, dict)
                                    and entry.get("action") == "picked"
                                    and entry.get("color") in counts):
                                counts[entry["color"]] += 1
                        self._ball_counts = counts
                        logger.info("[task4→task5] 采集统计: blue=%d yellow=%d",
                                    counts["blue"], counts["yellow"])
                        detail_reason = detail.get("reason") if isinstance(detail, dict) else None
                        if (detail_reason == "ir_odom_exit"
                                and defer_task5_handoff):
                            task4_task5_handoff = self._start_task4_task5_handoff(client)
                            logger.info("[task4→task5] handoff 已启动，先恢复巡航")
                time.sleep(wp.pause_after_s)
                if wp.task_id == 1:
                    if post_task1 is not None:
                        self._post_task1_maneuver(api, post_task1)
                # task2 结束后: 清零里程计 distance + 重挂 dis_buf 基线.
                # 后续 waypoint (task3_shoot dis_at_least_m=2.70) 从新基线起算.
                # dis_epoch 递增让 read_dis 重新起算 —— 否则单调保护把清零挡在 dis_buf 外,
                # task3_shoot 会用清零前的旧 dis 立即误触发.
                if wp.task_id == 2:
                    try:
                        client.execute("car", "reset_position", sync=True, timeout=10.0)
                        dis_epoch[0] += 1
                        dis_buf[0] = 0.0
                        logger.info("odometry reset after task2: distance from 0")
                    except Exception as exc:
                        logger.warning("odom reset after task2 failed: %s", exc)
                # task6 读完订单后: 清零里程 → 切断视觉 → 直行 → θ 顺时针转 120° → 恢复视觉.
                # 复用 _post_task1_maneuver (同款盲转段), 参数走 task_cfg.post_task6.
                if wp.task_id == 6:
                    try:
                        client.execute("car", "reset_position", sync=True, timeout=10.0)
                        logger.info("odometry reset after task6: next segment from distance 0")
                    except Exception as exc:
                        logger.warning("odom reset after task6 failed: %s", exc)
                    if post_task6 is not None:
                        self._post_task1_maneuver(api, post_task6)
                # task4→task5 handoff 已在后台处理关仓 + Phase 1 姿态，
                # 不能再启动通用 home reset 抢占同一条臂串口。
                if task4_task5_handoff is None:
                    if wp.task_id == 1:
                        # 2026-08-09 用户: task1 结束后, 前往 task3 识别区的
                        # 巡线途中后台摆好识别姿态 (来不及就等任务点停下调整).
                        from main.task.task3.arm_poses import RECOGNITION_ARM
                        self._schedule_arm_pose(RECOGNITION_ARM, "recognition",
                                                target_task_id=3)
                    elif wp.task_id == 2:
                        # 2026-08-09 用户: task2 结束后, 前往 task3 射击区的
                        # 巡线途中后台摆好射击姿态.
                        from main.task.task3.arm_poses import SHOOTING_ARM
                        self._schedule_arm_pose(SHOOTING_ARM, "shooting",
                                                target_task_id=8)
                    else:
                        self._schedule_arm_home_reset()
                self._resume_lane(runner)
                completed.append(wp.name)
        except KeyboardInterrupt:
            logger.info("interrupted by user")
        finally:
            # task4→task5 handoff 若仍在跑（中断/未到 task5），必须在
            # 清场前收尾，避免关仓/摆臂在 mission 结束后继续下发。
            if task4_task5_handoff is not None and task4_task5_handoff.is_alive():
                logger.info("[task4→task5] 收尾时 handoff 仍在跑, join 等它结束")
                task4_task5_handoff.join()
            # 终止 runner（#1）：stop() 唤醒 _pause.wait() 让 run() 看到 _stop，
            # join 等其 finally 块跑完（smoother 归零 + api.close()）。
            # 同时关掉 TUI + 下位机屏幕后台线程。
            tui_running.clear()
            display_running.clear()
            try:
                api.stop_wheel_speeds()
            except Exception:
                pass
            runner.stop()
            runner_thread.join(timeout=2.0)
            try:
                api.stop_lane_feed()
            except Exception:
                pass
            # 下位机屏幕清屏（best-effort）
            try:
                display_ui.clear()
                display_ui.render(throttle_s=0.0)
            except Exception:
                pass
            # 注意：不要再调 api.close() —— runner 的 finally 已经调过了
            logger.info("mission completed: %s", completed)
        return completed

    def _run_mission(self, waypoints: List[Waypoint]) -> List[str]:
        """核心逻辑：初始化底盘/巡线/IR/里程计/TUI，按 waypoints 列表顺序导航并执行任务。

        由 run()（全流程）和 run_single_task()（单任务测试）共用。
        返回 completed 列表。
        """
        state = self._init_mission(start_lane=True)
        return self._walk_waypoints(state, waypoints)

    # ── 一鍵啟動：MC602 板上鍵（--wait-key）─────────────────────────

    @staticmethod
    def _read_key_pressed(client) -> Optional[bool]:
        """讀一次下位機按鍵（走 realtime 快路徑 GET，不進 job_queue）。

        慢的 execute/sync 路徑在 feed 並發下每 call ~0.6s，20Hz 輪詢會全超時；
        GET /v1/realtime/key/state 單發 ~10ms。回傳 True/False；控制器掉線回傳 None。
        """
        try:
            resp = client.get(f"{client.api_prefix}/realtime/key/state", timeout=0.2)
        except Exception:
            return None
        if not isinstance(resp, dict) or not resp.get("ok"):
            return None
        return bool(resp.get("pressed"))

    def _wait_board_key(self, client, tui_buf: List[Dict[str, Any]]) -> None:
        """等待 MC602 板上鍵按下（邊沿 + 40ms 時間窗口去抖）。等待期間屏幕顯示 READY。"""
        det = PressDetector(confirm_samples=1)
        press_start = None
        confirm_duration_s = 0.04
        error_streak = 0
        tui_buf[0] = {"wp": "PRESS BOARD KEY", "dis": 0.0,
                      "ir_left": None, "ir_right": None, "state": "READY"}
        while True:
            pressed = self._read_key_pressed(client)
            if pressed is None:
                error_streak += 1
                tui_buf[0] = {"wp": "CTRL ERR", "dis": 0.0,
                              "ir_left": None, "ir_right": None, "state": "ERR"}
                time.sleep(min(0.5, 0.05 * error_streak))   # 退避重試
                continue
            error_streak = 0
            tui_buf[0] = {"wp": "PRESS BOARD KEY", "dis": 0.0,
                          "ir_left": None, "ir_right": None, "state": "READY"}
            if det.feed(pressed):
                press_start = time.time()
            if pressed:
                if press_start is None and not det.is_armed():
                    press_start = time.time()
                if press_start is not None and time.time() - press_start >= confirm_duration_s:
                    logger.info("board key pressed → mission start")
                    return
            else:
                press_start = None
            time.sleep(0.01)

    def wait_key_then_run(self) -> None:
        """--wait-key 模式：預先初始化（不挪車）→ 等 MC602 板上鍵 → 立即開跑完整任務。

        比賽計時從按下開始：等待階段完成全套初始化（含 5s IR feed 等待、lane 模型
        常駐熱載），按下瞬間只做「啟動 lane runner 線程 + 進 waypoint 迴圈」，
        beep 非阻塞不擋第一步挪車。任務完成後回 READY 可再按重跑。
        """
        state = self._init_mission(start_lane=False)
        while True:
            self._wait_board_key(state["client"], state["tui_buf"])
            # 按下 → 立刻開始：啟動 runner 線程（首幀輪速 ~20-40ms 內下發）
            state["runner_thread"].start()
            threading.Thread(target=self._beep_async,
                             args=(state["client"],), daemon=True).start()
            try:
                self._walk_waypoints(state, self.waypoints)
                threading.Thread(target=self._beep_async,
                                 args=(state["client"], 3), daemon=True).start()
            except KeyboardInterrupt:
                logger.info("interrupted by user, back to READY")
            except Exception as exc:
                logger.exception("mission failed: %s", exc)
            # 完成/失敗 → 重建機制（_walk_waypoints 已清理 runner/線程），回 READY
            state = self._init_mission(start_lane=False)

    @staticmethod
    def _beep_async(client, times: int = 1) -> None:
        """非阻塞蜂鳴：按下確認 beep×1 / 完成 beep×3，跑在背景線程，不擋主流程。"""
        try:
            for _ in range(times):
                client.execute("car", "beep", sync=True, timeout=2.0)
                time.sleep(0.3)
        except Exception:
            pass

    def run(self) -> None:
        """全流程 8 任务（巡线 + IR/里程计触发 + 顺序执行）。"""
        self._run_mission(self.waypoints)

    def run_tasks(self, task_ids: List[int]) -> None:
        """多任务模式：按给定顺序巡线到每个任务点位并执行。

        Args:
            task_ids: 任务编号列表，如 [1, 3, 5]。
        """
        # 收集所有可用 task_id
        available = sorted(
            {w.task_id for w in self.waypoints if w.task_id is not None}
        )
        # 筛选 + 按原始顺序保持
        selected: List[Waypoint] = []
        missing: List[int] = []
        for tid in task_ids:
            wp = next((w for w in self.waypoints if w.task_id == tid), None)
            if wp is None:
                missing.append(tid)
            else:
                selected.append(wp)
        if missing:
            raise ValueError(
                f"waypoints 中没有 task_id={missing}，可用: {available}"
            )
        logger.info("multi-task mode: task_ids=%s → %d waypoints",
                    task_ids, len(selected))
        self._run_mission(selected)

    def run_single_task(self, task_id: int) -> None:
        """单任务独立测试模式：只巡线到一个任务点位，触发后执行该任务，然后停止。

        Args:
            task_id: 任务编号 1-8 (8=task3_shoot, 2026-08-06 拆分).
        """
        wp = next((w for w in self.waypoints if w.task_id == task_id), None)
        if wp is None:
            available = sorted(
                {w.task_id for w in self.waypoints if w.task_id is not None}
            )
            raise ValueError(
                f"waypoints 中没有 task_id={task_id}，可用: {available}"
            )
        logger.info("single-task mode: task_id=%d → waypoint %s", task_id, wp.name)
        self._run_mission([wp])

# ── 后台线程 ────────────────────────────────────────────

    @staticmethod
    def _tui_loop(tui_buf: List[Dict[str, Any]],
                  tui_running: threading.Event) -> None:
        """每 200ms 刷一行 TUI 状态栏到终端。"""
        while tui_running.wait():
            info = tui_buf[0]
            state = info.get("state", "?")
            wp = info.get("wp", "")
            dis = info.get("dis", 0.0)
            il = info.get("ir_left")
            ir = info.get("ir_right")
            il_s = f"{il:.2f}" if il is not None else "---"
            ir_s = f"{ir:.2f}" if ir is not None else "---"
            line = f"\r[{state}] wp={wp:<14s} | dis={dis:6.2f}m | IR L:{il_s} R:{ir_s}   "
            sys.stdout.write(line)
            sys.stdout.flush()
            time.sleep(0.2)
        # 退出时换行
        sys.stdout.write("\n")
        sys.stdout.flush()

    @staticmethod
    def _display_ui_loop(display_ui, tui_buf: List[Dict[str, Any]],
                         display_running: threading.Event) -> None:
        """每 250ms 更新下位机 led_show 屏幕（带节流）。"""
        while display_running.wait():
            # 2026-08-08 修复: 未初始化 / 队列积压时跳过本轮刷屏。
            # 显示刷新 4Hz 比 show_text 串口处理快 → 队列无界增长 → current_job_id
            # 常驻 → runtime _auto_init_loop 被 gate 住 → 车永远初始化不了
            # (实测 3012+ show_text 排队, initialized=False 卡死)。
            # 排队 >40 或未初始化就跳过, 让 init 有机会完成后再恢复刷屏。
            try:
                hstate = (display_ui.api.get_health() or {}).get("state") or {}
                if (not hstate.get("initialized")) or (hstate.get("queued_jobs") or 0) > 40:
                    time.sleep(0.25)
                    continue
            except Exception:
                time.sleep(0.25)
                continue
            info = tui_buf[0]
            state = info.get("state", "?")
            wp = info.get("wp", "")
            dis = info.get("dis", 0.0)
            il = info.get("ir_left")
            ir = info.get("ir_right")
            try:
                display_ui.skin_dashboard(
                    state=state,
                    wp=wp,
                    dis=dis,
                    ir_left=il,
                    ir_right=ir,
                    battery=0.85,  # 可通过 runtime API 读取真实电量
                )
                display_ui.render(throttle_s=0.25)
            except Exception:
                pass
            time.sleep(0.25)
        # 退出时清屏
        try:
            display_ui.clear()
            display_ui.render(throttle_s=0.0)
        except Exception:
            pass

    # ── 主线程辅助 ──────────────────────────────────────────

    @staticmethod
    def _pause_lane(runner: DoubleLoopRunner, api: ChassisClient) -> None:
        """暂停外环（#1）：runner.pause() 同步等到外环确认停住（已补发零速），
        再主动发零速兜底（双保险，防止在途非零帧残留）。

        旧实现 pause() 是异步的：外环当前帧可能在 stop_wheel_speeds() 之后
        又下发非零，且随后阻塞不再补零 → 车停不下来。现在 pause() 同步后才返回。
        """
        paused = runner.pause()
        if not paused:
            logger.warning("runner.pause() 超时未确认，外环线程可能已退出，仍补发零速")
        try:
            api.stop_wheel_speeds()
        except Exception:
            pass

    @staticmethod
    def _resume_lane(runner: DoubleLoopRunner) -> None:
        """恢复外环（#1）：resume() 唤醒 + 清 smoother 记忆，从静止起步。"""
        runner.resume()

    @staticmethod
    def _wait_until_triggered(wp: Waypoint, api: ChassisClient,
                              dis_buf: list, tui_buf: List[Dict[str, Any]],
                              interval_s: float = 0.02) -> None:
        """轮询 IR + 里程计，直到 wp 的触发条件满足（默认 AND）。

        任一条件字段为 None 时视为「已满足」，避免任务永不触发。
        IR 分左右：wp.ir_side 取 left / right，"any" 表示两侧任一触发即可，
        "both" 表示两侧都要触发 (窄通道用).
        每轮更新 tui_buf 供 TUI 线程读取。
        """
        # 诊断(#task2-not-stop): 目标 IR 侧每次从"高于阈值"→"低于阈值"打一条日志,
        # 记录触发瞬间的 dis —— 用于判断是否 AND 窗口错位 (IR 触发时 dis 门限还没到).
        fired_before = False
        last_diag_log = 0.0
        while True:
            ir: dict = {}
            try:
                ir = read_ir(api, timeout=0.5)  # fast-path 不受影响；fallback 盲区从 0.6m→0.15m
            except Exception:
                pass
            right = ir.get("right") if isinstance(ir, dict) else None
            left = ir.get("left") if isinstance(ir, dict) else None
            dis = dis_buf[0]

            # 更新 TUI
            tui_buf[0] = {"wp": wp.name, "dis": dis,
                          "ir_left": left, "ir_right": right,
                          "state": "nav"}

            if (wp.ir_threshold_m is None
                    and wp.ir_threshold_left_m is None
                    and wp.ir_threshold_right_m is None):
                # 2026-08-06: 三种阈值都没设才跳过 IR 检查.
                # 旧逻辑只看 ir_threshold_m, 设了 per-side 也被短路成 True.
                ir_ok = True
            elif wp.ir_side == "left":
                ir_ok = left is not None and left < wp.ir_threshold_m
            elif wp.ir_side == "any":
                ir_ok = ((left is not None and left < wp.ir_threshold_m) or
                         (right is not None and right < wp.ir_threshold_m))
            elif wp.ir_side == "both":
                # 2026-08-06: 两侧 IR 都 < 各自阈值才触发.
                # 阈值优先用 per-side (ir_threshold_left_m / _right_m),
                # None 时 fallback 到 ir_threshold_m.
                left_th = (wp.ir_threshold_left_m
                           if wp.ir_threshold_left_m is not None
                           else wp.ir_threshold_m)
                right_th = (wp.ir_threshold_right_m
                            if wp.ir_threshold_right_m is not None
                            else wp.ir_threshold_m)
                ir_ok = (
                    (left is not None and left_th is not None
                     and left < left_th)
                    and
                    (right is not None and right_th is not None
                     and right < right_th)
                )
            else:  # "right"
                ir_ok = right is not None and right < wp.ir_threshold_m

            dis_ok = (wp.dis_at_least_m is None or dis >= wp.dis_at_least_m)

            # 诊断日志: 首次触发 + 持续触发期间每 ~1s 提醒 (看"等 dis"阶段), IR 回升则复位.
            if wp.ir_threshold_m is not None and ir_ok:
                if not fired_before:
                    fired_before = True
                    last_diag_log = time.time()
                    logger.info("[trigger] %s IR %s fired at dis=%.2f (gate=%s, dis_ok=%s)",
                                wp.name, wp.ir_side, dis, wp.dis_at_least_m, dis_ok)
                elif time.time() - last_diag_log >= 1.0:
                    last_diag_log = time.time()
                    logger.info("[trigger] %s IR %s still fired at dis=%.2f (gate=%s, dis_ok=%s)",
                                wp.name, wp.ir_side, dis, wp.dis_at_least_m, dis_ok)
            elif wp.ir_threshold_m is not None:
                fired_before = False  # IR 回升到阈值以上 → 复位, 下次再触发会重新打日志

            hit = (ir_ok and dis_ok) if wp.trigger_op == "AND" else (ir_ok or dis_ok)
            if hit:
                logger.info("triggered: %s (ir_left=%s ir_right=%s dis=%.2f)",
                            wp.name, left, right, dis)
                return
            time.sleep(interval_s)

    # ── task1 结束后: 盲转段位移 (2026-08-06) ─────────────────────

    def _post_task1_maneuver(self, api: ChassisClient,
                             seg: Dict[str, Any]) -> None:
        """task1 结束后: 切断视觉 → 里程计直行 → θ 转 turn_deg → 恢复视觉.

        巡线外环此时已暂停, 车停着. 盲转段不依赖 lane 帧, 先 stop_lane_feed
        让 lane_state 失效; 转完 start_lane_feed 并**等 lane 新鲜**再交还外环
        —— 否则 resume 首帧 age_ms>500ms 触发 watchdog 急停 (closed_loop.py).
        参数走 task_config.yml task_cfg.post_task1 (straight_m / turn_deg).
        """
        straight_m = float(seg.get("straight_m", 0.0))
        turn_deg = float(seg.get("turn_deg", 0.0))
        try:
            api.stop_lane_feed()
            logger.info("[post-task1] 切断视觉 (stop_lane_feed)")
        except Exception as exc:
            logger.warning("[post-task1] stop_lane_feed 失败: %s", exc)
        if straight_m:
            try:
                api.move_for(dx_m=straight_m)
                logger.info("[post-task1] 直行 %.2f m (里程计闭环 move_for)", straight_m)
            except Exception as exc:
                logger.warning("[post-task1] move_for %.2f 失败: %s", straight_m, exc)
        if turn_deg:
            self._turn_theta_deg(api, turn_deg)
        try:
            api.start_lane_feed(hz=self.lane_hz)
            logger.info("[post-task1] 恢复视觉 (start_lane_feed)")
        except Exception as exc:
            logger.warning("[post-task1] start_lane_feed 失败: %s", exc)

        # 2026-08-08: _wait_lane_fresh 可能因 inference 延迟 / 车头偏離车道线而超时，
        # 旧代码单次失败直接 resume → watchdog 急停 → 外环退出 → 卡死。
        # 改为重试机制：最多 3 次，每次重试前重启 lane_feed。
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            if self._wait_lane_fresh(api):
                logger.info("[post-task1] lane fresh 成功 (attempt %d/%d)",
                            attempt, max_retries)
                break
            if attempt < max_retries:
                logger.warning("[post-task1] lane fresh attempt %d/%d 失败，重启 feed 重试...",
                               attempt, max_retries)
                try:
                    api.stop_lane_feed()
                    time.sleep(0.5)
                    api.start_lane_feed(hz=self.lane_hz)
                except Exception as exc:
                    logger.warning("[post-task1] 重启 lane_feed 失败: %s", exc)
        else:
            logger.error("[post-task1] lane fresh %d 次均失败，将强制 resume（ risking watchdog）",
                         max_retries)

    @staticmethod
    def _turn_theta_deg(api: ChassisClient, turn_deg: float,
                        *, hz: float = 50.0, timeout_s: float = 15.0) -> None:
        """里程计 θ 闭环原地转弯 turn_deg (度), 转完自动零速.

        复用 odom_turn.OdomTurnPID (纯控制器无 IO) + realtime/chassis-velocity
        下发 (走 _realtime_gate, 不占 job_queue). turn_deg>0 = theta 增大
        (odom 逆时针); 实车方向反了在 task_config.yml 里取负.
        """
        from main.chassis.controllers.odom_turn import OdomTurnPID
        try:
            _, _, theta0 = api.get_odometry()
        except Exception:
            theta0 = 0.0
        turn = OdomTurnPID(turn_deg=turn_deg)
        turn.start(theta0)
        dt = 1.0 / max(hz, 1.0)
        deadline = time.monotonic() + max(timeout_s, 1.0)
        done = False
        while time.monotonic() < deadline:
            try:
                _, _, theta = api.get_odometry()
            except Exception:
                theta = theta0
            omega, done = turn.step(theta, dt)
            try:
                api.set_chassis_velocity(0.0, 0.0, omega)
            except Exception:
                pass
            if done:
                break
            time.sleep(dt)
        try:
            api.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass
        theta_end = 0.0
        try:
            _, _, theta_end = api.get_odometry()
        except Exception:
            pass
        logger.info("odom turn %+.1f° done=%s: theta %.3f→%.3f rad",
                    turn_deg, done, theta0, theta_end)

    @staticmethod
    def _wait_lane_fresh(api: ChassisClient, timeout_s: float = 5.0) -> bool:
        """start_lane_feed 后轮询 lane_state 直到新鲜 (age_ms<500ms)."""
        deadline = time.monotonic() + max(timeout_s, 0.5)
        while time.monotonic() < deadline:
            try:
                if api.read_lane().is_fresh:
                    return True
            except Exception:
                pass
            time.sleep(0.05)
        return False

    # ── 任务后机械臂归位 (2026-08-03) ───────────────────────────

    @staticmethod
    def _should_defer_task4_handoff(
        waypoints: List[Waypoint], index: int,
    ) -> bool:
        """仅在当前选中序列紧接 task5 时启用 task4→task5 交接。"""
        if index < 0 or index >= len(waypoints):
            return False
        if waypoints[index].task_id != 4 or index + 1 >= len(waypoints):
            return False
        return waypoints[index + 1].task_id == 5

    @staticmethod
    def _start_task4_task5_handoff(client: RuntimeApiClient):
        """后台并行完成 task4 关仓 + task5 Phase 1 入场姿态。

        线程立即启动，调用方可先恢复 lane 巡航；task5 waypoint 执行前再 join。
        """
        import threading
        from concurrent.futures import ThreadPoolExecutor, as_completed
        from main.arm import ArmClient
        from main.arm.each_task.task4.constants import (
            STORAGE_CLOSE_ANGLE_DEG, STORAGE_OPEN_SPEED,
        )
        # 交接姿态以 task5 权威常量为准, 避免两处硬编码漂移。
        from main.task.task5_sort import (
            PHASE1_ARM_DEG, PHASE1_X_MM, PHASE1_Y_MM, PHASE1_HAND_DEG,
        )

        state = {"ok": False, "storage_ok": False, "pose_ok": False, "error": None}

        def _run() -> None:
            try:
                arm = ArmClient(http=client)
                logger.info("[task4→task5] handoff 开始: 关仓 + Phase 1 姿态")

                def _close_storage():
                    return arm.set_storage_angle(
                        STORAGE_CLOSE_ANGLE_DEG,
                        speed=STORAGE_OPEN_SPEED,
                        timeout=10.0,
                    )

                def _move_phase1():
                    return arm.composite_run(
                        arm=PHASE1_ARM_DEG, x_mm=PHASE1_X_MM,
                        y_mm=PHASE1_Y_MM, hand=PHASE1_HAND_DEG,
                        speed=80, timeout=30.0,
                    )

                def _action_ok(result) -> bool:
                    if not isinstance(result, dict):
                        return False
                    if result.get("ok"):
                        return True
                    return (
                        result.get("status") == "succeeded"
                        and isinstance(result.get("result"), dict)
                        and bool(result["result"].get("ok", False))
                    )

                with ThreadPoolExecutor(max_workers=2,
                                        thread_name_prefix="task4-task5-handoff") as pool:
                    futures = {
                        pool.submit(_close_storage): "storage",
                        pool.submit(_move_phase1): "pose",
                    }
                    for future in as_completed(futures):
                        name = futures[future]
                        result = future.result()
                        ok = _action_ok(result)
                        state[f"{name}_ok"] = ok
                        if not ok:
                            raise RuntimeError(f"{name} failed: {result}")
                logger.info("[task4→task5] handoff 完成: 关仓 + Phase 1 姿态已到位")
            except Exception as exc:
                state["error"] = f"{type(exc).__name__}: {str(exc)[:160]}"
                logger.warning("[task4→task5] handoff 失败: %s", state["error"])
            finally:
                try:
                    # ⚠️ 不能用 client.call(..., sync=True) —— call() 把 sync 吞进
                    # kwargs, execute() 走异步立即返回 status=queued。必须显式
                    # execute(sync=True) 才会 wait_job 到 succeeded。
                    start_res = client.execute(
                        "car", "start_arm_feed",
                        kwargs={"hz": 20.0}, timeout=5.0, sync=True,
                    )
                    state["arm_feed_started"] = bool(
                        isinstance(start_res, dict)
                        and start_res.get("status") == "succeeded"
                    )
                except Exception as exc:
                    state["arm_feed_started"] = False
                    logger.warning("[task4→task5] arm_feed 恢复失败: %s", exc)
            # ok 必须包含 arm_feed 恢复成功 —— 否则 task5 会误信
            # phase1_pose_ready 跳过自身 Phase 1 摆臂。
            state["ok"] = bool(
                state["storage_ok"] and state["pose_ok"]
                and state.get("arm_feed_started", False)
            )
        thread = threading.Thread(target=_run, daemon=True,
                                  name="task4-task5-handoff")
        thread.handoff_state = state
        thread.start()
        return thread

    @staticmethod
    def _wait_task4_task5_handoff(thread) -> bool:
        """task5 入场前等待真实 handoff 线程结束，不使用固定 sleep。"""
        if thread is None:
            return True
        thread.join()
        return bool(getattr(thread, "handoff_state", {}).get("ok", False))

    @staticmethod
    def _schedule_arm_home_reset() -> None:
        """每个任务结束后, 把机械臂强制 reset 到 home 姿态。

        home 姿态: x=0, y=-150, arm=+90, hand=-90。
        走后台线程, 不阻塞 orchestrator 主线程 (_resume_lane 继续巡航)。
        失败 / 超时仅打 warning, 不影响后续任务。
        """
        import threading

        def _wait_job_silent(action_name: str, kwargs: dict, wait_s: float) -> bool:
            """fire-and-forget 提交一个 arm action, 阻塞等到 succeeded 或超时。"""
            from main.api_client import RuntimeApiClient
            try:
                client = RuntimeApiClient()
                job = client.execute("arm", action_name, kwargs=kwargs, sync=False)
                jid = job.get("id") if isinstance(job, dict) else None
                if jid:
                    client.wait_job(jid, timeout=wait_s)
                return True
            except Exception as exc:
                logger.warning("  [reset] %s 失败: %s", action_name, exc)
                return False

        def _bg_reset() -> None:
            try:
                logger.info("  [arm-home] reset 开始 (后台, 不阻塞巡航)")
                # 1) y 抬到 -150 (高于保护区的安全高度)
                _wait_job_silent("composite_run",
                                 dict(arm=None, x=None, y=-0.15, hand=None,
                                      speed=100, timeout=5.0),
                                 wait_s=5.0)
                # 2) x 编码器归 0
                _wait_job_silent("move_x_position",
                                 dict(target=0.0, v_max_mms=100.0,
                                      out_time=10.0, timeout=15.0),
                                 wait_s=15.0)
                # 3) arm=+90° (左边) + hand=-90° (正前面)
                #    一发 composite_run 并发
                _wait_job_silent("composite_run",
                                 dict(arm=90.0, x=None, y=None, hand=-90.0,
                                      speed=100, timeout=15.0),
                                 wait_s=15.0)
                logger.info("  [arm-home] reset 完成")
            except Exception as exc:
                logger.warning("  [arm-home] reset 失败: %s", exc)

        threading.Thread(target=_bg_reset, daemon=True,
                         name="arm-home-reset").start()

    def _schedule_arm_pose(self, pose, label: str,
                           target_task_id: Optional[int] = None) -> None:
        """后台线程把机械臂摆到目标姿态 (不阻塞巡航), 供 task3 识别/射击区途中准备.

        与 task3_shoot._set_shooting_pose 同款 subprocess 调 arm_seq_v9;
        _wait_pending_arm_pose 在目标任务点前等待完成 (车已停, 来不及就停下调整).
        """
        import subprocess

        done = threading.Event()

        def _bg() -> None:
            try:
                command = [
                    sys.executable, "-m", "main.task.task3.arm_seq_v9",
                    "--y1", pose[0], "--y2", pose[1], "--x", pose[2],
                    "--arm-angle", pose[3], "--hand-angle", pose[4],
                ]
                logger.info("[arm-pose] %s 后台摆臂启动: %s",
                            label, " ".join(command))
                rc = subprocess.run(command, check=False).returncode
                logger.info("[arm-pose] %s 后台摆臂%s", label,
                            "完成" if rc == 0 else f"失败 (rc={rc})")
            except Exception as exc:
                logger.warning("[arm-pose] %s 后台摆臂异常: %s", label, exc)
            finally:
                done.set()

        threading.Thread(target=_bg, daemon=True,
                         name=f"arm-pose-{label}").start()
        self._pending_arm_pose = {
            "label": label, "done": done, "target_task_id": target_task_id,
        }

    def _wait_pending_arm_pose(self, task_id: Optional[int],
                               timeout_s: float = 60.0) -> None:
        """目标任务点触发后 (车已停) 等待后台摆臂完成; 无 pending 直接返回.

        若 pending 姿态不是给当前任务准备的, 不等待 (留给后续目标任务).
        """
        pending = getattr(self, "_pending_arm_pose", None)
        if not pending:
            return
        if pending.get("target_task_id") not in (None, task_id):
            return
        self._pending_arm_pose = None
        done = pending["done"]
        label = pending["label"]
        if done.is_set():
            logger.info("[arm-pose] %s 途中摆臂已完成", label)
            return
        logger.info("[arm-pose] 车已停, 等待 %s 摆臂完成 (最多 %.0fs)...",
                    label, timeout_s)
        if done.wait(timeout_s):
            logger.info("[arm-pose] %s 摆臂就绪", label)
        else:
            logger.warning("[arm-pose] %s 摆臂超时未完成, 任务内会再次确认姿态", label)

    @staticmethod
    def _run_task(client: RuntimeApiClient, wp: Waypoint,
                  **task_kwargs) -> Dict[str, Any]:
        """按 task_id 查 TASK_RUNNERS 字典, 调 run(). 返回完整 result dict。"""
        if wp.task_id is None:
            return {"ok": True, "skipped": True}
        try:
            from main.task import TASK_RUNNERS
            runner = TASK_RUNNERS[wp.task_id]
        except (ImportError, KeyError) as exc:
            logger.warning("task_id=%d not registered in TASK_RUNNERS: %s",
                           wp.task_id, exc)
            return {"ok": False, "error": str(exc)}
        try:
            result = runner(client, **task_kwargs)
        except NotImplementedError as exc:
            logger.warning("task_id=%d not implemented, skipping: %s",
                           wp.task_id, exc)
            return {"ok": False, "error": str(exc), "skipped": True}
        except Exception:
            logger.exception("task %s raised exception", wp.name)
            return {"ok": False, "error": "exception"}
        if not isinstance(result, dict):
            result = {"ok": True, "raw": result}
        if not result.get("ok"):
            logger.warning("task %s failed: %s", wp.name,
                           result.get("error", result.get("detail", "?")))
        else:
            logger.info("task %s (id=%d) succeeded -> %s", wp.name, wp.task_id, result)
        return result


__all__ = ["Waypoint", "Orchestrator", "DEFAULT_WAYPOINTS"]
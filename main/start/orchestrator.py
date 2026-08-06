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
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=9.00, trigger_op="AND"),
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

    def _run_mission(self, waypoints: List[Waypoint]) -> List[str]:
        """核心逻辑：初始化底盘/巡线/IR/里程计/TUI，按 waypoints 列表顺序导航并执行任务。

        由 run()（全流程）和 run_single_task()（单任务测试）共用。
        返回 completed 列表。
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
        from main.task._config import load_crossroad_turn, load_post_task1
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

        # 后台 A：DoubleLoopRunner 50Hz 巡线（#1：用 runner.pause/resume 控制暂停）
        # max_seconds=inf：常驻，由 runner.stop() 终止
        runner_thread = threading.Thread(
            target=runner.run,
            kwargs={"max_seconds": math.inf},
            daemon=True, name="lane",
        )
        runner_thread.start()

        # 后台 B：里程计（全程累计，写共享 buffer）
        dis_buf = [0.0]
        threading.Thread(target=read_dis,
                         kwargs={"api": api, "hz": 20.0,
                                 "on_tick": lambda v: dis_buf.__setitem__(0, v)},
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

        completed: List[str] = []
        try:
            for wp in waypoints:
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
                if wp.task_module:
                    tui_buf[0] = {"wp": wp.name, "dis": dis_buf[0],
                                  "ir_left": None, "ir_right": None,
                                  "state": "task"}
                    ok = self._run_task(client, wp)
                    if not ok:
                        logger.warning("task %s did not succeed, continuing to next waypoint", wp.name)
                time.sleep(wp.pause_after_s)
                # task1 播种用 move_to_position 闭环跑格点, 结束时 odom 累积漂移
                # (实车实测 x=1.40 y=0.31 vs 期望 x≈0.30 y=0). 清零给下一段巡航干净基线.
                # 只清零底盘里程 (car.reset_position), 不碰机械臂; read_dis 的单调保护
                # 会保持上次读数, 不污染后续 waypoint 的累计 dis 阈值.
                if wp.task_id == 1:
                    try:
                        client.execute("car", "reset_position", sync=True, timeout=10.0)
                        logger.info("odometry reset after task1: next segment from distance 0")
                    except Exception as exc:
                        logger.warning("odom reset after task1 failed: %s", exc)
                    if post_task1 is not None:
                        self._post_task1_maneuver(api, post_task1)
                # 2026-08-03: 每个任务结束后强制 reset 机械臂到 home 姿态
                # (x=0, y=-150, arm=+90, hand=-90), 边重置边巡航 ——
                # reset 在后台线程跑, 不阻塞 _resume_lane。
                self._schedule_arm_home_reset()
                self._resume_lane(runner)
                completed.append(wp.name)
        except KeyboardInterrupt:
            logger.info("interrupted by user")
        finally:
            # 终止 runner（#1）：stop() 唤醒 _pause.wait() 让 run() 看到 _stop，
            # join 等其 finally 块跑完（smoother 归零 + api.close()）。
            # 同时关掉 TUI 后台线程.
            tui_running.clear()
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
            # 注意：不要再调 api.close() —— runner 的 finally 已经调过了
            logger.info("mission completed: %s", completed)
        return completed

    def run(self) -> None:
        """全流程 8 任务（巡线 + IR/里程计触发 + 顺序执行）。"""
        self._run_mission(self.waypoints)

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
        if not self._wait_lane_fresh(api):
            logger.warning("[post-task1] lane 未在超时内新鲜, resume 可能触发 watchdog 急停")

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

    @staticmethod
    def _run_task(client: RuntimeApiClient, wp: Waypoint) -> bool:
        """按 task_id 查 TASK_RUNNERS 字典, 调 run(). 返回 True 表示成功."""
        if wp.task_id is None:
            # 纯导航段或 finish, 不应到这里
            return True
        try:
            from main.task import TASK_RUNNERS
            runner = TASK_RUNNERS[wp.task_id]
        except (ImportError, KeyError) as exc:
            logger.warning("task_id=%d not registered in TASK_RUNNERS: %s",
                           wp.task_id, exc)
            return False
        try:
            result = runner(client)
        except NotImplementedError as exc:
            # 未实现 task (3/7) 抛 NotImplementedError, warning + 跳过
            logger.warning("task_id=%d not implemented, skipping: %s",
                           wp.task_id, exc)
            return False
        except Exception:
            logger.exception("task %s raised exception", wp.name)
            return False
        if isinstance(result, dict) and not result.get("ok"):
            logger.warning("task %s failed: %s", wp.name,
                           result.get("error", result.get("detail", "?")))
            return False
        logger.info("task %s (id=%d) succeeded -> %s", wp.name, wp.task_id, result)
        return True


__all__ = ["Waypoint", "Orchestrator", "DEFAULT_WAYPOINTS"]
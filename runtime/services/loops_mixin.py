#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""后台循环 / 内存降档 Mixin（从 runtime_service.py 拆出）。

- `_resource_probe_loop`：每 30s 读 RSS，高水位降档、低水位恢复 feed hz
- `_feed_watchdog_loop`：定期巡检 feed 守护线程，死了/卡了自动 restart
依赖聚合类提供 `self.car`、`self._resource_lock`、`self._feed_current_hz`、
`self.stream_service` 等属性。
"""
import logging
import os
import time

from runtime.core import settings

logger = logging.getLogger(__name__)


class LoopsMixin:
    """CarRuntimeService 的内存降档 / feed watchdog 行为。"""

    # 2026-08-01：内存压力降档（详见 .trae/specs/system-arch-optimization/spec.md）。
    def _resource_probe_loop(self):
        """后台守护线程：每 30s 读 psutil RSS，触发降档/恢复。

        不动业务；只调用各 feed 的 restart_*（已存在的幂等接口），把 hz 砍半；
        feeds.degraded 列表按降档顺序累计。恢复条件：RSS 持续 60s 低于阈值。
        """
        # 启动后给 60s 让 init 路径完成（按 spec 建议）
        time.sleep(60.0)
        pressure_mb = settings.get_car_memory_pressure_mb()
        hard_mb = settings.get_car_rss_limit_mb()
        last_below = None  # time.time() of first RSS <= (pressure-200)
        last_high_warn = 0.0
        while True:
            try:
                rss_mb = self._read_self_rss_mb()
                if rss_mb is None:
                    time.sleep(30.0)
                    continue
                # 1) 软限 warn（不动业务，只 log + 上报到 /v1/health）
                if rss_mb > hard_mb and (time.time() - last_high_warn) > 60.0:
                    print(
                        "[CarRuntimeService] RSS high: {:.0f}MB > hard limit {}MB".format(
                            rss_mb, hard_mb
                        )
                    )
                    last_high_warn = time.time()
                # 2) 高水位触发降档
                if rss_mb > pressure_mb:
                    self._degrade_one_step()
                    last_below = None
                # 3) 恢复
                elif rss_mb < pressure_mb - 200:
                    if last_below is None:
                        last_below = time.time()
                    elif (time.time() - last_below) >= 60.0:
                        self._restore_one_step()
                        last_below = time.time()  # 重置计数；下次再等 60s
                else:
                    last_below = None
            except Exception as exc:
                # 探测线程本身崩了不影响业务
                print("[CarRuntimeService] resource_probe err: {}".format(exc))
            time.sleep(30.0)

    def _read_self_rss_mb(self):
        try:
            import psutil as _psutil
            rss = _psutil.Process(os.getpid()).memory_info().rss
            return rss / (1024.0 * 1024.0)
        except Exception:
            return None

    def _degrade_one_step(self):
        """按 _degrade_order 顺序找第一个还没降档的 feed，把 hz 砍半。"""
        with self._resource_lock:
            for feed_name in self._degrade_order:
                if feed_name == "lane":
                    continue  # 永不降档
                cur = self._feed_current_hz.get(feed_name, 0.0)
                default = self._feed_default_hz.get(feed_name, cur)
                if cur >= default:
                    # 当前还在 default，下一次降到 default/2
                    new_hz = max(default / 2.0, 5.0)  # 最低 5Hz
                    self._feed_current_hz[feed_name] = new_hz
                    self._apply_feed_hz(feed_name, new_hz)
                    if feed_name not in self._feeds_degraded:
                        self._feeds_degraded.append(feed_name)
                    return feed_name
                # 已降过档，看看下一档空间
                if cur > 5.0:
                    new_hz = max(cur / 2.0, 5.0)
                    self._feed_current_hz[feed_name] = new_hz
                    self._apply_feed_hz(feed_name, new_hz)
                    return feed_name
        return None

    def _restore_one_step(self):
        """按 _degrade_order 反向找最后一个降过档的 feed，hz 翻倍。"""
        with self._resource_lock:
            for feed_name in reversed(self._degrade_order):
                if feed_name == "lane":
                    continue
                cur = self._feed_current_hz.get(feed_name, 0.0)
                default = self._feed_default_hz.get(feed_name, cur)
                if cur >= default:
                    if feed_name in self._feeds_degraded:
                        self._feeds_degraded.remove(feed_name)
                    continue
                new_hz = min(cur * 2.0, default)
                if new_hz >= default - 0.01:
                    new_hz = default
                    if feed_name in self._feeds_degraded:
                        self._feeds_degraded.remove(feed_name)
                self._feed_current_hz[feed_name] = new_hz
                self._apply_feed_hz(feed_name, new_hz)
                return feed_name
        return None

    def _apply_feed_hz(self, feed_name, hz):
        """调对应 feed 的 restart_*（已存在的幂等接口）。"""
        try:
            car = self.car
        except Exception:
            car = None
        if car is None:
            return
        try:
            if feed_name == "lane" and hasattr(car, "restart_lane_feed"):
                car.restart_lane_feed(hz=float(hz))
            elif feed_name == "arm" and hasattr(car, "restart_arm_feed"):
                car.restart_arm_feed(hz=float(hz))
            elif feed_name == "task" and hasattr(car, "restart_task_feed"):
                car.restart_task_feed(hz=float(hz))
            elif feed_name == "ir" and hasattr(car, "restart_ir_feed"):
                car.restart_ir_feed(hz=float(hz))
            elif feed_name == "odom" and hasattr(car, "restart_odom_feed"):
                car.restart_odom_feed(hz=float(hz))
            # 95% 高水位降 encoder quality/scale
            hard_mb = settings.get_car_rss_limit_mb()
            rss_mb = self._read_self_rss_mb()
            if (
                rss_mb is not None
                and rss_mb > hard_mb * 0.95
                and self.stream_service is not None
                and hasattr(self.stream_service, "set_encode_quality")
            ):
                self.stream_service.set_encode_quality(quality=60, scale=0.5)
        except Exception as exc:
            print("[CarRuntimeService] apply_feed_hz({}) err: {}".format(feed_name, exc))

    def set_memory_pressure_for_test(self, rss_mb):
        """测试入口：手动假装 RSS 是 rss_mb，触发一次降档/恢复判定。

        debug 用，真实环境由 ResourceProbeThread 接管。
        """
        pressure_mb = settings.get_car_memory_pressure_mb()
        if rss_mb > pressure_mb:
            return self._degrade_one_step()
        elif rss_mb < pressure_mb - 200:
            return self._restore_one_step()
        return None

    # === 2026-08-01：feed watchdog 自动复活 ===
    # 表驱动：feed_name → (thread_attr, health_attr, restart_method, default_hz)
    # - thread_attr：MyCar 上的守护线程属性
    # - health_attr：MyCar 上的心跳 dict 属性（可选）
    # - restart_method：MyCar 上的 stop+start 方法
    # - default_hz：默认频率
    _FEED_WATCHDOG_TABLE = (
        ("lane", "_lane_feed_thread", "_lane_feed_health", "restart_lane_feed", 50.0),
        ("task", "_task_feed_thread", "_task_feed_health", "restart_task_feed", 30.0),
        ("arm",  "_arm_feed_thread",  "_arm_feed_health",  "restart_arm_feed",  20.0),
        ("ir",   "_ir_feed_thread",   "_ir_feed_health",   "restart_ir_feed",   50.0),
        ("odom", "_odom_feed_thread", "_odom_feed_health", "restart_odom_feed", 50.0),
    )

    def _feed_watchdog_loop(self):
        """定期巡检 feed 守护线程，死了/卡了自动 restart。"""
        try:
            interval = float(os.environ.get("RAK_CAR_FEED_WATCHDOG_INTERVAL_S", "15"))
        except ValueError:
            interval = 15.0
        # 启动后给 30s 让 init 路径完成
        time.sleep(30.0)
        # stale 阈值：health.last_iter_at 距今超过这个秒数就算"卡了"
        # lane 50Hz (period=20ms) → 1s 内至少 50 次 iter；
        # 5s 阈值足够宽（容忍 init / 重连慢路径），又不会让车在路上失明太久。
        stale_iter_seconds = 5.0
        while True:
            try:
                self._feed_watchdog_tick(stale_iter_seconds)
            except Exception as exc:
                print("[CarRuntimeService] feed_watchdog tick err: {}".format(exc))
            time.sleep(interval)

    def _feed_watchdog_tick(self, stale_iter_seconds):
        try:
            car = self.car
        except Exception:
            car = None
        if car is None:
            return
        # car 引用可能在 init / recover 切换瞬间变 None，
        # 本 tick 直接跳过，不抛错。
        car_ref = car
        now = time.time()
        for (
            feed_name,
            thread_attr,
            health_attr,
            restart_method,
            default_hz,
        ) in self._FEED_WATCHDOG_TABLE:
            thread = getattr(car_ref, thread_attr, None)
            health = getattr(car_ref, health_attr, None)
            # 1) 线程不存在 / 已经死掉 → restart
            dead = (
                thread is None
                or not thread.is_alive()
            )
            # 2) 线程活着但 health 报告自己 alive=False → 已经显式 stop 过，不动
            # （业务调 stop_lane_feed 是用户的意图）
            explicit_stop = (
                isinstance(health, dict) and health.get("alive") is False
            )
            # 3) 线程活着 + health 报告 alive=True 但 last_iter_at 太久没动
            # → 卡在 ZMQ 永久 EAGAIN / cv2 永久失败 → restart
            stale = False
            if not dead and not explicit_stop and isinstance(health, dict):
                last_iter = health.get("last_iter_at") or 0.0
                last_ok = health.get("last_ok_at") or 0.0
                # (a) iter 完全不跑 → 卡死
                if last_iter > 0 and (now - last_iter) > stale_iter_seconds:
                    stale = True
                # (b) 2026-08-01：iter 在跑但推理持续失败（守护空转）。
                # lane_feed 50Hz 但推理一直 EAGAIN → ok_count 不涨 →
                # 车在路上失明。last_ok 距今 > 30s 视为守护空转，重启 client。
                elif (
                    last_ok > 0
                    and (now - last_ok) > stale_iter_seconds * 6
                ):
                    stale = True
                # (c) 健康从来没成功过（init 期僵尸）→ 30s 后重启
                elif last_ok == 0 and last_iter > 0 and (now - last_iter) > 30.0:
                    stale = True
            if not (dead or stale):
                continue
            reason = (
                "dead" if dead
                else ("stale_ok" if (
                    isinstance(health, dict)
                    and (health.get("last_ok_at") or 0) > 0
                ) else "stale_iter")
            )
            logger.warning(
                "feed watchdog: %s %s (thread=%s, alive_flag=%s, ok=%d, err=%d) → restart",
                feed_name, reason,
                "None" if thread is None else ("alive" if thread.is_alive() else "dead"),
                health.get("alive") if isinstance(health, dict) else "n/a",
                health.get("ok_count", 0) if isinstance(health, dict) else -1,
                health.get("err_count", 0) if isinstance(health, dict) else -1,
            )
            try:
                restart_fn = getattr(car_ref, restart_method, None)
                if restart_fn is None:
                    continue
                # restart_* 内部 stop + start; start 会创建新 ClintInterface,
                # 旧 socket 引用丢弃。EAGAIN 死 socket 的最干净修复。
                restart_fn(hz=float(default_hz))
            except Exception as exc:
                logger.warning(
                    "feed watchdog: restart %s failed: %s", feed_name, exc,
                )

    def get_feeds_degraded(self):
        with self._resource_lock:
            return list(self._feeds_degraded)

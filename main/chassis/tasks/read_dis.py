"""main/chassis/tasks/read_dis.py
实时读取车端里程计累计行驶距离，按频率回调当前距离。

用法::

    from main.chassis import ChassisClient
    from main.chassis.tasks.read_dis import read_dis

    api = ChassisClient.connect()

    def on_distance(value):
        # value = 当前累计行驶距离（m）
        ...

    read_dis(api, hz=20.0, max_seconds=60.0, on_tick=on_distance)
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from ..api import ChassisClient

# 每帧回调：当前累计行驶距离（m）
DisTickCallback = Callable[[float], None]

__all__ = ["read_dis", "DisTickCallback"]


def _read_distance(api: ChassisClient, *, timeout: float = 5.0) -> float:
    """单次读取车端累计行驶距离（m），失败返回 0.0。

    fast-path（2026-07-31）：runtime odom_feed 守护线程 50Hz 喂 streamer.odom_state，
    通过 /v1/realtime/odom/state 拉（不进 job_queue、不打 MC602、不抢 car_lock）。
    fallback：原 car.get_distance() 同步 execute(主要在 runtime 升级前/feed 异常时)。
    """
    try:
        payload = api.http.get_odom_state()
    except Exception:
        payload = None
    if isinstance(payload, dict):
        odom = payload.get("odom_state") or {}
        if odom.get("active") and odom.get("distance") is not None:
            return float(odom["distance"])
    # fallback：原 job_queue + car_lock 路径
    try:
        dist = api.http.execute("car", "get_distance", timeout=timeout, sync=True)
        if isinstance(dist, dict) and "result" in dist:
            dist = dist["result"]
        return float(dist)
    except Exception:
        return 0.0


def read_dis(
    api: ChassisClient,
    *,
    hz: float = 20.0,
    max_seconds: Optional[float] = None,
    on_tick: Optional[DisTickCallback] = None,
    timeout: float = 2.0,
) -> None:
    """实时轮询车端里程计累计距离，每帧调用 on_tick。

    Args:
        api: ChassisClient。
        hz: 采样频率。建议 10-50Hz，过高浪费 HTTP 调用。
        max_seconds: 最大运行时长，None 表示一直跑（依赖 Ctrl-C 终止）。
        on_tick: 每帧回调，签名 `(value)`，读取失败传 float("nan")。
        timeout: 单次 HTTP 调用超时秒数。
    """
    if hz <= 0:
        raise ValueError("hz must be > 0")
    dt = 1.0 / hz
    deadline = None if max_seconds is None else time.monotonic() + max_seconds

    try:
        while True:
            t0 = time.monotonic()

            if deadline is not None and t0 >= deadline:
                break

            value: float
            try:
                value = _read_distance(api, timeout=timeout)
            except Exception:
                value = float("nan")

            if on_tick is not None:
                try:
                    on_tick(value)
                except Exception:
                    pass

            elapsed = time.monotonic() - t0
            sleep_s = dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass


def _tui_render(
    distance: float,
    elapsed: float,
    frame_count: int,
    width: int = 40,
) -> str:
    """生成单帧 TUI 字符串（纯 ANSI，无外部依赖）。"""
    fps = frame_count / elapsed if elapsed > 0 else 0.0

    bar_width = width - 4
    filled = max(0, min(int((distance % 1.0) * bar_width), bar_width))
    bar = "\u2588" * filled + "\u2591" * (bar_width - filled)

    lines = [
        "\033[u\033[s",  # 恢复上次保存的光标位置，再立即保存
        "\033[1;36m" + "=" * width + "\033[0m\033[K",
        "\033[1;37m  RAK-CAR 里程计实时监控\033[0m\033[K",
        "\033[1;36m" + "=" * width + "\033[0m\033[K",
        "\033[K",
        f"  \033[1;33m累计距离\033[0m    \033[1;37m{distance:8.3f} m\033[0m\033[K",
        f"  \033[2m[{bar}]\033[0m  \033[2m{(distance % 1.0) * 100:.0f}%\033[0m\033[K",
        "\033[K",
        f"  \033[2m已运行  {elapsed:7.1f}s  |  帧数  {frame_count:6d}  |  FPS  {fps:5.1f}\033[0m\033[K",
        "\033[K",
        "  \033[2m按 Ctrl-C 退出\033[0m\033[K",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    def _main() -> None:
        api = ChassisClient.connect()
        t_start = time.monotonic()
        frame = 0
        last_distance: float = 0.0

        # 保存当前光标位置，后续每帧覆盖前先回到这里
        print("\033[s", end="", flush=True)

        def _tick(value: float):
            nonlocal frame, last_distance
            frame += 1
            if value != value:  # NaN
                value = last_distance
            else:
                last_distance = value
            elapsed = time.monotonic() - t_start
            print(_tui_render(value, elapsed, frame), end="", flush=True)

        try:
            read_dis(api, hz=10.0, on_tick=_tick)
        except KeyboardInterrupt:
            pass
        finally:
            print("\033[2J\033[H", end="", flush=True)
            api.close()

    _main()

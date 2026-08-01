"""main/chassis/loops/tui.py
底盘外环的 TUI 调试面板。替代 lane_trace 的滚动打印。

依赖：Python 标准库 + rich
用法（CLI 已接 --tui，底层接口）：
    with lane_tui(outer, title="正交寻路 TUI") as make_cb:
        on_tick = make_cb(runner)
        DoubleLoopRunner(..., on_tick=on_tick).run(max_seconds=60)

快捷键（不用回车，单键即生效）：
  r / z    清零 outer 的积分
  p        切换 dry_run
  s        手动急停：发 [0,0,0,0]
  q        退出
"""
from __future__ import annotations

import atexit
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, List, Optional, TYPE_CHECKING

from rich.align import Align
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from ..state import LaneState

if TYPE_CHECKING:
    from ..controllers.base import OuterLoop
    from .closed_loop import DoubleLoopRunner


# ── 横向 bar / spark 工具 ─────────────────────────────────────
def _pct_bar(pct: float, width: int, filled: str = "█", empty: str = "░") -> str:
    """pct ∈ [-1, +1] → 以 50% 为中心，左右半屏。"""
    pct = max(-1.0, min(1.0, float(pct)))
    half = width // 2
    if pct >= 0:
        f = int(round(pct * half))
        return " " * half + filled * f + empty * (half - f)
    else:
        f = int(round(-pct * half))
        return " " * (half - f) + filled * f + " " * half


def _abs_bar(mag: float, width: int, filled: str = "█", empty: str = "░") -> str:
    mag = max(0.0, min(1.0, float(mag)))
    f = int(round(mag * width))
    return filled * f + empty * (width - f)


def _spark_line(seq: List[float], width: int, *, lo: Optional[float] = None, hi: Optional[float] = None) -> str:
    if not seq:
        return " " * width
    xs = [float(v) for v in seq]
    if lo is None:
        lo = min(xs)
    if hi is None:
        hi = max(xs)
    span = hi - lo
    if span < 1e-9:
        return "_" * width
    chars = "_▁▂▃▄▅▆▇█"
    n = len(chars) - 1
    k = max(1, len(xs) // width)
    out: List[str] = []
    for i in range(width):
        a = i * k
        b = min(len(xs), a + k)
        v = xs[-1] if a >= b else (sum(xs[a:b]) / (b - a))
        lv = int(round((v - lo) / span * n))
        lv = max(0, min(n, lv))
        out.append(chars[lv])
    return "".join(out)


# ── 历史 buffer / 统计 ────────────────────────────────────────
@dataclass
class _Series:
    name: str
    buf: Deque[float] = field(default_factory=lambda: deque(maxlen=900))

    def push(self, v: float) -> None:
        self.buf.append(float(v))

    def range(self) -> tuple[float, float]:
        if not self.buf:
            return -1.0, 1.0
        lo = min(self.buf)
        hi = max(self.buf)
        if hi - lo < 1e-9:
            return lo - 0.5, hi + 0.5
        return lo, hi

    def reset(self) -> None:
        self.buf.clear()


# ── TUI 主体 ───────────────────────────────────────────────────
class LaneTUI:
    """底盘外环常驻面板。用 q 退出，别 Ctrl+C 硬杀（会导致终端恢复不全）。"""

    def __init__(
        self,
        outer: "OuterLoop",
        *,
        title: str = "底盘寻路 · TUI",
        ui_hz: float = 12.0,
        history_seconds: float = 18.0,
        ey_scale: float = 0.06,
        ea_scale: float = 0.105,
        vy_scale: float = 0.45,
        omega_scale: float = 1.40,
        wheel_scale: float = 0.70,
    ) -> None:
        self.outer = outer
        self.title = title
        self.ui_interval = 1.0 / max(1.0, float(ui_hz))
        self.ey_scale = float(ey_scale)
        self.ea_scale = float(ea_scale)
        self.vy_scale = float(vy_scale)
        self.omega_scale = float(omega_scale)
        self.wheel_scale = float(wheel_scale)

        self._started_at = time.time()
        self._last_state: Optional[LaneState] = None
        self._last_wheels: List[float] = [0.0, 0.0, 0.0, 0.0]
        self._last_debug: dict = {}

        cap = max(10, int(50 * history_seconds))
        self.s_ey = _Series("ey")
        self.s_ea = _Series("ea")
        self.s_vy = _Series("vy")
        self.s_om = _Series("omega")
        for s in (self.s_ey, self.s_ea, self.s_vy, self.s_om):
            s.buf = deque(s.buf, maxlen=cap)

        self._events_lock = threading.Lock()
        self._events = {
            "reset_int": False,
            "toggle_dry": False,
            "estop": False,
            "quit": False,
        }

        self._kb_thread: Optional[threading.Thread] = None
        self._kb_stop = threading.Event()
        self._console = Console(highlight=False, soft_wrap=True)
        self._live: Optional[Live] = None
        self._last_ui_ts = 0.0
        self._lock = threading.Lock()
        self._runner_ref: Optional["DoubleLoopRunner"] = None
        self._dry_run_now: bool = False
        atexit.register(self.close)

    def make_callback(self, runner: "DoubleLoopRunner") -> Callable[[LaneState, List[float]], None]:
        self._runner_ref = runner
        self._dry_run_now = bool(getattr(runner, "dry_run", False))

        def _on_tick(state: LaneState, wheels: List[float]) -> None:
            self._last_state = state
            ws = list(wheels) + [0.0, 0.0, 0.0, 0.0]
            self._last_wheels = ws[:4]
            self._last_debug = {}
            if hasattr(self.outer, "debug_snapshot"):
                try:
                    self._last_debug = self.outer.debug_snapshot() or {}
                except Exception:
                    self._last_debug = {}
            if state.error_y is not None:
                self.s_ey.push(state.error_y)
            if state.error_angle is not None:
                self.s_ea.push(state.error_angle)
            self.s_vy.push(float(self._last_debug.get("vy", 0.0)))
            self.s_om.push(float(self._last_debug.get("omega", 0.0)))

            with self._events_lock:
                ev = dict(self._events)
                for k in self._events:
                    self._events[k] = False
            if ev["reset_int"]:
                self._do_reset_int()
            if ev["toggle_dry"] and self._runner_ref is not None:
                try:
                    new = not bool(getattr(self._runner_ref, "dry_run", False))
                    setattr(self._runner_ref, "dry_run", new)
                    self._dry_run_now = new
                    if not new:
                        sm = getattr(self._runner_ref, "smoother", None)
                        if sm is not None and hasattr(sm, "reset"):
                            sm.reset([0.0, 0.0, 0.0, 0.0])
                except Exception:
                    pass
            if ev["estop"] and self._runner_ref is not None:
                try:
                    api = getattr(self._runner_ref, "api", None)
                    if api is not None and hasattr(api, "set_wheel_speeds"):
                        api.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
                    sm = getattr(self._runner_ref, "smoother", None)
                    if sm is not None and hasattr(sm, "reset"):
                        sm.reset([0.0, 0.0, 0.0, 0.0])
                except Exception:
                    pass
            if ev["quit"]:
                raise SystemExit(0)

            now = time.perf_counter()
            if now - self._last_ui_ts >= self.ui_interval:
                self._last_ui_ts = now
                try:
                    self._render()
                except Exception:
                    pass

        return _on_tick

    def start(self) -> None:
        if self._live is not None:
            return
        try:
            self._live = Live(
                self._render_once(init=True),
                console=self._console,
                refresh_per_second=16,
                screen=False,
            )
            self._live.start()
        except Exception:
            self._live = None
        self._kb_stop.clear()
        self._kb_thread = threading.Thread(target=self._kb_loop, name="tui-kb", daemon=True)
        self._kb_thread.start()

    def close(self) -> None:
        try:
            self._kb_stop.set()
        except Exception:
            pass
        kb = self._kb_thread
        self._kb_thread = None
        if kb is not None and kb.is_alive():
            try:
                kb.join(timeout=0.5)
            except Exception:
                pass
        live = self._live
        self._live = None
        if live is not None:
            try:
                live.stop()
            except Exception:
                pass
        try:
            self._console.print(self._summary_str(), style="dim")
        except Exception:
            pass

    def __enter__(self):
        self.start()
        return self.make_callback

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    # ── 键盘 ─────────────────────────────────────────────────
    def _kb_loop(self) -> None:
        try:
            if not sys.stdin.isatty():
                return
        except Exception:
            return
        fd = sys.stdin.fileno()
        is_unix = (os.name != "nt") and hasattr(os, "O_NONBLOCK")
        if is_unix:
            try:
                import select
                import termios
                import tty
            except Exception:
                return
            try:
                old = termios.tcgetattr(fd)
            except Exception:
                return
            try:
                tty.setcbreak(fd)
            except Exception:
                return
            try:
                while not self._kb_stop.is_set():
                    r, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if not r:
                        continue
                    try:
                        ch = sys.stdin.read(1)
                    except Exception:
                        ch = ""
                    self._handle_key(ch)
            finally:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
                except Exception:
                    pass
        else:  # windows
            try:
                import msvcrt  # type: ignore
            except Exception:
                return
            while not self._kb_stop.is_set():
                time.sleep(0.05)
                try:
                    if not msvcrt.kbhit():
                        continue
                    ch = msvcrt.getwch()
                except Exception:
                    ch = ""
                self._handle_key(ch)

    def _handle_key(self, ch: str) -> None:
        if not ch:
            return
        c = ch.lower()
        with self._events_lock:
            if c in ("r", "z"):
                self._events["reset_int"] = True
            elif c == "p":
                self._events["toggle_dry"] = True
            elif c == "s":
                self._events["estop"] = True
            elif c == "q":
                self._events["quit"] = True

    # ── 操作：清零积分 ───────────────────────────────────────
    def _do_reset_int(self) -> None:
        if hasattr(self.outer, "reset_integrals"):
            try:
                self.outer.reset_integrals()  # type: ignore[attr-defined]
            except Exception:
                pass
        for name in ("ey_integral", "ea_integral", "_ey_integral", "_ea_integral",
                     "_theta_integral", "_y_integral"):
            if hasattr(self.outer, name):
                try:
                    setattr(self.outer, name, 0.0)
                except Exception:
                    pass
        for name in ("straight_streak_ms", "_straight_streak_ms",
                     "straight_release_ts", "_release_ts"):
            if hasattr(self.outer, name):
                try:
                    setattr(self.outer, name, 0.0)
                except Exception:
                    pass
        for s in (self.s_ey, self.s_ea, self.s_vy, self.s_om):
            s.reset()

    # ── 渲染 ─────────────────────────────────────────────────
    def _render(self) -> None:
        if self._live is None:
            return
        with self._lock:
            try:
                self._live.update(self._render_once())
            except Exception:
                pass

    def _render_once(self, init: bool = False):
        dur_s = 0.0 if init else max(0.0, time.time() - self._started_at)
        m, s = divmod(int(dur_s), 60)
        hh, mm = divmod(m, 60)
        dur = f"{hh:02d}:{mm:02d}:{int(s):02d}"
        st = self._last_state
        wheels = self._last_wheels
        dbg = self._last_debug
        dry = "DRY" if self._dry_run_now else "RUN"
        locked = (
            "LOCK" if bool(dbg.get("locked_vx", False)) else
            ("CRUS" if float(dbg.get("vx", 0.0)) > 0 else "IDLE")
        )
        feed_ok = "✓" if (st is None or getattr(st, "is_fresh", True)) else "✗"
        feed_mode = str(getattr(st, "mode", "n/a")) if st is not None else "n/a"
        active = (
            "active" if (st is not None and st.has_error and st.is_fresh) else
            ("stale" if (st is not None and not st.is_fresh) else "idle")
        )
        ey = st.error_y if st is not None else None
        ea = st.error_angle if st is not None else None
        ey_cm = f"{ey*100:+.2f}cm" if ey is not None else "  -  "
        ea_deg = f"{ea*57.3:+.2f}°" if ea is not None else "  -  "
        ey_pct = (ey / self.ey_scale) if ey is not None else 0.0
        ea_pct = (ea / self.ea_scale) if ea is not None else 0.0
        ctl_type = dbg.get("type", type(self.outer).__name__)
        if ctl_type == "orthogonal":
            vx = float(dbg.get("vx", 0.0))
            vy = float(dbg.get("vy", 0.0))
            om = float(dbg.get("omega", 0.0))
            vy_p = float(dbg.get("vy_p_term", 0.0))
            vy_i = float(dbg.get("vy_i_term", 0.0))
            ey_dz = float(dbg.get("vy_dz", 0.0))
            o_p = float(dbg.get("omega_p_term", 0.0))
            o_i = float(dbg.get("omega_i_term", 0.0))
            ea_dz = float(dbg.get("omega_dz", 0.0))
            iy = getattr(self.outer, "_ey_integral", None)
            it = getattr(self.outer, "_ea_integral", None)
            iy_cap = getattr(self.outer, "ey_int_cap", None)
            it_cap = getattr(self.outer, "ea_int_cap", None)
            iy_s = f"{iy:+.4f}/{iy_cap}" if (iy is not None and iy_cap) else (
                "" if iy is None else f"{iy:+.4f}")
            it_s = f"{it:+.4f}/{it_cap}" if (it is not None and it_cap) else (
                "" if it is None else f"{it:+.4f}")
            vy_pct = 0.0 if abs(self.vy_scale) < 1e-9 else vy / self.vy_scale
            om_pct = 0.0 if abs(self.omega_scale) < 1e-9 else om / self.omega_scale
            ctl_block = Group(
                Text.assemble(
                    ("vy 通道（横移修 d_e）: ", "bold cyan"),
                    (f"{vy:+.4f} m/s  =  P {vy_p:+.4f}  +  I {vy_i:+.4f}    d_e' = {ey_dz:+.5f}\n", "white"),
                    ("  I_y = ", "dim"), (iy_s, "yellow"),
                    ("    上限 ±", "dim"), (f"{self.vy_scale}\n", "dim"),
                    ("  ", ""),
                    (f"{_abs_bar(abs(vy_pct), 38)}  {100*abs(vy_pct):+.1f}%  ({vy:+.3f}/{self.vy_scale:.2f})", "white"),
                ),
                Text(""),
                Text.assemble(
                    ("ω 通道（旋转修 d_a）:  ", "bold magenta"),
                    (f"{om:+.4f} rad/s =  P {o_p:+.4f}  +  I {o_i:+.4f}    d_a' = {ea_dz:+.5f}\n", "white"),
                    ("  I_θ = ", "dim"), (it_s, "yellow"),
                    ("    上限 ±", "dim"), (f"{self.omega_scale}\n", "dim"),
                    ("  ", ""),
                    (f"{_abs_bar(abs(om_pct), 38)}  {100*abs(om_pct):+.1f}%  ({om:+.3f}/{self.omega_scale:.2f})", "white"),
                ),
            )
        else:
            vx = float(dbg.get("vx", 0.0))
            vy = float(dbg.get("vy", 0.0))
            om = float(dbg.get("omega", 0.0))
            keys = [k for k in list(dbg.keys())[:10] if k not in ("type", "vx", "vy", "omega", "wheels")]
            ctl_block = Group(
                Text.assemble(
                    ("控制律 = ", "dim"), (f"{ctl_type}\n", "bold"),
                    ("vx=", "dim"), (f"{vx:+.4f}  ", "white"),
                    ("vy=", "dim"), (f"{vy:+.4f}  ", "white"),
                    ("ω =", "dim"), (f"{om:+.4f}\n", "white"),
                    ("附加: ", "dim"),
                    (", ".join(f"{k}={dbg[k]}" for k in keys), "white"),
                )
            )

        header = Panel(
            Align.left(
                Text.assemble(
                    ("[", "bold"),
                    (f"{locked:4s}", "bold green" if locked == "LOCK" else "bold yellow"),
                    ("]  ", "bold"),
                    ("运行时长 ", "dim"), (f"{dur}   ", "white"),
                    ("控制律 ", "dim"), (f"{ctl_type:14s}  ", "cyan"),
                    ("vx=", "dim"), (f"{vx:+.3f} m/s\n", "white"),
                    ("lane_feed: ", "dim"),
                    (f"{feed_mode:<16s}   fresh {feed_ok}   ",
                     "green" if feed_ok == "✓" else "red"),
                    ("状态 ", "dim"),
                    (active,
                     "green" if active == "active" else
                     ("yellow" if active == "stale" else "dim")),
                    ("   [", "bold"),
                    (dry, "bold yellow" if dry == "DRY" else "bold red"),
                    ("]", "bold"),
                ),
            ),
            title=f"[bold]{self.title}[/]",
            border_style="blue",
            subtitle="[dim]r/z=清零积分   p=切换 dry-run   s=急停零速   q=退出[/]",
            subtitle_align="left",
        )
        errors = Panel(
            Group(
                Text.assemble(
                    ("ey 横向偏差 ", "bold cyan"),
                    (f"{ey_cm:>8s}", "white"),
                    ("  / ±", "dim"), (f"{self.ey_scale*100:.0f}cm\n", "dim"),
                    (_pct_bar(ey_pct, 52), "bold cyan" if abs(ey_pct) > 0.85 else "cyan"),
                    ("   ◀ 左   ", "dim"), ("中线", "bold"), ("   右 ▶", "dim"),
                ),
                Text(""),
                Text.assemble(
                    ("ea 角度偏差 ", "bold magenta"),
                    (f"{ea_deg:>8s}", "white"),
                    ("  / ±", "dim"), (f"{self.ea_scale*57.3:.0f}°\n", "dim"),
                    (_pct_bar(ea_pct, 52), "bold magenta" if abs(ea_pct) > 0.85 else "magenta"),
                    ("   ◀ 车头偏左   ", "dim"), ("摆正", "bold"), ("   偏右 ▶", "dim"),
                ),
            ),
            title="lane 推理输入误差",
            border_style="cyan",
        )
        ctl = Panel(ctl_block,
                    title="控制律输出（十字正交：两通道完全解耦）",
                    border_style="yellow")
        wheels_p = Panel(self._render_wheels(wheels),
                         title="4 路电机目标线速度（m/s），麦轮逆解合成",
                         border_style="green")
        hist = Panel(self._render_history(),
                     title=f"最近 ~{self.s_ey.buf.maxlen // 50:d}s 历史曲线（ey / ea / vy / ω）",
                     border_style="magenta")
        grid = Table.grid(padding=(0, 1))
        grid.add_column()
        grid.add_row(header)
        grid.add_row(errors)
        grid.add_row(ctl)
        grid.add_row(wheels_p)
        grid.add_row(hist)
        return Panel(grid, border_style="none", expand=True)

    def _render_wheels(self, wheels: List[float]):
        def one(tag: str, v: float, col: str):
            pct = 0.0 if abs(self.wheel_scale) < 1e-9 else v / self.wheel_scale
            bar = _pct_bar(pct, 22)
            sign_c = "red" if v > 0 else ("blue" if v < 0 else "white")
            hi = abs(pct) > 0.75
            return Text.assemble(
                (f"{tag:>7s}:", f"bold {col}"),
                (f"{v:+.4f}  ", sign_c),
                (bar, f"bold green" if hi else "green"),
            )
        fl, fr, rl, rr = wheels[0], wheels[1], wheels[2], wheels[3]
        line1 = one("前左 FL", fl, "cyan") + Text("    ") + one("前右 FR", fr, "magenta")
        line2 = one("后左 RL", rl, "cyan") + Text("    ") + one("后右 RR", rr, "magenta")
        return Group(line1, Text(""), line2)

    def _render_history(self):
        w = 48
        def row(label: str, color: str, ser: _Series):
            buf = list(ser.buf)
            if not buf:
                return Text.assemble((f"{label:5s}:  ", color), (" " * w, "dim"))
            lo, hi = ser.range()
            sp = _spark_line(buf, w, lo=lo, hi=hi)
            return Text.assemble(
                (f"{label:5s}:  ", f"bold {color}"),
                (sp, color),
                (f"   末 {buf[-1]:+.4f}   范围 [{lo:+.3f}, {hi:+.3f}]", "dim"),
            )
        return Group(
            row("ey", "cyan", self.s_ey), Text(""),
            row("ea", "magenta", self.s_ea), Text(""),
            row("vy", "green", self.s_vy), Text(""),
            row("ω ", "yellow", self.s_om),
        )

    def _summary_str(self) -> str:
        dur = max(0.0, time.time() - self._started_at)
        st = self._last_state
        ey = (f"{st.error_y*100:+.2f}cm"
              if (st is not None and st.error_y is not None) else "-")
        ea = (f"{st.error_angle*57.3:+.2f}°"
              if (st is not None and st.error_angle is not None) else "-")
        return (f"[TUI 退出]  运行 {dur:.1f}s  末 ey={ey} ea={ea}  "
                f"控制律={self._last_debug.get('type', type(self.outer).__name__)}")


def lane_tui(outer, *, title: str = "底盘寻路 · TUI", ui_hz: float = 12.0, **kw):
    return LaneTUI(outer, title=title, ui_hz=ui_hz, **kw)

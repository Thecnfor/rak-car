#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/start/display_ui.py

MC602 下位机 led_show 屏幕高级 UI 引擎。

硬件能力
--------
- led_show (dev_id=0x0b): 100 字符 ASCII 点阵屏，协议格式 ``bbi`` (dev_id + mode + port_id + 100 bytes)。
  固件将 100 bytes 按代码页 437 解析，``\\n`` 为换行。
- nixietube (dev_id=0x0f): 4 位数码管，仅用于大数字高亮（可选）。

设计约束
--------
- 所有发往下位机的字符 ``ord()`` 必须在 0-255 范围内，否则高位被截断。
- 因此 **不能**直接用 Unicode box-drawing（U+2500 系列，ord > 255）。
- 方案：用代码页 437 的等价字形（ord <= 255），在 Python 代码中写 ``chr(cp437_byte)``，
  下位机固件按 cp437 显示为对应图形。

布局/分辨率
-----------
``LAYOUTS`` 定义了 100 字符屏的可选分辨率：
  - ``20x5`` : 宽扁仪表盘（默认，横向信息密度高）
  - ``25x4`` : 高密四行矩阵
  - ``16x6`` : 传统 LCD 风格
  - ``10x10``: 像素方块风

字符设计
--------
- Box Drawing: 用 ``chr(196/179/218/191/192/217/195/180/194/193/197)`` 等画边框、表格。
- Block Elements: ``chr(219)`` 全块、``chr(176-178)`` 阴影、``chr(220/223)`` 半块。
- 通过 ``draw_rect`` / ``draw_progress`` / ``draw_battery`` 等组件快速拼装。

用法
----
::

    from main.start.display_ui import Mc602Display

    display = Mc602Display(api_client, layout="20x5")
    display.clear()
    display.draw_rect(0, 0, display.width, display.height)
    display.draw_text(1, 1, "RAK-CAR READY", align="center")
    display.draw_progress(1, 2, display.width - 2, 0.75)
    display.draw_battery(display.width - 10, 3, level=0.8, charging=True)
    display.render()
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

# ── 代码页 437 图形字符表 ──────────────────────────────
# 每个字符的 ord() <= 255，可安全发往下位机。
# 固件按 IBM CP437 解析，显示为对应图形。
# Python 源码中用 chr(byte) 存储；注释标注其 Unicode 等价字形，方便阅读。

_CP437: Dict[str, str] = {
    # Box Drawing (Light)
    "h": chr(196),       # ─  BOX DRAWINGS LIGHT HORIZONTAL
    "v": chr(179),       # │  BOX DRAWINGS LIGHT VERTICAL
    "dl": chr(218),      # ┌  BOX DRAWINGS LIGHT DOWN AND RIGHT
    "dr": chr(191),      # ┐  BOX DRAWINGS LIGHT DOWN AND LEFT
    "ul": chr(192),      # └  BOX DRAWINGS LIGHT UP AND RIGHT
    "ur": chr(217),      # ┘  BOX DRAWINGS LIGHT UP AND LEFT
    "vl": chr(195),      # ├  BOX DRAWINGS LIGHT VERTICAL AND RIGHT
    "vr": chr(180),      # ┤  BOX DRAWINGS LIGHT VERTICAL AND LEFT
    "dh": chr(194),      # ┬  BOX DRAWINGS LIGHT DOWN AND HORIZONTAL
    "uh": chr(193),      # ┴  BOX DRAWINGS LIGHT UP AND HORIZONTAL
    "cross": chr(197),   # ┼  BOX DRAWINGS LIGHT VERTICAL AND HORIZONTAL
    # Box Drawing (Double)
    "dbar": chr(205),    # ═  BOX DRAWINGS DOUBLE HORIZONTAL
    # Block Elements / Shades
    "shade1": chr(176),  # ░  LIGHT SHADE
    "shade2": chr(177),  # ▒  MEDIUM SHADE
    "shade3": chr(178),  # ▓  DARK SHADE
    "block": chr(219),   # █  FULL BLOCK
    "uhalf": chr(223),   # ▀  UPPER HALF BLOCK
    "lhalf": chr(220),   # ▄  LOWER HALF BLOCK
    "lblock": chr(221),  # ▌  LEFT HALF BLOCK
    "rblock": chr(222),  # ▐  RIGHT HALF BLOCK
    # Misc
    "degree": chr(248),  # °  DEGREE SIGN
    "plus": chr(43),     # +
    "minus": chr(45),    # -
    "dot": chr(46),      # .
    "colon": chr(58),    # :
    "slash": chr(47),    # /
    "backslash": chr(92),# \\
    "bracket_l": chr(91),# [
    "bracket_r": chr(93),# ]
    "pipe": chr(124),    # |
    "star": chr(42),     # *
    "at": chr(64),       # @
    "hash": chr(35),     # #
    "dash": chr(45),     # -
}

# ── 布局定义 ──────────────────────────────────────────

LAYOUTS: Dict[str, Tuple[int, int]] = {
    "20x5": (20, 5),   # 宽扁仪表盘（默认）
    "25x4": (25, 4),   # 高密矩阵
    "16x6": (16, 6),   # 传统 LCD
    "10x10": (10, 10), # 像素方块
}


class Mc602Display:
    """MC602 下位机屏幕（100 字符）UI 渲染器。

    通过 ``RuntimeApiClient.execute("car", "show_text", ...)`` 更新屏幕。
    """

    def __init__(self, api_client, layout: str = "20x5"):
        if layout not in LAYOUTS:
            raise ValueError(f"未知布局: {layout}，可选: {list(LAYOUTS.keys())}")
        self.api = api_client
        self.width, self.height = LAYOUTS[layout]
        self._buf: List[List[str]] = [
            [" " for _ in range(self.width)] for _ in range(self.height)
        ]
        self._frame_id = 0
        self._last_render = 0.0

    # ── 基础操作 ──────────────────────────────────────

    def clear(self) -> None:
        """清空缓冲区。"""
        self._buf = [[" " for _ in range(self.width)] for _ in range(self.height)]

    def _clamp(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def _set(self, x: int, y: int, ch: str) -> None:
        if self._clamp(x, y):
            self._buf[y][x] = ch

    def _get(self, x: int, y: int) -> str:
        return self._buf[y][x] if self._clamp(x, y) else " "

    def _fill_rect(self, x: int, y: int, w: int, h: int, ch: str) -> None:
        """填充矩形区域。"""
        for iy in range(y, min(y + h, self.height)):
            for ix in range(x, min(x + w, self.width)):
                self._set(ix, iy, ch)

    # ── 组件 ──────────────────────────────────────────

    def draw_rect(self, x: int, y: int, w: int, h: int,
                  fill: bool = False, title: str = "") -> None:
        """画空心/实心矩形框，可选顶部标题。"""
        if w <= 0 or h <= 0:
            return
        x1, y1 = x, y
        x2, y2 = x + w - 1, y + h - 1
        if fill:
            self._fill_rect(x1, y1, w, h, _CP437["block"])
            return
        for iy in range(y1, y2 + 1):
            for ix in range(x1, x2 + 1):
                if iy == y1:
                    if ix == x1:
                        self._set(ix, iy, _CP437["dl"])
                    elif ix == x2:
                        self._set(ix, iy, _CP437["dr"])
                    else:
                        self._set(ix, iy, _CP437["h"])
                elif iy == y2:
                    if ix == x1:
                        self._set(ix, iy, _CP437["ul"])
                    elif ix == x2:
                        self._set(ix, iy, _CP437["ur"])
                    else:
                        self._set(ix, iy, _CP437["h"])
                else:
                    if ix == x1:
                        self._set(ix, iy, _CP437["v"])
                    elif ix == x2:
                        self._set(ix, iy, _CP437["v"])
        # 标题（覆盖上边框中间）
        if title:
            t = title[: max(0, w - 2)]
            tx = x1 + 1 + (w - 2 - len(t)) // 2
            for i, ch in enumerate(t):
                if 0 <= tx + i < self.width and y1 == y:
                    self._set(tx + i, y1, ch)

    def draw_text(self, x: int, y: int, text: str,
                  align: str = "left", max_width: Optional[int] = None) -> None:
        """在指定位置写文本，超出边界自动裁剪。"""
        if not (0 <= y < self.height):
            return
        text = text[: max_width or self.width]
        if align == "center":
            x = max(0, self.width // 2 - len(text) // 2)
        elif align == "right":
            x = max(0, self.width - len(text))
        for i, ch in enumerate(text):
            self._set(x + i, y, ch)

    def draw_progress(self, x: int, y: int, width: int, progress: float,
                      style: str = "block", label: str = "") -> None:
        """进度条（0.0~1.0）。

        style:
          - ``block``  : 全块填充，最醒目
          - ``shade``  : 阴影填充，柔和
          - ``bar``    : 半块交替，类似 braille 风格
        """
        progress = max(0.0, min(1.0, progress))
        filled = int(width * progress)
        for i in range(width):
            if i < filled:
                if style == "block":
                    self._set(x + i, y, _CP437["block"])
                elif style == "shade":
                    self._set(x + i, y, _CP437["shade3"])
                elif style == "bar":
                    self._set(x + i, y, _CP437["uhalf"] if i % 2 == 0 else _CP437["lhalf"])
            else:
                if style == "block":
                    self._set(x + i, y, _CP437["shade1"])
                elif style == "shade":
                    self._set(x + i, y, _CP437["shade1"])
                elif style == "bar":
                    self._set(x + i, y, " ")
        if label:
            self.draw_text(x, y - 1, label[:width])

    def draw_battery(self, x: int, y: int, level: float,
                     charging: bool = False, width: int = 4) -> None:
        """电池图标 + 百分比。

        :param level: 0.0~1.0
        :param charging: 是否充电中（显示 + 号）
        :param width: 电池内部宽度（默认 4）
        """
        level = max(0.0, min(1.0, level))
        w = width
        h = 3
        # 外框
        self._set(x, y, _CP437["dl"])
        self._set(x + w + 1, y, _CP437["dr"])  # 电池头
        for i in range(1, w + 1):
            self._set(x + i, y, _CP437["h"])
        for row in range(1, h + 1):
            self._set(x, y + row, _CP437["v"])
            self._set(x + w + 1, y + row, _CP437["v"])
        self._set(x, y + h, _CP437["ul"])
        self._set(x + w + 1, y + h, _CP437["ur"])
        for i in range(1, w + 1):
            self._set(x + i, y + h, _CP437["h"])
        # 电量填充
        fill_rows = int(h * level)
        for row in range(h):
            for col in range(1, w + 1):
                if row < fill_rows:
                    self._set(x + col, y + 1 + row, _CP437["block"])
                else:
                    self._set(x + col, y + 1 + row, " ")
        # 充电指示
        if charging:
            self._set(x + w // 2, y + 1, _CP437["plus"])
        # 百分比文字
        pct = f"{int(level * 100)}%"
        self.draw_text(x + w + 3, y, pct)

    def draw_gear(self, x: int, y: int, gear: int, max_gear: int = 5) -> None:
        """档位指示器（数字 + 进度条风格）。"""
        if not (0 <= gear <= max_gear):
            gear = 0
        self.draw_text(x, y, f"G{gear}")
        for i in range(max_gear):
            self._set(x + 2 + i, y, _CP437["block"] if i < gear else _CP437["shade1"])

    def draw_spinner(self, x: int, y: int, frame: int) -> None:
        """旋转动画帧。"""
        chars = ["|", _CP437["slash"], "-", "\\"]
        self._set(x, y, chars[frame % 4])

    def draw_task_list(self, x: int, y: int, tasks: List[Tuple[str, str]],
                       max_items: int = 5) -> None:
        """任务列表，状态用符号表示。

        :param tasks: [(name, status), ...]，status 为 ``done`` / ``run`` / ``wait`` / ``skip``
        """
        icons = {
            "done": chr(251),   # ✓  CHECK MARK
            "run": chr(62),     # ▶  BLACK RIGHT-POINTING POINTER
            "wait": chr(46),    # .  MIDDLE DOT
            "skip": chr(45),    # -  HYPHEN-MINUS
        }
        for idx, (name, status) in enumerate(tasks[:max_items]):
            icon = icons.get(status, "?")
            line = f"{icon} {name}"
            self.draw_text(x, y + idx, line[:self.width - x])

    def draw_bar_graph(self, x: int, y: int, width: int, values: List[float],
                       height: int = 3) -> None:
        """简易柱状图（竖向条形）。

        :param values: 0.0~1.0 的数值列表
        :param height: 柱子最大高度（字符行数）
        """
        if not values:
            return
        n = len(values)
        col_w = max(1, width // n)
        for col_idx, val in enumerate(values):
            val = max(0.0, min(1.0, val))
            bar_h = int(height * val)
            cx = x + col_idx * col_w
            for row in range(height):
                ch = _CP437["block"] if row < bar_h else _CP437["shade1"]
                for dx in range(col_w):
                    self._set(cx + dx, y + height - 1 - row, ch)

    # ── 渲染 ──────────────────────────────────────────

    def render(self, throttle_s: float = 0.2) -> bool:
        """将缓冲区发送到下位机屏幕（带简单帧率限制）。

        :param throttle_s: 最小发送间隔（秒），避免占满串口带宽。
        :return: 是否实际发送
        """
        now = time.monotonic()
        if now - self._last_render < throttle_s:
            return False
        self._last_render = now
        lines = ["".join(row) for row in self._buf]
        # 确保总字符数不超过 100
        max_chars = 100
        text = "\n".join(lines)
        if len(text) > max_chars:
            text = text[:max_chars]
        try:
            # 同步发送：显示动作必须串行完成，禁止高频异步提交堆满 job_queue。
            self.api.execute("car", "show_text", kwargs={"text": text},
                             sync=True, timeout=1.0)
            self._frame_id += 1
            return True
        except Exception:
            return False

    # ── 预设皮肤 ──────────────────────────────────────

    def skin_dashboard(self, state: str, wp: str, dis: float,
                       ir_left: Optional[float], ir_right: Optional[float],
                       battery: float = 1.0, charging: bool = False) -> None:
        """默认 20x5 仪表盘皮肤。"""
        self.clear()
        w, h = self.width, self.height
        # 外框
        self.draw_rect(0, 0, w, h)
        # 顶部标题栏
        self.draw_text(1, 1, "RAK-CAR 2026", align="center")
        # 状态
        self.draw_text(1, 2, f"ST:{state}")
        # 当前任务点
        self.draw_text(9, 2, f"WP:{wp}"[:12])
        # IR 距离
        il_s = f"{ir_left:.2f}m" if ir_left is not None else "----"
        ir_s = f"{ir_right:.2f}m" if ir_right is not None else "----"
        self.draw_text(1, 3, f"L:{il_s}")
        self.draw_text(w // 2, 3, f"R:{ir_s}", align="center")
        # 里程进度条（假设全程 ~17m）
        progress = max(0.0, min(1.0, dis / 17.0))
        self.draw_progress(1, 4, w - 2, progress, style="block")
        # 电池
        self.draw_battery(w - 10, 0, battery, charging)

    def skin_minimal(self, state: str, dis: float) -> None:
        """极简皮肤：大数字 + 小状态。"""
        self.clear()
        w, h = self.width, self.height
        self.draw_rect(0, 0, w, h)
        # 顶部状态
        self.draw_text(1, 1, state, align="center")
        # 大距离数字（尽可能放大显示）
        dis_str = f"{dis:.1f}m"
        self.draw_text(1, 2, dis_str, align="center")
        # 底部装饰条
        self.draw_progress(1, h - 1, w - 2, max(0.0, min(1.0, dis / 17.0)), style="bar")

    def skin_matrix(self, state: str, tasks: List[Tuple[str, str]],
                    dis: float) -> None:
        """25x4 矩阵皮肤：任务列表 + 进度。"""
        self.clear()
        w, h = self.width, self.height
        self.draw_rect(0, 0, w, h)
        self.draw_text(1, 1, state, align="center")
        self.draw_task_list(1, 2, tasks, max_items=h - 3)
        self.draw_progress(1, h - 1, w - 2, max(0.0, min(1.0, dis / 17.0)), style="shade")

    # ── 演示 ──────────────────────────────────────────

    @staticmethod
    def demo(api_client, layout: str = "20x5", loops: int = 50) -> None:
        """演示模式：循环展示各组件效果。"""
        d = Mc602Display(api_client, layout)
        for i in range(loops):
            d.clear()
            d.draw_rect(0, 0, d.width, d.height, title="DEMO")
            d.draw_text(1, 1, f"Layout {layout}", align="center")
            d.draw_progress(1, 2, d.width - 2, (i + 1) / loops, style="block")
            d.draw_battery(d.width - 10, 0, 0.6 + 0.4 * (i % 2), charging=(i % 2 == 0))
            d.draw_gear(d.width - 10, 3, (i % 6) + 1)
            d.draw_spinner(1, 3, i)
            d.draw_text(1, d.height - 1, f"#{i:03d}", align="right")
            d.render(throttle_s=0.12)
            time.sleep(0.12)

#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
快速采集工具 — 20类物体图像采集 (多线程优化版)
操作: [1-0/q-p] 选类  [空格] 拍照  [退格] 撤销  [c] 清空当前类  [ESC] 退出
"""

import cv2
import os
import sys
import json
import time
import threading
from concurrent.futures import ThreadPoolExecutor

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from camera import Camera
from log_info import logger
from vehicle import ScreenShow, Key4Btn

# ─────────────────────────────── 20 类定义 ───────────────────────────────
CATEGORIES = [
    ("animal",         "动物"),
    ("ball_blue",      "蓝色球"),
    ("ball_yellow",    "黄色球"),
    ("cylinder_1",     "圆柱体1"),
    ("cylinder_2",     "圆柱体2"),
    ("cylinder_3",     "圆柱体3"),
    ("cylinder_set",   "圆柱组合"),
    ("h_dou_jiao",     "豆角"),
    ("h_fan_qie",      "番茄"),
    ("h_jin_zhen_gu",  "金针菇"),
    ("h_mo_gu",        "蘑菇"),
    ("h_qin_cai",      "芹菜"),
    ("h_qing_jiao",    "青椒"),
    ("h_tu_dou",       "土豆"),
    ("h_xi_lan_hua",   "西兰花"),
    ("h_you_cai",      "油菜"),
    ("water",          "水容器"),
    ("water_l1",       "水容器L1"),
    ("water_l2",       "水容器L2"),
    ("water_l3",       "水容器L3"),
]

# cv2 按键 → 类别索引 (0-19)
KEY_MAP = {
    ord("1"): 0,  ord("2"): 1,  ord("3"): 2,  ord("4"): 3,  ord("5"): 4,
    ord("6"): 5,  ord("7"): 6,  ord("8"): 7,  ord("9"): 8,  ord("0"): 9,
    ord("q"): 10, ord("w"): 11, ord("e"): 12, ord("r"): 13, ord("t"): 14,
    ord("y"): 15, ord("u"): 16, ord("i"): 17, ord("o"): 18, ord("p"): 19,
}

BACKSPACE_KEYS = {8, 127}

# ANSI 转义
CLR = "\033[2J\033[H"
HOME = "\033[H"
HIDE_CUR = "\033[?25l"
SHOW_CUR = "\033[?25h"
BOLD = "\033[1m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RED = "\033[91m"
DIM = "\033[2m"
RESET = "\033[0m"
REVERSE = "\033[7m"

# TUI 刷新间隔（秒）—— 平衡流畅度与 I/O 开销
TUI_INTERVAL = 0.06  # ~16 fps, 足够流畅且不会抢 cv2 的 CPU


class QuickCollect:
    def __init__(self, cap_index: int = 1):
        self.base_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), "quick_collect")
        self.counter_file = os.path.join(self.base_dir, "counter.json")

        for name, _ in CATEGORIES:
            d = os.path.join(self.base_dir, name)
            if not os.path.exists(d):
                os.makedirs(d)

        self.counter = self._load_counter()
        self.selected = 0
        self.counts = [self._count_images(i) for i in range(len(CATEGORIES))]

        # 摄像头 + 硬件
        self.cap = Camera(cap_index, 640, 480)
        self.display = ScreenShow()
        self.key_btn = Key4Btn(1)

        # 线程安全锁：保护 selected / counts / counter / flash 状态
        self._lock = threading.Lock()
        # 磁盘 I/O 线程池（2 worker，避免 Jetson 过载）
        self._save_pool = ThreadPoolExecutor(max_workers=2)

        # TUI 状态
        self.flash_msg = ""
        self.flash_time = 0
        self._tui_dirty = threading.Event()    # 唤醒 TUI 线程
        self._tui_dirty_cat = False            # 类别行需要重绘
        self._tui_dirty_status = False         # 仅状态栏
        self._tui_full = True                  # 首次全量渲染
        self._tui_lines = []                   # TUI 行缓冲
        self._cat_start = 0
        self._status_start = 0
        self._running = True

        # TUI 后台渲染线程
        self._tui_thread = threading.Thread(target=self._tui_render_loop, daemon=True)
        self._tui_thread.start()

        # 硬件按键轮询线程
        self._hw_thread = threading.Thread(target=self._hw_poll_loop, daemon=True)
        self._hw_thread.start()

        self.run()

    # ──────────────── 持久化 ────────────────
    def _load_counter(self):
        if os.path.exists(self.counter_file):
            try:
                with open(self.counter_file, "r") as f:
                    return json.load(f).get("counter", 0)
            except Exception:
                pass
        max_n = 0
        for name, _ in CATEGORIES:
            d = os.path.join(self.base_dir, name)
            if os.path.isdir(d):
                for f in os.listdir(d):
                    if f.endswith(".jpg"):
                        try:
                            n = int(os.path.splitext(f)[0])
                            max_n = max(max_n, n)
                        except ValueError:
                            pass
        return max_n

    def _save_counter(self):
        try:
            with open(self.counter_file, "w") as f:
                json.dump({"counter": self.counter}, f)
        except Exception as e:
            logger.error(f"save counter failed: {e}")

    # ──────────────── 文件操作 ────────────────
    def _cat_dir(self, idx):
        return os.path.join(self.base_dir, CATEGORIES[idx][0])

    def _count_images(self, idx):
        d = self._cat_dir(idx)
        return len([f for f in os.listdir(d) if f.endswith(".jpg")]) if os.path.isdir(d) else 0

    def _save_image(self, img, idx):
        """拍照保存 — img 应已是 copy()，不含 overlay 文字。"""
        with self._lock:
            self.counter += 1
            self.counts[idx] += 1
            cnt = self.counter
            cat = CATEGORIES[idx][0]

        name = str(cnt).zfill(4) + ".jpg"
        path = os.path.join(self._cat_dir(idx), name)

        with self._lock:
            self.flash_msg = f"{GREEN}✓ saved {name} → {cat}{RESET}"
            self.flash_time = time.time()
        self._tui_wake(cat=True)

        # 磁盘 I/O 丢线程池
        self._save_pool.submit(cv2.imwrite, path, img)
        self._save_pool.submit(self._save_counter)
        logger.info(f"saved {path}")

    def _undo(self, idx):
        with self._lock:
            d = self._cat_dir(idx)
            files = sorted(f for f in os.listdir(d) if f.endswith(".jpg"))
            if not files:
                self.flash_msg = f"{YELLOW}nothing to undo in {CATEGORIES[idx][0]}{RESET}"
                self.flash_time = time.time()
                self._tui_wake(status=True)
                return
            last = files[-1]
            os.remove(os.path.join(d, last))
            self.counts[idx] -= 1
            self.flash_msg = f"{RED}✗ deleted {last} from {CATEGORIES[idx][0]}{RESET}"
            self.flash_time = time.time()
        self._tui_wake(cat=True)
        logger.info(f"deleted {os.path.join(d, last)}")

    def _clear_category(self, idx):
        """清空指定类别的全部图片。"""
        with self._lock:
            d = self._cat_dir(idx)
            files = [f for f in os.listdir(d) if f.endswith(".jpg")]
            count = len(files)
            if count == 0:
                self.flash_msg = f"{YELLOW}{CATEGORIES[idx][0]} already empty{RESET}"
                self.flash_time = time.time()
                self._tui_wake(status=True)
                return
            for f in files:
                os.remove(os.path.join(d, f))
            self.counts[idx] = 0
            cat = CATEGORIES[idx][0]
            self.flash_msg = f"{RED}🗑 cleared {count} images from {cat}{RESET}"
            self.flash_time = time.time()
        self._tui_wake(cat=True)
        logger.info(f"cleared {count} images from {CATEGORIES[idx][0]}")

    # ──────────────── TUI 唤醒信号 ────────────────
    def _tui_wake(self, cat=False, status=False, full=False):
        """通知 TUI 线程需要重绘。"""
        if full:
            self._tui_full = True
        if cat:
            self._tui_dirty_cat = True
        if status:
            self._tui_dirty_status = True
        self._tui_dirty.set()

    # ──────────────── TUI 后台渲染线程 ────────────────
    def _tui_render_loop(self):
        """独立线程：以固定频率渲染终端 UI，不阻塞主循环。"""
        sys.stdout.write(HIDE_CUR)
        sys.stdout.flush()
        try:
            while self._running:
                self._tui_dirty.wait(timeout=0.5)  # 最多等 500ms 兜底刷新
                self._tui_dirty.clear()

                if not self._running:
                    break

                with self._lock:
                    if self._tui_full:
                        self._build_full()
                        sys.stdout.write(CLR + "\n".join(self._tui_lines) + "\n")
                        sys.stdout.flush()
                        self._tui_full = False
                        self._tui_dirty_cat = False
                        self._tui_dirty_status = False
                    elif self._tui_dirty_cat:
                        self._build_cat_rows()
                        self._build_status()
                        self._flush_cat_tail()
                        self._tui_dirty_cat = False
                        self._tui_dirty_status = False
                    elif self._tui_dirty_status:
                        self._build_status()
                        self._flush_status_tail()
                        self._tui_dirty_status = False

                time.sleep(TUI_INTERVAL)
        finally:
            sys.stdout.write(SHOW_CUR)
            sys.stdout.flush()

    def _build_full(self):
        """构建完整 TUI 行缓冲。"""
        lines = []
        lines.append(f"{BOLD}{'═' * 58}{RESET}")
        lines.append(f"{BOLD}  Quick Collect  {CYAN}20类图像采集{RESET}")
        lines.append(f"{BOLD}{'═' * 58}{RESET}")
        lines.append(f"  {DIM}[1-0/q-p]选类  [空格]拍照  [退格]撤销  [c]清空  [ESC]退出{RESET}")
        lines.append(f"{DIM}{'─' * 58}{RESET}")
        self._cat_start = len(lines)
        self._build_cat_rows_data(lines)
        lines.append(f"{DIM}{'─' * 58}{RESET}")
        self._status_start = len(lines)
        self._build_status_data(lines)
        self._tui_lines = lines

    def _build_cat_rows(self):
        """原地更新类别行。"""
        lk = "1234567890"
        rk = "qwertyuiop"
        for row in range(10):
            li, ri = row, row + 10
            self._tui_lines[self._cat_start + row] = self._fmt_cat_row(
                li, ri, lk[row], rk[row])

    def _build_cat_rows_data(self, lines):
        """首次构建类别行。"""
        lk = "1234567890"
        rk = "qwertyuiop"
        for row in range(10):
            li, ri = row, row + 10
            lines.append(self._fmt_cat_row(li, ri, lk[row], rk[row]))

    def _fmt_cat_row(self, li, ri, lk, rk):
        ln, lc = CATEGORIES[li]
        rn, rc = CATEGORIES[ri]
        l_cnt = self.counts[li]
        r_cnt = self.counts[ri]
        sel = self.selected
        if li == sel:
            l = f"{REVERSE} {lk} {lc:8s}({ln:14s}) {l_cnt:>4d}img {RESET}"
        else:
            l = f" {lk} {lc:8s}({ln:14s}) {l_cnt:>4d}img "
        if ri == sel:
            r = f"{REVERSE} {rk} {rc:8s}({rn:14s}) {r_cnt:>4d}img {RESET}"
        else:
            r = f" {rk} {rc:8s}({rn:14s}) {r_cnt:>4d}img "
        return f"  {l}  │ {r}"

    def _build_status(self):
        """原地更新状态栏行。"""
        lines = self._tui_lines[:self._status_start]
        self._build_status_data(lines)
        self._tui_lines = lines

    def _build_status_data(self, lines):
        """追加状态栏行到 lines。"""
        cat_name, cat_cn = CATEGORIES[self.selected]
        count = self.counts[self.selected]
        lines.append(f"  {BOLD}{GREEN}>>> {self.selected+1:02d} {cat_cn} ({cat_name})  [{count} imgs]{RESET}")
        lines.append(f"  {DIM}counter: {self.counter:04d}  total: {sum(self.counts)} imgs{RESET}")
        if self.flash_msg and (time.time() - self.flash_time < 3):
            lines.append(f"\n  {self.flash_msg}")
        else:
            lines.append("")
        lines.append(f"{BOLD}{'═' * 58}{RESET}")

    def _flush_cat_tail(self):
        """光标移到类别行起始，覆写类别行 + 状态栏。"""
        tail = self._tui_lines[self._cat_start:]
        n = len(tail)
        sys.stdout.write(f"\033[{n}A" + "\n".join(tail) + "\n")
        sys.stdout.flush()

    def _flush_status_tail(self):
        """光标移到状态栏起始，仅覆写状态栏。"""
        tail = self._tui_lines[self._status_start:]
        n = len(tail)
        sys.stdout.write(f"\033[{n}A" + "\n".join(tail) + "\n")
        sys.stdout.flush()

    # ──────────────── 硬件屏幕 ────────────────
    def _hw_show(self):
        with self._lock:
            cat_name, cat_cn = CATEGORIES[self.selected]
            count = self.counts[self.selected]
            cnt = self.counter
        dis = f">{self.selected+1:02d} {cat_cn}\n{cat_name}\n{count}img c:{cnt}"
        try:
            self.display.show(dis)
        except Exception:
            pass

    # ──────────────── 硬件按键（独立线程轮询） ────────────────
    def _hw_poll_loop(self):
        """独立线程：轮询物理按键，不阻塞主循环。"""
        while self._running:
            try:
                btn = self.key_btn.get_key()
            except Exception:
                time.sleep(0.05)
                continue
            if btn == 0:
                time.sleep(0.02)
                continue
            if btn == 1:      # 上一类
                with self._lock:
                    self.selected = (self.selected - 1) % 20
                self._tui_wake(cat=True)
                self._hw_show()
            elif btn == 2:    # 下一类
                with self._lock:
                    self.selected = (self.selected + 1) % 20
                self._tui_wake(cat=True)
                self._hw_show()
            elif btn == 3:    # 拍照
                img = self.cap.read().copy()
                self._save_image(img, self.selected)
                self._hw_show()
            elif btn == 4:    # btn2 长按 → 清空当前类
                self._clear_category(self.selected)
                self._hw_show()
            elif btn == 5:    # btn1 长按 → 退出
                self._running = False
                self._tui_wake(full=True)
                break
            time.sleep(0.02)

    # ──────────────── 主循环（只管 cv2 显示 + 键盘） ────────────────
    def run(self):
        while self._running:
            img = self.cap.read()
            # 必须 copy：cap.read() 返回的是后台线程持续覆写的共享引用
            # 不 copy 的话 putText overlay 会污染保存的图片（水印）
            raw = img.copy()

            # overlay 只画在显示副本上
            with self._lock:
                sel = self.selected
                cnt = self.counter
                cat_name, cat_cn = CATEGORIES[sel]
                cat_count = self.counts[sel]

            info = f"[{sel+1:02d}] {cat_cn} ({cat_name})  {cat_count}img"
            cv2.putText(img, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img, f"counter:{cnt:04d}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
            cv2.imshow("Quick Collect", img)

            # waitKey(1) = 最小阻塞，最大化帧率
            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                break
            elif key in KEY_MAP:
                with self._lock:
                    self.selected = KEY_MAP[key]
                self._tui_wake(cat=True)
                self._hw_show()
            elif key == 32:  # Space → 拍照
                self._save_image(raw, sel)
                self._hw_show()
            elif key in BACKSPACE_KEYS:
                self._undo(sel)
                self._hw_show()
            elif key == ord("c"):  # c → 清空当前类
                self._clear_category(sel)
                self._hw_show()

            # 闪现消息过期 → 刷状态栏
            with self._lock:
                expired = self.flash_msg and (time.time() - self.flash_time >= 3)
                if expired:
                    self.flash_msg = ""
            if expired:
                self._tui_wake(status=True)

        self.close()

    def close(self):
        self._running = False
        self._tui_dirty.set()          # 唤醒 TUI 线程让它退出
        self._tui_thread.join(timeout=1)
        self._save_pool.shutdown(wait=False)
        self._save_counter()
        self.cap.close()
        cv2.destroyAllWindows()
        total = sum(self.counts)
        print(f"\n{GREEN}Done! Total: {total} images, counter: {self.counter:04d}{RESET}")
        for i, (name, cn) in enumerate(CATEGORIES):
            if self.counts[i] > 0:
                print(f"  {cn:8s} ({name:14s}): {self.counts[i]:>4d} imgs")


if __name__ == "__main__":
    QuickCollect(1)

#!/usr/bin/python3
# -*- coding: utf-8 -*-

import threading
import time

import cv2
import platform
import os


from .log_wrap import logger

# 连续 read 失败多少次后才真正 release fd（避免一遇错就关 fd 让内核 autosuspend）。
# 短时抖动（USB 噪声 / 一次坏帧）走"反复 read"路径；持续 ENODEV 才走"释放 + 重开"。
_CONSECUTIVE_READ_FAILURES_BEFORE_RELEASE = 8

# sysfs 里 power/control / autosuspend 的取值
_POWER_CONTROL_ON = "on"          # 永远不休眠
_POWER_CONTROL_AUTO = "auto"     # 内核默认（按 autosuspend timeout 决定）
_AUTOSUSPEND_NEVER = "-1"         # 单位秒；-1 = 禁用 autosuspend


def _find_usb_device_sysfs_path(video_dev_node):
    """
    给一个 /dev/videoN 节点（也接受 /dev/camN symlink），反推它对应的 USB 设备 sysfs 目录
    （如 /sys/devices/.../1-2.3）。

    链路：
      /sys/class/video4linux/videoN/device -> ../../../.../1-2.3:1.0  (USB 接口)
      /sys/class/video4linux/videoN/device/.. -> .../1-2.3                (USB 设备，含 power/)

    返回 USB 设备目录的绝对路径；找不到 / 不是 USB 设备 / 没权限时返回 None。
    """
    if not video_dev_node:
        return None
    try:
        # 先 realpath：/dev/camN 是 symlink → /dev/videoN，必须解到真节点。
        real_dev = os.path.realpath(video_dev_node)
        basename = os.path.basename(real_dev)
        v4l2_path = "/sys/class/video4linux/" + basename
        if not os.path.islink(v4l2_path):
            return None
        # video4linux/videoN/device 指向 USB 接口 (例 1-2.3:1.0)
        iface_link = os.path.join(v4l2_path, "device")
        iface_real = os.path.realpath(iface_link)
        # 上跳一级到 USB 设备目录（含 power/）
        usb_device = os.path.dirname(iface_real)
        if not os.path.isdir(usb_device):
            return None
        return usb_device
    except Exception:
        return None


def _try_write_sysfs(path, value):
    """尽量写一个 sysfs 文件；任何错误都吞掉（UAC/权限/不存在都常见）。"""
    try:
        with open(path, "w") as f:
            f.write(value)
        return True
    except Exception as e:
        logger.warning("写 sysfs {}={} 失败: {}".format(path, value, e))
        return False


def disable_usb_autosuspend_for(video_dev_node):
    """
    把指定 /dev/videoN 对应的 USB 设备的 power/control 设为 on，autosuspend 设为 -1。
    失败不抛异常（jetson 用户写 sysfs 会 PermissionError，udev 才是正经路子）。

    返回 True 表示至少 power/control 写成功；False 表示都失败（多半是 udev 没配）。
    """
    usb_device = _find_usb_device_sysfs_path(video_dev_node)
    if usb_device is None:
        logger.warning(
            "找不到 {} 对应的 USB sysfs 设备，跳过 autosuspend 禁用".format(video_dev_node)
        )
        return False
    control_path = os.path.join(usb_device, "power", "control")
    autosuspend_path = os.path.join(usb_device, "power", "autosuspend")
    wrote_control = _try_write_sysfs(control_path, _POWER_CONTROL_ON)
    _try_write_sysfs(autosuspend_path, _AUTOSUSPEND_NEVER)
    if wrote_control:
        logger.info(
            "已禁用 {} (USB 设备 {}) autosuspend".format(video_dev_node, usb_device)
        )
    return wrote_control


def read_usb_power_state(video_dev_node):
    """
    读取 /dev/videoN 对应的 USB 设备的 power/control 和 autosuspend，给 health 端用。
    读不到（无权限 / 不是 USB 设备）返回 (None, None)。
    """
    usb_device = _find_usb_device_sysfs_path(video_dev_node)
    if usb_device is None:
        return None, None
    state = {"control": None, "autosuspend": None}
    for key, filename in (("control", "control"), ("autosuspend", "autosuspend")):
        path = os.path.join(usb_device, "power", filename)
        try:
            with open(path, "r") as f:
                value = f.read().strip()
                state[key] = value or None
        except Exception:
            state[key] = None
    return state["control"], state["autosuspend"]


def _find_tty_usb_sysfs_path(tty_node):
    """
    给一个 /dev/ttyUSB* 或 /dev/ttyACM* 节点，反推它对应的 USB 设备 sysfs 目录
    （含 power/control）。

    链路：
      /sys/class/tty/ttyUSB0/device -> USB 接口
      .../1-2.2:1.0/tty/ttyUSB0    (interface)
      .../1-2.2/power/control       (USB device 才有 power/control)
    """
    if not tty_node:
        return None
    try:
        basename = os.path.basename(os.path.realpath(tty_node))
        tty_path = "/sys/class/tty/" + basename
        if not os.path.islink(tty_path):
            return None
        iface_link = os.path.join(tty_path, "device")
        start = os.path.realpath(iface_link)
        path = start
        for _ in range(12):
            if os.path.isdir(os.path.join(path, "power")) and os.path.exists(
                os.path.join(path, "power", "control")
            ):
                return path
            parent = os.path.dirname(path)
            if parent == path:
                return None
            path = parent
        return None
    except Exception:
        return None


def disable_usb_autosuspend_for_tty(tty_node):
    """
    跟 disable_usb_autosuspend_for 一样，但走 /sys/class/tty/ 路径。
    用于 serial_wrap._finish_connect_locked 给 ttyUSB 设备（CH340 MCU）写 power/control=on。
    解决"MCU 闲置 2s 内核 autosuspend → 下次串口操作返 device not ready → runtime 误判 controller 掉线"。
    """
    usb_device = _find_tty_usb_sysfs_path(tty_node)
    if usb_device is None:
        logger.debug("找不到 {} 对应的 USB sysfs 设备,跳过 autosuspend 禁用".format(tty_node))
        return False
    control_path = os.path.join(usb_device, "power", "control")
    autosuspend_path = os.path.join(usb_device, "power", "autosuspend")
    wrote_control = _try_write_sysfs(control_path, _POWER_CONTROL_ON)
    _try_write_sysfs(autosuspend_path, _AUTOSUSPEND_NEVER)
    if wrote_control:
        logger.info(
            "已禁用 {} (USB 设备 {}) autosuspend".format(tty_node, usb_device)
        )
    return wrote_control


def read_tty_usb_power_state(tty_node):
    """读 /dev/ttyUSB* 的 power/control 和 autosuspend。失败返 (None, None)。"""
    usb_device = _find_tty_usb_sysfs_path(tty_node)
    if usb_device is None:
        return None, None
    state = {"control": None, "autosuspend": None}
    for key, filename in (("control", "control"), ("autosuspend", "autosuspend")):
        path = os.path.join(usb_device, "power", filename)
        try:
            with open(path, "r") as f:
                value = f.read().strip()
                state[key] = value or None
        except Exception:
            state[key] = None
    return state["control"], state["autosuspend"]


class Camera:
    def __init__(self, index=1, width=640, height=480):
        # if src ==0:
        #     self.src = "/dev/video0"
        # elif src == 1:
        #     self.src = "/dev/video1"

        self.width = width
        self.height = height
        self.index = index

        # self.src =src
        self.src = None
        self.cap = None
        self.frame = None
        self.frame_updated_at = None
        self.opened_at = None
        self.last_open_attempt_at = None
        self.last_open_error = None
        self.reopen_requested = False
        # 连续 read 失败计数；到达阈值才真的 release fd。
        self._consecutive_read_failures = 0
        self._last_log_times = {}
        # 暂停标志
        self.pause_flag = False
        self.stop_flag = False

        self.init(max_wait=2.0)
        if self.cap is not None and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            # 兜底：启动时主动关 autosuspend（udev 不生效时的兜底）
            disable_usb_autosuspend_for(self.src)

        # thread是否运行标志
        self.flag_thread = False
        self.start_back_thread()
        # self.start()

    def _log_throttled(self, key, message, interval=3.0):
        now = time.time()
        last_at = self._last_log_times.get(key, 0.0)
        if now - last_at >= interval:
            logger.error(message)
            self._last_log_times[key] = now

    def _release_cap(self):
        if self.cap is None:
            return
        try:
            self.cap.release()
        except Exception:
            pass
        self.cap = None

    def _linux_source_candidates(self):
        primary = "/dev/cam" + str(self.index)
        # 当前双摄硬件的 video 节点是固定的，缺少 /dev/camN 软链接时允许回退。
        try:
            fallback_video_index = max((int(self.index) - 1) * 2, 0)
        except Exception:
            fallback_video_index = 0
        fallback = "/dev/video" + str(fallback_video_index)
        candidates = [primary, fallback]
        unique = []
        for candidate in candidates:
            if candidate not in unique:
                unique.append(candidate)
        return unique

    def _try_open_once(self):
        self.last_open_attempt_at = time.time()
        try:
            if 'Windows' in platform.platform():
                cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
                if cap is not None and cap.isOpened():
                    self.src = self.index
                    self._release_cap()
                    self.cap = cap
                    self.opened_at = time.time()
                    self.last_open_error = None
                    self.set_size(self.width, self.height)
                    return True
                if cap is not None:
                    cap.release()
                self.last_open_error = "摄像头{}打开失败".format(self.index)
                self._log_throttled("open-failed", self.last_open_error)
                return False

            missing_sources = []
            for candidate in self._linux_source_candidates():
                if not os.path.exists(candidate):
                    missing_sources.append(candidate)
                    continue
                cap = cv2.VideoCapture(candidate)
                if cap is not None and cap.isOpened():
                    self.src = candidate
                    self._release_cap()
                    self.cap = cap
                    self.opened_at = time.time()
                    self.last_open_error = None
                    self.set_size(self.width, self.height)
                    return True
                if cap is not None:
                    cap.release()
            if missing_sources:
                self.last_open_error = "摄像头候选设备不存在: {}".format(", ".join(missing_sources))
            else:
                self.last_open_error = "摄像头{}打开失败".format(self.index)
            self._log_throttled("open-failed", self.last_open_error)
            return False
        except Exception as e:
            self.last_open_error = "init:摄像头打开错误! {}".format(e)
            self._log_throttled("open-exception", self.last_open_error)
            self._release_cap()
            return False

    def init(self, max_wait=None):
        deadline = None if max_wait is None else time.time() + float(max_wait)
        while not self.stop_flag:
            if self._try_open_once():
                return True
            if deadline is not None and time.time() >= deadline:
                return False
            time.sleep(0.2)
        return False

    def request_reopen(self, reason="manual"):
        self.reopen_requested = True
        self.frame = None
        self.frame_updated_at = None
        self._release_cap()
        self._log_throttled("request-reopen", "摄像头{}触发重连: {}".format(self.index, reason), interval=2.0)
    
    def start_back_thread(self):
        # 如果未开启线程，开启线程
        if not self.flag_thread:
            self.cap_thread = threading.Thread(target=self.update, args=())
            self.cap_thread.daemon = True
            self.cap_thread.start()
            self.flag_thread = True
        time.sleep(0.5)
            
    def update(self):
        while True:
            if self.stop_flag:
                break
            if self.pause_flag:
                time.sleep(0.05)
                continue
            try:
                if self.reopen_requested or self.cap is None or (not self.cap.isOpened()):
                    self.reopen_requested = False
                    if not self.init(max_wait=1.0):
                        time.sleep(0.2)
                        continue
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.frame = frame
                    self.frame_updated_at = time.time()
                    self._consecutive_read_failures = 0
                    continue
                # ---- read 失败：不要立刻 release fd ----
                # 老逻辑：失败一次就 _release_cap()，内核看到 fd 关了 → 2 秒后 autosuspend
                #          → 下次 reopen 时 VIDIOC_REQBUFS 失败 → 死循环
                # 新逻辑：先累计 _consecutive_read_failures，连续 N 次失败才真的 release。
                #        这段时间里 fd 还开着，内核不会 autosuspend；偶发抖动也能被自愈。
                self._consecutive_read_failures += 1
                if self._consecutive_read_failures >= _CONSECUTIVE_READ_FAILURES_BEFORE_RELEASE:
                    self._log_throttled(
                        "read-failed",
                        "read:连续{}次读取失败，触发重连".format(
                            self._consecutive_read_failures
                        ),
                    )
                    self.request_reopen("read failed (consecutive)")
                    self._consecutive_read_failures = 0
                else:
                    self._log_throttled(
                        "read-blip",
                        "read:第{}次读取失败（保留 fd，避免触发 autosuspend）".format(
                            self._consecutive_read_failures
                        ),
                        interval=1.0,
                    )
                    # 新 fd 也尝试主动关 autosuspend（首次重开场景兜底）
                    if self.cap is None:
                        disable_usb_autosuspend_for(self.src)
                time.sleep(0.2)
            except Exception as e:
                self._consecutive_read_failures += 1
                self._log_throttled(
                    "read-exception",
                    "exception:摄像头错误!! {} (累计{})".format(
                        e, self._consecutive_read_failures
                    ),
                )
                if self._consecutive_read_failures >= _CONSECUTIVE_READ_FAILURES_BEFORE_RELEASE:
                    self.request_reopen("exception (consecutive)")
                    self._consecutive_read_failures = 0
                time.sleep(0.2)

    def set_size(self, width, height):
        self.width = width
        self.height = height
        if self.cap is None:
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def read(self):
        while self.frame is None:
            if self.stop_flag:
                raise RuntimeError("摄像头已关闭")
            time.sleep(0.1)
        return self.frame

    def close(self):
        self.stop_flag = True
        # 等待进程结束
        if hasattr(self, "cap_thread"):
            self.cap_thread.join(timeout=2)
        logger.info("{} close".format(self.src))
        self._release_cap()


def main():
    camera = Camera(1, 640, 480)
    # logger.info("camera test")
    # start_time = time.time()
    while True:
        try:
            img = camera.read()
            # print(img.shape)
            cv2.imshow("img", img)
            key = cv2.waitKey(1)
            # cost_time = time.time() - start_time
            # start_time = time.time()
            # print("fps:", 1 / cost_time)
            if key == ord('q'):
                time.sleep(0.1)
                break
        except Exception as e:
            logger.error(e)
    camera.close()
    logger.info("over")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

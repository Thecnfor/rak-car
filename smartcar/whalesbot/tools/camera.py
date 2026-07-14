#!/usr/bin/python3
# -*- coding: utf-8 -*-

import threading
import time

import cv2
import platform
import os


from .log_wrap import logger

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
        self._last_log_times = {}
        # 暂停标志
        self.pause_flag = False
        self.stop_flag = False

        self.init(max_wait=2.0)
        if self.cap is not None and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

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
                else:
                    self._log_throttled("read-failed", "read:读取图像错误!!!!")
                    self.request_reopen("read failed")
                    time.sleep(0.2)
            except Exception as e:
                self._log_throttled("read-exception", "exception:摄像头错误!! {}".format(e))
                self.request_reopen("exception")
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

# -*- coding: utf-8 -*-
"""Camera — minimal cv2-backed frame source.

Self-contained: only depends on OpenCV + numpy. Does NOT import any of the
rest of rak-car (vehicle, car_wrap, task_func, ...). This is deliberate so
`python3 -m tools.streamer` does not trigger the heavy hardware import chain
described in /home/jetson/.claude/projects/-home-jetson-workspace/memory/tools-streamer-run-gotchas.md.

Operating modes:
  * real      — opens /dev/cam<index> via V4L2 (the default on the car).
                If the device is missing or can't be opened, the camera
                reports `_no_signal=True` and every read returns a static
                "NO SIGNAL" placeholder. No synthetic content is ever
                generated — by design, "no fake data".

  * mock      — only enabled by env RAK_CAMERA_FORCE_MOCK=1. Synthesizes a
                dark frame with a moving timestamp overlay so dev machines
                without cameras can still see the UI working. The frame is
                watermarked "MOCK" so the operator can never mistake it for
                real footage. Never enabled by default.

Index convention (matches the existing rak-car / baidu_smartcar_2026 layout):
  side  = 1   → /dev/cam1
  front = 2   → /dev/cam2
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Optional

import cv2
import numpy as np


_logger = logging.getLogger(__name__)


class Camera:
    """Threaded capture. read() returns the latest BGR ndarray (or a "no
    signal" placeholder if the device is missing). Mirrors the original
    streamer.py's Camera shape (index, width, height)."""

    def __init__(self, index: int = 1, width: int = 640, height: int = 480) -> None:
        self.index = index
        self.width = width
        self.height = height
        self._cap: Optional[cv2.VideoCapture] = None
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._stop_flag = False
        self._thread: Optional[threading.Thread] = None
        # Opt-in only — default off. Mock mode is for dev machines without
        # cameras. Production always sees the real signal (or NO SIGNAL).
        self._mock = bool(int(os.environ.get("RAK_CAMERA_FORCE_MOCK", "0")))
        # Set when the real device was missing/unopenable. read() serves a
        # "NO SIGNAL" placeholder until the device shows up.
        self._no_signal: bool = False

    # -- public API -----------------------------------------------------

    def init(self) -> None:
        """Open the device. Idempotent."""
        if self._cap is not None or self._frame is not None or self._no_signal:
            return
        if self._mock:
            self._seed_mock()
            return
        cap = cv2.VideoCapture(self._device_path())
        if not cap.isOpened():
            cap.release()
            _logger.warning(
                "Camera(%d): %s not available — no signal will be served",
                self.index,
                self._device_path(),
            )
            self._no_signal = True
            self._frame = self._render_no_signal()
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap = cap
        self._start_back_thread()

    def start_back_thread(self) -> None:
        self.init()

    def read(self) -> Optional[np.ndarray]:
        """Returns the latest frame, or a static NO SIGNAL frame if the
        device is missing. Always returns a frame once init() has run;
        never None unless init() is still in progress."""
        with self._lock:
            if self._frame is not None:
                return self._frame.copy()
        # Frame not ready yet — wait briefly.
        deadline = time.time() + 1.0
        while True:
            with self._lock:
                if self._frame is not None:
                    return self._frame.copy()
            if time.time() > deadline:
                # init() never produced a frame (e.g. no_signal path already
                # produced one, but it's locked behind init being a no-op).
                return None
            time.sleep(0.02)

    def close(self) -> None:
        self._stop_flag = True
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:  # noqa: BLE001
                pass
            self._cap = None
        self._frame = None
        self._thread = None

    @property
    def is_mock(self) -> bool:
        return self._mock

    @property
    def is_no_signal(self) -> bool:
        return self._no_signal

    # -- helpers --------------------------------------------------------

    def _device_path(self) -> str:
        # rak-car convention: /dev/camN. Falls back to /dev/videoN on boxes
        # where the symlink doesn't exist (e.g. dev laptops).
        candidate = f"/dev/cam{self.index}"
        if os.path.exists(candidate):
            return candidate
        return f"/dev/video{max(0, self.index - 1)}"

    def _seed_mock(self) -> None:
        self._frame = self._render_mock(time.time())
        self._start_back_thread()

    def _render_no_signal(self) -> np.ndarray:
        """Plain black frame with a single 'NO SIGNAL' label. No
        synthetic content beyond the label itself — by design."""
        h, w = self.height, self.width
        img = np.zeros((h, w, 3), dtype=np.uint8)
        # center
        msg1 = "NO SIGNAL"
        msg2 = f"/dev/cam{self.index} unavailable"
        cv2.putText(img, msg1, (int(w * 0.30), int(h * 0.48)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (90, 90, 110), 2, cv2.LINE_AA)
        cv2.putText(img, msg2, (int(w * 0.28), int(h * 0.58)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (110, 110, 130), 1, cv2.LINE_AA)
        return img

    def _render_mock(self, now: float) -> np.ndarray:
        # Watermarked MOCK frame — only used when RAK_CAMERA_FORCE_MOCK=1.
        h, w = self.height, self.width
        img = np.zeros((h, w, 3), dtype=np.uint8)
        ys = np.linspace(0, 1, h, dtype=np.float32).reshape(h, 1)
        img[..., 0] = (24 + 12 * ys * 200).astype(np.uint8)
        img[..., 1] = (18 + 6 * ys * 200).astype(np.uint8)
        img[..., 2] = (10 + 4 * ys * 200).astype(np.uint8)
        cv2.putText(
            img, "MOCK", (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (84, 180, 255), 2, cv2.LINE_AA,
        )
        cv2.putText(
            img, f"cam{self.index} t={now:.1f}s", (12, 64),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 200, 220), 1, cv2.LINE_AA,
        )
        cv2.putText(
            img, "RAK_CAMERA_FORCE_MOCK=1", (12, h - 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 200, 220), 1, cv2.LINE_AA,
        )
        return img

    def _start_back_thread(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_flag = False
        self._thread = threading.Thread(target=self._update, daemon=True, name=f"cam{self.index}")
        self._thread.start()

    def _update(self) -> None:
        # Real capture loop
        if not self._mock and self._cap is not None and not self._no_signal:
            while not self._stop_flag:
                ok, frame = self._cap.read()
                if not ok or frame is None:
                    time.sleep(0.02)
                    continue
                with self._lock:
                    self._frame = frame
            return
        # Mock loop
        if self._mock:
            while not self._stop_flag:
                with self._lock:
                    self._frame = self._render_mock(time.time())
                time.sleep(0.05)
            return
        # no_signal — single static frame, no thread needed

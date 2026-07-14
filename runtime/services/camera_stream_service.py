#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import threading
import time
from pathlib import Path

try:
    import cv2
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError("缺少 opencv-python 依赖，无法输出摄像头视频流") from exc

try:
    import numpy as np
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError("缺少 numpy 依赖，无法生成视频占位画面") from exc


class CameraStreamService:
    """
    runtime 内置双摄像头流服务。

    从当前 MyCar 实例读取前视和侧视画面，缓存最新帧，并通过 FastAPI
    路由输出 MJPEG 和监控页面。
    """

    CAM_ALIASES = {
        "cam1": "cam1",
        "front": "cam1",
        "cam2": "cam2",
        "side": "cam2",
    }

    CAPTURE_DIRS = {
        "cam1": "dataset/image_set_lane/runtime_capture",
        "cam2": "dataset/image_set_object/runtime_capture",
    }

    def __init__(self, runtime_service, fps=20, quality=80):
        self.runtime_service = runtime_service
        self.fps = max(float(fps), 1.0)
        self.quality = int(quality)
        self.capture_interval = 1.0 / self.fps
        self.stale_timeout = max(self.capture_interval * 8, 2.0)
        self.recover_cooldown = 3.0
        self.project_root = Path(__file__).resolve().parents[2]
        self.running = False
        self.last_key = None
        self._thread = None
        self.frame_lock = threading.Lock()
        self.key_lock = threading.Lock()
        self.meta_lock = threading.Lock()
        self.frames = {}
        self.frame_meta = {}
        self.lane_state = self._default_lane_state()

    def start(self):
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        with self.frame_lock:
            self.frames.clear()
            self.frame_meta.clear()
        with self.meta_lock:
            self.lane_state = self._default_lane_state()

    def get_key(self, clear=True):
        with self.key_lock:
            key = self.last_key
            if clear:
                self.last_key = None
            return key

    def set_key(self, key):
        with self.key_lock:
            self.last_key = key
        return key

    def clear_frame(self, cam_id=None, preserve_meta=False):
        with self.frame_lock:
            if cam_id is None:
                self.frames.clear()
                if not preserve_meta:
                    self.frame_meta.clear()
                return
            normalized = self.normalize_cam_id(cam_id)
            if normalized in self.frames:
                del self.frames[normalized]
            if (not preserve_meta) and normalized in self.frame_meta:
                del self.frame_meta[normalized]

    def get_status(self):
        with self.frame_lock:
            active_cams = sorted(self.frames.keys())
            cameras = {}
            for cam_id in ("cam1", "cam2"):
                meta = dict(self.frame_meta.get(cam_id, {}))
                source_updated_at = meta.get("source_updated_at")
                frame_age = None
                stale = True
                if source_updated_at:
                    frame_age = round(max(time.time() - float(source_updated_at), 0.0), 3)
                    stale = frame_age > self.stale_timeout
                # USB autosuspend 状态：read 一下对应 USB 设备的 power/control。
                # 用于 /stream/health 一眼看出"摄像头为啥又黑屏了"。
                usb_power = self._read_usb_power(cam_id)
                cameras[cam_id] = {
                    "active": cam_id in self.frames and not stale,
                    "stale": stale,
                    "frame_age": frame_age,
                    "last_capture_at": meta.get("captured_at"),
                    "last_source_update_at": source_updated_at,
                    "last_recover_at": meta.get("last_recover_at"),
                    "last_issue": meta.get("last_issue"),
                    "usb_power_control": usb_power["control"],
                    "usb_autosuspend": usb_power["autosuspend"],
                    "usb_power_warn": usb_power["warn"],
                }
        return {
            "status": "running" if self.running else "stopped",
            "active_cams": active_cams,
            "fps": self.fps,
            "quality": self.quality,
            "stale_timeout": self.stale_timeout,
            "cameras": cameras,
        }

    @staticmethod
    def _usb_dev_for_cam(cam_id):
        """把 cam1/cam2 翻译成 /dev/videoN（和 smartcar Camera 类 _linux_source_candidates 一致）。"""
        from pathlib import Path
        try:
            cam_index = {"cam1": 1, "cam2": 2}.get(cam_id)
            if cam_index is None:
                return None
            primary = Path("/dev/cam")  # type: ignore
            primary_full = Path(str(primary) + str(cam_index))
            if primary_full.exists():
                return str(primary_full)
            fallback_idx = max((cam_index - 1) * 2, 0)
            fallback = "/dev/video" + str(fallback_idx)
            if os.path.exists(fallback):
                return fallback
        except Exception:
            return None
        return None

    @classmethod
    def _read_usb_power(cls, cam_id):
        """读 /dev/videoN 对应 USB 设备的 power/control 和 autosuspend；读不到返回 None。
        给 health 端用，明确告诉用户是不是 autosuspend 在搞事。"""
        try:
            from smartcar.whalesbot.tools.camera import read_usb_power_state
        except Exception:
            return {"control": None, "autosuspend": None, "warn": "camera 模块加载失败"}
        dev_node = cls._usb_dev_for_cam(cam_id)
        if dev_node is None:
            return {"control": None, "autosuspend": None, "warn": "设备节点不存在"}
        control, autosuspend = read_usb_power_state(dev_node)
        warn = None
        if control is None:
            warn = "无法读取 sysfs（多半 udev 没装/没权限）"
        elif control != "on":
            # auto / suspend 都是危险状态；默认内核是 auto + autosuspend=2s = 随时挂
            warn = "power/control={}（建议 udev 规则禁用 autosuspend）".format(control)
        return {"control": control, "autosuspend": autosuspend, "warn": warn}

    def get_stream_info(self, base_url):
        base = str(base_url).rstrip("/")
        return {
            "ok": True,
            "page_url": f"{base}/stream/",
            "health_url": f"{base}/stream/health",
            "lane_state_url": f"{base}/v1/vision/lane/state",
            "keypress_url": f"{base}/keypress",
            "capture_url": f"{base}/stream/capture",
            "cameras": {
                "cam1": self._camera_info(base, "cam1", ["cam1", "front"]),
                "cam2": self._camera_info(base, "cam2", ["cam2", "side"]),
            },
        }

    def set_lane_state(self, **updates):
        with self.meta_lock:
            state = dict(self.lane_state)
            for key, value in updates.items():
                state[key] = value
            state["updated_at"] = time.time()
            self.lane_state = state
            return dict(state)

    def clear_lane_state(self):
        with self.meta_lock:
            self.lane_state = self._default_lane_state()
            return dict(self.lane_state)

    def get_lane_state(self):
        with self.meta_lock:
            state = dict(self.lane_state)
        state["frame_url"] = "/stream/frame/cam1.jpg"
        state["preview_url"] = "/stream/"
        return state

    def normalize_cam_id(self, cam_id):
        return self.CAM_ALIASES.get(str(cam_id).lower(), "cam1")

    def get_frame(self, cam_id):
        normalized = self.normalize_cam_id(cam_id)
        with self.frame_lock:
            frame = self.frames.get(normalized)
            return None if frame is None else frame.copy()

    def _frame_is_fresh(self, cam_id):
        normalized = self.normalize_cam_id(cam_id)
        with self.frame_lock:
            meta = dict(self.frame_meta.get(normalized, {}))
        source_updated_at = meta.get("source_updated_at")
        if not source_updated_at:
            return False
        return (time.time() - float(source_updated_at)) <= self.stale_timeout

    def get_frame_or_placeholder(self, cam_id):
        normalized = self.normalize_cam_id(cam_id)
        frame = self.get_frame(normalized)
        if frame is not None and self._frame_is_fresh(normalized) and self._is_valid_image(frame):
            return frame
        if frame is not None:
            self.clear_frame(normalized, preserve_meta=True)
        return self._build_waiting_frame(normalized)

    def _update_frame_meta(self, cam_id, **updates):
        normalized = self.normalize_cam_id(cam_id)
        with self.frame_lock:
            meta = dict(self.frame_meta.get(normalized, {}))
            meta.update(updates)
            self.frame_meta[normalized] = meta

    def _request_camera_recover(self, camera, cam_id, reason):
        normalized = self.normalize_cam_id(cam_id)
        with self.frame_lock:
            meta = dict(self.frame_meta.get(normalized, {}))
        now = time.time()
        last_recover_at = meta.get("last_recover_at") or 0.0
        self._update_frame_meta(
            normalized,
            last_issue=reason,
            last_issue_at=now,
        )
        if now - float(last_recover_at) < self.recover_cooldown:
            return
        if camera is not None and hasattr(camera, "request_reopen"):
            try:
                camera.request_reopen(reason)
                self._update_frame_meta(normalized, last_recover_at=now)
            except Exception:
                pass

    def _get_camera_source_updated_at(self, camera):
        updated_at = getattr(camera, "frame_updated_at", None)
        if updated_at:
            return float(updated_at)
        return None

    def update_frame(self, image, cam_id, source_updated_at=None):
        if image is None:
            return
        if not self._is_valid_image(image):
            self._update_frame_meta(
                self.normalize_cam_id(cam_id),
                last_issue="empty image",
                last_issue_at=time.time(),
            )
            return
        normalized = self.normalize_cam_id(cam_id)
        captured_at = time.time()
        if source_updated_at is None:
            source_updated_at = captured_at
        with self.frame_lock:
            self.frames[normalized] = image.copy()
            meta = dict(self.frame_meta.get(normalized, {}))
            meta.update(
                {
                    "captured_at": captured_at,
                    "source_updated_at": float(source_updated_at),
                    "last_issue": None,
                }
            )
            self.frame_meta[normalized] = meta

    @staticmethod
    def _is_valid_image(image):
        if not hasattr(image, "size") or not hasattr(image, "shape"):
            return False
        try:
            if int(image.size) <= 0 or len(image.shape) < 2:
                return False
        except Exception:
            return False
        return True

    def stream_frames(self, cam_id):
        normalized = self.normalize_cam_id(cam_id)
        last_frame_time = 0.0
        while self.running:
            current_time = time.time()
            if current_time - last_frame_time < self.capture_interval:
                time.sleep(0.01)
                continue
            frame = self.get_frame_or_placeholder(normalized)
            try:
                ret, buffer = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.quality],
                )
            except cv2.error:
                self.clear_frame(normalized, preserve_meta=True)
                time.sleep(0.05)
                continue
            if ret:
                last_frame_time = current_time
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + buffer.tobytes()
                    + b"\r\n"
                )
            time.sleep(0.01)

    def encode_jpeg_bytes(self, cam_id, quality=None):
        frame = self.get_frame_or_placeholder(cam_id)
        quality = self.quality if quality is None else int(quality)
        try:
            ret, buffer = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), quality],
            )
        except cv2.error:
            self.clear_frame(self.normalize_cam_id(cam_id), preserve_meta=True)
            placeholder = self._build_waiting_frame(self.normalize_cam_id(cam_id))
            ret, buffer = cv2.imencode(
                ".jpg",
                placeholder,
                [int(cv2.IMWRITE_JPEG_QUALITY), quality],
            )
        if not ret:
            raise RuntimeError("JPEG 编码失败")
        return buffer.tobytes()

    def save_capture(self, cam_id, prefix="capture", subdir=None):
        normalized = self.normalize_cam_id(cam_id)
        frame = self.get_frame(normalized)
        if frame is None or not self._frame_is_fresh(normalized):
            raise RuntimeError("当前摄像头还没有可保存的实时画面")
        file_prefix = self._safe_slug(prefix or "capture")
        target_dir = self._get_capture_dir(normalized, subdir=subdir)
        target_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        millis = int((time.time() % 1) * 1000)
        filename = f"{file_prefix}_{normalized}_{timestamp}_{millis:03d}.jpg"
        file_path = target_dir / filename
        ok = cv2.imwrite(str(file_path), frame)
        if not ok:
            raise RuntimeError("截图保存失败")
        relative_path = file_path.relative_to(self.project_root)
        return {
            "cam_id": normalized,
            "filename": filename,
            "file_path": str(file_path),
            "relative_path": str(relative_path),
            "download_name": filename,
            "shape": list(frame.shape),
            "saved_at": time.time(),
            "subdir": str(target_dir.relative_to(self.project_root)),
        }

    def get_saved_capture_path(self, cam_id, filename, subdir=None):
        normalized = self.normalize_cam_id(cam_id)
        safe_name = os.path.basename(str(filename))
        if not safe_name:
            raise FileNotFoundError("缺少文件名")
        file_path = self._get_capture_dir(normalized, subdir=subdir) / safe_name
        if not file_path.exists():
            raise FileNotFoundError(safe_name)
        return file_path

    def render_page(self):
        return """
        <html lang="zh-CN">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>智能车监控系统</title>
                <style>
                    * { margin: 0; padding: 0; box-sizing: border-box; }
                    body {
                        font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
                        background: #0f1419;
                        min-height: 100vh;
                        color: #e0e0e0;
                    }
                    /* ---- 极简状态栏 ---- */
                    .statusbar {
                        position: sticky;
                        top: 0;
                        z-index: 10;
                        display: flex;
                        align-items: center;
                        gap: 18px;
                        padding: 8px 16px;
                        background: rgba(20, 25, 35, 0.92);
                        border-bottom: 1px solid rgba(255,255,255,0.08);
                        font-size: 13px;
                        font-family: 'SF Mono', Consolas, monospace;
                        backdrop-filter: blur(8px);
                    }
                    .statusbar .brand {
                        font-weight: 600;
                        background: linear-gradient(90deg, #00d4ff, #7b2cbf);
                        -webkit-background-clip: text;
                        -webkit-text-fill-color: transparent;
                    }
                    .statusbar .lane-dot {
                        display: inline-block;
                        width: 9px;
                        height: 9px;
                        border-radius: 50%;
                        background: #555;
                        margin-right: 6px;
                        vertical-align: middle;
                        transition: background 0.2s;
                    }
                    .statusbar .lane-dot.ok { background: #00e676; box-shadow: 0 0 8px #00e67688; }
                    .statusbar .lane-dot.warn { background: #ffb300; }
                    .statusbar .lane-dot.err { background: #ff5252; }
                    .statusbar .err { color: #8892b0; }
                    .statusbar .err b { color: #e6f1ff; font-weight: 500; }
                    .statusbar .key {
                        margin-left: auto;
                        color: #ff5252;
                        font-weight: 600;
                        min-width: 24px;
                        text-align: right;
                        transition: transform 0.15s;
                    }
                    .statusbar .key.active { transform: scale(1.4); }

                    /* ---- 原始流（无任何 overlay / 状态卡）---- */
                    .streams {
                        display: flex;
                        gap: 2px;
                        padding: 2px;
                    }
                    .stream-cell {
                        flex: 1;
                        min-width: 0;
                        background: #000;
                        position: relative;
                    }
                    .stream-cell img {
                        display: block;
                        width: 100%;
                        height: auto;
                    }
                    .stream-cell .label {
                        position: absolute;
                        top: 6px;
                        left: 8px;
                        font-size: 11px;
                        color: #fff;
                        background: rgba(0,0,0,0.55);
                        padding: 2px 6px;
                        border-radius: 4px;
                        font-family: monospace;
                    }
                </style>
            </head>
            <body>
                <div class="statusbar">
                    <span class="brand">RAK-CAR</span>
                    <span>
                        <span class="lane-dot" id="laneDot"></span>
                        <span id="laneText">lane: --</span>
                    </span>
                    <span class="err">
                        ey=<b id="laneEy">--</b>
                        ea=<b id="laneEa">--</b>
                    </span>
                    <span id="lastKey" class="key"></span>
                </div>
                <div class="streams">
                    <div class="stream-cell">
                        <span class="label">cam1 / front</span>
                        <span class="label" style="left:auto; right:8px; background: rgba(0,80,140,0.55);">
                            <label style="cursor:pointer; user-select:none;">
                                <input type="checkbox" id="laneOverlay" style="vertical-align:middle; margin-right:3px;">
                                lane overlay
                            </label>
                        </span>
                        <img id="cam1Img" src="/video_feed/cam1" alt="front camera">
                    </div>
                    <div class="stream-cell">
                        <span class="label">cam2 / side</span>
                        <img src="/video_feed/cam2" alt="side camera">
                    </div>
                </div>
                <script>
                    // 状态栏 lane 状态轮询（1Hz，轻量；MJPEG 自带 20Hz）
                    const laneDot = document.getElementById('laneDot');
                    const laneText = document.getElementById('laneText');
                    const laneEy = document.getElementById('laneEy');
                    const laneEa = document.getElementById('laneEa');
                    const lastKey = document.getElementById('lastKey');
                    let lastKeyText = '';
                    let keyTimer = null;

                    function fmt(v) {
                        if (v === null || v === undefined) return '--';
                        if (typeof v !== 'number') return String(v);
                        return v.toFixed(4);
                    }

                    async function pollLane() {
                        try {
                            const r = await fetch('/v1/vision/lane/state');
                            const d = await r.json();
                            // 点颜色
                            const age = d.updated_at ? (Date.now()/1000 - d.updated_at) : 999;
                            if (!d.updated_at || age > 2) {
                                laneDot.className = 'lane-dot err';
                            } else if (d.active) {
                                laneDot.className = 'lane-dot ok';
                            } else {
                                laneDot.className = 'lane-dot warn';
                            }
                            laneText.textContent = 'lane: ' + (d.active ? (d.mode || 'on') : 'idle');
                            laneEy.textContent = fmt(d.error_y);
                            laneEa.textContent = fmt(d.error_angle);
                        } catch (e) {
                            laneDot.className = 'lane-dot err';
                            laneText.textContent = 'lane: err';
                        }
                    }
                    pollLane();
                    setInterval(pollLane, 1000);  // 1Hz 足够（MJPEG 已经 20Hz）

                    // ---- cam1 车道 overlay 切换 ----
                    // 默认走 MJPEG /video_feed/cam1（干净流）。
                    // 勾上 checkbox 时切到 /v1/vision/lane/preview.jpg?cam_id=cam1，
                    // 该端点会读 streamer 缓存 + 画车道误差字，单帧 JPEG，
                    // 因此需要 JS 周期性 reload（默认 10Hz）。
                    const cam1Img = document.getElementById('cam1Img');
                    const laneOverlay = document.getElementById('laneOverlay');
                    let overlayTimer = null;
                    function refreshOverlay() {
                        // cache-buster 时间戳让浏览器拉新帧
                        cam1Img.src = '/v1/vision/lane/preview.jpg?cam_id=cam1&t=' + Date.now();
                    }
                    laneOverlay.addEventListener('change', () => {
                        if (laneOverlay.checked) {
                            refreshOverlay();
                            overlayTimer = setInterval(refreshOverlay, 100);  // 10Hz
                        } else {
                            if (overlayTimer) { clearInterval(overlayTimer); overlayTimer = null; }
                            cam1Img.src = '/video_feed/cam1';
                        }
                    });

                    // 键盘按键转发（保留）
                    let pageActive = false;
                    document.body.addEventListener('click', () => { pageActive = true; });
                    document.addEventListener('keydown', (ev) => {
                        if (!pageActive) return;
                        fetch('/keypress', {
                            method: 'POST',
                            headers: { 'Content-Type': 'application/json' },
                            body: JSON.stringify({ key: ev.key })
                        })
                        .then(r => r.json())
                        .then(d => {
                            if (lastKeyText) lastKey.textContent = '';
                            lastKeyText = d.received || '';
                            lastKey.textContent = lastKeyText;
                            lastKey.classList.remove('active');
                            void lastKey.offsetWidth;
                            lastKey.classList.add('active');
                            clearTimeout(keyTimer);
                            keyTimer = setTimeout(() => { lastKey.textContent = ''; lastKeyText = ''; }, 1500);
                        })
                        .catch(err => console.error('keypress err:', err));
                        if (['F5', 'F12'].includes(ev.key)) ev.preventDefault();
                    });
                </script>
            </body>
        </html>
        """

    def _capture_loop(self):
        while self.running:
            car = self.runtime_service.get_car_for_stream()
            cam1 = self.runtime_service.get_stream_camera("cam1")
            cam2 = self.runtime_service.get_stream_camera("cam2")
            if cam1 is None and car is not None:
                cam1 = getattr(car, "cap_front", None)
            if cam2 is None and car is not None:
                cam2 = getattr(car, "cap_side", None)
            if cam1 is None and cam2 is None and car is None:
                self.clear_frame()
                time.sleep(0.2)
                continue
            self._capture_from_camera(cam1, "cam1")
            self._capture_from_camera(cam2, "cam2")
            time.sleep(self.capture_interval)

    def _capture_from_camera(self, camera, cam_id):
        if camera is None:
            self.clear_frame(cam_id, preserve_meta=True)
            self._update_frame_meta(cam_id, last_issue="camera object missing", last_issue_at=time.time())
            return
        frame = getattr(camera, "frame", None)
        source_updated_at = self._get_camera_source_updated_at(camera)
        if frame is None or source_updated_at is None:
            self._request_camera_recover(camera, cam_id, "camera frame missing")
            self.clear_frame(cam_id, preserve_meta=True)
            return
        frame_age = time.time() - source_updated_at
        if frame_age > self.stale_timeout:
            self._request_camera_recover(camera, cam_id, "camera frame stale {:.2f}s".format(frame_age))
            self.clear_frame(cam_id, preserve_meta=True)
            return
        self.update_frame(frame, cam_id, source_updated_at=source_updated_at)

    def _build_waiting_frame(self, cam_id):
        title = {
            "cam1": "Waiting for cam1 / front...",
            "cam2": "Waiting for cam2 / side...",
        }.get(cam_id, "Waiting for camera...")
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            blank,
            title,
            (90, 240),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        return blank

    def _camera_info(self, base, cam_id, aliases):
        capture_dir = str(self._get_capture_dir(cam_id).relative_to(self.project_root))
        return {
            "aliases": aliases,
            "mjpeg_url": f"{base}/video_feed/{cam_id}",
            "frame_url": f"{base}/stream/frame/{cam_id}.jpg",
            "capture_url": f"{base}/stream/capture",
            "capture_download_url": f"{base}/stream/capture/{cam_id}/download",
            "default_capture_dir": capture_dir,
        }

    def _get_capture_dir(self, cam_id, subdir=None):
        normalized = self.normalize_cam_id(cam_id)
        base_dir = self.project_root / self.CAPTURE_DIRS[normalized]
        extra_dir = self._safe_slug(subdir) if subdir else ""
        if extra_dir:
            return base_dir / extra_dir
        return base_dir

    def _safe_slug(self, value):
        text = str(value or "").strip()
        if not text:
            return ""
        allowed = []
        for char in text:
            if char.isalnum() or char in {"-", "_"}:
                allowed.append(char)
            else:
                allowed.append("_")
        return "".join(allowed).strip("_") or "capture"

    def _default_lane_state(self):
        return {
            "active": False,
            "mode": "idle",
            "error_y": None,
            "error_angle": None,
            "forward_speed": None,
            "lateral_speed": None,
            "angular_speed": None,
            "distance": None,
            "frame_shape": None,
            "updated_at": None,
        }

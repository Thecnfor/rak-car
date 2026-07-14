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
                cameras[cam_id] = {
                    "active": cam_id in self.frames and not stale,
                    "stale": stale,
                    "frame_age": frame_age,
                    "last_capture_at": meta.get("captured_at"),
                    "last_source_update_at": source_updated_at,
                    "last_recover_at": meta.get("last_recover_at"),
                    "last_issue": meta.get("last_issue"),
                }
        return {
            "status": "running" if self.running else "stopped",
            "active_cams": active_cams,
            "fps": self.fps,
            "quality": self.quality,
            "stale_timeout": self.stale_timeout,
            "cameras": cameras,
        }

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
        if frame is not None and self._frame_is_fresh(normalized):
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

    def stream_frames(self, cam_id):
        normalized = self.normalize_cam_id(cam_id)
        last_frame_time = 0.0
        while self.running:
            current_time = time.time()
            if current_time - last_frame_time < self.capture_interval:
                time.sleep(0.01)
                continue
            frame = self.get_frame_or_placeholder(normalized)
            ret, buffer = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.quality],
            )
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
        ret, buffer = cv2.imencode(
            ".jpg",
            frame,
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
                    * {
                        margin: 0;
                        padding: 0;
                        box-sizing: border-box;
                    }
                    body {
                        font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
                        background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
                        min-height: 100vh;
                        color: #e0e0e0;
                        padding: 20px;
                    }
                    .container {
                        max-width: 1400px;
                        margin: 0 auto;
                    }
                    header {
                        display: flex;
                        justify-content: space-between;
                        align-items: center;
                        margin-bottom: 20px;
                        padding: 15px 20px;
                        background: rgba(255, 255, 255, 0.05);
                        border-radius: 15px;
                        border: 1px solid rgba(255, 255, 255, 0.1);
                        backdrop-filter: blur(10px);
                    }
                    h1 {
                        font-size: 1.5em;
                        background: linear-gradient(90deg, #00d4ff, #7b2cbf, #ff006e);
                        -webkit-background-clip: text;
                        -webkit-text-fill-color: transparent;
                        background-clip: text;
                    }
                    .key-panel-title {
                        font-size: 0.9em;
                        color: #8892b0;
                        display: flex;
                        align-items: center;
                        gap: 10px;
                    }
                    .key-panel-title .key-icon,
                    .key-panel-title #floatKeyDisplay {
                        font-size: 2.2em;
                    }
                    .key-panel-title #floatKeyDisplay.active {
                        transform: scale(1.2);
                    }
                    .stream-container {
                        display: flex;
                        justify-content: space-between;
                        gap: 20px;
                        margin-top: 20px;
                        width: 100%;
                    }
                    .status-grid {
                        display: grid;
                        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
                        gap: 12px;
                        margin-top: 20px;
                    }
                    .status-card {
                        background: rgba(255, 255, 255, 0.03);
                        border: 1px solid rgba(255, 255, 255, 0.1);
                        border-radius: 12px;
                        padding: 12px 14px;
                    }
                    .status-card .label {
                        font-size: 0.85em;
                        color: #8892b0;
                        margin-bottom: 6px;
                    }
                    .status-card .value {
                        font-size: 1.1em;
                        color: #e6f1ff;
                        font-family: monospace;
                    }
                    .stream-box {
                        background: rgba(255, 255, 255, 0.03);
                        border: 2px solid rgba(255, 255, 255, 0.1);
                        border-radius: 15px;
                        padding: 15px;
                        flex: 1;
                        min-width: 300px;
                        backdrop-filter: blur(5px);
                    }
                    .stream-box h3 {
                        text-align: center;
                        margin-bottom: 15px;
                        color: #ccd6f6;
                        font-size: 1.2em;
                    }
                    .stream-box img {
                        max-width: 100%;
                        height: auto;
                        border-radius: 10px;
                        border: 2px solid rgba(0, 0, 0, 0.3);
                        box-shadow: 0 5px 20px rgba(0, 0, 0, 0.5);
                    }
                    footer {
                        text-align: center;
                        margin-top: 30px;
                        color: #495670;
                        font-size: 0.9em;
                    }
                    @media (max-width: 1100px) {
                        .stream-container {
                            flex-direction: column;
                            align-items: center;
                        }
                        .stream-box {
                            width: 100%;
                            max-width: 600px;
                        }
                    }
                </style>
            </head>
            <body>
                <div class="container">
                    <header>
                        <h1>智能车监控系统</h1>
                        <h4 class="key-panel-title">
                            <span id="floatKeyDisplay"></span>
                            <span class="key-icon">⌨️</span>
                        </h4>
                    </header>
                    <div class="status-grid">
                        <div class="status-card">
                            <div class="label">Lane 状态</div>
                            <div class="value" id="laneActive">idle</div>
                        </div>
                        <div class="status-card">
                            <div class="label">横向误差</div>
                            <div class="value" id="laneError">--</div>
                        </div>
                        <div class="status-card">
                            <div class="label">角度误差</div>
                            <div class="value" id="laneAngle">--</div>
                        </div>
                        <div class="status-card">
                            <div class="label">横移速度</div>
                            <div class="value" id="laneYSpeed">--</div>
                        </div>
                        <div class="status-card">
                            <div class="label">角速度</div>
                            <div class="value" id="laneAngleSpeed">--</div>
                        </div>
                        <div class="status-card">
                            <div class="label">累计距离</div>
                            <div class="value" id="laneDistance">--</div>
                        </div>
                    </div>
                    <div class="stream-container">
                        <div class="stream-box">
                            <h3>前视画面 (cam1 / front)</h3>
                            <img src="/video_feed/cam1" alt="front camera">
                        </div>
                        <div class="stream-box">
                            <h3>侧视画面 (cam2 / side)</h3>
                            <img src="/video_feed/cam2" alt="side camera">
                        </div>
                    </div>
                    <footer>
                        <p>Powered by FastAPI & OpenCV</p>
                    </footer>
                </div>
                <script>
                    let isPageActive = false;
                    document.body.addEventListener('click', function() {
                        isPageActive = true;
                    });
                    document.addEventListener('keydown', function(event) {
                        if (!isPageActive) {
                            return;
                        }
                        fetch('/keypress', {
                            method: 'POST',
                            headers: { 'Content-Type': 'application/json' },
                            body: JSON.stringify({ key: event.key })
                        })
                        .then(response => response.json())
                        .then(data => {
                            const keyElement = document.getElementById('floatKeyDisplay');
                            keyElement.innerText = data.received || '';
                            keyElement.classList.remove('active');
                            void keyElement.offsetWidth;
                            keyElement.classList.add('active');
                        })
                        .catch(function(err) {
                            console.error('发送失败:', err);
                        });
                        if (['F5', 'F12'].includes(event.key)) {
                            event.preventDefault();
                        }
                    });

                    function formatNum(value, digits = 4) {
                        if (value === null || value === undefined) {
                            return '--';
                        }
                        if (typeof value !== 'number') {
                            return String(value);
                        }
                        return value.toFixed(digits);
                    }

                    async function refreshLaneState() {
                        try {
                            const response = await fetch('/v1/vision/lane/state');
                            const data = await response.json();
                            document.getElementById('laneActive').innerText = data.active ? (data.mode || 'tracking') : 'idle';
                            document.getElementById('laneError').innerText = formatNum(data.error_y);
                            document.getElementById('laneAngle').innerText = formatNum(data.error_angle);
                            document.getElementById('laneYSpeed').innerText = formatNum(data.lateral_speed);
                            document.getElementById('laneAngleSpeed').innerText = formatNum(data.angular_speed);
                            document.getElementById('laneDistance').innerText = formatNum(data.distance, 3);
                        } catch (error) {
                            console.error('lane state 获取失败:', error);
                        }
                    }

                    refreshLaneState();
                    setInterval(refreshLaneState, 300);
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

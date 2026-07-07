#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""streamer.py — rak-car's monitor service.

Exposes a *manifest-driven* panel surface to the persistent frontend in
/home/jetson/workspace/rak-hri. Two layers:

  1. **Panel manifest** at GET /api/panels
     A versioned, self-describing list of every panel the frontend can
     render (raw cameras, processed detection streams, pose, velocity,
     battery, IR, sensor grid, keypad, …). The frontend polls this
     periodically; new panels appear without code changes.

  2. **Data endpoints**
       /api/health                            process liveness
       /api/panels                            manifest
       /api/cameras/<front|side>/mjpeg        raw MJPEG
       /api/cameras/<front|side>/detections/mjpeg  processed MJPEG (boxes drawn)
       /api/cameras/<front|side>/snapshot     single JPEG (raw)
       /api/state/pose                        {x,y,theta}
       /api/state/velocity                    {vx,vy,w}
       /api/state/battery                     {voltage_v, percent}
       /api/sensors/all                       all sensor values
       /api/detections                        latest detection list (JSON)
       POST/GET/DELETE /api/keypress          keypress relay

All endpoints are self-contained: only depend on Flask, OpenCV, numpy.
No import from vehicle/, car_wrap.py, task_func.py. See
/home/jetson/.claude/.../tools-streamer-run-gotchas.md for why.

Run:
    python3 -m tools.streamer
    RAK_STREAMER_PORT=5000 python3 -m tools.streamer
    RAK_CAMERA_FORCE_MOCK=1 python3 -m tools.streamer   # no hardware
"""

from __future__ import annotations

import json
import logging
import os
import socket
import sys
import threading
import time
from typing import Any, Generator, Optional

# Make the repo root importable so `python3 -m tools.streamer` works.
sys.path.append(os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from flask import Flask, Response, jsonify, request  # noqa: E402

from .base.camera import Camera  # noqa: E402
from .detector import BaseDetector, LaneZmqDetector, MockDetector, NoneDetector, ZmqDetector, draw_detections  # noqa: E402
from .panels import Panel, REGISTRY, SIZE_LG, SIZE_MD, SIZE_SM  # noqa: E402
from .state import STATE  # noqa: E402


_logger = logging.getLogger("tools.streamer")

DEFAULT_PORT = int(os.environ.get("RAK_STREAMER_PORT", "5000"))
DEFAULT_HOST = os.environ.get("RAK_STREAMER_HOST", "0.0.0.0")
DEFAULT_FPS = int(os.environ.get("RAK_MJPEG_FPS", "20"))
DEFAULT_QUALITY = int(os.environ.get("RAK_MJPEG_QUALITY", "70"))

CAM_INDEX = {"side": 1, "front": 2}
VALID_CAMS = tuple(CAM_INDEX.keys())


# ---------------------------------------------------------------------------
# Streamer
# ---------------------------------------------------------------------------


class Streamer:
    _instances: dict[int, "Streamer"] = {}

    def __init__(
        self,
        port: int = DEFAULT_PORT,
        host: str = DEFAULT_HOST,
        fps: int = DEFAULT_FPS,
        quality: int = DEFAULT_QUALITY,
    ) -> None:
        self.port = port
        self.host = host
        self.fps = fps
        self.quality = quality
        # RAK_DETECTOR:
        #   'none' (default) → no boxes ever drawn; raw MJPEG only.
        #   'mock'           → opt-in synthetic PERSON/VEHICLE/SIGN/PLANT/PEST/QR
        #                      tracks for dev/demo. Never default — keeps the
        #                      production UI free of fake data.
        #   'zmq'            → real detector: routes each cam to the matching
        #                      rak-car ZMQ inference backend (see infer.yaml).
        #                      side cam → task (5002, task_wbt2025)
        #                      front cam → front (5003, front_model2)
        #                      Requires infer_back_end.py to be running.
        det_choice = os.environ.get("RAK_DETECTOR", "none").lower().strip()
        # RAK_LANE_DETECTOR=on|off — overlay the lane model next to the
        # YOLOE detector under RAK_DETECTOR=zmq. Default: on when zmq.
        lane_choice = os.environ.get("RAK_LANE_DETECTOR", "auto").lower().strip()
        if det_choice == "mock":
            self._detector_factory = lambda seed: MockDetector(seed=seed)
            self._lane_factory = lambda idx: NoneDetector()
        elif det_choice == "zmq":
            # idx (1/2) -> ZMQ service name (matches infer.yaml):
            #   side cam (idx=1) -> task (5002, task_wbt2025: 作物/方向)
            #   front cam (idx=2) -> front (5003, front_model2: 转向)
            _idx_to_service = {CAM_INDEX["side"]: "task", CAM_INDEX["front"]: "front"}
            def _make_zmq(idx: int) -> ZmqDetector:
                return ZmqDetector(_idx_to_service.get(idx, "front"))
            self._detector_factory = _make_zmq
            # Lane detector (front only by default — the side cam rarely
            # sees the road). Set RAK_LANE_CAM=front|side|both to override.
            lane_cams = os.environ.get("RAK_LANE_CAM", "front").lower().strip()
            _lane_cams = (
                {"front", "side"}
                if lane_cams == "both"
                else {lane_cams}
                if lane_cams in ("front", "side")
                else {"front"}
            )
            enabled_lane = lane_choice in ("on", "auto", "1", "true", "yes")
            if enabled_lane:
                def _make_lane(idx: int) -> BaseDetector:
                    cam_id = "side" if idx == CAM_INDEX["side"] else "front"
                    return LaneZmqDetector() if cam_id in _lane_cams else NoneDetector()
                self._lane_factory = _make_lane
            else:
                self._lane_factory = lambda idx: NoneDetector()
        else:
            self._detector_factory = lambda seed: NoneDetector()
            self._lane_factory = lambda idx: NoneDetector()
        self.cams: dict[str, Camera] = {}
        self.detectors: dict[str, BaseDetector] = {}
        self.lane_detectors: dict[str, BaseDetector] = {}
        self._last_detections: dict[str, list[dict]] = {"front": [], "side": []}
        self._detect_lock = threading.Lock()
        self.app = Flask(__name__)
        logging.getLogger("werkzeug").setLevel(logging.ERROR)
        self._server: Any = None
        self._server_thread: Optional[threading.Thread] = None
        self._running = False
        self._register_panels()
        self._setup_routes()

    # -- lifecycle ------------------------------------------------------

    def start(self) -> None:
        if self.port in Streamer._instances:
            _logger.warning("port %d in use, stopping old streamer", self.port)
            Streamer._instances[self.port].stop()
        if self._running:
            return
        for cam_id, idx in CAM_INDEX.items():
            cam = Camera(index=idx)
            cam.init()
            self.cams[cam_id] = cam
            self.detectors[cam_id] = self._detector_factory(idx)
            self.lane_detectors[cam_id] = self._lane_factory(idx)
        # Detection-overlay panels are only useful when a detector is
        # actually wired (i.e. not the default NoneDetector). Register
        # them now that detectors are known.
        self._register_detection_panels()
        from werkzeug.serving import make_server
        self._server = make_server(self.host, self.port, self.app, threaded=True)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True, name="streamer-http"
        )
        self._server_thread.start()
        self._running = True
        Streamer._instances[self.port] = self
        self._banner()

    def _detector_loop(self) -> None:
        """Removed in favor of synchronous detection. /api/detections and the
        annotated MJPEG/snapshot endpoints each call `_run_detector` on
        demand, which is fast (a few ms per frame) and avoids the
        threading complexity that bit us earlier."""
        pass

    def _run_detector(self, cam_id: str) -> list[dict]:
        """Read the latest frame, run the detector, return the result
        list. Updates the cached `_last_detections[cam_id]` so a subsequent
        /api/detections call returns the same data. Returns [] if no
        detector is wired (no fake data)."""
        cam = self.cams.get(cam_id)
        detector = self.detectors.get(cam_id)
        if cam is None or detector is None:
            return []
        # Fast-path: NoneDetector never produces detections.
        if type(detector).__name__ == "NoneDetector":
            return []
        frame = cam.read()
        if frame is None:
            return []
        try:
            detections = detector.detect(frame)
        except Exception:  # noqa: BLE001
            return []
        result = [
            {
                "class_id": d.class_id,
                "label": d.label,
                "confidence": round(d.confidence, 3),
                "bbox": [int(d.x), int(d.y), int(d.w), int(d.h)],
                "track_id": d.track_id,
                "age_s": round(d.age_s, 2),
            }
            for d in detections
        ]
        with self._detect_lock:
            self._last_detections[cam_id] = result
        return result

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._server is not None:
            try:
                self._server.shutdown()
            except Exception:  # noqa: BLE001
                pass
        for cam in self.cams.values():
            try:
                cam.close()
            except Exception:  # noqa: BLE001
                pass
        self.cams.clear()
        Streamer._instances.pop(self.port, None)

    def get_key(self, clear: bool = True) -> Optional[str]:
        rec = STATE.last_key()
        if clear and rec["key"]:
            STATE.record_key("")
        return rec["key"] or None

    def update_frame(self, image, cam_id: str = "front") -> None:
        cam = self.cams.get(cam_id)
        if cam is None or image is None:
            return
        cam._frame = image  # noqa: SLF001 — explicit daemon-side injection

    # -- panel manifest -------------------------------------------------

    def _register_panels(self) -> None:
        # Raw cameras are always present.
        REGISTRY.register(Panel(
            id="cam-front-raw",
            type="mjpeg",
            title="Front · Raw",
            description="Raw front-camera MJPEG, no processing. Shows NO SIGNAL when the device is missing.",
            endpoint="/api/cameras/front/mjpeg",
            size=SIZE_LG,
            poll_ms=0,
            meta={"camera_id": "front"},
        ))
        REGISTRY.register(Panel(
            id="cam-side-raw",
            type="mjpeg",
            title="Side · Raw",
            description="Raw side-camera MJPEG, no processing. Shows NO SIGNAL when the device is missing.",
            endpoint="/api/cameras/side/mjpeg",
            size=SIZE_LG,
            poll_ms=0,
            meta={"camera_id": "side"},
        ))
        # state panels — always present
        REGISTRY.register(Panel(
            id="pose",
            type="value",
            title="Pose",
            description="Robot odometry: x, y, theta (radians).",
            endpoint="/api/state/pose",
            size=SIZE_SM,
            poll_ms=500,
            meta={"fields": ["x", "y", "theta"], "units": {"theta": "rad"}},
        ))
        REGISTRY.register(Panel(
            id="velocity",
            type="value",
            title="Velocity",
            description="Current velocity command (vx, vy, ω).",
            endpoint="/api/state/velocity",
            size=SIZE_SM,
            poll_ms=500,
            meta={"fields": ["vx", "vy", "w"], "units": {"vx": "m/s", "vy": "m/s", "w": "rad/s"}},
        ))
        REGISTRY.register(Panel(
            id="battery",
            type="gauge",
            title="Battery",
            description="Main battery voltage and percent remaining.",
            endpoint="/api/state/battery",
            size=SIZE_SM,
            poll_ms=1000,
            meta={"field": "percent", "voltage_field": "voltage_v",
                  "min": 9.0, "max": 12.5, "unit": "V", "color": "acid"},
        ))
        REGISTRY.register(Panel(
            id="ir",
            type="sensor_grid",
            title="Infrared",
            description="Left and right infrared distance sensors.",
            endpoint="/api/sensors/all",
            size=SIZE_MD,
            poll_ms=500,
            meta={"fields": ["ir_left_m", "ir_right_m"], "unit": "m"},
        ))
        REGISTRY.register(Panel(
            id="sensors",
            type="sensor_grid",
            title="Sensors",
            description="Uptime + battery + last keypress.",
            endpoint="/api/sensors/all",
            size=SIZE_MD,
            poll_ms=1000,
            meta={"fields": ["uptime_s", "battery_v", "last_key"], "unit": ""},
        ))
        REGISTRY.register(Panel(
            id="keypad",
            type="keypad",
            title="Last Key",
            description="Last keyboard input received from the page.",
            endpoint="/api/keypress",
            size=SIZE_SM,
            poll_ms=500,
            meta={},
        ))

    def _register_detection_panels(self) -> None:
        """Called from start() once detectors are wired. Only registers
        the detection-overlay panels when a real or mock detector is
        active (NOT the default NoneDetector). The 'detections' value
        panel is also gated on having a real detector, since otherwise
        it would always return empty arrays."""
        detector_name = type(self.detectors.get("front", type(None))).__name__
        if detector_name == "NoneDetector":
            return
        REGISTRY.register(Panel(
            id="cam-front-detections",
            type="mjpeg",
            title="Front · Detections",
            description=f"Front camera with detection overlay (detector={detector_name}).",
            endpoint="/api/cameras/front/detections/mjpeg",
            size=SIZE_LG,
            poll_ms=0,
            meta={"camera_id": "front", "detector": detector_name},
        ))
        REGISTRY.register(Panel(
            id="cam-side-detections",
            type="mjpeg",
            title="Side · Detections",
            description=f"Side camera with detection overlay (detector={detector_name}).",
            endpoint="/api/cameras/side/detections/mjpeg",
            size=SIZE_LG,
            poll_ms=0,
            meta={"camera_id": "side", "detector": detector_name},
        ))
        REGISTRY.register(Panel(
            id="detections",
            type="value",
            title="Latest Detections",
            description=f"Most recent detection results from all cameras (detector={detector_name}).",
            endpoint="/api/detections",
            size=SIZE_MD,
            poll_ms=1000,
            meta={"fields": ["front", "side"]},
        ))

    # -- routes ---------------------------------------------------------

    def _setup_routes(self) -> None:
        @self.app.route("/api/health")
        def health():
            return jsonify({
                "running": self._running,
                "port": self.port,
                "fps": self.fps,
                "quality": self.quality,
                "cameras": [
                    {"id": cid, "index": CAM_INDEX[cid], "mock": cam._mock}  # noqa: SLF001
                    for cid, cam in self.cams.items()
                ],
                "manifest_hash": REGISTRY.manifest()["hash"],
            })

        @self.app.route("/api/panels")
        def panels():
            return jsonify(REGISTRY.manifest())

        @self.app.route("/api/cameras/<cam_id>/mjpeg")
        def video_feed(cam_id: str):
            if cam_id not in self.cams:
                return jsonify({"error": f"unknown camera: {cam_id}"}), 404
            return Response(
                self._generate_mjpeg(cam_id, with_detections=False),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.app.route("/api/cameras/<cam_id>/detections/mjpeg")
        def detections_mjpeg(cam_id: str):
            if cam_id not in self.cams:
                return jsonify({"error": f"unknown camera: {cam_id}"}), 404
            return Response(
                self._generate_mjpeg(cam_id, with_detections=True),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.app.route("/api/cameras/<cam_id>/snapshot")
        def snapshot(cam_id: str):
            if cam_id not in self.cams:
                return jsonify({"error": f"unknown camera: {cam_id}"}), 404
            return self._render_snapshot(cam_id, with_detections=False)

        @self.app.route("/api/cameras/<cam_id>/detections/snapshot.jpg")
        def detections_snapshot(cam_id: str):
            if cam_id not in self.cams:
                return jsonify({"error": f"unknown camera: {cam_id}"}), 404
            return self._render_snapshot(cam_id, with_detections=True)

        @self.app.route("/api/state/pose")
        def state_pose():
            return jsonify(asdict(STATE.pose()))

        @self.app.route("/api/state/velocity")
        def state_velocity():
            return jsonify(asdict(STATE.velocity()))

        @self.app.route("/api/state/battery")
        def state_battery():
            return jsonify(STATE.battery())

        @self.app.route("/api/sensors/all")
        def sensors_all():
            return jsonify(STATE.sensors())

        @self.app.route("/api/detections")
        def detections():
            # Run synchronously per camera. ~5ms per call with the mock detector.
            return jsonify({
                "front": self._run_detector("front"),
                "side": self._run_detector("side"),
            })

        @self.app.route("/api/keypress", methods=["POST"])
        def keypress_post():
            data = request.get_json(silent=True) or {}
            key = data.get("key")
            action = data.get("action")
            if not isinstance(key, str) or len(key) == 0 or len(key) > 32:
                return jsonify({"error": "invalid key"}), 400
            rec = STATE.record_key(key)
            if action:
                STATE.apply_command(action, data.get("params") or {})
            return jsonify({"status": "ok", "received": rec["key"], "at": rec["at"]})

        @self.app.route("/api/keypress", methods=["GET"])
        def keypress_get():
            clear = request.args.get("clear") == "1"
            rec = STATE.last_key()
            if clear and rec["key"]:
                STATE.record_key("")
                rec = {"key": "", "at": 0.0}
            if not rec["key"]:
                return jsonify({"status": "empty"})
            return jsonify({"status": "ok", "key": rec["key"], "at": rec["at"]})

        @self.app.route("/api/keypress", methods=["DELETE"])
        def keypress_delete():
            STATE.record_key("")
            return jsonify({"status": "cleared"})

    # -- mjpeg generation -----------------------------------------------

    def _generate_mjpeg(self, cam_id: str, *, with_detections: bool) -> Generator[bytes, None, None]:
        cam = self.cams[cam_id]
        detector = self.detectors.get(cam_id) if with_detections else None
        # Lane detector runs only when there's a real wired detector
        # (i.e. RAK_DETECTOR=zmq + RAK_LANE_DETECTOR enabled). Its
        # overlays compose with the box overlays on the same frame.
        lane = self.lane_detectors.get(cam_id) if with_detections else None
        has_lane = lane is not None and type(lane).__name__ != "NoneDetector"
        interval = 1.0 / max(1, self.fps)
        last = 0.0
        while self._running:
            now = time.time()
            if now - last < interval:
                time.sleep(0.01)
                continue
            frame = cam.read()
            if frame is None:
                blank = np.zeros((cam.height, cam.width, 3), dtype=np.uint8)
                cv2.putText(blank, f"WAITING FOR {cam_id}…",
                            (40, blank.shape[0] // 2), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (255, 255, 255), 2)
                ok, buf = cv2.imencode(".jpg", blank)
            else:
                composed: list = []
                if detector is not None:
                    try:
                        composed.extend(detector.detect(frame))
                    except Exception:  # noqa: BLE001
                        pass
                if has_lane:
                    try:
                        composed.extend(lane.detect(frame))
                    except Exception:  # noqa: BLE001
                        pass
                # store the latest detection list (per camera) — boxes
                # only, not lane, since the dashboard's "Latest
                # Detections" panel expects object-shaped boxes.
                with self._detect_lock:
                    self._last_detections[cam_id] = [
                        {
                            "class_id": d.class_id,
                            "label": d.label,
                            "confidence": round(d.confidence, 3),
                            "bbox": [int(d.x), int(d.y), int(d.w), int(d.h)],
                            "track_id": d.track_id,
                            "age_s": round(d.age_s, 2),
                        }
                        for d in composed
                        if d.label not in ("lane_l", "lane_r")
                    ]
                if composed:
                    draw_detections(frame, composed)
                ok, buf = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
                )
            if not ok:
                time.sleep(0.05)
                continue
            last = now
            yield (
                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
            )

    def _render_snapshot(self, cam_id: str, *, with_detections: bool):
        cam = self.cams[cam_id]
        frame = cam.read()
        if frame is None:
            return jsonify({"error": "no frame yet"}), 503
        if with_detections:
            detector = self.detectors.get(cam_id)
            if detector is not None:
                try:
                    detections = detector.detect(frame)
                    with self._detect_lock:
                        self._last_detections[cam_id] = [
                            {
                                "class_id": d.class_id,
                                "label": d.label,
                                "confidence": round(d.confidence, 3),
                                "bbox": [int(d.x), int(d.y), int(d.w), int(d.h)],
                                "track_id": d.track_id,
                                "age_s": round(d.age_s, 2),
                            }
                            for d in detections
                        ]
                    draw_detections(frame, detections)
                except Exception:  # noqa: BLE001
                    pass
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return jsonify({"error": "encode failed"}), 500
        return Response(buf.tobytes(), mimetype="image/jpeg")

    # -- helpers --------------------------------------------------------

    def _banner(self) -> None:
        ip = self._local_ip()
        m = REGISTRY.manifest()
        print(
            f"\n[streamer] rak-car monitor on http://{ip}:{self.port}\n"
            f"  panels  : http://{ip}:{self.port}/api/panels   ({m['count']} panels, hash={m['hash']})\n"
            f"  health  : http://{ip}:{self.port}/api/health\n"
            f"  raw mjpeg: http://{ip}:{self.port}/api/cameras/front/mjpeg\n"
            f"  det mjpeg: http://{ip}:{self.port}/api/cameras/front/detections/mjpeg\n"
            f"  keypress: POST /api/keypress  {{key}}\n"
            f"  backend : opencv (mock={self.cams['front']._mock})\n",  # noqa: SLF001
            flush=True,
        )

    @staticmethod
    def _local_ip() -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:  # noqa: BLE001
            return "127.0.0.1"


# Helper: asdict import shim (avoids circular import line noise)
from dataclasses import asdict  # noqa: E402


if __name__ == "__main__":
    logging.basicConfig(
        level=os.environ.get("RAK_STREAMER_LOG_LEVEL", "INFO"),
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    s = Streamer()
    s.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[streamer] shutting down…", flush=True)
    finally:
        s.stop()

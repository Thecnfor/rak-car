# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary

import threading
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from msgs.msg import DetectionArray, LaneResult
from rclpy.node import Node
from sensor_msgs.msg import Image

from .overlay_helpers import (
    draw_detection_overlay,
    draw_lane_overlay,
    image_data_sequence,
)

Frame = Tuple[np.ndarray, object, str]


class VisionOverlayNode(Node):
    """Publish one RViz overlay from one raw camera and one result topic."""

    def __init__(self) -> None:
        super().__init__("vision_overlay")
        self.declare_parameter("camera_topic", "/rak/sensors/camera/front/image_raw")
        self.declare_parameter("result_topic", "/rak/perception/lane")
        self.declare_parameter("output_topic", "/rak/visualization/front_overlay")
        self.declare_parameter("overlay_type", "lane")
        self.declare_parameter("publish_rate_hz", 10.0)

        self._overlay_type = str(self.get_parameter("overlay_type").value)
        if self._overlay_type not in {"lane", "detection"}:
            raise ValueError("overlay_type must be 'lane' or 'detection'")
        self._lock = threading.Lock()
        self._frame: Optional[Frame] = None
        self._lane: Optional[LaneResult] = None
        self._detections: Optional[DetectionArray] = None
        qos = 5
        camera_topic = self.get_parameter("camera_topic").value
        result_topic = self.get_parameter("result_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._image_sub = self.create_subscription(
            Image, camera_topic, self._on_image, qos)
        if self._overlay_type == "lane":
            self._result_sub = self.create_subscription(
                LaneResult, result_topic, self._on_lane, qos)
        else:
            self._result_sub = self.create_subscription(
                DetectionArray, result_topic, self._on_detections, qos)
        self._pub = self.create_publisher(Image, output_topic, qos)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"VisionOverlay up: {camera_topic} + {result_topic} "
            f"-> {output_topic} ({self._overlay_type}) @ {rate:g} Hz")

    def _on_image(self, msg: Image) -> None:
        try:
            row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = row[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA).copy()
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"overlay raw image decode failed: {exc}")
            return
        with self._lock:
            self._frame = (frame, msg.header.stamp, msg.header.frame_id)

    def _on_lane(self, msg: LaneResult) -> None:
        with self._lock:
            self._lane = msg

    def _on_detections(self, msg: DetectionArray) -> None:
        with self._lock:
            self._detections = msg

    @staticmethod
    def _to_image(frame: np.ndarray, stamp, frame_id: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = image_data_sequence(frame)
        return msg

    def _publish(self) -> None:
        with self._lock:
            if self._frame is None:
                return
            frame, stamp, frame_id = self._frame
            frame = frame.copy()
            lane = self._lane
            detections = self._detections
        if self._overlay_type == "lane":
            frame = draw_lane_overlay(frame, lane) if lane is not None else frame
        else:
            if detections is not None:
                scaled = DetectionArray()
                scaled.model_id = detections.model_id
                scaled.image_width = 320
                scaled.image_height = 240
                scaled.class_names = detections.class_names
                scaled.scores = detections.scores
                scaled.xs = [x * 0.5 for x in detections.xs]
                scaled.ys = [y * 0.5 for y in detections.ys]
                scaled.widths = [w * 0.5 for w in detections.widths]
                scaled.heights = [h * 0.5 for h in detections.heights]
                frame = draw_detection_overlay(frame, scaled)
        self._pub.publish(self._to_image(frame, stamp, frame_id))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

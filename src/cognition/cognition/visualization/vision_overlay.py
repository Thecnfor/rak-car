# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary

import threading
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from msgs.msg import DetectionArray, LaneResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from .overlay_helpers import draw_detection_overlay, draw_lane_overlay

Frame = Tuple[np.ndarray, object, str]


class VisionOverlayNode(Node):
    """Join live camera frames with structured inference results for RViz."""

    def __init__(self) -> None:
        super().__init__("vision_overlay")
        self.declare_parameter("front_image_topic", "/rak/sensors/camera/front/image_compressed")
        self.declare_parameter("side_image_topic", "/rak/sensors/camera/side/image_compressed")
        self.declare_parameter("lane_topic", "/rak/perception/lane")
        self.declare_parameter("detection_topic", "/rak/perception/detections/task")
        self.declare_parameter("publish_rate_hz", 10.0)

        self._lock = threading.Lock()
        self._front: Optional[Frame] = None
        self._side: Optional[Frame] = None
        self._lane: Optional[LaneResult] = None
        self._detections: Optional[DetectionArray] = None
        qos = 5
        self._front_sub = self.create_subscription(
            CompressedImage, self.get_parameter("front_image_topic").value,
            self._on_front, qos)
        self._side_sub = self.create_subscription(
            CompressedImage, self.get_parameter("side_image_topic").value,
            self._on_side, qos)
        self._lane_sub = self.create_subscription(
            LaneResult, self.get_parameter("lane_topic").value,
            self._on_lane, qos)
        self._detection_sub = self.create_subscription(
            DetectionArray, self.get_parameter("detection_topic").value,
            self._on_detections, qos)
        self._front_pub = self.create_publisher(
            Image, "/rak/visualization/front_overlay", qos)
        self._side_pub = self.create_publisher(
            Image, "/rak/visualization/side_overlay", qos)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "VisionOverlay up: front/side images + lane/detections -> RViz overlay topics")

    @staticmethod
    def _decode(msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        except Exception:
            return None

    def _on_front(self, msg: CompressedImage) -> None:
        frame = self._decode(msg)
        if frame is not None:
            with self._lock:
                self._front = (frame, msg.header.stamp, msg.header.frame_id)

    def _on_side(self, msg: CompressedImage) -> None:
        frame = self._decode(msg)
        if frame is not None:
            with self._lock:
                self._side = (frame, msg.header.stamp, msg.header.frame_id)

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
        msg.data = frame.tobytes()
        return msg

    def _publish(self) -> None:
        with self._lock:
            front = None if self._front is None else (
                self._front[0].copy(), self._front[1], self._front[2])
            side = None if self._side is None else (
                self._side[0].copy(), self._side[1], self._side[2])
            lane = self._lane
            detections = self._detections
        if front is not None:
            frame, stamp, frame_id = front
            if lane is not None:
                frame = draw_lane_overlay(frame, lane)
            self._front_pub.publish(self._to_image(frame, stamp, frame_id))
        if side is not None:
            frame, stamp, frame_id = side
            if detections is not None:
                frame = draw_detection_overlay(frame, detections)
            self._side_pub.publish(self._to_image(frame, stamp, frame_id))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
"""
inference_bridge — pure ROS2 inference bridge node.

Subscribes to raw image topics and publishes inference results as ROS2 topics:
  Subscribes: /rak/sensors/camera/<id>/image_raw
  Publishes:   /rak/perception/{lane,detections/task,detections/front,ocr}

When ENABLE_INFERENCE is not set or 0, the bridge stays alive but publishes
empty/mock results so downstream consumers can be tested without a real GPU
inference backend. Live model forward pass lands in a later phase.

This is the Python business layer's only node for now (C++ owns all
hardware nodes + the mission task framework). See docs/architecture-overhaul.md.
"""

from __future__ import annotations

import logging
import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from msgs.msg import LaneResult, DetectionArray


_logger = logging.getLogger(__name__)


class InferenceBridgeNode(Node):
    """Pure ROS2 inference bridge — image → perception topics."""

    def __init__(self) -> None:
        super().__init__("inference_bridge_node")

        # --- Gate: ENABLE_INFERENCE=0 → mock mode ---
        self._enable_inference = os.environ.get("ENABLE_INFERENCE", "").lower() in {
            "1", "true", "yes"
        }

        # --- Parameters ---
        self.declare_parameter("lane_topic", "/rak/perception/lane")
        self.declare_parameter("task_detect_topic",
                               "/rak/perception/detections/task")
        self.declare_parameter("front_detect_topic",
                               "/rak/perception/detections/front")
        self.declare_parameter("ocr_topic",
                               "/rak/perception/ocr")
        self.declare_parameter("camera_topic",
                               "/rak/sensors/camera/front/image_raw")
        self.declare_parameter("mock_rate_hz", 10.0)

        lane_topic = self.get_parameter("lane_topic").value
        task_topic = self.get_parameter("task_detect_topic").value
        front_topic = self.get_parameter("front_detect_topic").value
        ocr_topic = self.get_parameter("ocr_topic").value
        cam_topic = self.get_parameter("camera_topic").value
        mock_rate = float(self.get_parameter("mock_rate_hz").value)

        # --- QoS: sensor data is BEST_EFFORT (matches camera_node output) ---
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- Publishers ---
        self._lane_pub = self.create_publisher(
            LaneResult, lane_topic, sensor_qos)
        self._task_pub = self.create_publisher(
            DetectionArray, task_topic, sensor_qos)
        self._front_pub = self.create_publisher(
            DetectionArray, front_topic, sensor_qos)
        self._ocr_pub = self.create_publisher(String, ocr_topic, sensor_qos)

        # --- Subscribers ---
        self._img_sub = self.create_subscription(
            Image, cam_topic, self._on_image, sensor_qos)

        # --- Mock timer (runs when inference is disabled) ---
        self._mock_timer = self.create_timer(
            1.0 / mock_rate, self._publish_mock)
        self._mock_counter = 0

        mode = "mock" if not self._enable_inference else "live"
        self.get_logger().info(
            "InferenceBridgeNode[%s]: cam=%s lane=%s task=%s front=%s ocr=%s",
            mode, cam_topic, lane_topic, task_topic, front_topic, ocr_topic,
        )

    # ------------------------------------------------------------------
    # Image callback — in Phase 6-7 this runs the real model forward pass.
    # For now, it just triggers mock generation on the same timer.
    # ------------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        """Received a camera frame. In Phase 6+ this runs inference."""
        # Store the latest frame timestamp for mock mode synchronization.
        self._last_image_stamp = msg.header.stamp
        if self._enable_inference:
            # Phase 6-7: run PaddlePaddle inference here, publish results.
            pass

    # ------------------------------------------------------------------
    # Mock inference — synthetic results for dev / hardware-free testing.
    # ------------------------------------------------------------------
    def _publish_mock(self) -> None:
        self._mock_counter += 1
        t = time.time()

        # Lane: gentle sine wave to simulate lane curvature.
        lane = LaneResult()
        lane.header.stamp = self.get_clock().now().to_msg()
        lane.header.frame_id = "camera_optical"
        lane.deviation_distance = 0.02 * math.sin(t * 1.5)
        lane.deviation_angle = 0.05 * math.sin(t * 2.0)
        lane.confidence = 0.85
        lane.valid = True
        lane.inference_ms = 3.0
        self._lane_pub.publish(lane)

        # Task detections: one mock bounding box every 30 frames.
        task = DetectionArray()
        task.header = lane.header
        task.header.frame_id = "camera_optical"
        task.model_id = "task_wbt2025"
        if self._mock_counter % 30 == 0:
            task.class_names = ["target_plant"]
            task.class_ids = [1]
            task.scores = [0.92]
            task.xs = [200.0]
            task.ys = [150.0]
            task.widths = [80.0]
            task.heights = [80.0]
        else:
            task.class_names = []
            task.class_ids = []
            task.scores = []
            task.xs = []
            task.ys = []
            task.widths = []
            task.heights = []
        self._task_pub.publish(task)

        # Front detections: sparse (every 60 frames).
        front = DetectionArray()
        front.header = lane.header
        front.header.frame_id = "camera_optical"
        front.model_id = "front_model2"
        if self._mock_counter % 60 == 0:
            front.class_names = ["obstacle"]
            front.class_ids = [0]
            front.scores = [0.78]
            front.xs = [320.0]
            front.ys = [240.0]
            front.widths = [60.0]
            front.heights = [60.0]
        else:
            front.class_names = []
            front.class_ids = []
            front.scores = []
            front.xs = []
            front.ys = []
            front.widths = []
            front.heights = []
        self._front_pub.publish(front)

        # OCR: static text (readable by downstream consumers).
        ocr = String()
        ocr.data = "TASK-01" if self._mock_counter % 20 < 10 else ""
        self._ocr_pub.publish(ocr)

    def destroy_node(self) -> None:
        self.get_logger().info("InferenceBridgeNode shutting down")
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = InferenceBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

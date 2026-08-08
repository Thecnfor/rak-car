# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
#
# DetectorNode — PP-YOLOE-Plus TRT 检测节点(竞赛物体检测)。
# Spec: docs/superpowers/specs/2026-08-09-ros2-layering-interfaces-design.md §5.4
#
# 订阅前视相机 → TRT 推理 → 发 /rak/perception/detections/<model_id>(DetectionArray)。
# 检测框为 NMS 后 [x1,y1,x2,y2,score,class];模型是竞赛检测器(water/蔬菜/cylinder/ball)。
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from msgs.msg import DetectionArray

from .detector import Detector


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        self.declare_parameter("camera_topic", "/rak/sensors/camera/side/image_compressed")
        self.declare_parameter("image_transport", "compressed")
        self.declare_parameter("engine_path", "/home/xrak/models/ppyoloe_plus_crn_s_80e_coco/model_fp16.engine")
        self.declare_parameter("labels_file", "/home/xrak/models/ppyoloe_plus_crn_s_80e_coco/labels.txt")
        self.declare_parameter("model_id", "ppyoloe")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("score_threshold", 0.3)
        self.declare_parameter("input_scale", 1.0)
        self.declare_parameter("bgr_to_rgb", False)

        topic = self.get_parameter("camera_topic").value
        transport = self.get_parameter("image_transport").value
        rate = self.get_parameter("publish_rate_hz").value

        # 引擎加载失败即 raise(no-mocks)。
        self._det = Detector(
            self.get_parameter("engine_path").value,
            input_scale=self.get_parameter("input_scale").value,
            bgr_to_rgb=self.get_parameter("bgr_to_rgb").value,
        )
        self._labels = self._load_labels(
            self.get_parameter("labels_file").value)
        self._model_id = self.get_parameter("model_id").value
        self._score_thr = self.get_parameter("score_threshold").value
        self.get_logger().info(
            f"TRT detector loaded: {self._det.engine.name} "
            f"({len(self._labels)} classes)")

        if transport == "raw":
            self._sub = self.create_subscription(
                Image, topic, self._on_image, 5)
        else:
            self._sub = self.create_subscription(
                CompressedImage, topic, self._on_compressed, 5)

        self._pub = self.create_publisher(
            DetectionArray, f"/rak/perception/detections/{self._model_id}", 5)
        self._frame = None
        self._frame_lock = threading.Lock()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"DetectorNode up: {topic} ({transport}) -> "
            f"/rak/perception/detections/{self._model_id} @ {rate} Hz")

    @staticmethod
    def _load_labels(path):
        try:
            with open(path) as f:
                return [line.strip() for line in f if line.strip()]
        except Exception as e:
            raise RuntimeError(f"labels load failed: {e}")

    # ------------------------------------------------------------------ #
    def _on_compressed(self, msg):
        import cv2
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"imdecode failed: {e}")
            return
        if frame is not None:
            with self._frame_lock:
                self._frame = frame

    def _on_image(self, msg):
        try:
            frame = np.frombuffer(msg.data, np.uint8).reshape(
                (msg.height, msg.width, 3))
        except Exception as e:
            self.get_logger().warn(f"raw reshape failed: {e}")
            return
        with self._frame_lock:
            self._frame = frame.copy()

    def _tick(self):
        with self._frame_lock:
            frame = self._frame
        if frame is None:
            return
        t0 = time.perf_counter()
        try:
            boxes, num = self._det.infer(frame)
        except Exception as e:
            self.get_logger().warn(f"detect failed: {e}")
            return
        inference_ms = (time.perf_counter() - t0) * 1000.0

        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.model_id = self._model_id
        msg.image_width = frame.shape[1]
        msg.image_height = frame.shape[0]
        for i in range(num):
            x1, y1, x2, y2, score, cls = boxes[i]
            if score < self._score_thr:
                continue
            cid = int(cls)
            msg.class_names.append(
                self._labels[cid] if cid < len(self._labels) else f"c{cid}")
            msg.class_ids.append(cid)
            msg.scores.append(float(score))
            msg.xs.append(float(x1))
            msg.ys.append(float(y1))
            msg.widths.append(float(x2 - x1))
            msg.heights.append(float(y2 - y1))
        msg.inference_ms = float(inference_ms) if hasattr(msg, "inference_ms") else 0.0
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

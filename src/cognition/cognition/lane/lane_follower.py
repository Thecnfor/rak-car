# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
#
# LaneFollower — 循线感知节点(偏航 correction_cnn + 偏角 cnn_lane)。
# Spec: docs/superpowers/specs/2026-08-09-ros2-layering-interfaces-design.md §5.4
#
# 订阅前视相机 → 两个模型各推一次(共享 128x128 预处理) → 发 /rak/perception/lane。
# behavior 层消费 LaneResult 做转向控制;本节点只做感知,不碰底盘。
#
# 字段映射(参数可调,融合权重是调参域):
#   deviation_angle   ← angle_source: "correction"(correction_cnn steer)
#                     | "lane"(cnn_lane d_e) | "blend"(按 blend_weight 线性混合)
#   deviation_distance ← cnn_lane d_a
#
# 模型路径走参数(默认指向 Orin ~/models/lane,可由 launch 覆盖);加载失败 raise(no-mocks)。
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from msgs.msg import LaneResult

from .models import CorrectionPredictor, LanePredictor


class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")
        self.declare_parameter("camera_topic", "/rak/sensors/camera/front/image_compressed")
        self.declare_parameter("image_transport", "compressed")   # compressed | raw
        self.declare_parameter("correction_weights", "/home/xrak/models/lane/correction_cnn/correction_cnn.pdparams")
        self.declare_parameter("cnn_lane_dir", "/home/xrak/models/lane/cnn_lane")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("angle_source", "correction")      # correction | lane | blend
        self.declare_parameter("blend_weight", 0.5)               # blend 时 correction 的权重
        self.declare_parameter("steer_scale", 1.0)                # correction steer -> rad 缩放

        topic = self.get_parameter("camera_topic").value
        transport = self.get_parameter("image_transport").value
        rate = self.get_parameter("publish_rate_hz").value

        # 模型加载失败即 raise(no-mocks: 不静默发假数据)。
        self._correction = CorrectionPredictor(
            self.get_parameter("correction_weights").value)
        self._lane = LanePredictor(self.get_parameter("cnn_lane_dir").value)
        self.get_logger().info("lane models loaded: correction_cnn + cnn_lane")

        # 订阅相机:只缓存最新一帧,定时器按 publish_rate_hz 取帧推理。
        if transport == "raw":
            self._sub = self.create_subscription(
                Image, topic, self._on_image, 5)
        else:
            self._sub = self.create_subscription(
                CompressedImage, topic, self._on_compressed, 5)

        self._pub = self.create_publisher(LaneResult, "/rak/perception/lane", 5)
        self._frame = None
        self._frame_lock = threading.Lock()

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"LaneFollower up: {topic} ({transport}) -> /rak/perception/lane @ {rate} Hz")

    # ------------------------------------------------------------------ #
    # 订阅回调(只存帧,不推理,避免阻塞订阅)                               #
    # ------------------------------------------------------------------ #
    def _on_compressed(self, msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"imdecode failed: {e}")
            return
        if frame is not None:
            with self._frame_lock:
                self._frame = frame

    def _on_image(self, msg: Image):
        try:
            frame = np.frombuffer(msg.data, np.uint8).reshape(
                (msg.height, msg.width, 3))
        except Exception as e:
            self.get_logger().warn(f"raw image reshape failed: {e}")
            return
        with self._frame_lock:
            self._frame = frame.copy()

    # ------------------------------------------------------------------ #
    # 定时推理 + 发 LaneResult                                             #
    # ------------------------------------------------------------------ #
    def _tick(self):
        with self._frame_lock:
            frame = self._frame
        if frame is None:
            return

        t0 = time.perf_counter()
        try:
            steer = self._correction.predict(frame)          # 偏航修正 [-1,1]
            d_a, d_e = self._lane.predict(frame)             # [距离, 偏航]
        except Exception as e:
            self.get_logger().warn(f"inference failed: {e}")
            return
        inference_ms = (time.perf_counter() - t0) * 1000.0

        source = self.get_parameter("angle_source").value
        scale = self.get_parameter("steer_scale").value
        if source == "lane":
            angle = d_e
        elif source == "blend":
            w = self.get_parameter("blend_weight").value
            angle = w * steer * scale + (1.0 - w) * d_e
        else:  # correction
            angle = steer * scale

        msg = LaneResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.deviation_angle = float(angle)
        msg.deviation_distance = float(d_a)
        msg.confidence = 1.0
        msg.valid = True
        msg.inference_ms = float(inference_ms)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
#
# CorrectionCNN — 车端循线偏航修正模型(6 conv + 3 FC)。
# 从 correction_cnn_v1/smartcar/paddlebaidu/paddle_jetson/base/correction_cnn.py
# 原样落地到 ROS2 包(保持训练/推理一致)。
#
# 输入 128x128x3 RGB, 输出 steer ∈ [-1, +1]。
# 训练: train/train_final_correction.py, 权重: correction_cnn.pdparams。
import paddle
import paddle.nn as nn


class CorrectionCNN(nn.Layer):
    """6 conv + 3 FC, 输出 steer ∈ [-1, 1]。"""

    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2D(3, 16, 3, stride=2, padding=1), nn.ReLU(),    # 128→64
            nn.Conv2D(16, 32, 3, stride=2, padding=1), nn.ReLU(),   # 64→32
            nn.Conv2D(32, 64, 3, stride=2, padding=1), nn.ReLU(),   # 32→16
            nn.Conv2D(64, 64, 3, stride=2, padding=1), nn.ReLU(),   # 16→8
            nn.Conv2D(64, 64, 3, stride=2, padding=1), nn.ReLU(),   # 8→4
            nn.Conv2D(64, 64, 3, stride=2, padding=1), nn.ReLU(),   # 4→2
        )
        self.head = nn.Sequential(
            nn.Linear(64 * 2 * 2, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Tanh(),
        )

    def forward(self, x):
        x = self.features(x)
        x = paddle.flatten(x, 1)
        return self.head(x)

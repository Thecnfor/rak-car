# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
#
# 两个循线模型的推理封装(在 Orin paddle 3.4 CPU 下验证可加载):
#   - CorrectionPredictor: correction_cnn 动态图 → steer ∈ [-1,1] (偏航修正)
#   - LanePredictor:       cnn_lane 静态图(PaddleInference) → [d_a, d_e]
#
# 共享预处理(与训练/车端一致):
#   BGR 帧 → resize 128x128 → BGR->RGB → (img/127.5)-1 → HWC->CHW → [1,3,128,128]
#
# 模型路径由节点参数传入,不在此硬编码。
import cv2
import numpy as np
import paddle
from paddle.inference import Config, create_predictor

from .correction_cnn import CorrectionCNN


class CorrectionPredictor:
    """correction_cnn 动态图:128x128 BGR 帧 → steer ∈ [-1, 1]。"""

    def __init__(self, weights_path: str):
        self.model = CorrectionCNN()
        state = paddle.load(weights_path)
        self.model.set_state_dict(state)

    def predict(self, img_bgr: np.ndarray) -> float:
        img = cv2.resize(img_bgr, (128, 128))
        img = img[:, :, ::-1]  # BGR -> RGB
        x = (img.astype(np.float32) / 127.5) - 1.0
        x = x.transpose(2, 0, 1)
        with paddle.no_grad():
            out = self.model(paddle.to_tensor(x[None, ...]))
        return float(out.numpy()[0, 0])


class LanePredictor:
    """cnn_lane 静态图(PaddleInference):BGR 帧 → [d_a(距离), d_e(偏航)]。"""

    def __init__(self, model_dir: str):
        cfg = Config(
            f"{model_dir}/cnn_lane.pdmodel",
            f"{model_dir}/cnn_lane.pdiparams",
        )
        cfg.disable_gpu()
        cfg.switch_ir_optim()
        cfg.enable_memory_optim()
        self.predictor = create_predictor(cfg)
        self.input_names = self.predictor.get_input_names()
        self.output_names = self.predictor.get_output_names()

    def preprocess(self, img_bgr: np.ndarray) -> np.ndarray:
        img = cv2.resize(img_bgr, (128, 128))
        img = img.astype(np.float32) / 127.5 - 1.0
        img = img[:, :, ::-1]  # BGR -> RGB
        img = img.transpose((2, 0, 1))
        return img[np.newaxis, :]

    def predict(self, img_bgr: np.ndarray) -> np.ndarray:
        x = self.preprocess(img_bgr)
        handle = self.predictor.get_input_handle(self.input_names[0])
        handle.copy_from_cpu(x)
        self.predictor.run()
        out = self.predictor.get_output_handle(
            self.output_names[0]).copy_to_cpu()
        return out[0]  # [d_a, d_e]

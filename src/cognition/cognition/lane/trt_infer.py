# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
#
# 两个 lane 模型的 TensorRT 推理封装(替代 paddle CPU,省内存 + 加速)。
#
#   TRTCorrectionPredictor: correction_cnn_fp16.engine → steer ∈ [-1,1]
#   TRTLanePredictor:       cnn_lane_fp16.engine → [d_a, d_e]
#
# 预处理与 paddle 路径一致(训练/车端同款): BGR → resize 128x128 → BGR->RGB
# -> (img/127.5)-1 -> HWC->CHW -> [1,3,128,128]。
# 静态 IO:缓冲在 __init__ 分配一次复用,close() 用 DeviceAllocation.free()。
import numpy as np
import tensorrt as trt
import cv2

try:
    import pycuda.driver as cuda
    import pycuda.autoinit  # noqa: F401
    _HAVE_PYCUDA = True
except Exception:  # pragma: no cover
    cuda = None
    _HAVE_PYCUDA = False


def preprocess128(img_bgr: np.ndarray) -> np.ndarray:
    """共享预处理:BGR 帧 → [1,3,128,128] float32。"""
    img = cv2.resize(img_bgr, (128, 128))
    img = img[:, :, ::-1]  # BGR -> RGB
    img = (img.astype(np.float32) / 127.5) - 1.0
    return img.transpose((2, 0, 1))[np.newaxis, :]


class _StaticRunner:
    """通用静态 IO TRT 执行器:缓冲分配一次复用。"""

    def __init__(self, engine_path: str):
        if not _HAVE_PYCUDA:
            raise RuntimeError("pycuda not available")
        logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(logger)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"failed to deserialize engine: {engine_path}")
        self.ctx = self.engine.create_execution_context()

        self.inputs = {}    # name -> DeviceAllocation
        self.outputs = {}   # name -> DeviceAllocation
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            shape = self.engine.get_tensor_shape(name)
            shape = tuple(d if d > 0 else 1 for d in shape)   # 动态维填 1
            if mode == trt.TensorIOMode.INPUT:
                self.ctx.set_input_shape(name, shape)
            alloc = cuda.mem_alloc(trt.volume(shape) * np.float32().itemsize)
            self.ctx.set_tensor_address(name, int(alloc))
            (self.inputs if mode == trt.TensorIOMode.INPUT else self.outputs)[name] = alloc

        self.stream = cuda.Stream()

    def run(self, x: np.ndarray):
        """x = 已预处理 [1,3,128,128]。返回 {输出名: numpy 数组}。"""
        in_name = next(iter(self.inputs))
        cuda.memcpy_htod_async(self.inputs[in_name], np.ascontiguousarray(x), self.stream)
        self.ctx.execute_async_v3(self.stream.handle)
        self.stream.synchronize()
        out = {}
        for name, alloc in self.outputs.items():
            shape = self.ctx.get_tensor_shape(name)
            shape = tuple(d if d > 0 else 1 for d in shape)
            buf = np.empty(shape, dtype=np.float32)
            cuda.memcpy_dtoh_async(buf, alloc, self.stream)
            self.stream.synchronize()
            out[name] = buf
        return out

    def close(self):
        for a in list(self.inputs.values()) + list(self.outputs.values()):
            a.free()
        self.inputs.clear()
        self.outputs.clear()


class TRTCorrectionPredictor:
    """correction_cnn(偏航修正)→ steer ∈ [-1,1]。"""

    def __init__(self, engine_path: str):
        self._r = _StaticRunner(engine_path)
        self._out_name = next(iter(self._r.outputs))

    def predict(self, img_bgr: np.ndarray) -> float:
        out = self._r.run(preprocess128(img_bgr))
        return float(out[self._out_name].flatten()[0])

    def close(self):
        self._r.close()


class TRTLanePredictor:
    """cnn_lane(偏角/距离)→ [d_a, d_e]。"""

    def __init__(self, engine_path: str):
        self._r = _StaticRunner(engine_path)
        self._out_name = next(iter(self._r.outputs))

    def predict(self, img_bgr: np.ndarray) -> np.ndarray:
        out = self._r.run(preprocess128(img_bgr))
        return out[self._out_name].flatten()[:2]

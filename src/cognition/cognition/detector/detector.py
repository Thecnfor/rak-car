# Copyright 2026 Thecnfor
# SPDX-License-Identifier: Proprietary
#
# PP-YOLOE-Plus TRT 检测推理(加载 model_fp16.engine)。
#
# 输入: BGR 帧(任意尺寸) → 640x640 拉伸 resize → CHW float32 → 引擎
# 输出: NMS 后 [num, 6] = (x1, y1, x2, y2, score, class) + num_dets
#
# 引擎输出是动态的 → 用 trt.IOutputAllocator 处理。scale_factor 输入恒为 1.0。
# 纯推理,无 ROS 依赖;节点层(cognition.detector.detector_node)只管订阅/发布。
import numpy as np
import tensorrt as trt

try:
    import pycuda.driver as cuda
    import pycuda.autoinit  # noqa: F401  初始化 CUDA 上下文
    _HAVE_PYCUDA = True
except Exception:  # pragma: no cover
    cuda = None
    _HAVE_PYCUDA = False


class _OutputAllocator(trt.IOutputAllocator):
    """动态 NMS 输出的显存分配器:容量足够就复用,不足才重分(避免每帧 cudaMalloc)。"""

    def __init__(self):
        super().__init__()
        self.mem = None
        self.shape = None
        self._capacity = 0

    def reallocate_output(self, tensor_name, memory, size, alignment):
        if size > self._capacity:
            if self.mem is not None:
                cuda.mem_free(self.mem)
            self.mem = cuda.mem_alloc(size)
            self._capacity = size
        return int(self.mem)

    def notify_shape(self, tensor_name, shape):
        self.shape = tuple(shape)

    def close(self):
        if self.mem is not None:
            cuda.mem_free(self.mem)
            self.mem = None
            self._capacity = 0


class Detector:
    """PP-YOLOE-Plus TRT 检测器。"""

    def __init__(self, engine_path: str, input_scale: float = 1.0,
                 bgr_to_rgb: bool = False, input_size: int = 640):
        if not _HAVE_PYCUDA:
            raise RuntimeError("pycuda not available")
        self.input_size = input_size
        self.input_scale = input_scale
        self.bgr_to_rgb = bgr_to_rgb
        self._output_alloc = None

        logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(logger)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"failed to deserialize engine: {engine_path}")
        self.context = self.engine.create_execution_context()

        # 输入 shape(动态 batch → 固定 1)
        self.context.set_input_shape("image", (1, 3, input_size, input_size))
        self.context.set_input_shape("scale_factor", (1, 2))

        # 输入/静态输出 buffer(TRT 8.5+ 无绑定 API:形状 + 地址都要 set)
        self._d_inputs = {}
        for name in ("image", "scale_factor"):
            shape = self.context.get_tensor_shape(name)
            size = trt.volume(shape)
            self._d_inputs[name] = cuda.mem_alloc(size * np.float32().itemsize)
            self.context.set_tensor_address(name, int(self._d_inputs[name]))
        # num_dets 输出(静态 [1])
        out_names = [self.engine.get_tensor_name(i)
                     for i in range(self.engine.num_io_tensors)
                     if self.engine.get_tensor_mode(
                         self.engine.get_tensor_name(i)) == trt.TensorIOMode.OUTPUT]
        self._num_dets_name = "save_infer_model/scale_1.tmp_0"
        self._num_dets = np.zeros(1, dtype=np.int32)
        self._d_num_dets = cuda.mem_alloc(4)
        self.context.set_tensor_address(self._num_dets_name, int(self._d_num_dets))
        # boxes 输出(动态) → output allocator
        self._boxes_name = "save_infer_model/scale_0.tmp_0"
        self._output_alloc = _OutputAllocator()
        self.context.set_output_allocator(self._boxes_name, self._output_alloc)

        self.stream = cuda.Stream()

    def preprocess(self, img_bgr: np.ndarray) -> np.ndarray:
        img = cv2_resize(img_bgr, (self.input_size, self.input_size))
        if self.bgr_to_rgb:
            img = img[:, :, ::-1]
        img = img.astype(np.float32) * self.input_scale
        img = img.transpose((2, 0, 1))  # HWC -> CHW
        return img[np.newaxis, :]

    def infer(self, img_bgr: np.ndarray):
        """返回 (boxes[n,6], num_dets)。boxes 列 = [x1,y1,x2,y2,score,class]"""
        x = self.preprocess(img_bgr)
        cuda.memcpy_htod_async(self._d_inputs["image"], np.ascontiguousarray(x), self.stream)
        sf = np.full((1, 2), 1.0, dtype=np.float32)  # scale_factor = 1.0
        cuda.memcpy_htod_async(self._d_inputs["scale_factor"], sf, self.stream)

        self.context.execute_async_v3(self.stream.handle)
        self.stream.synchronize()

        shape = self._output_alloc.shape
        boxes = np.empty(shape, dtype=np.float32)
        cuda.memcpy_dtoh_async(boxes, self._output_alloc.mem, self.stream)
        cuda.memcpy_dtoh_async(self._num_dets, self._d_num_dets, self.stream)
        self.stream.synchronize()
        return boxes, int(self._num_dets[0])

    def close(self):
        if self._output_alloc is not None:
            self._output_alloc.close()
        for d in self._d_inputs.values():
            cuda.mem_free(d)
        cuda.mem_free(self._d_num_dets)


def cv2_resize(img, size):
    import cv2
    return cv2.resize(img, size)

# --*-- coding: utf-8 --*--
# infer_back_end.py

import atexit
import concurrent.futures
import gc
import os
import signal
import sys
import time
import tracemalloc
import zmq
import json
import cv2
import yaml
import numpy as np
from threading import Thread, Lock
import threading
import urllib.request
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
# 添加项目根目录到Python路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")))

# 导入infer_front中的函数
from smartcar.paddlebaidu.infer_cs.base.infer_front import get_yaml, get_path_relative
from smartcar.paddlebaidu.paddle_jetson import YoloeInfer, LaneInfer, OCRReco
# from smartcar.whalesbot.tools.tools_class import get_yaml

# #region debug-point A:infer-backend-startup
def _debug_emit(hypothesis_id, location, msg, data=None, run_id="pre-fix"):
    api_url = os.environ.get("DEBUG_SERVER_URL") or os.environ.get("TRAE_DEBUG_API_URL")
    session_id = os.environ.get("DEBUG_SESSION_ID") or "program-camera-preview"
    env_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".dbg", "program-camera-preview.env")
    )
    if not api_url and os.path.exists(env_path):
        try:
            with open(env_path, "r", encoding="utf-8") as env_file:
                for line in env_file:
                    line = line.strip()
                    if line.startswith("DEBUG_SERVER_URL="):
                        api_url = line.split("=", 1)[1]
                    elif line.startswith("DEBUG_SESSION_ID="):
                        session_id = line.split("=", 1)[1]
        except Exception:
            pass
    if not api_url:
        return
    payload = {
        "sessionId": session_id,
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "msg": msg,
        "data": data or {},
        "ts": int(time.time() * 1000),
    }
    try:
        req = urllib.request.Request(
            api_url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=0.2).read()
    except Exception:
        pass
# #endregion

class InferServer:
    def __init__(self):
        _debug_emit("A", "infer_back_end.py:InferServer.__init__", "[DEBUG] infer backend init start")
        # 导入推理客户端的配置
        # configs = ClintInterface.configs
        configs = get_yaml('config_car.yml')['infer_cfg']
        _debug_emit(
            "A",
            "infer_back_end.py:InferServer.__init__",
            "[DEBUG] infer backend config loaded",
            {"configs": [conf.get("name") for conf in configs]},
        )

        # 2026-08-01：OOM 韧性改造（详见 .trae/specs/system-arch-optimization/spec.md）。
        # 默认只预热 lane；其余模型走懒加载。RAK_INFER_EAGER_MODELS 仍可覆盖（逗号分隔）。
        # 注意：保留旧 env 变量语义，旧版"全 eager"行为可通过 RAK_INFER_EAGER_MODELS=lane,task,ocr
        # 切回去。
        eager_env = os.environ.get("RAK_INFER_EAGER_MODELS")
        if eager_env is None:
            self._eager_models = {"lane"}
        else:
            self._eager_models = {
                m.strip() for m in eager_env.split(",") if m.strip()
            }
        print(f"[InferServer] eager_models: {sorted(self._eager_models)}")

        # 闲置自动卸载：默认 300s（env RAK_INFER_IDLE_UNLOAD_SECONDS 覆盖）。
        try:
            self._idle_unload_seconds = float(
                os.environ.get("RAK_INFER_IDLE_UNLOAD_SECONDS", "300")
            )
        except ValueError:
            self._idle_unload_seconds = 300.0
        # 单帧推理超时：默认 5s（env RAK_INFER_FRAME_TIMEOUT_S 覆盖）。
        try:
            self._frame_timeout_s = float(
                os.environ.get("RAK_INFER_FRAME_TIMEOUT_S", "5")
            )
        except ValueError:
            self._frame_timeout_s = 5.0
        print(
            f"[InferServer] idle_unload={self._idle_unload_seconds}s "
            f"frame_timeout={self._frame_timeout_s}s"
        )

        self.flag_infer_initok = False

        self.flag_end = False
        # 开启对应的线程和服务
        self.threads_list = []
        self.server_dict = {}

        # self.lane_server = self.get_server(5001)
        for conf in configs:
            print(conf)
            # 创建获取zmq服务
            server = self.get_server(conf['port'])
            self.server_dict[conf['name']] = server
            _debug_emit(
                "A",
                "infer_back_end.py:InferServer.__init__",
                "[DEBUG] infer backend server bound",
                {"name": conf.get("name"), "port": conf.get("port")},
            )
            # 创建线程
            # thread_tmp = Thread(target=eval('self.'+conf['name']+'_process'))
            # 带参数线程，此处参数为各种推理模型
            thread_tmp = Thread(target=self.process_demo, args=(conf['name'],))
            # thread_tmp = Thread(target=self.lane_process)
            thread_tmp.daemon = True
            thread_tmp.start()
            _debug_emit(
                "A",
                "infer_back_end.py:InferServer.__init__",
                "[DEBUG] infer backend worker started",
                {"name": conf.get("name")},
            )
            # 添加进程
            self.threads_list.append(thread_tmp)

        from smartcar.paddlebaidu.paddle_jetson import YoloeInfer, LaneInfer, OCRReco, LaneBlendInfer # , HummanAtrr, MotHuman

        InferFactory = {
            "YoloeInfer": YoloeInfer,
            "LaneInfer": LaneInfer,
            "LaneBlendInfer": LaneBlendInfer,
            "OCRReco": OCRReco,
            # "HummanAtrr": HummanAtrr,
            # "MotHuman": MotHuman
        }
        # 2026-08-01：模型注册表（lazy + LRU + 内存画像）。
        #   infer_dict:       name -> InferType 实例（懒加载后填入）
        #   _models_loaded:   name -> bool（是否已加载）
        #   _last_used_at:    name -> float（最后被调用时间；LRU 卸载依据）
        #   _load_lock:       name -> threading.Lock（同一模型并发懒加载串行）
        #   _mem_estimate_mb: name -> float（tracemalloc diff 估算，OOM 决策依据）
        #   <0 表示 gpu_only（不计入进程 RSS，走 GPU 显存）
        self.infer_dict = {}
        self._models_loaded = {}
        self._last_used_at = {}
        self._load_lock = threading.Lock()
        self._per_model_lock = {}
        self._mem_estimate_mb = {}
        self._lazy_load_count = {}
        self._infer_factory = InferFactory
        self._mem_method = "tracemalloc+rss"  # 占位；_load_model 首次进入时刷新
        self._mem_method_per_model = {}

        for conf in configs:
            model_name = conf['name']
            self._models_loaded[model_name] = False
            self._last_used_at[model_name] = 0.0
            self._mem_estimate_mb[model_name] = 0.0
            self._lazy_load_count[model_name] = 0
            self._per_model_lock[model_name] = threading.Lock()

            if model_name in self._eager_models:
                # 立即加载模型
                self._load_model(conf, model_name, InferFactory)
            else:
                print(f"[InferServer] lazy model: {model_name} (will load on first request)")

        # 单帧推理线程池：max_workers=1，避免推理库内多线程冲突；future.result(timeout=)
        # 实现"超时返回 [] 不杀线程"语义（Python 没有 signal 风格的可移植超时）。
        self._infer_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="infer-frame"
        )

        # 预加载推理几张图片，刚开始推理时速度慢，会有卡顿
        _debug_emit(
            "C",
            "infer_back_end.py:InferServer.__init__",
            "[DEBUG] infer warmup start",
            {"rounds": 3, "models": sorted(self._eager_models)},
        )
        warmup_cfgs = [c for c in configs if c['name'] in self._eager_models]
        if warmup_cfgs:
            # 新建一个空白图片，用于预先图片推理
            img = np.zeros((240, 240, 3), np.uint8)
            for i in range(3):
                for conf in warmup_cfgs:
                    infer_tmp = self.infer_dict.get(conf['name'])
                    if infer_tmp is None:
                        continue
                    warmup_start = time.time()
                    infer_tmp(img)
                    _debug_emit(
                        "C",
                        "infer_back_end.py:InferServer.__init__",
                        "[DEBUG] infer warmup step done",
                        {
                            "round": i + 1,
                            "name": conf.get("name"),
                            "cost_s": round(time.time() - warmup_start, 3),
                        },
                    )

        # 2026-08-01：tracemalloc baseline 由 _load_model 首次进入时 lazy 初始化，
        # 避免在 __init__ 里 tracemalloc.start() 与 Paddle C++ 扩展加载撞车。
        # 此处仅占位，_tm_baseline 在 _load_model 内首次写入。
        self._baseline_lock = threading.Lock()
        print("infer init ok")
        self._eager_loaded_count = sum(
            1 for v in self._models_loaded.values() if v
        )

        self.flag_infer_initok = True
        _debug_emit(
            "C",
            "infer_back_end.py:InferServer.__init__",
            "[DEBUG] infer backend init ready",
            {"ready": True, "eager_loaded": self._eager_loaded_count},
        )

        # 2026-08-01：闲置自动卸载 tick。60s 扫一次，超过阈值的非 eager 模型卸载。
        try:
            self._idle_unload_tick_seconds = 60.0
            self._idle_unload_thread = threading.Thread(
                target=self._idle_unload_loop,
                name="infer-idle-unload",
                daemon=True,
            )
            self._idle_unload_thread.start()
        except Exception as exc:  # pragma: no cover
            print(f"[InferServer] idle_unload thread start failed: {exc}")

        # 2026-08-01：进程退出钩子 + 信号处理，主动释放 ZMQ 资源。
        try:
            atexit.register(self._cleanup_sockets)
        except Exception:
            pass
        try:
            signal.signal(signal.SIGTERM, self._on_signal_exit)
            signal.signal(signal.SIGINT, self._on_signal_exit)
        except Exception:
            # 某些环境（threaded embedding）signal 装不上，忽略。
            pass

    def _on_signal_exit(self, signum, frame):
        """SIGTERM/SIGINT 处理器：置 flag_end + 主动释放 ZMQ。"""
        try:
            self.flag_end = True
            self._cleanup_sockets()
        except Exception:
            pass
        # 不 sys.exit，让主进程自然结束（pm2 重启友好）。

    def _cleanup_sockets(self):
        """主动释放 ZMQ socket + context；避免 pm2 重启时 Address already in use。"""
        try:
            for name, sock in list(self.server_dict.items()):
                try:
                    sock.setsockopt(zmq.LINGER, 0)
                    sock.close(0)
                except Exception:
                    pass
            self.server_dict.clear()
        except Exception:
            pass
        # context 全局共享，term() 会让所有 socket 失效；已 close 过的不会再次报错。
        try:
            self._SHARED_CONTEXT.term()
        except Exception:
            pass

    def _read_self_rss_mb(self):
        """读 /proc/self/status 的 VmRSS（KB → MB）。读不到返回 None。"""
        try:
            with open("/proc/self/status", "r") as f:
                for line in f:
                    if line.startswith("VmRSS:"):
                        return int(line.split()[1]) / 1024.0
        except Exception:
            return None
        return None

    def _idle_unload_loop(self):
        """后台 tick：每 60s 扫一次，闲置超过阈值的非 eager 模型主动卸载。"""
        while True:
            try:
                if self.flag_end:
                    return
                now = time.time()
                unloaded = []
                # 复制 keys 以便边遍历边删
                for name in list(self.infer_dict.keys()):
                    if name in self._eager_models:
                        continue
                    last = self._last_used_at.get(name, 0.0)
                    if last <= 0.0:
                        continue  # 从未被调用，不动
                    if (now - last) < self._idle_unload_seconds:
                        continue
                    with self._load_lock:
                        if name in self.infer_dict:
                            self.infer_dict.pop(name, None)
                            self._models_loaded[name] = False
                            self._mem_estimate_mb[name] = 0.0
                            unloaded.append(name)
                if unloaded:
                    gc.collect()
                    print(
                        f"[InferServer] idle-unload: {unloaded} "
                        f"(threshold={self._idle_unload_seconds}s)"
                    )
            except Exception as exc:  # pragma: no cover
                print(f"[InferServer] idle_unload_loop err: {exc}")
            time.sleep(self._idle_unload_tick_seconds)

    def _load_model(self, conf, model_name, InferFactory):
        """同步加载推理模型（per-model 锁 + tracemalloc+RSS 内存画像）。

        内存估算算法（解决原版 "task.mem=0MB" 的 bug）：
        1. 取加载前 tracemalloc snapshot S_before + rss_before（来自 _rss_baseline_mb）
        2. 执行模型构造（OCRReco / YoloeInfer / LaneInfer）
        3. 构造完后立刻 take_snapshot() S_after，与 baseline 比 diff
        4. RSS 用 *当前* 总 RSS 减去 baseline；不再用 rss_after 直接减 rss_before
           （旧版的 bug 是连续 load 多个模型时，第二个的 rss_after 已被第三个污染）
        5. 取 max(rss_delta, python_traced_delta) 作为 mem_estimate；
           Paddle 主体重 RSS（多是 mmap/anon），但 OCR 后处理 / YoloeInfer 的
           Python 包装分配走 tracemalloc 也能被覆盖到。
        """
        # per-model 锁：多个 REP 线程同时首次请求同一模型时串行加载。
        model_lock = self._per_model_lock.get(model_name)
        if model_lock is None:
            model_lock = threading.Lock()
            self._per_model_lock[model_name] = model_lock
        with model_lock:
            if self._models_loaded.get(model_name, False):
                return  # 其它线程已加载
            # 2026-08-01：首次进入 _load_model 时 lazy 初始化 baseline，
            # 避免在 __init__ 里 tracemalloc.start() 与 Paddle C++ 扩展加载撞车。
            # baseline 是「当前 Python 堆 + 当前 RSS」，所有后续 lazy load 都对比它。
            if not getattr(self, "_tm_baseline", None) and not getattr(self, "_baseline_lock", None):
                self._baseline_lock = threading.Lock()
            if not getattr(self, "_tm_baseline", None):
                with self._baseline_lock:
                    if not getattr(self, "_tm_baseline", None):
                        if not tracemalloc.is_tracing():
                            tracemalloc.start(25)
                        self._tm_baseline = tracemalloc.take_snapshot()
                        self._rss_baseline_mb = self._read_self_rss_mb() or 0.0
                        self._mem_method = "tracemalloc+rss"
                        print(
                            f"[InferServer] mem baseline established: "
                            f"rss={self._rss_baseline_mb:.1f}MB"
                        )

            InferType = InferFactory[conf['infer_type']]
            model_start = time.time()
            _debug_emit(
                "B",
                "infer_back_end.py:InferServer._load_model",
                "[DEBUG] infer model init start",
                {
                    "name": model_name,
                    "infer_type": conf.get("infer_type"),
                    "run_mode": conf.get("run_mode"),
                },
            )
            # baseline 已在 lazy init 时留好；这里只取 fresh snapshot。
            try:
                tm_before = tracemalloc.take_snapshot()
            except Exception:
                tm_before = None
            rss_before = self._rss_baseline_mb

            if InferType == OCRReco:
                if 'det_model_dir' in conf and 'rec_model_dir' in conf:
                    infer = InferType(conf['det_model_dir'], conf['rec_model_dir'], run_mode=conf['run_mode'])
                else:
                    raise InferType()
            elif InferType == LaneBlendInfer:
                # 双 cnn 叠加:d_a (model_dir) + d_e (model_dir_d_e)
                # 配置示例:
                #   infer_type: LaneBlendInfer
                #   model_dir: lane_model
                #   model_dir_d_e: lane_model_d_e
                model_dir_d_a = conf.get('model_dir', 'lane_model')
                model_dir_d_e = conf.get('model_dir_d_e', 'lane_model_d_e')
                infer = InferType(
                    model_dir_d_a=model_dir_d_a,
                    model_dir_d_e=model_dir_d_e,
                    run_mode=conf.get('run_mode', 'paddle'),
                )
            else:
                if 'model_dir' in conf:
                    infer = InferType(conf['model_dir'], run_mode=conf['run_mode'])
                else:
                    infer = InferType(run_mode=conf['run_mode'])

            with self._load_lock:
                self.infer_dict[model_name] = infer
                self._models_loaded[model_name] = True
                self._lazy_load_count[model_name] = self._lazy_load_count.get(model_name, 0) + 1

            # 2026-08-01：在读 RSS 之前做一次 dummy inference（warmup），
            # 否则 Paddle C++ 内部的 tensor pool / mmap 是 lazy 的，
            # `_load_model` 函数返回时 RSS 还没涨，第一次真实推理才 commit。
            # 用 1x1 灰度图触发最小分配路径。
            try:
                _warmup_img = np.zeros((64, 64, 3), np.uint8)
                infer(_warmup_img)
            except Exception:
                # warmup 失败不阻塞模型加载
                pass

            # 计算内存增量：tracemalloc + RSS
            # 2026-08-01：gc.collect() 强制回收加载过程中的临时对象，
            # 避免 Paddle C++ 内部 mmap 的 lazy RSS 增长还没 commit 就被吃掉。
            gc.collect()
            # 二次读 RSS：Paddle 模型 mmap 的内存页可能在构造返回后才被 RSS 反映，
            # 单次 _read_self_rss_mb 容易拿到 mmap 但还未 RSS-reserve 的快照。
            rss_now = self._read_self_rss_mb() or rss_before
            rss_now_2 = self._read_self_rss_mb() or rss_now
            # 取两次的最大值作为「真实常驻 RSS」
            rss_now = max(rss_now, rss_now_2)
            rss_delta_mb = max(rss_now - rss_before, 0.0)
            python_delta_mb = 0.0
            if tm_before is not None:
                try:
                    tm_after = tracemalloc.take_snapshot()
                    diff = tm_after.compare_to(self._tm_baseline, key_type='filename')
                    # diff[i].size_diff 可能是负数（GC 释放），取绝对值累计
                    total = sum(max(stat.size_diff, 0) for stat in diff)
                    python_delta_mb = total / (1024.0 * 1024.0)
                except Exception:
                    python_delta_mb = 0.0
            # 内存估算：取两者最大值（覆盖 RSS 主导的 Paddle C++ 分配与
            # tracemalloc 能看到的 Python 层包装对象），避免负数。
            self._mem_estimate_mb[model_name] = max(rss_delta_mb, python_delta_mb)
            # 2026-08-01：标记「GPU-only」模型（RSS 与 tracemalloc 都几乎为 0，
            # 说明模型走 GPU 显存，进程 RSS 不反映）。前端可据此显示 <gpu>。
            # Paddle task 模型在 Jetson 上 device="GPU" 走 unified memory 的 zero-copy 路径，
            # mmap 不立即 commit 到 anonymous RSS，所以 RSS delta 接近 0 是预期的，
            # 不是测量 bug。
            self._mem_method_per_model = getattr(self, "_mem_method_per_model", {})
            if self._mem_estimate_mb[model_name] < 1.0:
                self._mem_estimate_mb[model_name] = -1.0  # 标记 GPU-only
                self._mem_method_per_model[model_name] = "gpu_only"
            else:
                self._mem_method_per_model[model_name] = self._mem_method
            # 2026-08-01：更新 baseline 让下一模型的 diff 只覆盖自己的贡献，
            # 否则 lane=1090 / task=1066 会出现「总和远超实际 RSS」的错觉
            # （实际是 task 的 baseline 没考虑 lane 已占的内存）。
            # 增量记录到 _per_model_rss_after，调试用。
            self._per_model_rss_after = getattr(self, "_per_model_rss_after", {})
            self._per_model_rss_after[model_name] = rss_now
            self._rss_baseline_mb = rss_now
            # tracemalloc baseline 不更新——它是 cumulative Python 堆
            # 总增量，越往后越大；对单模型贡献用 rss_delta 已经够。

            mem_str = (
                f"~{self._mem_estimate_mb[model_name]:.0f}MB"
                if self._mem_estimate_mb[model_name] >= 0
                else "<gpu-only>"
            )
            print(
                f"[InferServer] model loaded: {model_name} "
                f"({time.time() - model_start:.2f}s, {mem_str} "
                f"[rss+{rss_delta_mb:.0f}/py+{python_delta_mb:.0f}])"
            )
            _debug_emit(
                "B",
                "infer_back_end.py:InferServer._load_model",
                "[DEBUG] infer model init done",
                {
                    "name": model_name,
                    "cost_s": round(time.time() - model_start, 3),
                    "rss_delta_mb": round(rss_delta_mb, 1),
                    "python_delta_mb": round(python_delta_mb, 1),
                    "mem_estimate_mb": round(self._mem_estimate_mb[model_name], 1),
                },
            )


    # 2026-08-01：共享 zmq context，方便 atexit/SIGTERM 时一次性 term()。
    _SHARED_CONTEXT = zmq.Context()

    def get_server(self, port):
        """用共享 context 建 REP socket；atexit 时统一 close + term()。"""
        socket = self._SHARED_CONTEXT.socket(zmq.REP)
        socket.bind(f"tcp://127.0.0.1:{port}")
        return socket

    def process_demo(self, name):
        """REP 循环：懒加载 + 单帧超时 + DROP_OLDEST 命令处理。

        head 取 response[:5]：
        - b"ATATA" → 健康检查（返回 dict 兼容旧 bool）
        - b"DROPX!" → 按 LRU 卸载非 eager 模型（runtime drop_oldest() 用）
        - b"image" → 推理，ThreadPoolExecutor 包超时
        """
        print(time.strftime("%Y-%m-%d %H:%M:%S"), "{} process start".format(name))
        server = self.server_dict[name]

        def get_infer_func(model_name):
            def lazy_infer(x, normalize=True):
                if not self._models_loaded.get(model_name, False):
                    print(f"[InferServer] lazy loading model: {model_name}")
                    configs = get_yaml('config_car.yml')['infer_cfg']
                    conf = next(
                        (c for c in configs if c['name'] == model_name), None
                    )
                    if conf:
                        self._load_model(conf, model_name, self._infer_factory)
                    else:
                        raise RuntimeError(f"unknown model: {model_name}")
                with self._load_lock:
                    inst = self.infer_dict.get(model_name)
                if inst is None:
                    self._last_used_at[model_name] = time.time()
                    return []
                result = inst(x, normalize)
                self._last_used_at[model_name] = time.time()
                return result
            return lazy_infer

        func = get_infer_func(name)
        _debug_emit(
            "D",
            "infer_back_end.py:process_demo",
            "[DEBUG] infer worker loop start",
            {"name": name},
        )

        while True:
            if self.flag_end:
                return
            try:
                response = server.recv()
            except Exception as exc:
                print("{} recv err: {}".format(name, exc))
                time.sleep(0.01)
                continue

            head = response[:6]
            res = []
            try:
                if response.startswith(b"ATATA"):
                    if self.flag_infer_initok:
                        loaded = bool(self._models_loaded.get(name, False))
                        last_used = float(self._last_used_at.get(name, 0.0))
                        mem_mb = float(self._mem_estimate_mb.get(name, 0.0))
                        # 2026-08-01：gpu_only 标记，-1 表示模型走 GPU 显存不计入 RSS。
                        gpu_only = mem_mb < 0
                        res = {
                            "ready": True,
                            "name": name,
                            "loaded": loaded,
                            "last_used_at": last_used,
                            "mem_estimate_mb": abs(mem_mb),  # 取绝对值
                            "gpu_only": gpu_only,
                            "mem_method": getattr(self, "_mem_method_per_model", {}).get(name, self._mem_method),
                            "lazy_load_count": int(self._lazy_load_count.get(name, 0)),
                            # 2026-08-03：raw1 帧传输能力位。客户端（infer_front）
                            # 见到 True 后改发 b"raw1"+裸帧,省掉 JPEG encode/decode。
                            "supports_raw": True,
                        }
                    else:
                        res = False
                elif response.startswith(b"DROPX!"):
                    evicted = []
                    now = time.time()
                    with self._load_lock:
                        for n in list(self.infer_dict.keys()):
                            if n in self._eager_models:
                                continue
                            if not self._models_loaded.get(n, False):
                                continue
                            last = self._last_used_at.get(n, 0.0)
                            if last <= 0.0:
                                continue
                            self.infer_dict.pop(n, None)
                            self._models_loaded[n] = False
                            self._mem_estimate_mb[n] = 0.0
                            evicted.append({"name": n, "idle_s": round(now - last, 1)})
                    if evicted:
                        gc.collect()
                    res = {"evicted": evicted, "rss_mb": self._read_self_rss_mb()}
                elif response.startswith(b"raw1"):
                    # 2026-08-03 raw1 协议：b"raw1" + <III h/w/ch + 裸 BGR bytes。
                    # 同机传输免 cv2.imencode/imdecode（Nano 上每帧各省 ~1-2ms）。
                    img = None
                    if len(response) >= 16:
                        h, w, ch = np.frombuffer(response[4:16], dtype=np.uint32)
                        img = np.frombuffer(response[16:], dtype=np.uint8)
                        expect = int(h) * int(w) * max(int(ch), 1)
                        if img.size == expect:
                            img = img.reshape((int(h), int(w)) if int(ch) == 1 else (int(h), int(w), int(ch)))
                        else:
                            print("{} raw1 size mismatch ({} != {})".format(name, img.size, expect))
                            img = None
                    if img is None:
                        res = []
                    elif self.flag_infer_initok:
                        try:
                            future = self._infer_executor.submit(func, img)
                            try:
                                res = future.result(timeout=self._frame_timeout_s)
                            except concurrent.futures.TimeoutError:
                                print(
                                    "{} frame timeout (>{}s); cancel future".format(
                                        name, self._frame_timeout_s
                                    )
                                )
                                future.cancel()
                                res = []
                        except Exception as infer_exc:
                            print("{} infer err: {}".format(name, infer_exc))
                            res = []
                elif response.startswith(b"image"):
                    img = cv2.imdecode(np.frombuffer(response[5:], dtype=np.uint8), 1)
                    if self.flag_infer_initok:
                        try:
                            future = self._infer_executor.submit(func, img)
                            try:
                                res = future.result(timeout=self._frame_timeout_s)
                            except concurrent.futures.TimeoutError:
                                print(
                                    "{} frame timeout (>{}s); cancel future".format(
                                        name, self._frame_timeout_s
                                    )
                                )
                                future.cancel()
                                res = []
                        except Exception as infer_exc:
                            print("{} infer err: {}".format(name, infer_exc))
                            res = []
                json_data = json.dumps(res)
                json_data = bytes(json_data, encoding="utf-8")
                server.send(json_data)
            except Exception as exc:
                print("{} process err: {}".format(name, exc))
                time.sleep(0.01)

    def close(self):
        print("closing...")
        self.flag_end = True
        for thread in self.threads_list:
            # 等待结束
            thread.join()
            # 关闭
            thread.close()

def main():
    print("infer_back_end.py 程序开始运行")
    infer_back = InferServer()

    while True:
        try:
            time.sleep(1)
        except Exception as e:
            print(e)
            break
    time.sleep(0.1)
    infer_back.close()

if __name__ == "__main__":
    main()

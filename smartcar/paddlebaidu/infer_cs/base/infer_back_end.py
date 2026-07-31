# --*-- coding: utf-8 --*--
# infer_back_end.py

import zmq
import json
import cv2
import yaml
import numpy as np
from threading import Thread, Lock
import threading
import time
import os
import sys
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
        
        # 支持环境变量控制哪些模型常驻加载（节省内存）
        # RAK_INFER_EAGER_MODELS=eager,lane,task  只加载 eager/lane/task，OCR 按需加载
        eager_models_env = os.environ.get("RAK_INFER_EAGER_MODELS", "")
        self._eager_models = set(m.strip() for m in eager_models_env.split(",") if m.strip()) if eager_models_env else None
        print(f"[InferServer] eager_models config: {self._eager_models}")
        
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
        
        from smartcar.paddlebaidu.paddle_jetson import YoloeInfer, LaneInfer, OCRReco # , HummanAtrr, MotHuman

        InferFactory = {
            "YoloeInfer": YoloeInfer,
            "LaneInfer": LaneInfer,
            "OCRReco": OCRReco,
            # "HummanAtrr": HummanAtrr,
            # "MotHuman": MotHuman
        }
        # 创建推理模型（支持按需加载节省内存）
        self.infer_dict = {}
        self._models_loaded = {}  # name -> True/False
        self._model_lock = threading.Lock()

        for conf in configs:
            model_name = conf['name']
            # 检查是否需要常驻加载
            should_load = self._eager_models is None or model_name in self._eager_models
            
            self._models_loaded[model_name] = False
            
            if should_load:
                # 立即加载模型
                self._load_model(conf, model_name, InferFactory)
            else:
                print(f"[InferServer] lazy model: {model_name} (will load on first request)")
        
        # 预加载推理几张图片，刚开始推理时速度慢，会有卡顿
        _debug_emit(
            "C",
            "infer_back_end.py:InferServer.__init__",
            "[DEBUG] infer warmup start",
            {"rounds": 3, "models": [conf.get("name") for conf in configs]},
        )
        warmup_cfgs = [c for c in configs if self._models_loaded.get(c['name'], False)]
        if warmup_cfgs:
            # 新建一个空白图片，用于预先图片推理
            img = np.zeros((240, 240, 3), np.uint8)
            for i in range(3):
                for conf in warmup_cfgs:
                    infer_tmp = self.infer_dict[conf['name']]
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
        print("infer init ok")
        self._eager_loaded_count = sum(1 for v in self._models_loaded.values() if v)

        self.flag_infer_initok = True
        _debug_emit(
            "C",
            "infer_back_end.py:InferServer.__init__",
            "[DEBUG] infer backend init ready",
            {"ready": True, "eager_loaded": self._eager_loaded_count},
        )

    def _load_model(self, conf, model_name, InferFactory):
        """同步加载推理模型"""
        InferType = InferFactory[conf['infer_type']]
        model_start = time.time()
        _debug_emit(
            "B",
            "infer_back_end.py:InferServer.__init__",
            "[DEBUG] infer model init start",
            {
                "name": model_name,
                "infer_type": conf.get("infer_type"),
                "run_mode": conf.get("run_mode"),
            },
        )
        if InferType == OCRReco:
            if 'det_model_dir' in conf and 'rec_model_dir' in conf:
                infer = InferType(conf['det_model_dir'], conf['rec_model_dir'], run_mode=conf['run_mode'])
            else:
                raise InferType()
        else:
            if 'model_dir' in conf:
                infer = InferType(conf['model_dir'], run_mode=conf['run_mode'])
            else:
                infer = InferType(run_mode=conf['run_mode'])
        
        with self._model_lock:
            self.infer_dict[model_name] = infer
            self._models_loaded[model_name] = True
        
        print(f"[InferServer] model loaded: {model_name} ({time.time() - model_start:.2f}s)")
        _debug_emit(
            "B",
            "infer_back_end.py:InferServer.__init__",
            "[DEBUG] infer model init done",
            {
                "name": model_name,
                "cost_s": round(time.time() - model_start, 3),
            },
        )


    def get_server(self, port):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://127.0.0.1:{port}")
        return socket
    
    def process_demo(self, name):
        
        print(time.strftime("%Y-%m-%d %H:%M:%S"), "{} process start".format(name))
        server:zmq.Socket = self.server_dict[name]
        
        # lambda定义推理函数，含有归一化处理参数为True, 此处定义方便后续调用
        # 注意：这里闭包捕获的是 self.infer_dict，需要支持懒加载
        def get_infer_func(model_name):
            def lazy_infer(x, normalize=True):
                # 检查模型是否已加载，未加载则先加载
                if not self._models_loaded.get(model_name, False):
                    print(f"[InferServer] lazy loading model: {model_name}")
                    configs = get_yaml('config_car.yml')['infer_cfg']
                    conf = next((c for c in configs if c['name'] == model_name), None)
                    if conf:
                        from smartcar.paddlebaidu.paddle_jetson import YoloeInfer, LaneInfer, OCRReco
                        InferFactory = {"YoloeInfer": YoloeInfer, "LaneInfer": LaneInfer, "OCRReco": OCRReco}
                        self._load_model(conf, model_name, InferFactory)
                return self.infer_dict[model_name](x, normalize)
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
                # recv 阶段异常（少见，但 ZMQ socket 状态可能被外部打断）
                print("{} recv err: {}".format(name, exc))
                time.sleep(0.01)
                continue

            head = response[:5]
            res = []
            try:
                if head == b"ATATA":
                    if self.flag_infer_initok:
                        res = True
                    else:
                        res = False
                elif head == b"image":
                    # 把bytes转为jpg格式
                    img = cv2.imdecode(np.frombuffer(response[5:], dtype=np.uint8), 1)
                    if self.flag_infer_initok:
                        # 2026-07-31 加固:
                        # 推理异常不能杀死 REP 线程, 否则 lane/task/ocr 任一模型抛错
                        # 整个端口没人 recv, 前端 ClintInterface.get_infer 一直
                        # zmq.Again, lane_feed 守护线程会卡住 / 触发前端 backoff 风暴.
                        try:
                            res = func(img)
                        except Exception as infer_exc:
                            print("{} infer err: {}".format(name, infer_exc))
                            # 返回 [] 让前端知道本次推理失败但不阻塞,
                            # 不要让异常向上抛出杀掉这个 REP 线程.
                            res = []
                # 任何分支都到这里, send 响应保证 ZMQ REQ/REP 同步不被打乱.
                json_data = json.dumps(res)
                json_data = bytes(json_data, encoding="utf-8")
                server.send(json_data)
            except Exception as exc:
                # send 失败 / json 编码失败 / 其他: 这次响应丢了, REQ 端会超时.
                # 但 REP 状态已对齐(没 send 也算消耗了一帧), 下次循环能继续.
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

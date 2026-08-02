#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import zmq
import cv2
import numpy as np
import json
import yaml

import time, os, sys, struct
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..","..", "..")))
from smartcar.whalesbot.tools.log_wrap import logger
# from smartcar.whalesbot.tools.tools_class import get_yaml

def get_yaml(path):
    root_path = get_path_relative("..", "..", "..", "..")
    config_path = os.path.join(root_path, "config_car.yml")
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.load(f, Loader=yaml.FullLoader)
    except Exception as e:
        print('{} not found'.format(config_path))
        print(e)
        return None

def get_path_relative(*args):
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)


def get_zmp_client(port):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    res = socket.connect(f"tcp://127.0.0.1:{port}")
    # print(res)
    return socket

class Bbox:
    def __init__(self, box=None, rect=None, size=[640, 480]) -> None:
        self.size = np.array(size) / 2
        self.size_concat = np.concatenate((self.size, self.size))

        if box is not None:
            box_np = np.array(box)
            # 如果所有值的绝对值都小于1，表示归一化
            # np.abs(box_np, out=box_np)
            if (np.abs(box_np) < 2).all():
                self.box_normalise = box_np
                self.box = self.denormalise(box_np, self.size)
                # print(self.box)
            else:
                self.box = box_np
                self.box_normalise = self.normalise(box_np, self.size)
            self.rect = self.box_to_rect(self.box, self.size)
        elif rect is not None:
            self.rect = np.array(rect)
            self.box = self.rect_to_box(self.rect, self.size)
            self.box_normalise = self.normalise(self.box, self.size)
    
    def get_rect(self):
        return self.rect
    
    def get_box(self):
        return self.box

    @staticmethod
    def normalise(box, size):
        return box / np.concatenate((size, size))
    
    @staticmethod
    # 去归一化
    def denormalise(box_nor, size):
        return (box_nor * np.concatenate((size, size))).astype(np.int32)

    @staticmethod
    def rect_to_box(rect, size):
        pt_tl = rect[:2]
        pt_br = rect[2:]
        pt_center = (pt_tl + pt_br) / 2 - size
        box_wd = pt_br - pt_tl
        return np.concatenate((pt_center, box_wd)).astype(np.int32)

    @staticmethod
    def box_to_rect(box, size):
        pt_center = box[:2]
        box_wd = box[2:]
        pt_tl = (size + pt_center - box_wd / 2).astype(np.int32)
        pt_br = (size + pt_center + box_wd / 2).astype(np.int32)
        # print(pt_tl, pt_br)
        rect = np.concatenate((pt_tl, pt_br))
        # 限制最大最小值
        max_size = np.concatenate((size, size))*2
        # print(max_size)
        np.clip(rect, 0, max_size, out=rect)
        return rect

class ClintInterface:
    # configs = [
    #         {'name':'lane', 'infer_type': 'LaneInfer', 'params': [], 'port':5001, 'img_size':[128, 128]},
    #         {'name':'task', 'infer_type': 'YoloeInfer', 'params': ['task_model3'], 'port':5002, 'img_size':[416, 416]},
    #         {'name':'front', 'infer_type':'YoloeInfer', 'params': ['front_model2'], 'port':5003, 'img_size':[416, 416]},
    #         {'name':'ocr', 'infer_type':'OCRReco', 'params': [], 'port':5004,'img_size':None},
    #         {'name':'humattr', 'infer_type':'HummanAtrr', 'params': [], 'port':5005, 'img_size':None},
    #         {'name':'mot', 'infer_type':'MotHuman', 'params': [], 'port':5006, 'img_size':None}
    #         ]
    
    def __init__(self, name, socket_timeout_ms=None):
        self.configs = get_yaml('config_car.yml')['infer_cfg']
        self.name = name
        # 2026-07-31：保护 zmq.REQ/REP 的 send/recv 与 reset_client，
        # 否则并发场景会触发 EFSM（"Operation cannot be accomplished in current state"）
        self._socket_lock = threading.Lock()
        # 2026-08-01：超时缩短到 500ms（默认 2000ms）。
        # lane_feed 50Hz 一次 2s 超时直接卡死守护线程；500ms 配合守护线程的
        # backoff 退避比 "等 2s 再重试" 鲁棒得多。
        # env: RAK_CAR_INFER_CLIENT_TIMEOUT_MS（毫秒）可覆盖
        if socket_timeout_ms is None:
            env_ms = os.environ.get("RAK_CAR_INFER_CLIENT_TIMEOUT_MS")
            if env_ms and env_ms.isdigit():
                socket_timeout_ms = int(env_ms)
            else:
                socket_timeout_ms = 500
        self._socket_timeout_ms = int(socket_timeout_ms)
        # 2026-08-01：reset 限速。EFSM/死 socket 时连续 reset 会让 ZMQ 状态机
        # 雪崩（旧 bug 之一：连续 EAGAIN → 反复 close/open → 上下文抖）。
        # 1s 内最多 reset 3 次。
        self._reset_count = 0
        self._reset_window_start = 0.0
        self._reset_max_per_window = 3
        self._reset_window_s = 1.0
        # 统计：让上层能看到"客户端活着但后端抖"
        self.stats = {
            "send_count": 0,
            "recv_count": 0,
            "err_count": 0,
            "reset_count": 0,
            "timeout_count": 0,
            "last_err": None,
            "last_err_at": 0.0,
            "last_recv_at": 0.0,
            "last_state_at": 0.0,
        }
        # 2026-08-03：raw 帧传输。同机 ZMQ 传 JPEG 是纯浪费（encode+decode 各
        # ~1-2ms @ Nano）。协商式启用：后端 ATATA 响应带 supports_raw=True 才走
        # raw1 头；旧后端继续走 b"image"+JPEG，滚动升级零风险。
        self._supports_raw = False
        logger.info("{}连接服务器...".format(name))
        model_cfg = self.get_config(name)
        self.img_size = model_cfg['img_size']
        self.raw_stats = {"raw_frames": 0, "jpeg_frames": 0}
        self.port = model_cfg['port']
        self.client = self.get_zmp_client(self.port)

        flag = False
        deadline = time.time() + float(os.getenv("RAK_CAR_INFER_CLIENT_READY_TIMEOUT", "45"))
        while time.time() < deadline:
            state = self.get_state()
            if state:
                if flag:
                    logger.info("")
                break
            # 输出一个提示信息，不换行
            print('.', end='', flush=True)
            time.sleep(1)
            flag = True
        else:
            raise RuntimeError(
                "{}推理服务未就绪，请先确认 runtime 已托管启动 infer_back_end.py".format(name)
            )
        # print(self.client)
        # print("连接服务器成功")
        logger.info("{}连接服务器成功".format(name))

    def get_config(self, name):
        for conf in self.configs:
            if conf['name'] == name:
                return conf
            
    @staticmethod
    def get_zmp_client(port, socket_timeout_ms=500):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        # 2026-08-01：超时参数化（默认 500ms）。REQ/REP 默认阻塞，
        # 短超时让上层 backoff 退避更细；2s 太长。
        socket.setsockopt(zmq.RCVTIMEO, socket_timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, socket_timeout_ms)
        socket.connect(f"tcp://127.0.0.1:{port}")
        return socket

    def _maybe_rate_limit_reset_locked(self):
        """reset 限速：1s 窗内最多 3 次，超过就直接放弃。返回 True=允许 reset。"""
        now = time.time()
        if now - self._reset_window_start > self._reset_window_s:
            self._reset_window_start = now
            self._reset_count = 0
        if self._reset_count >= self._reset_max_per_window:
            return False
        self._reset_count += 1
        return True

    def reset_client(self):
        with self._socket_lock:
            self._reset_client_locked()

    def _reset_client_locked(self):
        """必须在 self._socket_lock 已持有的前提下调用,避免 race。

        2026-08-01：限速 + 激进自愈：
          - 1s 窗内最多 reset 3 次（防止 EFSM 雪崩）
          - 连续 30s 拿不到有效 state/recv → 尝试重启 infer_back_end.py（后端进程可能被 OOM killer 杀）
        """
        # 激进自愈:若 reset_count 累计 > 20 且距上次成功 recv/send > 30s,
        # 说明单纯重连 socket 没用,后端进程可能死了 → 尝试重启 infer_back_end.py。
        now = time.time()
        last_activity = max(
            self.stats.get("last_err_at") or 0.0,
            self.stats.get("last_state_at") or 0.0,
        )
        total_reset = self.stats.get("reset_count", 0) + self._reset_count
        backend_considered_dead = (
            total_reset >= 20
            and last_activity > 0
            and (now - last_activity) > 30.0
        )
        # rate limit (1s 窗 3 次)
        if now - self._reset_window_start > self._reset_window_s:
            self._reset_window_start = now
            self._reset_count = 0
        if self._reset_count >= self._reset_max_per_window:
            self.stats["err_count"] += 1
            self.stats["last_err"] = "reset_rate_limited"
            self.stats["last_err_at"] = now
            raise RuntimeError(
                "{} client reset rate-limited (>{} in {}s)".format(
                    self.name, self._reset_max_per_window, self._reset_window_s,
                )
            )
        self._reset_count += 1

        if backend_considered_dead:
            logger.warning(
                "%s infer backend appears dead (reset_count=%d, last_activity=%.0fs ago) → trying restart infer_back_end.py",
                self.name, total_reset, now - last_activity,
            )
            try:
                self._try_restart_infer_backend_locked()
            except Exception as exc:
                logger.warning("%s try_restart_infer_backend failed: %s", self.name, exc)

        try:
            self.client.close()
        except Exception:
            pass
        self.client = self.get_zmp_client(self.port, self._socket_timeout_ms)
        self.stats["reset_count"] += 1

    def __call__(self, *args, **kwds):
        return self.get_infer(*args, **kwds)

    def get_state(self):
        data = bytes('ATATA', encoding='utf-8')
        with self._socket_lock:
            try:
                self.client.send(data)
                self.stats["send_count"] += 1
                response = self.client.recv()
                self.stats["recv_count"] += 1
                now = time.time()
                self.stats["last_recv_at"] = now
                self.stats["last_state_at"] = now
            except zmq.Again:
                self.stats["timeout_count"] += 1
                self.stats["err_count"] += 1
                self.stats["last_err"] = "state_timeout"
                self.stats["last_err_at"] = time.time()
                logger.warning("{}服务器状态探测超时".format(self.name))
                try:
                    self._reset_client_locked()
                except RuntimeError:
                    pass
                return None
            except zmq.ZMQError as exc:
                self.stats["err_count"] += 1
                self.stats["last_err"] = "state_zmq_err:{}".format(exc)
                self.stats["last_err_at"] = time.time()
                logger.warning("{}服务器状态探测失败: {}".format(self.name, exc))
                try:
                    self._reset_client_locked()
                except RuntimeError:
                    pass
                return None
        response = json.loads(response)
        if isinstance(response, dict):
            self._supports_raw = bool(response.get("supports_raw"))
        return response

    def get_infer(self, img):
        if self.img_size is not None:
            img = cv2.resize(img, self.img_size)
        if self._supports_raw:
            # raw1 协议：4B magic + <III h/w/ch + 裸 BGR bytes（免 encode/decode）
            if not img.flags["C_CONTIGUOUS"]:
                img = np.ascontiguousarray(img)
            h, w = img.shape[0], img.shape[1]
            ch = 1 if img.ndim == 2 else img.shape[2]
            data = b"raw1" + struct.pack("<III", h, w, ch) + img.tobytes()
            self.raw_stats["raw_frames"] += 1
        else:
            img = cv2.imencode('.jpg', img)[1].tobytes()
            data = bytes('image', encoding='utf-8') + img
            self.raw_stats["jpeg_frames"] += 1
        with self._socket_lock:
            try:
                self.client.send(data)
                self.stats["send_count"] += 1
                response = self.client.recv()
                self.stats["recv_count"] += 1
                self.stats["last_recv_at"] = time.time()
            except zmq.Again:
                self.stats["timeout_count"] += 1
                self.stats["err_count"] += 1
                self.stats["last_err"] = "infer_timeout"
                self.stats["last_err_at"] = time.time()
                try:
                    self._reset_client_locked()
                except RuntimeError:
                    pass
                raise RuntimeError("{}推理请求超时".format(self.name))
            except zmq.ZMQError as exc:
                self.stats["err_count"] += 1
                self.stats["last_err"] = "infer_zmq_err:{}".format(exc)
                self.stats["last_err_at"] = time.time()
                try:
                    self._reset_client_locked()
                except RuntimeError:
                    pass
                raise RuntimeError("{}推理请求失败: {}".format(self.name, exc))
        response = json.loads(response)
        return response

    def _try_restart_infer_backend_locked(self):
        """激进自愈：lane/task 后端进程被 OOM killer 杀后,尝试重启 infer_back_end.py。

        只在当前进程自己有启动权限、且环境里没 RAK_CAR_INFER_AUTO_START=0 才做。
        runtime 托管的后端进程 PID 记录在 smartcar/paddlebaidu/infer_cs/base 下
        的 .infer_backend_${PORT}.pid, 这里也查一下。
        """
        import os as _os
        import subprocess as _subp
        if _os.environ.get("RAK_CAR_INFER_AUTO_START", "1") == "0":
            return
        # 后端脚本: smartcar/paddlebaidu/infer_cs/base/infer_back_end.py
        base_dir = _os.path.dirname(_os.path.abspath(__file__))
        script = _os.path.join(base_dir, "infer_back_end.py")
        if not _os.path.isfile(script):
            return
        python = _os.environ.get("PYTHON", "/usr/bin/python3")
        pid_file = _os.path.join(base_dir, ".infer_backend_{}.pid".format(self.port))
        # 若 PID 文件存在,检查进程还活着
        if _os.path.isfile(pid_file):
            try:
                with open(pid_file) as fp:
                    old_pid = int(fp.read().strip() or "0")
                if old_pid > 1 and _os.path.isdir("/proc/{}".format(old_pid)):
                    # 进程还活着, 不要重复启动
                    return
            except Exception:
                pass
        # 启动一个后台实例,stdout/stderr 全部吞掉,避免阻塞 REP 循环
        try:
            proc = _subp.Popen(
                [python, script],
                cwd=base_dir,
                stdout=_subp.DEVNULL, stderr=_subp.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            raise RuntimeError("Popen infer_back_end.py failed: {}".format(exc))
        # 写 PID 文件 (best effort)
        try:
            with open(pid_file, "w") as fp:
                fp.write(str(proc.pid))
        except Exception:
            pass
        logger.warning(
            "%s infer_backend restarted: pid=%s (base_dir=%s)",
            self.name, proc.pid, base_dir,
        )


def main_client():
    from camera import Camera
    cap = Camera(1, 640, 480)
    # cap.set_size(640, 480)
    # cap.start_back_thread()
    # infer_client = ClintInterface('lane')
    infer_client = ClintInterface("ocr")
    # infer_client = ClintInterface('task')
    # infer_client = ClintInterface('mot')
    # infer_client = ClintInterface('front')
    # infer_client = infer_clint
    # while True:
    #     print(infer_client.get_state())
    #     time.sleep(1)
    # infer_client = TaskDetectClient()
    last_time = time.time()
    while True:
        img = cap.read()
        # img = cv2.resize(img, (128, 128))
        dets_ret = infer_client.get_infer(img[300:, 200:460])
        # dets_ret = infer_client.get_infer(img)
        # dets_ret = infer_client.get_infer(img)
        print(dets_ret)
        # for det in dets_ret:
        #     cls_id, obj_id, label, score, bbox = det[0], det[1], det[2], det[3], det[4:]
        #     rect = Bbox(box=bbox, size=[640, 480]).get_rect()
        #     print(rect)
        #     cv2.rectangle(img, rect[0:2], rect[2:4], (255, 0, 0), 2)
        
        # response = task_det_client.get_infer(img)
        cv2.imshow("img", img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        # print(response)
        fps = 1 / (time.time() - last_time)
        last_time = time.time()
        print("fps:", fps)
    cap.close()
    cv2.destroyAllWindows()

def stop_process(py_str):
    py_lists = get_python_processes()
    print(py_lists)
    for py_procees in py_lists:
        pid_id, py_name = py_procees[0], py_procees[1]
        # print(pid_id, py_name)
        if py_str in py_procees[1]:
            psutil.Process(pid_id).terminate()
            print("stop", py_name)
            return


if __name__ == '__main__':
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument('--op', type=str, default="infer")
    args = args.parse_args()
    print(args)
    if args.op == "infer":
        main_client()
    if args.op == "stop":
        stop_process("infer_back_end.py")

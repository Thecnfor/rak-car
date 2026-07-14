#!/usr/bin/python
# -*- coding: utf-8 -*-
import zmq
import cv2
import numpy as np
import json
import yaml

import time, os, sys
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
    
    def __init__(self, name):
        self.configs = get_yaml('config_car.yml')['infer_cfg']
        self.name = name
        logger.info("{}连接服务器...".format(name))
        model_cfg = self.get_config(name)
        self.img_size = model_cfg['img_size']
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
    def get_zmp_client(port):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        socket.setsockopt(zmq.SNDTIMEO, 2000)
        socket.connect(f"tcp://127.0.0.1:{port}")
        return socket

    def reset_client(self):
        try:
            self.client.close()
        except Exception:
            pass
        self.client = self.get_zmp_client(self.port)

    def __call__(self, *args, **kwds):
        return self.get_infer(*args, **kwds)

    def get_state(self):
        data = bytes('ATATA', encoding='utf-8')
        try:
            self.client.send(data)
            response = self.client.recv()
        except zmq.Again:
            logger.warning("{}服务器状态探测超时".format(self.name))
            self.reset_client()
            return None
        except zmq.ZMQError as exc:
            logger.warning("{}服务器状态探测失败: {}".format(self.name, exc))
            self.reset_client()
            return None
        response = json.loads(response)
        return response

    def get_infer(self, img):
        if self.img_size is not None:
            img = cv2.resize(img, self.img_size)
        img = cv2.imencode('.jpg', img)[1].tobytes()
        data = bytes('image', encoding='utf-8') + img
        try:
            self.client.send(data)
            response = self.client.recv()
        except (zmq.Again, zmq.ZMQError) as exc:
            self.reset_client()
            raise RuntimeError("{}推理请求失败: {}".format(self.name, exc))
        response = json.loads(response)
        return response

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

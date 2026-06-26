#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import platform
import signal
from camera import Camera
import numpy as np
from vehicle import ArmBase, ScreenShow, Key4Btn, Infrared, LedLight,CarBase, Beep
from simple_pid import PID
import difflib
import cv2, math
from task_func import MyTask
from infer_cs import ClintInterface, Bbox
from ernie_bot import ErnieBotWrap, ActionPrompt, HumAttrPrompt
from tools import CountRecord, get_yaml, IndexWrap
import sys, os

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 
from log_info import logger

def sellect_program(programs, order, win_order):
    dis_str = ''
    start_index = 0
    
    start_index = order - win_order
    for i, program in enumerate(programs):
        if i < start_index:
            continue

        now = str(program)
        if i == order:
            now = '>>> ' + now
        else:
            now = str(i+1) + '.' + now
        if len(now) >= 19:
            now = now[:19]
        else:
            now = now + '\n'
        dis_str += now
        if i-start_index == 4:
            break
    return dis_str

def kill_other_python():
    import psutil
    pid_me = os.getpid()
    # logger.info("my pid ", pid_me, type(pid_me))
    python_processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if 'python' in proc.info['name'].lower() and len(proc.info['cmdline']) > 1 and len(proc.info['cmdline'][1]) < 30:
                    python_processes.append(proc.info)
            # 出现异常的时候捕获 不存在的异常，权限不足的异常， 僵尸进程
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
    for process in python_processes:
        # logger.info(f"PID: {process['pid']}, Name: {process['name']}, Cmdline: {process['cmdline']}")
        # logger.info("this", process['pid'], type(process['pid']))
        if int(process['pid']) != pid_me:
            os.kill(int(process['pid']), signal.SIGKILL)
            time.sleep(0.3)
            
def limit(value, value_range):
    return max(min(value, value_range), 0-value_range)

# 两个pid集合成一个
class PidCal2():
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        self.pid_y = PID(**cfg_pid_y)
        self.pid_angle = PID(**cfg_pid_angle)
    
    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out

class LanePidCal():
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        # y_out_limit = 0.7
        # self.pid_y = PID(5, 0, 0)
        # self.pid_y.setpoint = 0
        # self.pid_y.output_limits = (-y_out_limit, y_out_limit)
        # print(cfg_pid_y)
        # print(cfg_pid_angle)
        self.pid_y = PID(**cfg_pid_y)
        # print(self.pid_y)

        angle_out_limit = 1.5
        self.pid_angle = PID(3, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)
    
    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out
    
class DetPidCal():
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        y_out_limit = 0.7
        self.pid_y = PID(0.3, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        angle_out_limit = 1.5
        self.pid_angle = PID(2, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)
    
    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out
    

class LocatePidCal():
    def __init__(self):
        y_out_limit = 0.3
        self.pid_y = PID(0.5, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        x_out_limit = 0.3
        self.pid_x = PID(0.5, 0, 0)
        self.pid_x.setpoint = 0
        self.pid_x.output_limits = (-x_out_limit, x_out_limit)
    
    def set_target(self, x, y):
        self.pid_y.setpoint = y
        self.pid_x.setpoint = x

    def get_out(self, error_x, error_y):
        pid_y_out = self.pid_y(error_y)
        pid_x_out = self.pid_x(error_x)
        return pid_x_out, pid_y_out

class MyCar(CarBase):
    STOP_PARAM = True
    def __init__(self):
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        logger.info("my car init ok {}".format(time.time() - start_time))
        # 任务
        self.task = MyTask()
        # 显示
        self.display = ScreenShow()
        
        # 获取自己文件所在的目录路径
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = os.path.join(self.path_dir, "config_car.yml")
        # 获取配置
        cfg = get_yaml(self.yaml_path)
        # 根据配置设置sensor
        self.sensor_init(cfg)

        self.car_pid_init(cfg)
        self.ring = Beep()
        self.camera_init(cfg)
        # paddle推理初始化
        self.paddle_infer_init()
        # 文心一言分析初始化
        # self.ernie_bot_init()

        # 相关临时变量设置
        # 程序结束标志
        self._stop_flag = False
        # 按键线程结束标志
        self._end_flag = False
        self.thread_key = threading.Thread(target=self.key_thread_func)
        self.thread_key.setDaemon(True)
        self.thread_key.start()
        
        self.beep()
    
    def beep(self):
        self.ring.rings()
        time.sleep(0.2)

    def sensor_init(self, cfg):
        cfg_sensor = cfg['io']
        # print(cfg_sensor)
        self.key = Key4Btn(cfg_sensor['key'])
        self.light = LedLight(cfg_sensor['light'])
        self.left_sensor = Infrared(cfg_sensor['left_sensor'])
        self.right_sensor = Infrared(cfg_sensor['right_sensor'])
    
    def car_pid_init(self, cfg):
        # lane_pid_cfg = cfg['lane_pid']
        # self.pid_y = PID(lane_pid_cfg['y'], 0, 0)
        # self.lane_pid = LanePidCal(**cfg['lane_pid'])
        # self.det_pid = DetPidCal(**cfg['det_pid'])
        self.lane_pid = PidCal2(**cfg['lane_pid'])
        self.det_pid = PidCal2(**cfg['det_pid'])

    def camera_init(self, cfg):
        # 初始化前后摄像头设置
        self.cap_front = Camera(cfg['camera']['front'])
        # 侧面摄像头
        self.cap_side = Camera(cfg['camera']['side'])        
    '''  
    def paddle_infer_init(self):
    # 修改后端服务路径
        backend_path = "/home/jetson/workspace/vehicle_wbt/infer_cs/base/infer_back_end.py"
    
    try:
        # 启动后端服务
        command = f"cd {os.path.dirname(backend_path)} && python {os.path.basename(backend_path)}"
        # 其他初始化代码...
    except Exception as e:
        logger.error(f"初始化失败: {e}")    
    

    def ernie_bot_init(self):
        self.hum_analysis = ErnieBotWrap()
        self.hum_analysis.set_promt(str(HumAttrPrompt()))

        self.action_bot = ErnieBotWrap()
        self.action_bot.set_promt(str(ActionPrompt()))


    '''
    def paddle_infer_init(self):
        self.crusie = ClintInterface('lane')
        # 前置左右方向识别
        self.front_det = ClintInterface('front')
        # 任务识别
        self.task_det = ClintInterface('task')
        # ocr识别
        self.ocr_rec = ClintInterface('ocr')
        # 识别为None
        self.last_det = None

    def ernie_bot_init(self):
        self.hum_analysis = ErnieBotWrap()
        self.hum_analysis.set_promt(str(HumAttrPrompt()))

        self.action_bot = ErnieBotWrap()
        self.action_bot.set_promt(str(ActionPrompt()))






    @staticmethod
    def get_cfg(path):
        from yaml import load, Loader
        # 把配置文件读取到内存
        with open(path, 'r') as stream:
            yaml_dict = load(stream, Loader=Loader)
        port_list = yaml_dict['port_io']
        # 转化为int
        for port in port_list:
            port['port'] = int(port['port'])
        # print(yaml_dict)

    # 延时函数
    def delay(self, time_hold):
        start_time = time.time()
        while True:
            if self._stop_flag:
                return
            if time.time() - start_time > time_hold:
                break
            
    # 按键检测线程
    def key_thread_func(self):
        while True:
            if not self._stop_flag:
                if self._end_flag:
                    return
                key_val = self.key.get_key()
                # print(key_val)
                if key_val == 3:
                    self._stop_flag = True
                time.sleep(0.2)
    
    
    # 根据某个值获取列表中匹配的结果
    @staticmethod
    def get_list_by_val(list, index, val):
        for det in list:
            if det[index] == val:
                return det
        return None
    
    def move_base(self, sp, end_fuction, stop=STOP_PARAM):
        self.set_velocity(sp[0], sp[1], sp[2])
        while True:
            if self._stop_flag:
                return
            if end_fuction():
                break
            self.set_velocity(sp[0], sp[1], sp[2])
        if stop:
            self.set_velocity(0, 0, 0)


    #  高级移动，按着给定速度进行移动，直到满足条件
    def move_advance(self, sp, value_h=None, value_l=None, times=1, sides=1, dis_out=0.2, stop=STOP_PARAM):
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        _sensor_usr = self.left_sensor
        if sides == -1:
            _sensor_usr = self.right_sensor
        # 用于检测开始过渡部分的标记
        flag_start = False
        def end_fuction():
            nonlocal flag_start
            val_sensor = _sensor_usr.read()
            # print("val:", val_sensor)
            if val_sensor < value_h and val_sensor > value_l:
                return flag_start
            else:
                flag_start = True
                return False
        for i in range(times):
            self.move_base(sp, end_fuction, stop=False)
        if stop:
            self.stop()

    
    def move_time(self, sp, dur_time=1, stop=STOP_PARAM):
        end_time = time.time() + dur_time
        end_func = lambda: time.time() > end_time
        self.move_base(sp, end_func, stop)

    def move_distance(self, sp, dis=0.1, stop=STOP_PARAM):
        end_dis = self.get_dis_traveled() + dis
        end_func = lambda: self.get_dis_traveled() > end_dis
        self.move_base(sp, end_func, stop)

    # 计算两个坐标的距离
    def calculation_dis(self, pos_dst, pos_src):
        return math.sqrt((pos_dst[0] - pos_src[0])**2 + (pos_dst[1] - pos_src[1])**2)
    
    def det2pose(self, det, w_r=0.06): 
        # r 真实  v 成像  f 焦点
        # rf 真实到焦点的距离  vf 相到焦点的距离
        vf_dis = 1.445
        x_v, y_v, w_v, h_v = det
        
        rf_dis = vf_dis * w_r / w_v
        x_r = x_v * rf_dis / vf_dis
        y_r = y_v * rf_dis / vf_dis
        return x_r, y_r, rf_dis
    
   
        
        
        
    def lane_det_location_v4(self, speed, pt_tar=[0, 1, 'pedestrian', 0, -0.15, -0.48, 0.24, 0.82], dis_out=0.4, side=-1,
                             det='task'):
        """
        将圆柱微调的误差阈值做了修改
        """
        infer = self.task_det
        if det != "task":
            infer = self.mot_hum
        # pid_x = PID(0.5, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        # pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.02, 0.02))
        # pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.02, 0.02))
        pid_x = PID(0.4, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.15, 0.15))
        pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.08, 0.08))
        _start_x, _start_y, _start_omage = self.get_odometry()  # 用来计算距离
    
        # 用于相同记录结果的计数类
        x_count = CountRecord(1)  ##################################
        y_count = CountRecord(1)
        w_count = CountRecord(1)
        back_count=CountRecord(4)
    
        out_x = speed
        out_y = 0
        # 坐标位置error转换相对位置
        error_adjust = np.array([-1, 1, 1, 1])
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])
    
        # 此时设置相对初始位置
        # self.set_pos_relative()
        find_tar = False
        # 类别, id, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_cls, tar_id, tar_label, tar_score, tar_bbox = pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]
        flag_location = False
        while True:
            if self._stop_flag:
                return
            _pos_x, _pos_y, _pos_omage = self.get_odometry()  # 用来计算距离
            _move_x,_move_y,_move_omage=_pos_x - _start_x, _pos_y - _start_y, _pos_omage - _start_omage
            #print('偏移量',_move_x,_move_y,_move_omage)
            if abs(_move_x) > dis_out or abs(_move_y) > dis_out:
                if not find_tar:
                    print("\n","task location dis out","\n")
                    break
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)
            dets_ret.sort(key=lambda x: (x[4] - tar_bbox[0]) ** 2 + (x[5] - tar_bbox[1]) ** 2)
            #print(dets_ret)
            # det = self.get_list_by_val_v1(dets_ret, 2, tar_label)
    
            # 如果没有，就重新获取
            if len(dets_ret) > 0 and dets_ret[0][0] == pt_tar[0]:
                det = dets_ret[0]
                find_tar = True
                # 结果分解
                det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                # 计算偏差, 并进行偏差转换为输入pid的输入值
                bbox_error = ((np.array(det_bbox) - np.array(tar_bbox)) * error_adjust).tolist()
                # 离得远时. ywh值进行滤波为0，最终仅使用了w的值
                if abs(bbox_error[0]) > 0.1:
                    bbox_error[1] = 0
                    bbox_error[2] = 0
                    bbox_error[3] = 0
                out_x = pid_x(bbox_error[0])
                #out_x=-0.2
                if abs(out_x+speed)<0.01:
                    print("能计数")
                flag_back=back_count(abs(out_x+speed)<0.001)
                if flag_back:
                    print("条件成立")
                    break
                # out_y = pid_y(bbox_error[1])
                out_y = pid_w(bbox_error[2])
                # 检测偏差值连续小于阈值时，跳出循环
                # print(bbox_error)
                # flag_x = x_count(abs(bbox_error[0]) < 0.02)######################
                flag_x = x_count(abs(bbox_error[0]) < 0.025)
                flag_y = y_count(abs(bbox_error[2]) < 0.03)
                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                flag_y = True
                if flag_x and flag_y:
                    print("location ok")
                    flag_location = True
                    self.beep()
                    break
    
                print(
                    "error_x:{:.2f}, error_y:{:.2f}, out_x:{:.2f}, out_y:{:.2f}".format(bbox_error[0], bbox_error[2], out_x,
                                                                                        out_y))
            else:
                x_count(False)
                y_count(False)
                back_count(False)
            self.set_velocity(out_x, out_y, 0)
        # 停止
        self.set_velocity(0, 0, 0)
        return flag_location,abs(_move_x)
    
    
    def lane_det_location_vert(self, speed, pt_tar=[0, 1, 'pedestrian', 0, -0.15, -0.48, 0.24, 0.82], dis_out=0.4,
                               side=-1, det='task'):
        """
        将圆柱微调的误差阈值做了修改
        """
        infer = self.task_det
        if det != "task":
            infer = self.mot_hum
        # pid_x = PID(0.5, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        # pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.02, 0.02))
        # pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.02, 0.02))
        pid_x = PID(0.4, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.15, 0.15))
        pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.08, 0.08))
        _start_x, _start_y, _start_omage = self.get_odometry()  # 用来计算距离
    
        # 用于相同记录结果的计数类
        x_count = CountRecord(2)  ##################################
        y_count = CountRecord(2)
        w_count = CountRecord(2)
    
        out_x = speed
        out_y = 0
        # 坐标位置error转换相对位置
        error_adjust = np.array([-1, 1, 1, 1])
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])
    
        # 此时设置相对初始位置
        # self.set_pos_relative()
        find_tar = False
        # 类别, id, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_cls, tar_id, tar_label, tar_score, tar_bbox = pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]
        print(f"{pt_tar}\n\n")
        flag_location = False
        while True:
            if self._stop_flag:
                return
            _pos_x, _pos_y, _pos_omage = self.get_odometry()  # 用来计算距离
            _move_x, _move_y, _move_omage = _pos_x - _start_x, _pos_y - _start_y, _pos_omage - _start_omage
            if abs(_move_x) > dis_out or abs(_move_y) > dis_out:
                if not find_tar:
                    print("task location dis out")
                    break
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)
            dets_ret.sort(key=lambda x: (x[4] - tar_bbox[0])**2)
            print(dets_ret,len(dets_ret))
            # det = self.get_list_by_val_v1(dets_ret, 2, tar_label)
    
            # 如果没有，就重新获取
            if len(dets_ret) > 1 and (dets_ret[0][0] == pt_tar[0] or dets_ret[1][0] == pt_tar[0]):
                det = dets_ret[0]
                find_tar = True
                # 结果分解
                det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                # 计算偏差, 并进行偏差转换为输入pid的输入值
                bbox_error = ((np.array(det_bbox) - np.array(tar_bbox)) * error_adjust).tolist()
                # 离得远时. ywh值进行滤波为0，最终仅使用了w的值
                # if abs(bbox_error[0]) > 0.1:
                # bbox_error[1] = 0
                # bbox_error[2] = 0
                # bbox_error[3] = 0
                out_x = pid_x(bbox_error[0])
                # out_y = pid_y(bbox_error[1])
                out_y = pid_w(bbox_error[2])
                # 检测偏差值连续小于阈值时，跳出循环
                # print(bbox_error)
                # flag_x = x_count(abs(bbox_error[0]) < 0.02)######################
                flag_x = x_count(abs(bbox_error[0]) < 0.025)  #
                print(f"x:abs(bbox_error[0])={abs(bbox_error[0])}")
                flag_y = y_count(abs(bbox_error[2]) < 0.025)  #bbox_error[2]) < 0.025
                print(f"y:abs(bbox_error[2])={abs(bbox_error[2])}")
                
                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                flag_y = True
                if flag_x and flag_y:
                    print("location ok")
                    flag_location = True
                    self.beep()
                    break
    
                print("error_x:{:.2f}, error_y:{:.2f}, out_x:{:.2f}, out_y:{:.2f}".format(bbox_error[0], bbox_error[2],
                                                                                          out_x, out_y))
            else:
                x_count(False)
                y_count(False)
            self.set_velocity(out_x, out_y, 0)
            print(f"out_y={out_y}\n\n")
        # 停止
        self.set_velocity(0, 0, 0)
        return flag_location,abs(_move_x)
        
        
        
        
        
        
        
   
           
           

    
    
    def lane_det_location_v8_multi(self, speed, targets, dis_out=0.5, side=-1):    # dis_out=0.4
        """
        改造为支持识别二维目标列表中的任意一个目标
        targets: 二维列表，例如 [[cls, id, label, score, x, y, w, h], ...]
        """
        infer = self.task_det
        pose_dict = {}
        # PID参数
        pid_x = PID(0.4, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.15, 0.15))
        pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.08, 0.08))

        # 计数器
        x_count = CountRecord(1)
        y_count = CountRecord(1)
        w_count = CountRecord(1)

        # 调整方向
        error_adjust = np.array([-1, 1, 1, 1])
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])

        # 拷贝一份targets用于操作
        remaining_targets = [list(t) for t in targets]

        while True:
            if self._stop_flag:
                return

            _pos_x, _pos_y, _pos_omega = self.get_odometry()
            print(_pos_x,_pos_y)
            
            # 超出范围返回失败
            if abs(_pos_x) > dis_out or abs(_pos_y) > dis_out:
                self.set_velocity(0, 0, 0)
                if not remaining_targets: 
                    logger.info("All targets located successfully.")
                    return pose_dict ######
                else:
                    logger.info("Out of range while locating targets.")
                    return False

            img_side = self.cap_side.read()
            dets_ret = infer(img_side)

            # 检查是否有目标被识别到
            found = False
            matched_target = None
            if len(dets_ret) > 0:
                # 按照误差排序，取最近的一个检测框
                dets_ret.sort(key=lambda x: x[4]**2 + x[5]**2)
                closest_det = dets_ret[0]
                det_cls = closest_det[0]

                # 查找 remaining_targets 中是否有同类别目标
                for tar in remaining_targets:
                    if tar[0] == det_cls:
                        found = True
                        matched_target = tar
                        break

                if found:
                    print("Found target:", closest_det)

                    # 分解目标信息
                    tar_cls, tar_id, tar_label, tar_score, tar_bbox = matched_target[0], matched_target[1], matched_target[2], matched_target[3], matched_target[4:]
                    det_bbox = closest_det[4:]

                    # 计算偏差
                    bbox_error = ((np.array(det_bbox) - np.array(tar_bbox)) * error_adjust).tolist()

                    # 如果太远，只使用bbox_error[0]进行控制
                    if abs(bbox_error[0]) > 0.1:
                        bbox_error[1] = 0
                        bbox_error[2] = 0
                        bbox_error[3] = 0

                    # PID控制
                    out_x = pid_x(bbox_error[0])
                    out_y = pid_w(bbox_error[2])

                    # 判断是否稳定
                    flag_x = x_count(abs(bbox_error[0]) < 0.025)
                    flag_y = y_count(abs(bbox_error[2]) < 0.032)   #########0.03

                    if flag_x:
                        out_x = 0
                    if flag_y:
                        out_y = 0

                    # 如果都稳定了，移除目标
                    if flag_x and flag_y:
                        print("Target located successfully:", closest_det)
                        remaining_targets.remove(matched_target)
                        # 定位完后,记录当前位置
                        pose_dict[tar_id] =self.get_odometry().copy()
                        # 如果所有目标已完成
                        if not remaining_targets:
                            print("All targets located.")
                            return pose_dict

                    self.set_velocity(out_x, out_y, 0)

                else:
                    # 没有识别到目标
                    x_count(False)
                    y_count(False)
                    self.set_velocity(speed, 0, 0)  # 向前移动寻找目标

           
    
    def lane_det_location_plant(self, speed, targets, dis_out=0.4, side=-1, confidence_threshold=0.80):
        """
        改造为支持识别二维目标列表中的任意一个目标
        只要识别到targets中的任意一个目标就停止
        targets: 二维列表，例如 [[cls, id, label, score, x, y, w, h], ...]
        """
        infer = self.task_det
        pose_dict = {}
        _start_x, _start_y, _start_omage = self.get_odometry()  # 用来计算距离
        
        # PID参数
        pid_x = PID(0.4, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.15, 0.15))
        pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.08, 0.08))
        
        # pid_x = PID(0.2, 0.01, 0.1, setpoint=0, output_limits=(-speed*0.8, speed*0.8))  # 降低P，增加D
        # pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.15, 0.15))
        # pid_w = PID(0.6, 0.005, 0.08, setpoint=0, output_limits=(-0.06, 0.06))  # 降低输出限制
        
        # 计数器
        x_count = CountRecord(1)
        y_count = CountRecord(1)
        w_count = CountRecord(1)
    
        # 调整方向
        error_adjust = np.array([-1, 1, 1, 1])
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])
    
        # 拷贝一份targets用于操作
        remaining_targets = [list(t) for t in targets]
        
        # 新增：记录已找到的目标
        found_targets = []
    
        while True:
            if self._stop_flag:
                return
    
            _pos_x, _pos_y, _pos_omage = self.get_odometry()
            print(_pos_x, _pos_y)
            
            # 超出范围返回失败
            _move_x, _move_y, _move_omage = _pos_x - _start_x, _pos_y - _start_y, _pos_omage - _start_omage
            if abs(_move_x) > dis_out or abs(_move_y) > dis_out:
                self.set_velocity(0, 0, 0)
                if found_targets: 
                    logger.info(f"Found {len(found_targets)} targets before out of range.")
                    return pose_dict  # 返回已找到的目标
                else:
                    logger.info("Out of range without finding any target.")
                    return False
    
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)
    
            # 检查是否有目标被识别到
            found = False
            matched_target = None
            if len(dets_ret) > 0:
                # 按照误差排序，取最近的一个检测框
                dets_ret.sort(key=lambda x: x[4]**2 + x[5]**2)
                closest_det = dets_ret[0]
                det_cls = closest_det[0]
                det_confidence = closest_det[3]  # 获取置信度
                det_label = closest_det[2]
                
                
                
                if det_confidence < confidence_threshold:
                    # 置信度不够，视为未识别到目标
                    logger.info(f"Low confidence: {det_confidence:.3f},seen:{det_label}, skipping")
                    x_count(False)
                    y_count(False)
                    self.set_velocity(speed, 0, 0)
                    continue  # 跳过后续处理
                
    
                # 查找 remaining_targets 中是否有同类别目标
                for tar in remaining_targets:
                    if tar[0] == det_cls:
                        found = True
                        matched_target = tar
                        break
    
                if found:
                    print("Found target:", closest_det)
    
                    # 分解目标信息
                    tar_cls, tar_id, tar_label, tar_score, tar_bbox = matched_target[0], matched_target[1], matched_target[2], matched_target[3], matched_target[4:]
                    det_bbox = closest_det[4:]
    
                    # 计算偏差
                    bbox_error = ((np.array(det_bbox) - np.array(tar_bbox)) * error_adjust).tolist()
    
                    # 如果太远，只使用bbox_error[0]进行控制
                    if abs(bbox_error[0]) > 0.1:
                        bbox_error[1] = 0
                        bbox_error[2] = 0
                        bbox_error[3] = 0
    
                    # PID控制
                    out_x = pid_x(bbox_error[0])
                    out_y = pid_w(bbox_error[2])
    
                    # 判断是否稳定
                    flag_x = x_count(abs(bbox_error[0]) < 0.025)   #########0.025
                    flag_y = y_count(abs(bbox_error[2]) < 0.032)   #########0.032
    
                    if flag_x:
                        out_x = 0
                    if flag_y:
                        out_y = 0
    
                    # 如果都稳定了，记录目标并立即返回
                    if flag_x and flag_y:
                        print("Target located successfully:", closest_det)
                        remaining_targets.remove(matched_target)
                        found_targets.append(matched_target)
                        
                        # 定位完后,记录当前位置
                        pose_dict[tar_id] = self.get_odometry().copy()
                        
                        # 关键修改：只要找到一个目标就立即返回
                        logger.info(f"Found target {tar_id}, stopping immediately.")
                        self.set_velocity(0, 0, 0)  # 停止运动
                        return pose_dict  # 立即返回找到的目标
    
                    self.set_velocity(out_x, out_y, 0)
                    
                else:
                    # 没有识别到目标
                    x_count(False)
                    y_count(False)
                    self.set_velocity(speed, 0, 0)  # 向前移动寻找目标
            else:
                # 没有检测到任何目标，继续向前移动
                x_count(False)
                y_count(False)
                self.set_velocity(speed, 0, 0)
    
    
           
           
           
           
            
    def lane_base(self, speed, end_fuction, stop=STOP_PARAM):
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            error_y, error_angle = self.crusie(image)
            y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
            # speed_dy, angle_speed = process(image)
            self.set_velocity(speed, y_speed, angle_speed)
            if end_fuction():
                break
        if stop:
            self.stop()

    def lane_det_base(self, speed, end_fuction, stop=STOP_PARAM):
        # 初始化速度和角度速度
        y_speed = 0
        angle_speed = 0
        w_r=0.06
        # 无限循环
        while True:
            # 读取前摄像头图像
            image = self.cap_front.read()
            dets_ret = self.front_det(image)
            # 此处检测简单不需要排序
            # dets_ret.sort(key=lambda x: x[4]**2 + (x[5])**2)
            if len(dets_ret)>0:
                det = dets_ret[0]
                det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                _x, _y, _dis = self.det2pose(det_bbox, w_r)
                # error_y = det_bbox[0]
                # dis_x = 1 - det_bbox[1]
                if end_fuction(_dis):
                    break
                error_angle = _x /_dis
                y_speed, angle_speed = self.det_pid.get_out(_x, error_angle)
                # print("_x:{:.2}, _angle:{:.2}, y_vel:{:.2}, angle_vel:{:.2}, dis{:.2}".format(_x, error_angle, y_speed, angle_speed, _dis))
            self.set_velocity(speed, y_speed, angle_speed)
            # if end_fuction(0):
            #     break
        if stop:
            self.stop()
            
    def lane_det_time(self, speed, time_dur, stop=STOP_PARAM):
        time_end = time.time() + time_dur
        end_fuction = lambda x: time.time() > time_end
        self.lane_det_base(speed, end_fuction, stop=stop)

    def lane_det_dis2pt(self, speed, dis_end, stop=STOP_PARAM):
        # lambda定义endfunction
        end_fuction = lambda x: x < dis_end and x != 0
        self.lane_det_base(speed, end_fuction, stop=stop)

    def lane_time(self, speed, time_dur, stop=STOP_PARAM):
        time_end = time.time() + time_dur
        end_fuction = lambda: time.time() > time_end
        self.lane_base(speed, end_fuction, stop=stop)
    
    # 巡航一段路程
    def lane_dis(self, speed, dis_end, stop=STOP_PARAM):
        # lambda重新endfunction
        end_fuction = lambda: self.get_dis_traveled() > dis_end
        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis_offset(self, speed, dis_hold, stop=STOP_PARAM):
        dis_start = self.get_dis_traveled()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, stop=stop)

    def lane_sensor(self, speed, value_h=None, value_l=None, dis_offset=0.0, times=1, sides=1, stop=STOP_PARAM):
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        _sensor_usr = self.left_sensor
        if sides == -1:
            _sensor_usr = self.right_sensor
        # 用于检测开始过渡部分的标记
        flag_start = False
        def end_fuction():
            nonlocal flag_start
            val_sensor = _sensor_usr.read()
            # print("val:", val_sensor)
            if val_sensor < value_h and val_sensor > value_l:
                return flag_start
            else:
                flag_start = True
                return False

        for i in range(times):
            self.lane_base(speed, end_fuction, stop=False)
        # 根据需要是否巡航
        self.lane_dis_offset(speed, dis_offset, stop=stop)

    def get_card_side(self):
        # 检测卡片左右指示
        count_side = CountRecord(3)
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            dets_ret = self.front_det(image)
            if len(dets_ret) == 0:
                count_side(-1)
                continue
            det = dets_ret[0]
            print(det)
            det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
            # 联系检测超过3次
            if count_side(det_label):
                if det_label == 'turn_right':
                    return -1
                elif det_label == 'turn_left':
                    return 1
                    
    def ocr(image):
        def get_access_token():
            """
            使用 AK，SK 生成鉴权签名（Access Token）
            :return: access_token，或是None(如果错误)
            """
            url = "https://aip.baidubce.com/oauth/2.0/token"
            
            API_KEY = "js7RZ6BHSIKygBpUp990PNyq"
            SECRET_KEY = "vSxUjmCB4UIaHaPHd5pH0vZG4N7T88g6"
            params = {"grant_type": "client_credentials", "client_id": API_KEY, "client_secret": SECRET_KEY}
            result = requests.post(url, params=params).json()
            return result.get("access_token")
        
        
        import cv2
        import base64
        import numpy as np
        import requests
        # 方法 1：强制连续内存（直接处理 ndarray）
        # image = np.ascontiguousarray(image)
        # img_base64 = base64.b64encode(image).decode('utf-8')
    
        # 方法 2：推荐！用 cv2.imencode 转为字节流
        success, img_encoded = cv2.imencode('.jpg', image)
        if not success:
            raise ValueError("图像编码失败")
        img_base64 = base64.b64encode(img_encoded).decode('utf-8')
    
        # 调用百度 OCR API
        url = "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate_basic?access_token=" + get_access_token()
        payload = {
            'image': img_base64,
            'detect_direction': 'false',
        }
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        
        response = requests.post(url, headers=headers, data=payload)
        result = response.json()
        
        if 'words_result' in result:
            return ''.join(item['words'] for item in result['words_result'])
        else:
            raise ValueError("OCR 识别失败")
    
    
    
    def get_ocr_list_plus(self, time_out=10):
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(1)
        text_out = None
        while True:
            texts=[]
            if self._stop_flag:
                return
            if time.time() > time_stop:
                return None
            img = self.cap_side.read()
            response = self.task_det(img)
            # 升序排序（从小到大）
            response.sort(key=lambda det: det[5])
            if len(response) > 0:
                for det in response:
                    det_cls_id, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                    if det_cls_id == 16:
                        x1, y1, w, h = det_bbox
                        x1 = img.shape[1] * (1+x1) / 2 - img.shape[1] * w / 4
                        x2 = x1 + img.shape[1] * w / 2
                        y1 = img.shape[0] * (1+y1) / 2 - img.shape[0] * w / 4
                        y2 = y1 + img.shape[0] * h / 2
                        x1 = 0 if x1 < 0 else int(x1)
                        x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                        y1 = 0 if y1 < 0 else int(y1)
                        y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                        print(y1,y2, x1,x2)
                        img_txt = img[y1:y2, x1:x2]
                        # text = self.ocr_rec(img_txt)
                        try:
                            text=MyCar.ocr(img_txt)
                        except:
                            continue
                        # print(text)
                        texts.append(text)
                        
                texts_str= "".join(texts)
                if text_out==None:
                    text_out = texts_str

                return texts
                
                
                
    def get_ocr_list_plus(self, time_out=10):
        time_stop = time.time() + time_out
        text_count = CountRecord(1)
        text_out = None
        while True:
            texts = []
            if self._stop_flag:
                return
            if time.time() > time_stop:
                return None
            img = self.cap_side.read()
            response = self.task_det(img)
            response.sort(key=lambda det: det[5])
            
            # 标志位，标记当前帧是否发生错误
            frame_error = False
    
            if len(response) > 0:
                for det in response:
                    det_cls_id, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                    if det_cls_id == 16:
                        x1, y1, w, h = det_bbox
                        x1 = img.shape[1] * (1 + x1) / 2 - img.shape[1] * w / 4
                        x2 = x1 + img.shape[1] * w / 2
                        y1 = img.shape[0] * (1 + y1) / 2 - img.shape[0] * w / 4
                        y2 = y1 + img.shape[0] * h / 2
                        x1 = 0 if x1 < 0 else int(x1)
                        x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                        y1 = 0 if y1 < 0 else int(y1)
                        y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                        img_txt = img[y1:y2, x1:x2]
    
                        try:
                            text = MyCar.ocr(img_txt)
                            texts.append(text)
                        except:
                            # 捕获异常后标记帧出错，并跳出 for 循环
                            print('ocr识别错误！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！')
                            frame_error = True
                            break  # 跳出 for 循环
    
                        
    
                # 如果帧出错，跳过后续处理，直接进入下一轮 while 循环
                if frame_error:
                    continue
    
                texts_str = "".join(texts)
                if text_out is None:
                    text_out = texts_str
    
                return texts
                        

    
    def get_ocr_list(self, time_out=10):
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(2)
        text_out = None
        while True:
            texts=[]
            if self._stop_flag:
                return
            if time.time() > time_stop:
                return None
            img = self.cap_side.read()
            response = self.task_det(img)
            # 升序排序（从小到大）
            response.sort(key=lambda det: det[5])
            if len(response) > 0:
                for det in response:
                    det_cls_id, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                    if det_cls_id == 16:
                        x1, y1, w, h = det_bbox
                        x1 = img.shape[1] * (1+x1) / 2 - img.shape[1] * w / 4
                        x2 = x1 + img.shape[1] * w / 2
                        y1 = img.shape[0] * (1+y1) / 2 - img.shape[0] * w / 4
                        y2 = y1 + img.shape[0] * h / 2
                        x1 = 0 if x1 < 0 else int(x1)
                        x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                        y1 = 0 if y1 < 0 else int(y1)
                        y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                        print(y1,y2, x1,x2)
                        img_txt = img[y1:y2, x1:x2]
                        print(y1,y2, x1,x2)
                        text = self.ocr_rec(img_txt)
                        print(text)
                        texts.append(text)
                        
                texts_str= "".join(texts)
                if text_out==None:
                    text_out = texts_str
                else:
                    # 文本相似度比较
                    matcher = difflib.SequenceMatcher(None, text_out, texts_str).ratio()
                    if text_count(matcher > 0.6):
                        return texts
    
    
    



    

    def get_ocr(self, time_out=3):
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(3)
        text_out = None
        while True:
            if self._stop_flag:
                return
            if time.time() > time_stop:
                return None
            img = self.cap_side.read()
            response = self.task_det(img)
            #print(response)
            if len(response) > 0:
                for det in response:
                    det_cls_id, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                    if det_cls_id == 9:
                        x1, y1, w, h = det_bbox
                        # print(img.shape)
                        # print(x1, y1, w, h)
                        x1 = img.shape[1] * (1+x1) / 2 - img.shape[1] * w / 4
                        x2 = x1 + img.shape[1] * w / 2
                        y1 = img.shape[0] * (1+y1) / 2 - img.shape[0] * w / 4
                        y2 = y1 + img.shape[0] * h / 2
                        x1 = 0 if x1 < 0 else int(x1)
                        x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                        y1 = 0 if y1 < 0 else int(y1)
                        y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                        img_txt = img[y1:y2, x1:x2]
                        text = self.ocr_rec(img_txt)
                        if text_out==None:
                            text_out = text
                        else:
                            # 文本相似度比较
                            matcher = difflib.SequenceMatcher(None, text_out, text).ratio()
                            if text_count(matcher > 0.6):
                                return text_out
                            else:
                                text_out = text
                            # if matcher > 0.85:
                            #     text_count(T)
                        # print(text)
                        # print(res.bbox)
                        # print(text)
                        # if text_count(text):
                        #     return text
            
    def yiyan_get_humattr(self, text):
        return self.hum_analysis.get_res_json(text)
    
    def yiyan_get_actions(self, text):
        return self.action_bot.get_res_json(text)
    
    def debug(self):
        # self.arm.arm_init()
        # self.set_xyz_relative(0, 100, 60, 0.5)
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            res = self.crusie(image)
            det_front = self.front_det(image)
            error = res[0]
            angle = res[1]
            image = self.cap_side.read()
            det_task = self.task_det(image)
            # det_hum = self.mot_hum(image)
            
            logger.info("")
            logger.info("--------------")
            logger.info("error:{} angle{}".format(error, angle))
            logger.info("front:{}".format(det_front))
            det_task.sort(key=lambda x: (x[4])**2 + (x[5])**2)
            logger.info("task:{}".format(det_task))
            # logger.inf
            if len(det_task) > 0:
                for det in det_task:
                    
                    dis = self.det2pose(det[4:])
                    logger.info("det:{} dis:{}".format(det, dis))
                # logger.info("hum_det:{}".format(det_hum))
                # logger.info("left:{} right:{}".format(self.left_sensor.read(), self.right_sensor.read()))
                # self.delay(0.5)
            # self.det2pose(det_task[4:])
            # logger.info("hum_det:{}".format(det_hum))
            logger.info("left:{} right:{}".format(self.left_sensor.read(), self.right_sensor.read()))
            self.delay(1)

    def walk_lane_test(self):
        end_function = lambda: True
        self.lane_base(0.3, end_function, stop=self.STOP_PARAM)

    def close(self):
        self._stop_flag = False
        self._end_flag = True
        self.thread_key.join()
        self.cap_front.close()
        self.cap_side.close()
        # self.grap_cam.close()

    def manage(self, programs_list:list, order_index=0):

        def all_task():
            time.sleep(4)
            for func in programs_list:
                func()
        
        def lane_test():
            self.lane_dis_offset(0.3, 30)

        programs_suffix = [all_task, lane_test, self.task.arm.reset, self.debug]
        programs = programs_list.copy()
        programs.extend(programs_suffix)
        # print(programs)
        # 选中的python脚本序号
        # 当前选中的序号
        win_num = 5
        win_order = 0
        # 把programs的函数名转字符串
        logger.info(order_index)
        programs_str = [str(i.__name__) for i in programs]
        logger.info(programs_str)
        dis_str = sellect_program(programs_str, order_index, win_order)
        self.display.show(dis_str)

        self.stop()
        run_flag = False
        stop_flag = False
        stop_count = 0
        while True:
            # self.button_all.event()
            btn = self.key.get_key()
            # 短按1=1,2=2,3=3,4=4
            # 长按1=5,2=6,3=7,4=8
            # logger.info(btn)
            # button_num = car.button_all.clicked()
            
            if btn != 0:
                # logger.info(btn)
                # 长按1按键，退出
                if btn == 5:
                    # run_flag = True
                    self._stop_flag = True
                    self._end_flag = True
                    break
                else:
                    if btn == 4:
                        # 序号减1
                        self.beep()
                        if order_index == 0:
                            order_index = len(programs)-1
                            win_order = win_num-1
                        else:
                            order_index -= 1
                            if win_order > 0:
                                win_order -= 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 2:
                        self.beep()
                        # 序号加1
                        if order_index == len(programs)-1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num-1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 3:
                        # 确定执行
                        # 调用别的程序
                        dis_str = "\n{} running......\n".format(str(programs_str[order_index]))
                        self.display.show(dis_str)
                        self.beep()
                        self._stop_flag = False
                        programs[order_index]()
                        self._stop_flag = True
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.stop()
                        self.beep()

                        # 自动跳转下一条
                        # if order_index == len(programs)-1:
                        #     order_index = 0
                        #     win_order = 0
                        # else:
                        #     order_index += 1
                        #     if len(programs) < win_num:
                        #         win_num = len(programs)
                        #     if win_order != win_num-1:
                        #         win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)
                    logger.info(programs_str[order_index])
            else:
                self.delay(0.02)
                
            time.sleep(0.02)

        for i in range(2):
            self.beep()
            time.sleep(0.4)
        time.sleep(0.1)
        self.close()

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    time.sleep(0.4)
    # my_car.task.get_ingredients(1, arm_set=True)

    def start_det_loc():
        det1 = [15, 60, "cylinder3", 0, 0, 0, 0.47, 0.7]
        det2 = [14, 80, "cylinder2", 0, 0, 0, 0.69, 0.7]
        det3 = [13, 100, "cylinder1", 0,  0, 0, 0.77, 0.7]          
        dets = [det1, det2, det3]
        my_car.lane_det_location(0.2, dets)

    def lane_det_test():
        my_car.lane_det_dis2pt(0.2, 0.16)

    def move_test():
        my_car.set_vel_time(0.3, 0, -0.6, 1)

    def ocr_test():
        print(my_car.get_ocr())

    my_car.manage([start_det_loc, lane_det_test, move_test, ocr_test])
    # my_car.lane_time(0.3, 5)
    
    # my_car.lane_dis_offset(0.3, 1.2)
    # my_car.lane_sensor(0.3, 0.5)
    # my_car.debug() 

    # text = "犯人没有带着眼镜，穿着短袖"
    # criminal_attr = my_car.hum_analysis.get_res_json(text)
    # print(criminal_attr)
    # my_car.task.reset()
    # pt_tar = my_car.task.punish_crimall(arm_set=True)
    # hum_attr = my_car.get_hum_attr(pt_tar)
    # print(hum_attr)
    # res_bool = my_car.compare_humattr(criminal_attr, hum_attr)
    # print(res_bool)
    # pt_tar = [0, 1, 'pedestrian',  0, 0.02, 0.4, 0.22, 0.82]
    # for i in range(4):
    #     my_car.set_pos_offset([0.07, 0, 0])
    #     my_car.lane_det_location(0.1, pt_tar, det="mot", side=-1)
    # my_car.close()
    # text = my_car.get_ocr()
    # print(text)
    # pt_tar = my_car.task.pick_up_ball(arm_set=True)
    # my_car.lane_det_location(0.1, pt_tar)
    
    my_car.close()
    # my_car.debug()
    # while True:
    #     text = my_car.get_ocr()
    #     print(text)

    # my_car.task.reset()
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.lane_time(0.3, 5)
    # my_car.debug()
    
    # my_car.debug()

            
    # my_car.task.pick_up_block()
    # my_car.task.put_down_self_block()
    # my_car.lane_time(0.2, 2)
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.task.pick_up_block()
    # my_car.close()
    # logger.info(time.time())
    # my_car.lane_task_location(0.3, 2)


    # my_car.debug()
    # programs = [func1, func2, func3, func4, func5, func6]
    # my_car.manage(programs)
    # import sys
    # test_ord = 0
    # if len(sys.argv) >= 2:
    #     test_ord = int(sys.argv[1])
    # logger.info("test:", test_ord)
    # car_test(test_ord)

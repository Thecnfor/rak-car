#!/usr/bin/python
# -*- coding: utf-8 -*-
"""运动 / 车道保持 / 目标定位 Mixin。

从 my_car.py 拆出：car_pid_init、delay、move_base / move_time / move_distance、
calculation_dis / det2pose、lane_det_location、lane_base / lane_time / lane_dis /
lane_dis_offset、move_to_detection_target、adjust_arm_position、get_list_by_val。
依赖 MyCar 聚合后提供 set_velocity / get_odometry / get_distance / streamer /
cap_side / task_det / lane_pid 等。
"""
import math
import time
from typing import Union

import cv2

from smartcar import PID, logger
from smartcar.whalesbot.tools import CountRecord, get_yaml

from .pid import PidCal2


class MotionMixin:
    """MyCar 的运动 / 车道 / 定位行为。"""

    def car_pid_init(self, cfg):
        """
        初始化PID控制器

        根据配置初始化车道保持和目标检测的PID控制器。

        参数:
            cfg: 配置字典，包含PID控制器的配置信息
        """
        # lane_pid_cfg = cfg['lane_pid']
        # self.pid_y = PID(lane_pid_cfg['y'], 0, 0)
        # self.lane_pid = LanePidCal(**cfg['lane_pid'])
        # self.det_pid = DetPidCal(**cfg['det_pid'])
        self.lane_pid = PidCal2(**cfg["lane_pid"])
        self.det_pid = PidCal2(**cfg["det_pid"])

    # 延时函数
    def delay(self, time_hold):
        """
        延时函数

        延时指定的时间，期间会检查停止标志。

        参数:
            time_hold: 延时时间（秒）
        """
        start_time = time.time()
        while True:
            if self._must_exit():
                return
            if time.time() - start_time > time_hold:
                break

    # 按键检测线程已移除（2026年）：硬件按键仅做 arm jog,不再触发 _stop_flag。
    @staticmethod
    def get_list_by_val(list, index, val):
        """
        根据某个值获取列表中匹配的结果

        参数:
            list: 要搜索的列表
            index: 要匹配的索引位置
            val: 要匹配的值

        返回:
            匹配的元素，如果没有匹配的则返回None
        """
        for det in list:
            if det[index] == val:
                return det
        return None

    def move_base(self, sp, end_fuction, stop=None):
        """
        基础移动方法

        设置车辆速度并持续移动，直到满足结束条件。

        参数:
            sp: 速度向量 [x, y, z]
            end_fuction: 结束条件函数，返回True时停止移动
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        stop = self.resolve_stop(stop)
        self.set_velocity(sp[0], sp[1], sp[2])
        while True:
            if self._must_exit():
                return
            if end_fuction():
                break
            self.set_velocity(sp[0], sp[1], sp[2])
        if stop:
            self.set_velocity(0, 0, 0)

    def move_time(self, sp, dur_time=1, stop=None):
        """
        按时间移动

        以给定速度移动指定的时间。

        参数:
            sp: 速度向量 [x, y, z]
            dur_time: 移动时间（秒），默认为1
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        stop = self.resolve_stop(stop)
        self.set_velocity_for_duration(sp[0], sp[1], sp[2], dur_time)
        if stop:
            self.stop()

    def move_distance(self, sp, dis=0.1, stop=None):
        """
        按距离移动

        以给定速度移动指定的距离。

        参数:
            sp: 速度向量 [x, y, z]
            dis: 移动距离，默认为0.1
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        stop = self.resolve_stop(stop)
        end_dis = self.get_distance() + dis

        def end_func():
            return self.get_distance() > end_dis

        self.move_base(sp, end_func, stop)

    # 计算两个坐标的距离
    def calculation_dis(self, pos_dst, pos_src):
        """
        计算两个坐标的距离

        计算两个二维坐标点之间的欧几里得距离。

        参数:
            pos_dst: 目标坐标 [x, y]
            pos_src: 源坐标 [x, y]

        返回:
            float: 两个坐标之间的距离
        """
        return math.sqrt(
            (pos_dst[0] - pos_src[0]) ** 2 + (pos_dst[1] - pos_src[1]) ** 2
        )

    def det2pose(self, det, w_r=0.06):
        """
        将检测结果转换为真实世界坐标

        根据检测结果和物体实际宽度，计算物体在真实世界中的坐标和距离。

        参数:
            det: 检测结果，包含 [x, y, w, h]（归一化坐标）
            w_r: 物体实际宽度（米），默认为0.06

        返回:
            tuple: (x坐标, y坐标, 距离)，单位为米
        """
        # r 真实  v 成像  f 焦点
        # rf 真实到焦点的距离  vf 相到焦点的距离
        vf_dis = 1.445
        x_v, y_v, w_v, h_v = det

        rf_dis = vf_dis * w_r / w_v
        x_r = x_v * rf_dis / vf_dis
        y_r = y_v * rf_dis / vf_dis
        return x_r, y_r, rf_dis

    # 侧面摄像头进行位置定位
    def lane_det_location(
        self,
        speed,
        pts_tar=[[0, 70, "text_det", 0, 0, 0, 0.70, 0.70]],
        dis_out=0.05,
        side=1,
        time_out=2,
        det="task",
    ):
        """
        侧面摄像头进行位置定位

        使用侧面摄像头检测目标并进行位置定位，通过PID控制调整车辆位置。

        参数:
            speed: 移动速度
            pts_tar: 目标点列表，每个元素包含 [id, 宽度, 标签, 置信度, x, y, w, h]
            dis_out: 距离限制，默认为0.05
            side: 方向，1为正方向，-1为反方向
            time_out: 超时时间（秒），默认为2
            det: 检测类型，默认为'task'

        返回:
            int: 目标索引，如果超时或距离超出限制则返回False
        """
        end_time = time.time() + time_out
        infer = self.task_det
        loc_pid = get_yaml(self.yaml_path)["location_pid"]  # type: ignore
        pid_x = PID(**loc_pid["pid_x"])
        pid_x.output_limits = (-speed, speed)
        pid_y = PID(**loc_pid["pid_y"])
        pid_y.output_limits = (-0.15, 0.15)
        # pid_w = PID(1.0, 0, 0.00, setpoint=0, output_limits=(-0.15, 0.15))

        # 用于相同记录结果的计数类
        x_count = CountRecord(5)
        dis_count = CountRecord(5)

        out_x = speed
        out_y = 0

        # 此时设置相对初始位置
        # self.set_pos_relative()
        # self.dis_tra_st = self.get_distance()
        x_st, y_st, _ = self.get_odometry()
        find_tar = False
        tar = []
        for pt_tar in pts_tar:
            # id, 物体宽度，置信度, 归一化bbox[x_c, y_c, w, h]
            tar_id, tar_width, tar_label, tar_score, tar_bbox = (
                pt_tar[0],
                pt_tar[1],
                pt_tar[2],
                pt_tar[3],
                pt_tar[4:],
            )
            tar_width *= 0.001
            tar_x, tar_y, tar_dis = self.det2pose(tar_bbox, tar_width)
            tar.append([tar_id, tar_width, tar_x, tar_y, tar_dis])
        # logger.info("tar x:{} dis:{}".format(tar_x, tar_dis))
        tar_id, tar_width, tar_x, tar_y, tar_dis = tar[0]
        pid_x.setpoint = tar_x
        pid_y.setpoint = tar_dis
        tar_index = 0
        flag_location = False
        while True:
            if self._must_exit():
                return
            if time.time() > end_time:
                logger.info("time out")
                self.set_velocity(0, 0, 0)
                return False
            _pos_x, _pos_y, _pos_omage = self.get_odometry()  # 用来计算距离

            if abs(_pos_x - x_st) > dis_out or abs(_pos_y - y_st) > dis_out:
                if not find_tar:
                    logger.info("task location dis out")
                    self.set_velocity(0, 0, 0)
                    return False
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)

            img_side_show = img_side.copy()
            for det in dets_ret:
                det_cls_id, det_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                x_c, y_c, w, h = det_bbox
                # 将归一化坐标转换为像素坐标
                img_h, img_w = img_side.shape[:2]
                x_c = int((x_c + 1) / 2 * img_w)
                y_c = int((y_c + 1) / 2 * img_h)
                w = int(w * img_w / 2)
                h = int(h * img_h / 2)
                x1 = int(x_c - w / 2)
                y1 = int(y_c - h / 2)
                x2 = int(x_c + w / 2)
                y2 = int(y_c + h / 2)
                # 绘制矩形框
                cv2.rectangle(img_side_show, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 绘制标签
                label_text = f"{det_label}:{det_score:.2f}"
                cv2.putText(
                    img_side_show,
                    label_text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            self.streamer.update_frame(img_side_show, "cam2")

            # dets_ret = self.mot_hum(img_side)
            # cv2.imshow("side", img_side)
            # cv2.waitKey(1)

            # 进行排序，此处排列按照自中心由近及远的顺序
            dets_ret.sort(key=lambda x: (x[4]) ** 2 + (x[5]) ** 2)
            print(dets_ret)
            # # 找到最近对应的类别，类别存在第一个位置
            # det = self.get_list_by_val(dets_ret, 2, tar_label)

            # 如果没有，就重新获取
            if len(dets_ret) > 0:
                det = dets_ret[0]
                # 结果分解
                det_id, obj_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                # if find_tar is False:
                # tar_index = 0
                # for tar_pt in tar:
                for index, tar_pt in enumerate(tar):
                    if det_id == tar_pt[0]:
                        tar_index = index
                        tar_id, tar_width, tar_x, tar_y, tar_dis = tar_pt
                        pid_x.setpoint = tar_x
                        pid_y.setpoint = tar_dis
                        find_tar = True
                        # print("find tar", tar_id)
                        break

                if det_id == tar_id:
                    _x, _y, _dis = self.det2pose(det_bbox, tar_width)
                    out_x = pid_x(_x) * side  # type: ignore
                    out_y = pid_y(_dis) * side  # pyright: ignore[reportOptionalOperand]
                    # out_y = pid_y(_dis)
                    # out_y = pid_w(bbox_error[2])
                    # 检测偏差值连续小于阈值时，跳出循环
                    # print(bbox_error)
                    # print("err x:{:.2}, dis:{:.2}, tar x:{:.2}, tar dis:{:.2}".format(_x, _dis, tar_x, tar_dis))
                    flag_x = x_count(abs(_x - tar_x) < 0.01)
                    flag_dis = dis_count(abs(_dis - tar_dis) < 0.01)
                    if flag_x:
                        out_x = 0
                    if flag_dis:
                        out_y = 0
                    if flag_x and flag_dis:
                        logger.info("location{} ok".format(tar_id))
                        # flag_location = True
                        # 停止
                        self.set_velocity(0, 0, 0)
                        return tar_index

                # print("error_x:{:.2}, error_y:{:.2}, out_x:{:.2}, out_y:{:2}".format(bbox_error[0], bbox_error[2], out_x, out_y))
            else:
                x_count(False)
                dis_count(False)
            self.set_velocity(out_x, out_y, 0)

    def lane_base(self, speed, end_fuction, stop=None):
        """
        车道保持基础方法

        使用前置摄像头进行车道检测和保持，根据检测结果调整车辆方向。

        参数:
            speed: 行驶速度
            end_fuction: 结束条件函数，返回True时停止
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        stop = self.resolve_stop(stop)
        while True:
            if self._must_exit():
                if hasattr(self.streamer, "set_lane_state"):
                    self.streamer.set_lane_state(
                        active=False,
                        mode="stopped",
                        forward_speed=0.0,
                        lateral_speed=0.0,
                        angular_speed=0.0,
                        distance=self.get_distance(),
                    )
                return

            error_y, error_angle = self.get_lane_results()
            y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
            self.set_velocity(speed, y_speed, angle_speed)
            if hasattr(self.streamer, "set_lane_state"):
                self.streamer.set_lane_state(
                    active=True,
                    mode="tracking",
                    error_y=error_y,
                    error_angle=error_angle,
                    forward_speed=speed,
                    lateral_speed=y_speed,
                    angular_speed=angle_speed,
                    distance=self.get_distance(),
                )
            if end_fuction():
                break
        if stop:
            self.stop()
        if hasattr(self.streamer, "set_lane_state"):
            self.streamer.set_lane_state(
                active=False,
                mode="idle",
                forward_speed=0.0,
                lateral_speed=0.0,
                angular_speed=0.0,
                distance=self.get_distance(),
            )

    def lane_time(self, speed, time_dur, stop=None):
        """
        车道保持定时方法

        使用前置摄像头进行车道保持，持续指定的时间。

        参数:
            speed: 行驶速度
            time_dur: 持续时间（秒）
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        stop = self.resolve_stop(stop)
        time_end = time.time() + time_dur

        def end_fuction():
            return time.time() > time_end

        self.lane_base(speed, end_fuction, stop=stop)

    # 巡航一段路程
    def lane_dis(self, speed, dis_end, stop=None):
        """
        车道保持定距方法

        使用前置摄像头进行车道保持，直到行驶距离超过指定值。

        参数:
            speed: 行驶速度
            dis_end: 目标距离
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """

        stop = self.resolve_stop(stop)
        # lambda重新endfunction
        def end_fuction():
            return self.get_distance() > dis_end

        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis_offset(self, speed, dis_hold, stop=None):
        """
        车道保持距离偏移方法

        使用前置摄像头进行车道保持，行驶指定的距离偏移量。

        参数:
            speed: 行驶速度
            dis_hold: 距离偏移量
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        stop = self.resolve_stop(stop)
        dis_start = self.get_distance()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, stop=stop)

    def move_to_detection_target(
        self,
        delta_x=0.0,
        delta_y: Union[float, None] = 0.0,
        label=None,
        time_out=2.0,
        sort_pos=(0, 0),
        num=0,
    ):
        """
        前往目标位置

        参数:
            cls_id : 指定检测目标的 cls_id，默认None为距离中心最近的目标
            time_out: 设置超时时间
            包含目标检测信息的列表，格式为 [cls_id, obj_id,label, score, x_c, y_c, w, h]
        """
        time_stop = time.time() + time_out
        x_count = CountRecord(3)
        y_count = CountRecord(3)

        # pid_x.output_limits((-0.7, 0.7))

        out_x = 0
        out_y = 0
        # print(f"手柄方向：{self.arm.side}")
        if self.arm.side == "RIGHT":
            kp_y = -0.2
            kp_x = -0.25
            ki_x = -0.05
        else:
            kp_y = 0.2
            kp_x = 0.25
            ki_x = 0.05

        pid_x = PID(kp_x, ki_x)
        pid_x.setpoint = delta_x
        while True:
            if self._must_exit():
                self.set_velocity(0, 0, 0)
                self.arm.x_speed(0)
                return -1, "None"

            # 2026-07-16 改造:读 task_state cache 替代同步 ZMQ
            # 原版每 50ms 调 get_detection_results -> cap_side.read() + self.task_det() (2s 超时),
            # 与 task_feed 30Hz 守护线程共用 self.task_det REQ socket 串行排队,卡 arm 闭环
            # 现读 streamer.task_state (task_feed 写,meta_lock 微秒级),detections 已经是
            # task_feed 过滤过的最新结果;limit_x/limit_y 默认 1 不做过滤,sort_pos 按
            # 中心距离排序保留。
            state = self.streamer.get_task_state() if hasattr(self, "streamer") and self.streamer else None
            raw_dets = (state or {}).get("detections", []) if state else []
            # 转换 dict 格式 -> 旧 tuple 格式,保持后续 PID 逻辑不变
            dets = []
            for d in raw_dets:
                bbox = d.get("bbox_norm", {}) or {}
                dets.append((
                    d.get("cls_id"),
                    d.get("det_id"),
                    d.get("label"),
                    d.get("score"),
                    bbox.get("x_center", 0.0),
                    bbox.get("y_center", 0.0),
                    bbox.get("width", 0.0),
                    bbox.get("height", 0.0),
                ))
            # 按 sort_pos 距离排序 (中心 -> 远)
            dets.sort(
                key=lambda x: (x[4] - sort_pos[0]) ** 2 + (x[5] - sort_pos[1]) ** 2
            )

            if label is not None:
                dets = [item for item in dets if item[2] == label]

            if len(dets) > num:
                det = dets[num]
                dx, dy = det[4:6]
                # print(f"dx:{dx} dy:{dy}")
                out_x = -pid_x(dx)  # type: ignore
                if delta_y is None:
                    out_y = 0
                else:
                    out_y = kp_y * (dy - delta_y)

                flag_x = x_count(abs(dx) < 0.04)
                flag_y = y_count(abs(dy) < 0.02)
                if delta_y is None:
                    flag_y = True

                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                if flag_x and flag_y:
                    # logger.info(f"location{self.get_odometry()} ok, arm_pose{self.arm.x_pose_now}")
                    self.set_velocity(0, 0, 0)
                    self.arm.x_speed(0)
                    # return det[0],det[2]
            else:
                x_count(False)
                y_count(False)
            self.set_velocity(out_x, 0, 0)
            self.arm.x_speed(out_y)
            time.sleep(0.05)

            if time.time() > time_stop:
                self.set_velocity(0, 0, 0)
                self.arm.x_speed(0)
                logger.error("对齐目标超时")
                # logger.info(f"location{self.get_odometry()} ok, arm_pose{self.arm.x_pose_now}")

                try:
                    return det[0], det[2]
                except:
                    return (None, None)

    def adjust_arm_position(self, dis=0.05):
        # print(f"arm side:{self.arm.side}")
        x_position = self.arm.x_get_position()
        if self.arm.side == "LEFT":
            self.arm.move_x_position(x_position + dis)
        elif self.arm.side == "RIGHT":
            self.arm.move_x_position(x_position - dis)

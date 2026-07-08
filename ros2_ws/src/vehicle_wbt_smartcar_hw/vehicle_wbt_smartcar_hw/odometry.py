#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Mecanum轮底盘控制模块

此模块实现了Mecanum轮底盘的完整控制功能，包括：
- 运动学计算（正逆解）
- 速度控制
- 里程计更新
- 坐标系转换
- PID位置控制

所有功能均内置，不依赖外部文件，可直接执行或集成到更大的系统中。
"""

import math
import numpy as np

# logger / WheelWrap / PID were only used by MecanumDriver (removed below);
# Odometry and MecanumChassis are pure numpy compute and need no logging.
pass  # logger via print


class Odometry:
    """
    里程计基础类

    用于计算和更新车辆的位姿信息，以及处理坐标系转换
    """

    def __init__(self):
        """
        初始化里程计

        初始位姿为 [0.0, 0.0, 0.0]，初始速度为 [0.0, 0.0, 0.0]
        """
        # x, y, theta
        self.position = np.array(
            [0.0, 0.0, 0.0], dtype=np.float32
        )  # 世界坐标系下的位姿
        # 速度
        self.velocity = np.array(
            [0.0, 0.0, 0.0], dtype=np.float32
        )  # 世界坐标系下的速度
        # 车子整体前进的路程变量
        self.distance = 0.0

    def update(self, d_vector):
        """
        更新里程计数据

        参数:
            d_vector: 车辆坐标系下的位移向量 [dx, dy, dtheta]
        """
        # 位置变化矩阵
        z_angle = self.position[2]
        d_pose_transform = np.array(
            [
                [math.cos(z_angle), math.sin(z_angle)],
                [-math.sin(z_angle), math.cos(z_angle)],
            ],
            dtype=np.float32,
        )
        # 车子坐标变化转为世界坐标变化
        d_pose_xy = np.dot(d_vector[:2], d_pose_transform)
        # 更新路程
        self.distance += float(np.sum(d_vector[:2] ** 2, keepdims=True) ** 0.5)
        # 增加角度变化量
        d_pose = np.append(d_pose_xy, values=d_vector[2]).astype(np.float64)
        # 更新世界坐标位置
        self.position += d_pose

    def reset(self, x=None, y=None, z=None, distance=None):
        """
        重置位姿为指定位置, 默认保持原有值不变
        
        参数:
            x: x轴位置，为 None 时保持原值
            y: y轴位置，为 None 时保持原值
            z: theta角度，为 None 时保持原值
            distance: 距离参数，为 None 时保持原值
        """
        # 取出当前的 x, y, z 值
        current_x, current_y, current_z = self.position
        current_distance = self.distance
        
        # 逐个判断：传入非 None 则更新，否则用原值
        new_x = x if x is not None else current_x
        new_y = y if y is not None else current_y
        new_z = z if z is not None else current_z
        # 赋值新位置
        self.distance = distance if distance is not None else current_distance
        self.position = np.array([new_x, new_y, new_z], dtype=np.float32)
        
            

    def world_to_car_velocity(self, vel_world, angle_car):
        """
        世界坐标系速度转换为车辆坐标系速度

        参数:
            vel_world: 世界坐标系下的速度向量 [vx, vy, vtheta]
            angle_car: 车辆当前角度（弧度）

        返回:
            numpy.ndarray: 车辆坐标系下的速度向量 [vx, vy, vtheta]
        """
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # 世界坐标系到车辆坐标系的转换矩阵
        transform = np.array([[cos_car, -sin_car, 0], [sin_car, cos_car, 0], [0, 0, 1]])
        vel_car = np.array(vel_world).dot(transform)
        return vel_car

    def car_to_world_velocity(self, vel_car, angle_car):
        """
        车辆坐标系速度转换为世界坐标系速度

        参数:
            vel_car: 车辆坐标系下的速度向量 [vx, vy, vtheta]
            angle_car: 车辆当前角度（弧度）

        返回:
            numpy.ndarray: 世界坐标系下的速度向量 [vx, vy, vtheta]
        """
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # 车辆坐标系到世界坐标系的转换矩阵
        transform = np.array([[cos_car, -sin_car, 0], [sin_car, cos_car, 0], [0, 0, 1]])
        vel_world = np.array(vel_car).dot(transform)
        return vel_world


class MecanumChassis:
    """
    麦克纳姆轮底盘类

    轮子布局：
        [2]**[1]
        ********
        ********
        [3]**[4]

    从上方往下方看是x形排布, 轮子接触地面是O形排布
    轮子速度定义为轮子顺时针转动为正
    """

    def __init__(self, track=0.30, wheel_base=0.28, wheel_radius=0.03):
        """
        初始化麦克纳姆轮底盘

        参数:
            track: 轮距
            wheel_base: 轴距
            wheel_radius: 轮子半径
        """
        # 初始化里程计
        self.odometry = Odometry()
        # 轮距 轴距
        self.half_wheel_base = wheel_base / 2
        self.half_track = track / 2
        self.wheel_radius = wheel_radius
        self.init_parameters()

    def init_parameters(self):
        """
        初始化麦克纳姆轮底盘的转换矩阵
        """
        roller_angle = math.pi / 4 * 1.052
        tan_roller = math.tan(roller_angle)
        wheel_constant = self.half_track * tan_roller + self.half_wheel_base
        # 根据小车四轮运动计算小车运动，正解
        self.wheel_to_vehicle_matrix = np.array(
            [
                [1 / 4, 1 / 4 / tan_roller, 1 / wheel_constant / 4],
                [-1 / 4, 1 / 4 / tan_roller, 1 / wheel_constant / 4],
                [-1 / 4, -1 / 4 / tan_roller, 1 / wheel_constant / 4],
                [1 / 4, -1 / 4 / tan_roller, 1 / wheel_constant / 4],
            ]
        )

        # 根据小车运动计算小车四轮运动，逆解
        self.vehicle_to_wheel_matrix = np.array(
            [
                [1, -1, -1, 1],
                [tan_roller, tan_roller, -tan_roller, -tan_roller],
                [wheel_constant, wheel_constant, wheel_constant, wheel_constant],
            ]
        )

    def forward_kinematics(self, wheel_velocity: np.ndarray) -> np.ndarray:
        """
        正解计算：轮子速度 → 车辆速度

        参数:
            wheel_velocity: 轮子速度向量

        返回:
            numpy.ndarray: 车辆速度向量
        """
        return wheel_velocity @ self.wheel_to_vehicle_matrix

    def inverse_kinematics(self, car_velocity: np.ndarray) -> np.ndarray:
        """
        逆解计算：车辆速度 → 轮子速度

        参数:
            car_velocity: 车辆速度向量 [vx, vy, vtheta]

        返回:
            numpy.ndarray: 轮子速度向量
        """
        return car_velocity @ self.vehicle_to_wheel_matrix

    def calculate_wheel_velocities(self, x: float, y: float, z: float) -> np.ndarray:
        """
        计算轮子速度

        参数:
            x: x轴线速度
            y: y轴线速度
            z: theta角度

        返回:
            numpy.ndarray: 轮子线速度向量
        """
        # 计算小车每个轮子的线速度
        wheel_velocities = self.inverse_kinematics(np.array([x, y, z]))
        return wheel_velocities

    def update_odometry(self, wheel_displacements: np.ndarray):
        """
        更新里程计数据

        参数:
            wheel_displacements: 轮子位移向量
        """
        # 计算小车的位置变化
        car_displacement = self.forward_kinematics(wheel_displacements)
        # 更新小车的位姿
        self.odometry.update(car_displacement)


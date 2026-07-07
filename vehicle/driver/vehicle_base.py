#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import math, threading
import numpy as np


from threading import Thread
import yaml, os, sys

import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 
# 导入自定义log模块
from log_info import logger
from vehicle import MotorConvert, Motors, WheelWrap
from tools import PID, limit_val

# 把该文件夹目录加入环境变量
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

def get_path_relative(*args):
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)
    
class OdometryBase:
    def __init__(self) -> None:
        # x, y, theta
        self.pose = np.array([.0, .0, .0])
        self.twist = np.array([.0, .0, .0])
        # 车子整体前进的路程变量
        self.dis_traveled = 0

    # odometry update,间隔时间不宜过长
    def odom_update(self, d_vect):
        # 位置变化矩阵
        z_angle = self.pose[2]
        d_pose_transform = np.array([[math.cos(z_angle), math.sin(z_angle)], 
                                    [-math.sin(z_angle), math.cos(z_angle)]])
        # print("z_angle:", z_angle, "d_pose_transform:", d_pose_transform)
        # 车子坐标变化转为世界坐标变化
        d_pose_xy = np.dot(d_vect[:2], d_pose_transform)
        # print("vect:",d_vect, "tans:",d_pose_xy)
        # 更新路程
        self.dis_traveled += np.sum(d_vect[:2]**2, keepdims=True)**0.5
        # 增加角度变化量
        d_pose = np.append(d_pose_xy, values=d_vect[2])
        # self.twist[2] += d_pose[2]
        # 更新世界坐标位置
        self.pose += d_pose
    
    def reset(self):
        self.pose = np.array([.0, .0, .0])
        
# 底盘功能抽象
class ChassisBase:
    def __init__(self):
        self.odom = OdometryBase()
        self.wheel_radius = 0.1
    
    def params_init(self):
        # 运动正解逆解转换矩阵参数初始化,可能包含尺寸参数
        self.transform_forward = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.transform_inverse = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def get_velocity(self, linear_vx, linear_vy, angular_v):
        """
        设置小车的线速度和角速度.
        参数:
        linear_vx (float): x轴线速度,
        linear_vy (float): y轴线速度
        angular_v (float): 角速度,范围为 [-max_angular_speed, max_angular_speed]
        """
        pass

    def forward_cal(self, wheel_vel):
        return np.dot(wheel_vel, self.transform_forward)
    
    def inverse_cal(self, car_vel):
        return np.dot(car_vel, self.transform_inverse)

    def get_velocity(self, linear_vx, linear_vy, angular_v):
        # 计算小车每个轮子的线速度
        wheel_vel = self.inverse_cal(np.array([linear_vx, linear_vy, angular_v]))
        return wheel_vel
        # 计算每个小车轮子的角速度
        wheel_angular = wheel_vel / self.wheel_radius
        return wheel_angular

    def updata_odom(self, wheel_vect):
        # 计算小车的位置变化
        car_vect = self.forward_cal(wheel_vect)
        # print("car_vect:", car_vect)
        # 更新小车的位姿
        self.odom.odom_update(car_vect)

class TricycleChassis(ChassisBase):
    '''
    wheel list
        *****A*****
        ***********
        ***B***C***
    '''
    def __init__(self, raduis_base=0.1) -> None:
        super().__init__()
        # 三轮全向车轮子到中心的距离半径
        self.radius = raduis_base
        self.params_init()

    def params_init(self):
        r = self.radius
        cos_a = math.cos(math.pi/6)
        sin_a = math.sin(math.pi/6)
        # 根据小车三轮的运动计算小车运动， 正解
        self.transform_forward = np.array([[           0,  2/3, 1/(3*r)],
                                           [-1/(2*cos_a), -1/3, 1/(3*r)],
                                           [ 1/(2*cos_a), -1/3, 1/(3*r)]])
        # 根据小车运动计算小车三轮的运动， 逆解
        self.transform_inverse = np.array([[0, -cos_a,  cos_a],
                                           [1, -sin_a, -sin_a],
                                           [r,      r,      r]])
        

# 差速车定义
class Diff2Chassis(ChassisBase):
    '''
    ********
    1******2
    ********
    差速二轮序号定义, 轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, track=0.2) -> None:
        logger.info("diff init")
        super().__init__()
        # 配置文件存储路径
        # 轮距
        self.track = track
        self.params_init()
    
    def params_init(self):
        r = self.track
        # 根据小车差速二轮运动计算小车运动， 正解
        self.transform_forward = np.array([[-1/2, 0, 1/r],
                                           [ 1/2, 0, 1/r]])
    
        # 根据小车运动计算小车差速二轮运动， 逆解
        self.transform_inverse = np.array([[ -1,  1],
                                           [ 0,    0],
                                           [ r/2, r/2]])

# 差速车定义
class Diff4Chassis(ChassisBase):
    '''
    2******1
    ********
    ********
    3******4
    差速四轮序号定义, 轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, track=0.2) -> None:
        logger.info("diff4 init")
        super().__init__()
        # 配置文件存储路径
        self.track = track
        self.params_init()
    
    def params_init(self):
        r = self.track / 2
        # 根据小车差速四轮运动计算小车运动， 正解
        self.transform_forward = np.array([[ 1/4, 0, 1/4/r],
                                           [-1/4, 0, 1/4/r],
                                           [-1/4, 0, 1/4/r],
                                           [ 1/4, 0, 1/4/r]])
    
        # 根据小车运动计算小车差速四轮运动， 逆解
        self.transform_inverse = np.array([[ 1, -1, -1,  1],
                                           [ 0,  0,  0,  0],
                                           [ r,  r,  r, r]])

# 麦克纳姆轮定义
class MecanumChassis(ChassisBase):
    '''
    2******1
    ********
    ********
    3******4
    麦克纳姆轮序号定义, 从上方往下方看是x形排布, 轮子接触地面是O形排布
    轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, track=0.17, wheel_base=0.2) -> None:
        logger.info("mecanum init")
        super().__init__()
        # 轮距 轴距 
        self.ry = wheel_base / 2
        self.rx = track / 2
        self.params_init()
    
    def params_init(self):
        # 这里定义辊子安装的弧度偏移，或轮子安装时的偏移比例
        k_rolls = 1.052
        rad_rolls = math.pi/4*k_rolls
        # rad_rolls = math.pi/4
        t = math.tan(rad_rolls)
        r = self.rx*t + self.ry
        # 根据小车四轮运动计算小车运动， 正解
        self.transform_forward = np.array([[1/4,  1/4/t, 1/r/4],
                                          [-1/4,  1/4/t, 1/r/4],
                                          [-1/4, -1/4/t, 1/r/4],
                                          [ 1/4, -1/4/t, 1/r/4]])

        # 根据小车运动计算小车四轮运动， 逆解
        self.transform_inverse = np.array([[1, -1, -1,  1],
                                           [t,  t, -t, -t],
                                           [r,  r,  r,  r]])

# 四轮全向轮定义
class QuadricycleChassis(ChassisBase):
    '''
    2/****\1
    ********
    ********
    3\****/4
    四轮全向序号定义, 轮子速度定义为轮子顺时针转动为正
    '''
    def __init__(self, raduis_base=0.1115) -> None:
        logger.info("Quadricycle init")
        super().__init__()
        # 轴距 轮距 轮直径转周长
        self.rx = raduis_base
        self.ry = raduis_base
        self.params_init()
    
    def params_init(self):
        s_th = math.sin(math.pi/4)
        r = (self.rx + self.ry) / 2
        # 根据小车四轮运动计算小车运动, 正解
        self.transform_forward = np.array([[ 1/4/s_th,  1/4/s_th, 1/r/4],
                                           [-1/4/s_th,  1/4/s_th, 1/r/4],
                                           [-1/4/s_th, -1/4/s_th, 1/r/4],
                                           [ 1/4/s_th, -1/4/s_th, 1/r/4]])

        # 根据小车运动计算小车四轮运动, 逆解
        self.transform_inverse = np.array([[s_th, -s_th, -s_th,  s_th],
                                           [s_th,  s_th, -s_th, -s_th],
                                           [   r,     r,     r,     r]])

# 底盘类型注册表(替代 eval(chassis_type), 消除任意代码执行风险)
CHASSIS_REGISTRY = {
    "TricycleChassis": TricycleChassis,
    "Diff2Chassis": Diff2Chassis,
    "Diff4Chassis": Diff4Chassis,
    "MecanumChassis": MecanumChassis,
    "QuadricycleChassis": QuadricycleChassis,
}

class MapWrap():
    def __init__(self) -> None:
        pass

    def load_map(self, path):
        import json
        with open(path, "r", encoding='utf-8') as f:
            self.map = json.load(f)
        self.map_length = len(self.map)
        self.map_index = 0

    def get_path(self, pose_start, pose_end):
        path = [[1, 1], [2, 2]]
        return path

class RoadMap():
    def __init__(self, path):
        self.path = path
        self.road_map = []
        self.road_map_index = 0
        self.road_map_length = 0
        self.load_road_map()
    
    def load_road_map(self):
        import json
        with open(self.path, "r", encoding='utf-8') as f:
            self.road_map = json.load(f)
    

class Pos2VelPid():
    def __init__(self):
        self.pid_x = 0
        self.pid_y = 0


class _Offset:
    """单轴偏移代理: 支持 car.offset.x += 100 (mm) 触发底盘移动。移植自 baidu mecanum.py。"""
    def __init__(self, driver, axis):
        object.__setattr__(self, "_driver", driver)
        object.__setattr__(self, "_axis", axis)

    def _make_offset(self, delta):
        return {"x": [delta, 0, 0], "y": [0, delta, 0], "z": [0, 0, delta]}[self._axis]

    def __iadd__(self, delta):
        self._driver.offset_by(self._make_offset(delta))
        return self

    def __isub__(self, delta):
        self._driver.offset_by(self._make_offset(-delta))
        return self


class _OffsetGroup:
    """car.offset.{x,y,z} += mm/deg 的语法糖容器。"""
    def __init__(self, driver):
        object.__setattr__(self, "_x", _Offset(driver, "x"))
        object.__setattr__(self, "_y", _Offset(driver, "y"))
        object.__setattr__(self, "_z", _Offset(driver, "z"))

    def __getattr__(self, name):
        if name in ("x", "y", "z"):
            return object.__getattribute__(self, "_" + name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        # 仅允许 += / -= 回写 _Offset 实例; 阻止直接标量赋值
        if name in ("x", "y", "z") and isinstance(value, _Offset):
            object.__setattr__(self, "_" + name, value)
        else:
            raise AttributeError("use car.offset.{}.+= / -= (mm/deg)".format(name))


class CarBase():
    def __init__(self):
        path = get_path_relative("cfg_vehicle.yaml")
        cfg = yaml.load(open(path, "r", encoding='utf-8'), Loader=yaml.FullLoader)
        
        self.chassis_init(cfg)
        # self.world_odom = OdometryBase()

        # self.motor_convert = MotorConvert(math.pi * cfg["vehicle_cfg"]["wheel_diameter"])
        self.end_flag = False
        # 里程计读写锁: 守护 chassis.odom.pose 免于 daemon 线程与主线程的撕裂读
        self._pose_lock = threading.Lock()
        # 单位友好偏移 DSL: car.offset.x += 100 (mm)
        self.offset = _OffsetGroup(self)
        self.odom_process = Thread(target=self.odomery_update)
        self.odom_process.daemon = True
        self.odom_process.start()

    def reset_pose(self):
        with self._pose_lock:
            self.chassis.odom.reset()

    def _read_pose(self):
        # 加锁读取世界系位姿, 返回副本
        with self._pose_lock:
            return np.array(self.chassis.odom.pose)

    def chassis_init(self, cfg):
        try:
            # 获取底盘类型
            chassis_type = cfg["vehicle_cfg"]["chassis_type"]
            wheel_params1 = cfg["vehicle_cfg"]["wheel"]

            logger.info(chassis_type)
            chassis_params = cfg["vehicle_cfg"][chassis_type]["size"]
            motor_ports = cfg["vehicle_cfg"][chassis_type]["wheel"]["port_list"]
            # 获取底盘参数(查表替代 eval, 避免任意代码执行)
            if chassis_type not in CHASSIS_REGISTRY:
                raise KeyError(
                    "unknown chassis_type {!r}; known: {}".format(
                        chassis_type, list(CHASSIS_REGISTRY)))
            self.chassis:ChassisBase = CHASSIS_REGISTRY[chassis_type](**chassis_params)
            # 获取电机接口
            # self.motors_chassis = Motors(cfg["vehicle_cfg"][chassis_type]["motor_ports"])
            self.wheels_chassis = WheelWrap(motor_ports, **wheel_params1)

            self.pid_x = PID(**cfg["pid_vel_params"]["pid_x"])
            self.pid_y = PID(**cfg["pid_vel_params"]["pid_y"])
            self.pid_yaw = PID(**cfg["pid_vel_params"]["pid_yaw"])
        except Exception as e:
            # 不再 while True: time.sleep(1) 挂死, 抛出让 systemd / 调用方感知
            logger.error("chassis cfg error: {}".format(e))
            raise RuntimeError("chassis init failed: {}".format(e)) from e

    @staticmethod
    def sp_world2car(vel_world, angle_car):
        '''
        世界坐标系到车坐标系
        '''
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # print(sin_car, cos_car)
        transform = np.array([[cos_car, -sin_car, 0], 
                            [sin_car,  cos_car, 0],
                            [       0,       0, 1]])
        vel_car = np.array(vel_world).dot(transform)
        return vel_car
    
    @staticmethod
    def sp_car2world(vel_car, angle_car):
        '''
        车坐标系到世界坐标系
        '''
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # print(sin_car, cos_car)
        transform = np.array([[cos_car, -sin_car, 0], 
                            [sin_car,  cos_car, 0],
                            [      0,        0, 1]])
        vel_world= np.array(vel_car).dot(transform)
        return vel_world
    
    def set_velocity(self, linear_vx, linear_vy, angular_v):
        # 根据速度计算四个轮子速度
        wheel_linear = self.chassis.get_velocity(linear_vx, linear_vy, angular_v)
        # print(wheel_linear)
        # 根据线速度计算轮胎的角速度, 计算A,B,C的电机速度
        # sp_motors = self.motor_convert.sp2angluar(wheel_linear)
        # sp_motors = self.motor_convert.sp2virtual(wheel_linear)
        # print(sp_motors)
        self.wheels_chassis.set_linear(wheel_linear)
    
    def set_vel_time(self, linear_vx, linear_vy, angular_v, during=1):
        time_st = time.time()
        while True:
            if time.time() - time_st > during:
                break
            self.set_velocity(linear_vx, linear_vy, angular_v)
            time.sleep(0.01)
        self.set_velocity(0, 0, 0)

    def odomery_update(self):
        # encoders_last = np.array(self.motors_chassis.get_encoder())
        linear_wheel_last = np.array(self.wheels_chassis.get_linear())
        # print(linear_wheel_last)
        while True:
            if self.end_flag:
                break
            '''
            
            # 计算编码器变化
            encoders_now = np.array(self.motors_chassis.get_encoder())
            encoders_d = encoders_now - encoders_last
            encoders_last = encoders_now
            d_vect_wheel = self.motor_convert.dis2true(encoders_d)
            '''
            linear_wheel_now = self.wheels_chassis.get_linear()
            try:
                # print(linear_wheel_now)
                # 获取每个轮子的位移
                linear_wheel_d = linear_wheel_now - linear_wheel_last
                linear_wheel_last = linear_wheel_now
            except:
                linear_wheel_d = np.zeros(4)
            # print(linear_wheel_last)
            # 里程计根据轮子的位置变化更新
            with self._pose_lock:
                self.chassis.updata_odom(linear_wheel_d)
            # print(self.chassis.odom.pose)
            time.sleep(0.05)

    def get_odometry(self):
        return self._read_pose()
    
    def get_dis_traveled(self):
        return self.chassis.odom.dis_traveled
    
    def stop(self):
        self.set_velocity(0, 0, 0)

    def close(self):
        self.end_flag = True
        self.odom_process.join()
    
    
    def set_pose(self, pose, during=None, vel=[0.15, 0.15, math.pi/3], threshold=[0.002, 0.002, 0.01],
                 timeout=None, max_iterations=None, loop_sleep=0.0):
        # timeout / max_iterations / loop_sleep 默认 None/0 => 与历史行为完全一致(不限时/不限次/不休眠)
        # 计算当前位置和目标位置的差值
        # 根据差值和小车的位置和角度计算小车到目标的位置和角度
        # 根据函数参数来计算或者设置小车的速度限制
        # 根据小车的运动学模型计算小车的线速度和角速度
        if during is None:
            during = float((np.array(pose) / np.array(vel)).max())

        vel = (np.abs(np.array(pose) - self._read_pose())) / during
 
        # print(vel)
        self.pid_x.setpoint = pose[0]
        vel_thresh = limit_val(vel[0], 0.1, 0.5)
        self.pid_x.output_limits = (-vel_thresh, vel_thresh)
        self.pid_y.setpoint = pose[1]
        vel_thresh = limit_val(vel[1], 0.1, 0.5)
        self.pid_y.output_limits = (-vel_thresh, vel_thresh)

        pose_threshold = np.array(threshold)

        # self.pid_turn.setpoint = math.pi  # math.pi
        self.pid_yaw.setpoint = pose[2]
        vel_thresh = limit_val(vel[2], 0.1, 1.5)
        self.pid_yaw.output_limits = (-vel_thresh, vel_thresh)
        count_exit = 0
        iter_count = 0
        time_start = time.time()
        while True:
            if timeout is not None and time.time() - time_start > timeout:
                logger.warning("set_pose timeout {}s, target={}".format(timeout, list(pose)))
                break
            if max_iterations is not None:
                iter_count += 1
                if iter_count > max_iterations:
                    logger.warning("set_pose exceeded max_iterations={}".format(max_iterations))
                    break
            pose_now = self._read_pose()
            err = np.abs(pose_now - pose)
            # print("err:{}, threshold:{}".format(err, pose_threshold))
            err_ret = (err < pose_threshold)
            # print(err_ret)
            if err_ret.all():
                count_exit += 1
                if count_exit > 20:
                    break
            else:
                count_exit = 0

            # print(pose)
            vel_x_pid = self.pid_x(pose_now[0])
            vel_y_pid = self.pid_y(pose_now[1])
            vel_yaw_out = self.pid_yaw(pose_now[2])
            # print(vel_x_pid, vel_y_pid, vel_yaw_out)
            # print(vel_x_pid)
            # 世界坐标速度转换车子坐标速度
            vel_out = self.sp_world2car([vel_x_pid, vel_y_pid, vel_yaw_out], pose_now[2])
            self.set_velocity(*vel_out)
            # print(vel_out)
            # self.set_velocity(vel_x_pid, vel_y_pid, vel_yaw_out)
            if loop_sleep:
                time.sleep(loop_sleep)
        self.set_velocity(0, 0, 0)

    def set_pose_offset(self, pose, during=None, vel=[0.2, 0.2, math.pi/3], threshold=[0.002, 0.002, 0.01],
                        timeout=None, max_iterations=None, loop_sleep=0.0):
        start_pos = self._read_pose()
        tar_pos = [0, 0, 0]
        tar_pos[0] = start_pos[0] + pose[0]*math.cos(start_pos[2]) - pose[1]*math.sin(start_pos[2])
        tar_pos[1] = start_pos[1] + pose[1]*math.cos(start_pos[2]) + pose[0]*math.sin(start_pos[2])
        tar_pos[2] = start_pos[2] + pose[2]
        # print(tar_pos)
        self.set_pose(tar_pos, during, vel, threshold,
                      timeout=timeout, max_iterations=max_iterations, loop_sleep=loop_sleep)

    # ---- 吸收自 baidu mecanum.py 的路点 API(薄封装 set_pose/set_pose_offset, 加超时/有界循环) ----
    def move_to_position(self, pose, duration=None, max_velocities=(0.2, 0.2, math.pi/3),
                         tolerance=(0.002, 0.002, 0.01), timeout=30.0):
        """驶向世界系目标位姿 [x, y, theta](米/弧度)。带 30s 超时 + 1000 次迭代上限, 不可达不再挂死。"""
        self.set_pose(list(pose), during=duration, vel=list(max_velocities),
                      threshold=list(tolerance), timeout=timeout,
                      max_iterations=1000, loop_sleep=0.01)

    def move_for(self, offset, duration=None, max_velocities=(0.2, 0.2, math.pi/3),
                 tolerance=(0.002, 0.002, 0.02), timeout=30.0):
        """相对当前位姿的机体系偏移 [dx, dy, dtheta](米/弧度)。"""
        self.set_pose_offset(list(offset), during=duration, vel=list(max_velocities),
                             threshold=list(tolerance), timeout=timeout,
                             max_iterations=1000, loop_sleep=0.01)

    def offset_by(self, offset, duration=None, max_velocities=(0.2, 0.2, math.pi/3),
                  tolerance=(0.002, 0.002, 0.02), timeout=30.0):
        """单位友好偏移: offset=[mm, mm, deg] -> 内部换算为 [m, m, rad]。"""
        offset_si = [offset[0] / 1000.0, offset[1] / 1000.0, math.radians(offset[2])]
        self.move_for(offset_si, duration=duration, max_velocities=max_velocities,
                      tolerance=tolerance, timeout=timeout)

    def move_x(self, mm):
        self.offset_by([mm, 0, 0])

    def move_y(self, mm):
        self.offset_by([0, mm, 0])

    def move_z(self, deg):
        self.offset_by([0, 0, deg])


if __name__ == '__main__':

    # logger.info("init ok")
    # car = CarBase()
    # for i in range(10):
    #     car.set_velocity(0, 0, math.pi/2)
    #     time.sleep(0.1)
    #     pose, traveled = car.get_odometry()
    #     # print(pose)
    # car.set_velocity(0, 0, 0)
    # pose, traveled = car.get_odometry()
    # logger.info("pose:{}, traveled{}".format(pose, traveled))
    # diff = Diff2Chassis()
    # diff4 = Diff4Chassis()
    # tric = TricycleChassis()
    # mecanum = MecanumChassis()
    # quad = QuadricycleChassis()
    car = CarBase()
    
    # car.set_pose_offset([0,0,math.pi*1.3], 4)
    car.set_pose([0.2,0,0], 0.4)
    car.set_pose([0,0,0], 0.4)

    # for i in range(5):
    #     # car.set_pose([0.4,0.3,math.pi], 2)
    #     # car.set_pose_offset([0.2,0,math.pi/2], 2)
    #     car.set_pose([0.3, 0, 0], 1)
    #     car.set_pose([0.3, 0.3, 0], 1)
    #     car.set_pose([0, 0.3, 0], 1)
    #     car.set_pose([0, 0, 0], 1)
    # car.set_pose_offset([0, -0.1, 0], 1)
    # for i in range(20):
        # pose, trav = car.get_odometry()
        # angle = pose[2]
        
        # vel_car = sp_world2car([0.1, 0, math.pi/2], angle)
        # print(vel_car, angle)
        # car.set_velocity(*vel_car)
        # car.set_velocity(0, 0, math.pi/2)
        # car.set_velocity(0, -0.1, 0)
        # time.sleep(0.089)
    # car.set_pose([0,0,math.pi/2], 3)
    # car.move_to([0.2,0,0], 2)
    # st = time.time()
    # while True:
    #     if time.time()-st > 2:
    #         break
    #     car.set_velocity(0, 0, math.pi/2)
    #     time.sleep(0.05)
    # car.set_velocity(0, 0, 0)
    print(car.get_odometry())
    # car.move_test([0,0.2,-math.pi/2], 1)
    # print(car.get_odometry())
    print(time.time())
    car.close()
    '''
    for i in range(20):
        # pose, trav = car.get_odometry()
        # angle = pose[2]
        
        # vel_car = sp_world2car([0.1, 0, math.pi/2], angle)
        # print(vel_car, angle)
        # car.set_velocity(*vel_car)
        car.set_velocity(0, 0, math.pi/2)
        # car.set_velocity(0, -0.1, 0)
        time.sleep(0.089)
    # print(time.time())
    pose, traveled = car.get_odometry()
    car.set_velocity(0, 0, 0)
    print(pose, traveled)
    '''
    # print(pose)
    # car.set_velocity(0, 0, 0)
    # /dev/video9
    # print("vel:", car.set_velocity(0, 0, math.pi/2))

#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import os
from queue import Queue
from serial.tools import list_ports
from threading import Lock, Thread
from multiprocessing import Process 
import time, math
import struct
import sys
# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
pass  # logger via print
# from pydownload import Scratch_Download_MC602P
from .serial import serial_mc602

# def set_serial_mc602(ser:SerialWrap):
#     global serial_mc602
#     serial_mc602 = ser

ctl602_dev_list = {
    "motor4":{"dev_id":0x01, "format":"bbbbb"},
    "motor":{"dev_id":0x02, "format":"bbb"},
    "encoder4":{"dev_id":0x03, "format":"biiii"},
    "encoder":{"dev_id":0x04, "format":"bbi"},
    "servo_pwm":{"dev_id":0x05, "format":"bbBB"},
    "servo_bus":{"dev_id":0x06, "format":"bbbbh"},
    "sensor_analog":{"dev_id":0x07, "mode":0, "format":"bbH"},
    "sensor_infrared":{"dev_id":0x07, "mode":1, "format":"bbH"},
    "sensor_touch":{"dev_id":0x07, "mode":2, "format":"bbH"},
    "sensor_ultrasonic":{"dev_id":0x07, "mode":3, "format":"bbH"},
    "sensor_ambient_light":{"dev_id":0x07, "mode":4, "format":"bbH"},
    "sensor_analog_a":{"dev_id":0x08, "mode":0, "format":"bbH"},
    "bluetooth":{"dev_id":0x09, "format":"BBBBi"},
    "beep":{"dev_id":0x0a, "format":"BBB"},
    "led_show":{"dev_id":0x0b, "format":"b"*101},
    "power":{"dev_id":0x0c, "format":"bi"},
    "board_key":{"dev_id":0x0d, "format":"bbb"},
    "led_light":{"dev_id":0x0e, "format":"bbBBBB"},
    "nixietube":{"dev_id":0x0f, "format":"bbi"},
    "dout":{"dev_id":0x10, "format":"bbb"},
    "stepper":{"dev_id":0x11, "format":"bbii"}
}

# 导入自定义log模块
pass  # logger via print

class StructData():
    def __init__(self, format=None) -> None:
        if format is None:
            format=''
        self.format = '<b'+ format
        self.size = struct.calcsize(self.format)
        self.len = len(self.format)-1
        
    def set_format(self, format):
        self.format = '<b'+ format
        self.size = struct.calcsize(self.format)
        self.len = len(self.format)-1
        
    def __sizeof__(self) -> int:
        return self.size
    
    def unpack_data(self, data, index_start):
        try:
            s = index_start
            e = index_start + self.size
            
            # print(data[s:e])
            re_list = list(struct.unpack(self.format, data[s:e]))
        except Exception as e:
            pass
            return []
        return re_list

    def pack_data(self, data):
        bytes_t = struct.pack(self.format, *data)
        return bytes_t

    # 定义len函数的定义
    def __len__(self):
        return self.len

class DevCmdInterface:
    def __init__(self, dev_id=None, mode=None, port_id=None, format='bb') -> None:
        global serial_mc602
        self.ser = serial_mc602
        self.data_struct = StructData(format)
        self.dev_id = dev_id
        self.mode = mode
        self.port_id = port_id

        self.time_out = 0.2
        self.last_data = None
        # 参数保存位置
        self.arg_reg = 1

    def set_time_out(self, time_out):
        self.time_out = time_out

    def set_port(self, port_id):
        self.port_id = port_id
    
    def get_bytes(self, *args, mode=None, port_id=None):
        # 根据参数补充所有参数
        data = []
        # print(args)
        data.append(self.dev_id)
        self.arg_reg = 3
        # 根据需要添加操作参数
        if mode is not None:
            data.append(mode)
        elif self.mode is not None:
            data.append(self.mode)
        else:
            self.arg_reg -= 1
            data.append(0)
        # 根据需要添加端口参数
        if port_id is not None:
            data.append(port_id)
        elif self.port_id is not None:
            data.append(self.port_id)
        else:
            self.arg_reg -= 1
        d_len = len(self.data_struct) - len(data)
        args_list= list(args)
        # 根据情况去除参数或者补齐参数
        while True:
            if len(args_list) > d_len:
                args_list.pop(0)
            elif len(args_list) < d_len:
                args_list.append(0)
            else:
                break
        data = data + args_list
        return self.data_struct.pack_data(data)
    
    def get_result(self, bytes_all, index=0):
        data = self.data_struct.unpack_data(bytes_all, index)[self.arg_reg:]
        # 如果只有一个结果
        if len(data) == 1:
            data = data[0]
        return data
    
    def send_get(self, bytes_tmp:bytes):
        ret = self.ser.get_anwser(bytes_tmp, self.time_out)
        if ret is not None:
            self.last_data = self.get_result(ret)
        return self.last_data
    
    def act_mode(self, *args, mode=None, port_id=None):
        data_bytes = self.get_bytes(*args, mode=mode, port_id=port_id)
        return self.send_get(data_bytes)
    
    def reset(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, mode=3, port_id=port_id)
        return self.send_get(data_bytes)
    
    # 设置操作
    def set(self, *args, port_id=None):
        # print(args)
        data_bytes = self.get_bytes(*args, mode=2, port_id=port_id)
        # print(data_bytes.hex(" "))
        return self.send_get(data_bytes)
    
    # 获取操作
    def get(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, mode=1, port_id=port_id)
        # print(data_bytes)
        return self.send_get(data_bytes)
    
    # 没有操作符号时
    def no_act(self, port_id=None):
        data_bytes = self.get_bytes(port_id=port_id)
        # print(data_bytes)
        return self.send_get(data_bytes)
    
    def act_default(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, port_id=port_id)
        return data_bytes

class DevListWrap:
    def __init__(self, dev_list=None) -> None:
        if dev_list is None:
            self.dev_list = []
        else:
            self.dev_list = dev_list

    def get_all(self, args, mode=1):
        bytes_all = b''
        for i in range(len(self.dev_list)):
            bytes_all += self.dev_list[i].get_bytes(args[i], mode=mode)
            # bytes_all += self.dev_list[i].act_default(args[i])
        # print(bytes_all.hex(' '))
        res = serial_mc602.get_anwser(bytes_all)
        data_ret = []
        if res is not None:
            index = 0
            for i in range(len(self.dev_list)):
                data = self.dev_list[i].get_result(res, index)
                index += self.dev_list[i].data_struct.size
                data_ret.append(data)
        else:
            return [0,0,0,0]
        return data_ret
    def __getattr__(self, name):
        return getattr(self.dev_list, name)
    
class Buzzer_2(DevCmdInterface):
    def __init__(self) -> None:
        super().__init__(**ctl602_dev_list["beep"])

    def rings(self, freq=262, duration=0.2):
        # 音调hz 时间s
        res = super().set(int(freq/2), int(duration*20))
        return res
    
class Motor_2(DevCmdInterface):
    def __init__(self, port_id=None, reverse=1) -> None:
        super().__init__(**ctl602_dev_list["motor"], port_id=port_id)
        self.reverse = reverse
    
    def set_dir(self, reverse):
        self.reverse = reverse
        
    def set_speed(self, *args):
        args = list(args)
        if len(args) == 2:
            args[1] = int(args[1] * self.reverse)
        else:
            args[0] = int(args[0] * self.reverse)
        # print(args)
        return self.set(*args)
    
class AnalogInput_2(DevCmdInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["sensor_analog"], port_id=port_id)

# 红外传感器
class Infrared_2(DevCmdInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["sensor_infrared"], port_id=port_id)

class Sensor_Analog2_2(DevCmdInterface):
    def __init__(self, port_id=None):
        super().__init__(**ctl602_dev_list["sensor_analog_a"], port_id=port_id)
    def read(self):
        return self.no_act()

class BoardKey_2(DevCmdInterface):
    def __init__(self) -> None:
        super().__init__(**ctl602_dev_list["board_key"])

    def no_act(self):
        return super().no_act()[1:]


# === P 口 0x07 不同 mode 的子类(触摸/超声/环境光) ===
class Touch_2(DevCmdInterface):
    """P 口触摸按键 mode=2。读返回 0/1。"""
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["sensor_touch"], port_id=port_id)


class Ultrasonic_2(DevCmdInterface):
    """P 口超声波测距 mode=3。返回 mm(0~65535)。"""
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["sensor_ultrasonic"], port_id=port_id)


class Ambient_2(DevCmdInterface):
    """P 口环境光传感器 mode=4。返回 0~4095 亮度值。"""
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["sensor_ambient_light"], port_id=port_id)


# === 第二路模拟 0x08 ===
class Sensor_Analog2_2(DevCmdInterface):
    """dev_id=0x08 第二路通用 ADC 输入(0~4095)。"""
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["sensor_analog_a"], port_id=port_id)

    def read(self):
        return self.no_act()


# === 4 键按键板 Key4Btn_2(继承 AnalogInput_2,带查表 + 状态机) ===
KEY_MAP_4BTN = {3: 355, 1: 1366, 2: 2137, 4: 2988}  # ADC 中值 → 按键编号


class Key4Btn_2(AnalogInput_2):
    """P 口 4 键按键板。继承 AnalogInput_2,读 ADC 后查表返回按键编号 1~4/0。

    实硬件测试中用作人工调试(机械臂上下左右)。
    """
    btn_sta = []  # class-level state,SDK 也有
    state = []

    def __init__(self, port_id=4) -> None:
        super().__init__(port_id=port_id)
        self.threshold = 300  # ADC 偏差容差

    def _closest_key(self, adc: int) -> int:
        """ADC → 按键编号 1~4,误差大返回 0。"""
        if abs(adc) < 100:  # 没按
            return 0
        for key_id, mid in KEY_MAP_4BTN.items():
            if abs(adc - mid) < self.threshold:
                return key_id
        return 0

    def no_act(self, port_id=None) -> list:
        raw = super().no_act(port_id=port_id)
        # raw = [dev_id, mode, port, value]
        if raw is None or len(raw) < 4:
            return [0, 0]
        # 返回 [key1, key2] 模拟 SDK 行为(2 键 bit field)
        v = raw[3]
        return [(v >> 0) & 1, (v >> 1) & 1]

    def read(self) -> int:
        """返回 1/2/3/4 表示按下了哪个键,0 表示松开。"""
        raw = super().no_act()
        if raw is None or len(raw) < 4:
            return 0
        return self._closest_key(raw[3])


# === 蓝牙手柄 BluetoothPad_2(dev_id=0x09) ===
class BluetoothPad_2(DevCmdInterface):
    """蓝牙手柄。读返回 [left_x, left_y, right_x, right_y, btn_bitmask](归一化到 [-1, 1])。

    SDK 协议:format='BBBBi' = 4 byte sticks + 4 byte button bitmask (int32)。
    """
    def __init__(self) -> None:
        super().__init__(**ctl602_dev_list["bluetooth"])
        self.throsheld_mid = [97, 97, 97, 97, 0]
        self.stick_min = 40
        self.stick_max = 160
        self.divisor_min = [42, 42, 42, 42]
        self.divisor_max = [56, 56, 56, 56]
        self.margin = 6

    def calibrate(self):
        info = self.no_act()
        if info is None or len(info) < 4:
            return
        for i in range(4):
            if abs(info[i] - self.throsheld_mid[i]) < 10:
                self.throsheld_mid[i] = info[i]
        for i in range(4):
            self.divisor_max[i] = self.stick_max - self.throsheld_mid[i] - self.margin
            self.divisor_min[i] = self.throsheld_mid[i] - self.stick_min - self.margin

    def get_stick(self) -> list:
        """返回 [lx, ly, rx, ry, btn_int32] (归一化 -1~1 + 按键 bitmask)。"""
        data = self.no_act()
        if data is None or len(data) < 5:
            return [0.0, 0.0, 0.0, 0.0, 0]
        re_data = []
        for i in range(4):
            tmp = float(data[i] - self.throsheld_mid[i])
            if abs(tmp) < self.margin:
                tmp = 0.0
            elif tmp > 0:
                tmp = (tmp - self.margin) / max(self.divisor_max[i], 1)
            else:
                tmp = (tmp + self.margin) / max(self.divisor_min[i], 1)
            tmp = max(-1.0, min(1.0, tmp))
            re_data.append(tmp)
        re_data.append(int(data[4]))
        return re_data


# === LED 灯条 LedLight_2(dev_id=0x0e) ===
class LedLight_2(DevCmdInterface):
    """LED 灯条。set_light(led_id, r, g, b) 设置单个 LED 颜色(RGB 0~255)。"""
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["led_light"], port_id=port_id)

    def set_light(self, led_id: int, r: int, g: int, b: int, port_id: int | None = None):
        return super().set(led_id, int(r), int(g), int(b), port_id=port_id)

    def set(self, *args, port_id: int | None = None):  # noqa: A003
        return super().set(*args, port_id=port_id)


# === 数码管 NixieTube_2(dev_id=0x0f) ===
class NixieTube_2(DevCmdInterface):
    """数码管显示整数(0~9999)。"""
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["nixietube"], port_id=port_id)

    def set_number(self, num: int, port_id: int | None = None):
        return super().set(int(num), port_id=port_id)


# === 屏幕 ScreenShow_2(dev_id=0x0b) ===
class ScreenShow_2(DevCmdInterface):
    """屏幕显示 ASCII 字符串(最多 100 字符)。"""
    def __init__(self) -> None:
        super().__init__(**ctl602_dev_list["led_show"])

    def show(self, text):
        if not isinstance(text, str):
            text = str(text)
        # 截断到 100 字符(ScreenShow 协议是 101 字节 = 1 dev_id + 100 ASCII)
        text = text[:100]
        int_values = tuple(ord(c) for c in text)
        self.set(*int_values)

class Motor4_2(DevCmdInterface):
    def __init__(self) -> None:
        super().__init__(**ctl602_dev_list["motor4"])
    
    def set_speed(self, speeds):
        return super().set(*speeds)

class Motors_2():
    def __init__(self, ports, reverse=False) -> None:
        self.moto_ports = ports
        self.motors = []
        self.encoders = []
        self.args_none = []
        self.reverse = reverse
        for i in ports:
            self.motors.append(Motor_2(i))
            self.encoders.append(EncoderMotor_2(i))
            self.args_none.append(0)
        self.motors_wrap = DevListWrap(self.motors)
        self.encoders_wrap = DevListWrap(self.encoders)
        
    # 设置速度
    def set_speed(self, speeds):
        if not self.reverse:
            speeds = [-i for i in speeds]
        # print(speeds)
        return self.motors_wrap.get_all(speeds, mode=2)

    def get_speed(self):
        speed = self.motors_wrap.get_all(self.args_none, mode=1)
        if self.reverse:
            speed = [-i for i in speed]
        return speed
    
    def get_encoder(self):
        encoders = self.encoders_wrap.get_all(self.args_none, mode=1)
        if isinstance(encoders[0], list):
            encoders = encoders[0]  # 解开嵌套列表

        if not self.reverse:
            encoders = [-i for i in encoders]
        return encoders

    def reset_encoder(self):
        return self.encoders_wrap.get_all(self.args_none, mode=3)
    
    def reset(self):
        self.motors_wrap.get_all(self.args_none, mode=3)
        return self.encoders_wrap.get_all(self.args_none, mode=3)
    
class EncoderMotor_2(DevCmdInterface):
    def __init__(self, port_id=None, reverse=-1) -> None:
        self.reverse = reverse
        super().__init__(**ctl602_dev_list["encoder"], port_id=port_id)
    
    def get_encoder(self):
        return self.get()*self.reverse

class EncoderMotors4_2(DevCmdInterface):
    def __init__(self) -> None:
        super().__init__(**ctl602_dev_list["encoder4"])

class ServoPwm_2(DevCmdInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(**ctl602_dev_list["servo_pwm"], port_id=port_id)

    def set_angle(self, angle, speed=100):
        self.set(int(speed), int(angle))

class ServoBus_2(DevCmdInterface):
    def __init__(self,port_id=None) -> None:
        super().__init__(**ctl602_dev_list["servo_bus"], port_id=port_id)
        self.set_time_out(1)
    
    def set_angle(self, angle, speed=100):
        self.act_mode(1, speed, angle, mode=2)

    def set_speed(self, speed):
        self.act_mode(2, speed, mode=2)

class Battry_2(DevCmdInterface):
    def __init__(self) -> None:
        super().__init__(**ctl602_dev_list["power"])

    def read(self):
        res = super().get()
        bat = float(res) / 1000
        return bat
    
class PoutD_2(DevCmdInterface):
    def __init__(self, port_id=1) -> None:
        super().__init__(**ctl602_dev_list["dout"], port_id=port_id)
    
    def set(self, *args):
        super().set(*args)
        
class Stepper_2(DevCmdInterface):
    def __init__(self, port_id=1) -> None:
        super().__init__(**ctl602_dev_list["stepper"], port_id=port_id)

    def set_pwm(self, freq):
        super().set(int(freq))
    
    def get_step(self):
        return super().get()[1]

def beep_test():
    beep = Buzzer_2()
    for i in range(10):
        beep.set(200, 10)
        time.sleep(0.5)

def motor_test():
    motor = Motor_2(1)
    for i in range(10):
        motor.set(10)
        time.sleep(1)
        motor.set(0)
        time.sleep(1)

def motors_test():
    motors = Motors_2([1, 4], True)
    motors.reset_encoder()
    for i in range(10):
        motors.set_speed([10,10])
        time.sleep(0.1)
        # print(motors.get_encoder())
        # motors.set_speed([0, 0, 0])
        # time.sleep(1)
    motors.set_speed([0,0])
    
def motor4_test():
    motor4 = Motor4_2()
    while True:
        for i in range(10):
            motor4.set_speed([32, -16, -16, 30])
            time.sleep(1)
        motor4.set_speed([0, 0, 0, 0])
        time.sleep(1)

def encoders_test():
    encoders = EncoderMotors4_2()
    while True:
        res = encoders.get()
        print(res)
        time.sleep(1)

def sensor_anolog_test():
    sensor4 = AnalogInput_2(1)
    while(1):
        print(sensor4.no_act())
        time.sleep(1)

def sensor_infrared_test():
    infrared1 = Infrared_2(1)
    while(1):
        print(infrared1.no_act())
        time.sleep(1)

def board_key_test():
    key = BoardKey_2()
    while True:
        res = key.no_act()
        print(res)
        time.sleep(0.1)



def servo_bus_test():
    servo_bus = ServoBus_2(2)
    while(1):
        servo_bus.set_angle(100, 60)
        time.sleep(1)
        servo_bus.set_angle(50, 60)
        time.sleep(1)

def servo_pwm_test():
    servo_pwm = ServoPwm_2(2)
    while(1):
        servo_pwm.set_angle(60, 60)
        time.sleep(1)
        servo_pwm.set_angle(70, 60)
        time.sleep(1)


        


def dout_test():
    dout = PoutD_2(1)
    dout.set(0)
    # time.sleep

def stepper_test():
    stepper = Stepper_2(2)
    stepper.set(2000)
    time.sleep(1)
    stepper.set(0)

if __name__ == "__main__":
    serial_mc602.assert_dev("mc602")
    beep = Buzzer_2()
    beep.rings()
    # bluetooth_pad_test()
    # dev_list_test()
    # board_key_test()
    # led_light_test()
    # beep_test()
    # motor_test()
    # motors_test()
    # dout = Dout_2()
    # stepper_test()
    # dout_test()
    # motor4_test()
    # encoders_test()
    # sensor_anolog_test()
    # sensor_infrared_test()
    # show_test()
    # key_test()
    # servo_bus_test()
    # nixie_tube_test()
    # servo_pwm_test()


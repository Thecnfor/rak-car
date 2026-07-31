#!/usr/bin/python
# -*- coding: utf-8 -*-
"""状态 / 急停 / 调试 Mixin。

从 my_car.py 拆出：紧急停止、协作取消、arm_state 查询、调试手柄循环、
close 收尾。依赖 MyCar 在 __init__ 里设置 `self.arm` / `self.streamer` /
`self._estop_event` 等属性。
"""
import time

from smartcar import logger


class StateMixin:
    """MyCar 的状态管理 / 急停 / 调试行为。"""

    def resolve_stop(self, stop=None):
        """
        解析动作结束后的停车开关

        参数:
            stop: 显式传入的停车开关，None 时使用实例当前 STOP_PARAM
        """
        if stop is None:
            return self.STOP_PARAM
        return stop

    def emergency_stop(self):
        """
        软件急停：立即停三轴 + 置硬件协作停止标志。

        设计要点（配合 runtime 无锁调用）：
          - 底层串口每条指令自带锁并各自成包，即使 worker 线程正在跑长动作，
            这里并发下发停车指令也不会串包，故 runtime 侧无需再抢 car_lock；
          - 仅置 _hardware_stop / _estop_event：**不**再置 _stop_flag。两个标志分工：
              · _hardware_stop（仅 emergency_stop 设置）→ 硬件 loop（move_base /
                lane_base / reset_* 等）响应；feed 守护线程（lane_feed / arm_feed /
                task_feed）不响应，上位机视觉/推理仍对外提供数据流。
              · _stop_flag（仅 cancel_job 路径设置）→ feed 守护线程响应。
            底盘/机械臂 PID 循环读到 _hardware_stop 退出、y_speed/x_speed 被
            _estop chokepoint 强制 0,正在跑的 reset_y / move_* 循环随即因电
            机不动而收敛退出,不会又把电机驱起来。

        返回:True。
        """
        self._hardware_stop = True
        self._estop_event.set()
        errors = []
        for label, fn in (
            ("arm_y", lambda: self.arm.y_speed(0)),
            ("arm_x", lambda: self.arm.x_speed(0)),
            ("chassis", self.stop),
        ):
            try:
                fn()
            except Exception as exc:  # 急停尽力而为，单轴失败不拖累其它轴
                errors.append("{}:{}".format(label, exc))
        if errors:
            logger.warning("emergency_stop 部分轴停车异常: {}".format(errors))
        return True

    def clear_stop(self):
        """
        解除软件急停：清除停止标志，允许后续动作重新驱动电机。
        急停后必须显式调用本方法（或 runtime reset_stop）才能恢复运动。
        同时清 _hardware_stop 与 _stop_flag、_estop_event。
        """
        self._stop_flag = False
        self._hardware_stop = False
        self._estop_event.clear()
        return True

    def _must_exit(self):
        """硬件控制循环退出条件：硬件急停或任务取消任一命中都退出。

        Feed 守护线程只看 _stop_flag（emergency_stop 不杀它们），
        硬件 loop 同时看两个标志，确保 emergency_stop 立即生效。
        """
        return self._hardware_stop or self._stop_flag

    def beep(self):
        """
        发出蜂鸣音

        控制蜂鸣器发出一声蜂鸣音，并等待0.2秒。
        """
        self.ring.rings()
        time.sleep(0.2)

    def get_arm_state(self):
        #region debug-point runtime-init-queue-arm-state
        import json as _json
        import os as _os
        import urllib.request as _urllib_request

        def _debug_emit(msg, data=None):
            api_url = _os.environ.get("DEBUG_SERVER_URL") or _os.environ.get("TRAE_DEBUG_API_URL")
            if not api_url:
                return
            payload = {
                "sessionId": "runtime-init-queue",
                "hypothesisId": "H1",
                "location": "car_wrap_2026.get_arm_state",
                "msg": msg,
                "data": data or {},
            }
            try:
                req = _urllib_request.Request(
                    api_url,
                    data=_json.dumps(payload).encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                _urllib_request.urlopen(req, timeout=0.2).read()
            except Exception:
                pass

        _debug_emit("开始读取机械臂状态")
        #endregion debug-point runtime-init-queue-arm-state
        return {
            "x": self.arm.x_get_position(),
            "y": self.arm.y_get_position(),
            "side": getattr(self.arm, "side", None),
            "arm_angle": getattr(self.arm, "angle", None),
            "hand_angle": getattr(self.arm, "hand_angle", None),
            "y_limit": self.arm.y_reset_check(),
        }

    def debug(self, inference=False):
        """
        调试方法,显示摄像头图像和检测结果，用于调试和测试。

        inference: 是否进行推理，默认为False
        """
        inference_flag = False
        grasp_flag = False
        while True:
            if self._must_exit():
                return

            keys_val = self.blue_pad.read()

            # ==================== 1. 蓝牙手柄连接检测 ====================
            if keys_val == [-1, -1, -1, -1, 0]:
                self.car_state = [0.0, 0.0, 0.0]
                logger.error("未检测到蓝牙手柄")
                self.display.show("can't find bluetooth pad\n")
                self.beep()
                time.sleep(1)
                continue

            if inference_flag:  # 按键1: 显示车道检测结果
                self.get_lane_results()
                self.get_detection_results()
            else:
                self.streamer.update_frame(self.cap_front.read(), "cam1")
                self.streamer.update_frame(self.cap_side.read(), "cam2")

            # 执行车辆控制
            self.set_velocity(keys_val[1], -keys_val[0], -keys_val[2])

            # 射击 按下【4】
            if keys_val[4] == (1 << 11):
                self.shooting()

            if keys_val[4] == (1 << 14):  # 按键[1]: 切换推理显示
                inference_flag = not inference_flag
                self.beep()
                time.sleep(0.5)

            # 执行机械臂控制
            if keys_val[4] == (1 << 4):  # 按键△ : 向上移动机械臂
                self.arm.motor_y.set_velocity(0.5)
            elif keys_val[4] == (1 << 6):  # 按键▽: 向下移动机械臂
                self.arm.motor_y.set_velocity(-0.5)
            else:
                self.arm.motor_y.set_velocity(0.0)

            if keys_val[4] == (1 << 7):  # 按键◁ : 向左移动机械臂
                self.arm.motor_x.set_angular(50)
            elif keys_val[4] == (1 << 5):  # 按键▷: 向右移动机械臂
                self.arm.motor_x.set_angular(-50)
            else:
                self.arm.motor_x.set_angular(0.0)

            if keys_val[4] == (1 << 0):  # 按键^ : 控制手臂向上<>^v
                self.arm.set_hand_angle("UP")
            elif keys_val[4] == (1 << 2):  # 按键V: 控制手臂向下<>^v
                self.arm.set_hand_angle("DOWN")

            if keys_val[4] == (1 << 1):
                self.arm.set_arm_angle("LEFT")
            elif keys_val[4] == (1 << 3):
                self.arm.set_arm_angle("RIGHT")
            elif keys_val[4] == (1 << 10):
                self.arm.set_arm_angle(-110)
                self.arm.set_hand_angle(30)

            if keys_val[4] == (1 << 9):
                grasp_flag = not grasp_flag
                self.arm.grasp(grasp_flag)
                time.sleep(0.3)
            if keys_val[4] == (1 << 8):
                self.servo_1_flag = (self.servo_1_flag + 1) % 2
                angle = self.servo_1_angle_list[self.servo_1_flag]
                print(angle)
                self.servo_1.set_angle(angle)
                time.sleep(0.3)
            time.sleep(0.05)

    def walk_lane_test(self):
        """
        车道行走测试

        测试车道保持功能，以固定速度行驶。
        """

        def end_function():
            return True

        self.lane_base(0.3, end_function, stop=self.STOP_PARAM)

    # 注意：close() 定义在 MyCar 聚合类里（my_car/__init__.py）。
    # 它必须调用 super(MyCar, self).close() → MecanumDriver.close()；
    # 若放进 mixin，super() 会沿 MRO 跳过 MecanumDriver，基类关闭逻辑静默丢失。

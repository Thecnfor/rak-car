#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""arm_base.y_pid_moveto 收敛死区闩锁离线单测（2026-08-09，无硬件依赖）。

背景: 机械臂 Y 轴（步进 + MC602 步数反馈）在电压偏高时振荡 —— 欠阻尼 PID
(Kp=6, Kd=1.0 → ζ≈0.2) 在 setpoint 附近持续下发微速度, 位置绕目标做极限环,
"连续 5 帧 |err|<1.5mm" 永远凑不满 → move_y_position 挂死 → 上层 wait_job 超时。

修复 (arm_base.y_pid_moveto): |err| 进入 POSITION_ERROR_THRESHOLD 内立即
velocity=0（死区闩锁），轴静置 → 5 帧自然凑满收敛。本测试钉住这个语义:
  1) 带外: 仍走 PID (非零速度, 限幅内)
  2) 带内: 速度指令为 0 (轴停住, 不再喂微速度)
  3) 收敛: 连续 5 帧带内 → 返回 True
  4) 理想被控对象上整段移动仍正常收敛 (回归保护: 死区不破坏常规收敛)

真机验证才是最终凭证 (跑 run.py --task 1 看 Y 是否停摆); 此处只钉控制律语义。

跑法 (repo 根目录):
    RAK_CAR_SERIAL_AUTO_CONNECT=0 /usr/bin/python3 -m unittest smartcar.test.test_arm_y_deadband -v
或直接:
    RAK_CAR_SERIAL_AUTO_CONNECT=0 /usr/bin/python3 smartcar/test/test_arm_y_deadband.py
"""
import os
import sys
import types
import unittest
from pathlib import Path

# ---- 环境：禁止 import 时自动连串口（无硬件的开发机必须） ----
os.environ["RAK_CAR_SERIAL_AUTO_CONNECT"] = "0"

REPO_ROOT = Path(__file__).resolve().parents[2]


def _bootstrap_stub_packages():
    """桩包链: arm_base 的 ``from ..`` / ``from ...tools`` 只取类引用,
    不实例化硬件; 这里把 package 挂成 stub, 并把真实 PID/CountRecord/limit_val
    从 tools_class 挂到 tools stub 上 (tools/__init__ 会拖 camera/streamer, 跳过)."""

    def stub(name, path=None):
        mod = types.ModuleType(name)
        mod.__path__ = [str(path)] if path is not None else []
        sys.modules[name] = mod
        return mod

    for name in ("smartcar", "smartcar.whalesbot",
                 "smartcar.whalesbot.vehicle",
                 "smartcar.whalesbot.vehicle.arm"):
        sys.modules.pop(name, None)

    stub("smartcar", REPO_ROOT / "smartcar")
    stub("smartcar.whalesbot", REPO_ROOT / "smartcar" / "whalesbot")
    stub("smartcar.whalesbot.vehicle",
         REPO_ROOT / "smartcar" / "whalesbot" / "vehicle")
    stub("smartcar.whalesbot.vehicle.arm",
         REPO_ROOT / "smartcar" / "whalesbot" / "vehicle" / "arm")

    import logging

    tools = stub("smartcar.whalesbot.tools",
                 REPO_ROOT / "smartcar" / "whalesbot" / "tools")
    from smartcar.whalesbot.tools.tools_class import (  # type: ignore
        PID, CountRecord, limit_val,
    )
    tools.PID = PID
    tools.CountRecord = CountRecord
    tools.limit_val = limit_val
    tools.get_yaml = None
    tools.logger = logging.getLogger("arm-y-deadband-test")

    # vehicle 包需要 arm_base 顶层 `from .. import (...)` 的名字（仅引用，不实例化）
    vehicle = sys.modules["smartcar.whalesbot.vehicle"]

    class _Dummy:
        pass

    for name in ("AnalogInput", "MotorWrap", "Key4Btn", "ServoPwm",
                 "ServoBus", "StepperWrap", "PoutD"):
        setattr(vehicle, name, _Dummy)


_bootstrap_stub_packages()

from smartcar.whalesbot.vehicle.arm import arm_base  # noqa: E402
from smartcar.whalesbot.tools.tools_class import (  # noqa: E402
    PID, CountRecord,
)


class _Clock:
    """可控单调时钟: 每帧 +dt, 让 PID 的 sample_time=10ms 语义在测试里确定。"""

    def __init__(self, t0=1000.0):
        self.t = t0

    def now(self):
        return self.t


class _FakeStepper:
    """步进位置反馈桩: get_dis() 返回可控值。"""

    def __init__(self, pos_m=0.0):
        self.pos = pos_m

    def get_dis(self):
        return self.pos


def _make_arm(pos_m: float) -> "arm_base.ArmController":
    """用 object.__new__ 绕过 __init__（需要 yaml + 串口硬件），只喂 y_pid_moveto
    需要的属性。y_speed 桩记录每次速度指令。"""
    ctl = object.__new__(arm_base.ArmController)
    ctl.motor_y = _FakeStepper(pos_m)
    ctl.y_pose_start = 0.0
    ctl.y_pose_last = pos_m
    ctl.clock = _Clock()
    ctl.y_pid = PID(Kp=6.0, Ki=0.0, Kd=1.0, setpoint=-0.1,
                    output_limits=(-0.1, 0.1), sample_time=0.01,
                    time_fn=ctl.clock.now)
    ctl.y_pid_flag = CountRecord(5)
    ctl._sent_vel = []
    ctl.y_speed = lambda v: ctl._sent_vel.append(float(v))
    return ctl


class TestYDeadband(unittest.TestCase):
    """钉住 y_pid_moveto 死区闩锁语义。target 恒取 -0.1 m（-100mm）。"""

    TARGET = -0.1

    def test_inside_band_commands_zero(self):
        """|err| < 1.5mm 带内 → 速度指令必须为 0（轴停住，不再喂微速度）。"""
        ctl = _make_arm(pos_m=-0.099)  # 离目标 1mm
        ctl.y_pid.setpoint = self.TARGET
        ctl.y_pid_moveto(self.TARGET)
        self.assertEqual(ctl._sent_vel[-1], 0.0)

    def test_outside_band_uses_pid(self):
        """带外仍走 PID：非零速度 + 主限幅 ±0.1 内，且一帧不收敛。"""
        ctl = _make_arm(pos_m=-0.05)  # 离目标 50mm
        ctl.y_pid.setpoint = self.TARGET
        ok = ctl.y_pid_moveto(self.TARGET)
        self.assertFalse(ok)
        self.assertEqual(len(ctl._sent_vel), 1)
        v = ctl._sent_vel[0]
        self.assertNotAlmostEqual(v, 0.0, places=9)
        self.assertLessEqual(abs(v), 0.1)

    def test_converges_after_5_frames_in_band(self):
        """带内连续 5 帧 → 返回 True（CountRecord(5) 语义不被破坏）。"""
        ctl = _make_arm(pos_m=-0.099)
        ctl.y_pid.setpoint = self.TARGET
        ok = False
        for _ in range(8):
            ctl.clock.t += 0.01
            ok = ctl.y_pid_moveto(self.TARGET)
        self.assertTrue(ok)
        # 入带后每一帧都是 0 速度
        self.assertTrue(all(abs(v) < 1e-9 for v in ctl._sent_vel))

    def test_full_move_converges_ideal_plant(self):
        """理想被控对象（pos += v*dt）上整段移动仍正常收敛——回归保护，
        确保死区闩锁不破坏常规到位（旧行为也一样收敛，这里防改动引入回归）。"""
        ctl = _make_arm(pos_m=-0.05)
        ctl.y_pid.setpoint = self.TARGET
        dt = 0.01
        ok = False
        for _ in range(300):
            ctl.clock.t += dt
            pos_before = ctl.motor_y.pos
            ok = ctl.y_pid_moveto(self.TARGET)
            if ok:
                break
            vel = ctl._sent_vel[-1]
            ctl.motor_y.pos = pos_before + vel * dt
        self.assertTrue(ok, "死区闩锁不应破坏常规收敛")
        self.assertLessEqual(abs(ctl.motor_y.pos - self.TARGET), 1.5e-3 + 1e-9)
        # 收敛后末段 5 帧全是 0 速度（轴停住）
        self.assertTrue(all(abs(v) < 1e-9 for v in ctl._sent_vel[-5:]))


if __name__ == "__main__":
    unittest.main()

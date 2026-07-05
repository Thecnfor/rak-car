# Issue: 代码文件与 MC602 硬件接口映射文档

## 概述

本文档记录了 Jetson 上位机代码与 MC602 单片机物理接口的完整对应关系，包含所有传感器、执行器、电机的接线位置和代码调用链路。

## 完整文档

完整的 Markdown 文档位于仓库 `hardware-mapping-doc/` 目录下：
- **`hardware-mapping-doc/代码文件与MC602硬件接口对应表.md`**

## 速查表

### M 口（直流电机）
| 接口 | 硬件 | 代码初始化 |
|:----:|------|-----------|
| M1 | 右前轮 | `WheelWrap([1,2,3,4])` |
| M2 | 左前轮 | ↑ |
| M3 | 左后轮 | ↑ |
| M4 | 右后轮 | ↑ |
| M5 | 弹射推进 | `MotorWrap(5, -1)` |
| M6 | 机械臂水平 | `MotorWrap(6, -1)` |

### S 口（舵机）
| 接口 | 硬件 | 代码初始化 |
|:----:|------|-----------|
| S2 | 展示牌舵机 | `ServoBus(2)` |
| S3 | 手爪旋转舵机 | `ServoBus(3)` |
| S7 | 手爪开合舵机 | `ServoPwm(7, mode=270)` |

### P 口（多功能 IO）
| 接口 | 硬件 | 代码初始化 |
|:----:|------|-----------|
| P1 | 4 按键菜单 | `Key4Btn(1)` |
| P2 | LED 灯带 | `LedLight(2)` |
| P2 | 真空泵 | `PoutD(2)` |
| P3 | 电磁阀 | `PoutD(3)` |
| P4 | 弹射气阀 | `PoutD(4)` |
| P6 | 竖直限位传感器 | `AnalogInput(6)` |
| P7 | 右侧红外 | `Infrared(7)` |
| P8 | 左侧红外 | `Infrared(8)` |

### 步进电机口
| 接口 | 硬件 | 代码初始化 |
|:----:|------|-----------|
| 步进 1 | 弹射角度调节 | `StepperWrap(1)` |
| 步进 3 | 机械臂竖直 | `StepperWrap(3, -1)` |

## 涉及文件清单

- `vehicle/driver/cfg_vehicle.yaml` — 底盘端口配置
- `vehicle/driver/vehicle_base.py` — 运动学与 CarBase
- `vehicle/base/controller_wrap.py` — 统一硬件抽象层
- `vehicle/base/mc602_ctl2.py` — MC602 协议实现
- `vehicle/arm/arm_cfg.yaml` — 机械臂端口配置
- `vehicle/arm/arm_base.py` — 机械臂控制
- `config_car.yml` — IO 和传感器配置
- `task_func.py` — 任务原语（弹射/抓取等）

## 相关 Issue

关联硬件接线图与端口分配讨论。

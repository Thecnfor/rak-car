# 已知问题与技术债务

## 🔴 Critical — 必须立即修复

### 1. 硬编码 API 密钥

| 文件 | 行号 | 密钥类型 |
|------|:---:|---------|
| `car_wrap.py` | 950-951 | 百度 OCR API_KEY / SECRET_KEY |
| `ernie_bot/base/ernie_bot_wrap.py` | 272 | 百度 AI Studio access_token |
| `ernie_bot/base/ernie_bot_tmp.py` | 176 | 百度 AI Studio access_token (旧) |
| `ernie_bot/base/answer.py` | 12 | DeepSeek API key |
| `ernie_bot/base/gpt_bot_wrap.py` | — | 阿里 DashScope API key |
| `ernie_bot/base/weather_api.py` | 5 | 高德天气 API key |

**风险：** 代码泄露 = 密钥泄露。已提交到 git 历史中。

**修复：** 迁移到 `.env` 文件 + `python-dotenv`，`.gitignore` 排除 `.env`。

### 2. eval() 执行外部输入

| 文件 | 行号 | 风险 |
|------|:---:|------|
| `ernie_bot/base/answer.py` | 95, 122, 144 | `eval(answer)` — LLM 返回什么执行什么 |
| `vehicle/driver/vehicle_base.py` | 309 | `eval(chassis_type)` — YAML 配置值 |

**修复：** `answer.py` 改用 `json.loads()`；`vehicle_base.py` 用字典映射替代 `eval`。

### 3. 裸 except: 吞掉所有异常

| 文件 | 行号 | 后果 |
|------|:---:|------|
| `car_wrap.py` | 1022, 1070 | OCR 异常被静默忽略 |
| `vehicle/driver/vehicle_base.py` | 317 | 底盘初始化失败 → 无限挂起 |
| `vehicle/driver/vehicle_base.py` | 390 | 里程计异常 → 归零掩盖 |
| `tools/base/tools_class.py` | 327 | CountRecord 异常 → 返回 None |
| `mc602_ctl2.py` | 77 | struct 解包异常 → 返回空列表 |

**修复：** 改为 `except Exception as e:` + 日志记录。

### 4. 错误时无限挂起

| 文件 | 行号 | 触发条件 |
|------|:---:|---------|
| `serial_wrap.py` | 60-67 | 找不到控制器 |
| `serial_wrap.py` | 155-157 | `assert_dev` 失败 |
| `controller_wrap.py` | 92-95 | `NoneDev` 方法被调用 |
| `camera.py` | `init()` | 摄像头打不开 |
| `infer_front.py` | health check | 推理服务未启动 |

**修复：** 加超时，抛异常或返回错误码。

---

## 🟠 High — 应尽快修复

### 5. 同文件重复定义

| 文件 | 重复内容 | 行号 |
|------|---------|:---:|
| `controller_wrap.py` | `Motors` 类定义两次 | 156, 353 |
| `controller_wrap.py` | `MotorWrap` 类定义两次 | 341, 536 |
| `car_wrap.py` | `get_ocr_list_plus` 方法定义两次 | 989, 1035 |
| `main/qqq.py` | `bmi()` 函数定义两次 | 265, 340 |

第二个定义静默覆盖第一个。

### 6. ernie_bot/__init__.py 命名冲突

`ernie_bot_wrap.py` 和 `gpt_bot_wrap.py` 都导出同名类：
- `HumAttrPrompt`, `ActionPrompt`, `EduCounselerPrompt`
- `FoodGetPrompt`, `FoodPutPrompt`, `BMIAnaPrompt`

后导入的覆盖先导入的，取决于 `__init__.py` 的 import 顺序。

### 7. import 时硬件初始化

```python
# serial_wrap.py:352 — import 时扫描串口
serial_wrap = SerialWrap()

# controller_wrap.py:37 — import 时读取全局 serial_wrap
ctl_id = get_devid()

# main/main.py:630 — import 时初始化全部硬件
my_car = MyCar()
```

意味着 `import vehicle` 就会触发硬件扫描。无法在无硬件环境下运行或测试。

### 8. paddle_jetson 反向依赖 ernie_bot

```python
# paddle_jetson/base/infer_wrap.py:26
from ernie_bot import HumAttrPrompt
```

底层推理库依赖高层 LLM 模块。`HumAttrPrompt` 的 schema 应该提取到共享模块。

---

## 🟡 Medium — 计划修复

### 9. 大量代码重复

| 重复项 | 出现次数 |
|--------|:------:|
| `get_key_by_value()` 函数 | 17 个文件 |
| `index_form` 字典 | 15+ 个文件 |
| `lane_det_location_v4` / `vert` | car_wrap.py 内 2 个 100+ 行近似方法 |
| `lane_det_location_v8_multi` / `plant` | car_wrap.py 内 2 个 100+ 行近似方法 |
| 竞赛任务函数 (hanoi, bmi, camp...) | 4+ 个入口文件 |

### 10. sys.path.append 滥用

30+ 处 `sys.path.append`，包括硬编码绝对路径：
```python
sys.path.append("/home/jetson/workspace/vehicle_wbt/")
```

### 11. 通配符 import

```python
# controller_wrap.py
from mc601_ctl2 import *   # 导入所有名称
from mc602_ctl2 import *

# tools/__init__.py
from .base.tools_class import *
```

### 12. MC601 编码器是模拟值

`Motor_1.get_encoder()` 不读取真实编码器，而是速度×时间积分。误差会累积。

### 13. 两个 PID 实现

- `tools.PID` — 自定义实现
- `simple_pid.PID` — 第三方库

两者都在 `car_wrap.py` 中使用。

---

## 🔵 Low — 有空再修

### 14. 命名拼写错误

| 错误 | 正确 | 文件 |
|------|------|------|
| `sellect_program` | `select_program` | car_wrap.py |
| `end_fuction` | `end_function` | car_wrap.py (多处) |
| `get_anwser` | `get_answer` | serial_wrap.py |
| `updata_odom` | `update_odom` | vehicle_base.py |
| `raduis` | `radius` | controller_wrap.py |
| `scripy` | `script` | main/scripy*.py |
| `Battry` | `Battery` | controller_wrap.py |

### 15. 乱码注释

`# ??????????` 出现在 9+ 个文件中，是 UTF-8 编码问题。

### 16. 未使用的代码

| 类型 | 内容 | 位置 |
|------|------|------|
| 类 | `LanePidCal`, `DetPidCal`, `LocatePidCal` | car_wrap.py |
| 类 | `MapWrap` (返回硬编码路径) | vehicle_base.py |
| 类 | `PositionPID` (含 matplotlib 画图) | controller_wrap.py |
| 方法 | `get_cfg()` (从未调用) | car_wrap.py |
| import | `difflib`, `platform` | car_wrap.py |

### 17. 魔法数字

- PID 参数直接写在方法里，不读配置文件
- 机械臂位置全是裸数字（0.255, 0.175, 0.215...），无物理单位注释
- 检测 class_id 用裸数字（`if det_cls_id == 16:`）

### 18. git 仓库中的运行时产物

- 4 个 `nohup.out` 文件
- 2 个 `.swp` Vim 交换文件
- `Overload` 空文件
- 523KB 截图

---

## 技术债务全景

```
                    可重构 ─────────────────────── 不可动
                         │                           │
  car_wrap.py (God Object)    vehicle/base/ (协议层)
  main/*.py (重复脚本)         vehicle/driver/ (运动学)
  ernie_bot/ (接口不统一)      vehicle/arm/ (机械臂)
  infer_cs/ (错误恢复)         camera/ (采集)
  根目录草稿文件               paddle_jetson/ (模型)
```

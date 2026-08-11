# debug-task7-position2-2026-08-03.md

> **2026-08-03 上下文压缩文档**：task7 新建 `target.py` / `dipan.py` / `position2.py` 三个自包含脚本
> 的完整会话记录。**下次 task7 相关工作第一件事读这个**。
>
> 配套约定见 [[arm-business-layer-only]] —— 业务层只能写 `main/**`，
> `smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、
> `car_task_function.py` 都不动。

---

## TL;DR

| 文件 | 用途 | 当前状态 |
| --- | --- | --- |
| `main/arm/each_task/task7/target.py` | 4 步位姿 + 1×OCR + 2×3 网格解析 → 6 位置映射 + JSON 落盘 | ✅ 已 v3（OCR 一次 + 网格解析，不再 6×扫描） |
| `main/arm/each_task/task7/dipan.py` | 底盘直线位移 (默认前进 **60cm**) | ✅ 默认 dist=+600mm |
| `main/arm/each_task/task7/position1.py` | 位置 1 三阶段序列（后退 13cm → 5 步臂 → 前进 13cm） | ✅ 2026-08-03 新建；臂 5 步与 position2 同款 (y_up=-190/hand=-90/y_down=-70/x=-260/x=0) |
| `main/arm/each_task/task7/position2.py` | 位置 2 位姿序列（5 步） | ✅ y_down=-70, x=-260, y_up=-190, hand=-90, x_return=0 |
| `main/arm/each_task/task7/position3.py` | 位置 3 三阶段序列（前进 13cm → 5 步臂 → 后退 13cm） | ✅ 2026-08-03 新建；底盘顺序与 position1 相反；臂 5 步与 position2 完全同款 (x_to=-260) |
| `main/arm/each_task/task7/position5.py` | 位置 5 七步序列（吸气 → y up → arm → hand → y down → x → 放气） | ✅ 2026-08-03 新建；**v2 修订**：原 6 步版 y=0 触底隐患消除，y=-50 出保护区；hand=-45°（中间档，非 UP）；x=-250；加 grasp 真空阀开关 |
| `main/arm/each_task/task7/get_position1.py` | 位置 1 抓取（与 position1.py 区别：纯臂序列，不动底盘） | ✅ 2026-08-03 新建；底盘归位专用 |
| `main/arm/each_task/task7/get_position2.py` | 位置 2 释放（与 position2.py 区别：纯臂序列，不动底盘） | ✅ 2026-08-03 新建；**v2 修订**：4 步收尾 y=-150/arm=+85°/hand=0°/x=0，移除 y_down |
| `main/arm/each_task/task7/pingcang.py` | 储存仓角度设置（raw 协议值直传，绕开 LEFT/RIGHT 两档） | ✅ 2026-08-03 新建；默认 angle=**+90°**（用户指定），合法区间 [-128, 127]，target=car |

**task7 还有 2 个位置文件没建**（position4 / position6），等用户给坐标再建。

### 2026-08-03 /compact 续接
- 用户用 `/compact` 重新拉起 session 后, **新增 position1.py + position3.py**：
  - position1: 底盘后退 13cm → 臂 5 步（与 position2 完全同款）→ 底盘前进 13cm。CLI `--back` / `--forward` 默认 130，内部强制转符号（back→负、forward→正）防误传撞墙。**x_to=-230** (用户后期微调, 比 position2 浅 30mm)。
  - position3: 底盘前进 13cm → 臂 5 步（与 position2 完全同款）→ 底盘后退 13cm。CLI 同样 `--forward` / `--back`，但 phase 顺序相反（Phase 1=forward, Phase 3=back）。**x_to=-260** (与 position2 一致, 跟 position1 故意不同)。

---

## 1. 本会话参考的文档（按引用次数排序）

| 文档 | 路径 | 用了什么 |
| --- | --- | --- |
| **CLAUDE.md** | repo root | 业务层硬约束；三层架构；runtime 入口 |
| **main/arm/ARM_API.md** | `main/arm/ARM_API.md` | §0 坐标系约定；§1.1 ArmClient 业务动作；§1.3 ArmRunner 不对称 API；§3.1 ARM_ACTIONS；§7 软限位；§9 reset_x / reset_all / composite_run；§10 丢步核对 |
| **main/arm/ARM_API.md §7** | 同上 | y 保护区 [0, -30] 业务硬限 |
| **main/misc/test_order_read.py** | `main/misc/test_order_read.py` | ERNIE-4.5-turbo-vL 多模态调用 pattern（直接 POST aistudio，base64 + multipart） |
| **main/misc/llm_config.yml** | `main/misc/llm_config.yml` | ERNIE access_token（已配置 `47bcf191...`）+ 业务 prompt (order_read / delivery_detect / veggie_detect) |
| **main/arm/each_task/common.py** | `main/arm/each_task/common.py` | `move_x_with_split()`（belt-slip + wall + overshoot 安全分段移动） |
| **main/arm/each_task/task5/target.py** | 同目录 | target.py v2 写法的原始模板（4 步 + 1 OCR） |
| **main/arm/each_task/task5/dipan.py** | 同目录 | dipan.py 完整模板（move_for + sync=True + 自适应 timeout） |
| **main/arm/each_task/task5/__init__.py** | 同目录 | "曾被外部清空"教训 → 自包含约束 |
| **main/arm/each_task/task6/b2_ocr_lm.py** | 同目录 | OCR + LLM 解析 stub（之前是被注释掉的占位） |
| **main/arm/api/setters.py:36** | `main/arm/api/setters.py` | `set_hand_angle` 签名：timeout 必填位置参 |
| **main/arm/loops/runner.py** | `main/arm/loops/runner.py` | 24 个 def，**没有 `set_hand_angle`**（只有 `set_arm_angle` + `set_storage`） |
| **main/api_client.py:33** | `main/api_client.py` | `_request` 末尾 `response.json()` 对 JPEG bytes 直接 JSONDecodeError → 不能用 `client.http.get()` 抓图像 |
| **runtime/VISION_API.md** | `runtime/VISION_API.md` | OCR 模型 cam2；`/v1/vision/ocr` endpoint；stream URL 命名约定 |
| **config_car.yml** | repo root | `ernie_access_token` 字段；OCR 用 `OCRReco` 走 port 5004 |
| **main/arm/each_task/common.py:174** | 同前 | `move_x_with_split(target_x_mm, ...)` 函数签名 |

---

## 2. 当前文件状态

### 2.1 task7 目录现状

```
main/arm/each_task/task7/
├── __init__.py        (task7 = "产品配送 - 将 task6 拿到的货物投递到对应住户(配送点)。与 task6 强绑定。")
├── target.py          (~600 行, OCR 大模型入口 + 6 位置映射 + JSON 落盘)
├── dipan.py           (~150 行, 底盘前进 60cm = 600mm)
├── position1.py       (~250 行, 三阶段: 后退 13cm → 5 步臂 → 前进 13cm, x_to=-230)
├── position2.py       (~150 行, 5 步位姿序列, x_to=-260)
├── position3.py       (~280 行, 三阶段: 前进 13cm → 5 步臂 → 后退 13cm, x_to=-260)
├── position5.py       (~270 行, 7 步位姿: 吸气→y up→arm→hand→y down→x→放气)
├── get_position1.py   (~150 行, 底盘归位专用, 不动底盘)
├── get_position2.py   (~195 行, **v2**: 4 步纯臂序列 y=-150/arm=+85°/hand=0°/x=0)
└── pingcang.py        (~155 行, 储存仓角度设置, raw 协议值直传, 默认 +90°)
```

**还没建**：`position4.py` / `position6.py`，**等用户给坐标再建**。

### 2.2 `target.py` —— 入口设计

**完整流程**（v3 重构后）：

```
[setup 1/4] move_y(-80mm)             # 出保护区
[setup 2/4] set_arm_angle(+90°)        # 大臂复位位
[setup 3/4] set_hand_angle(-90°)       # 手爪 UP
[setup 4/4] move_x_with_split(-80mm)   # x 观察位 (belt-slip 安全)
[ocr]        ERNIE 多模态 VL 调一次    # 一帧 cam2 拍全 6 名
[parse]     _parse_grid() → 2×3 网格
[map]       按 id=1..6 row-major 映射到 6 个位置
[record]    写 JSON 到 $HOME/.remember/logs/task7_ocr_<ts>.json
```

**关键常量**（顶部集中）：

```python
TARGET_Y_MM        = -80.0    # 工作深度 (出保护区)
TARGET_ARM_DEG     = 90.0     # 复位位 (+90, 业务硬限上界)
TARGET_HAND_DEG    = -90.0    # init UP (业务硬限下界)
TARGET_X_MM        = -80.0    # 观察位 (-80, 历史占位, 基本不再用)
ANGLE_SPEED        = 80

POSITIONS = [
    (1, "上左"),  # 2-tuple, 无 x_mm
    (2, "上中"),
    (3, "上右"),
    (4, "下左"),
    (5, "下中"),
    (6, "下右"),
]

ERNIE_CHAT_URL    = "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions"
DEFAULT_OCR_MODEL = "ernie-4.5-turbo-vl"
OCR_CAM           = "cam2"
```

**6 位置编号约定**（用户 2026-08-03 指定）：

```
    [1] [2] [3]      ← 上排左→右扫
    [4] [5] [6]      ← 下排左→右扫
```

**v3 重构根因**：v2 跑了 6 次 move_x + 6 次 OCR，每次 cam2 拍到同一张图（X 不变），OCR 结果重复 6 次（每位置挂 6 个名）。v3 改成 1×OCR，把"张三 熊九 孙八 \n 田一 孟三 白七"按 row-major 切 6 个名映射到位置。

### 2.3 `dipan.py` —— 底盘直线位移

**当前默认**：前进 **60cm (600mm)**，`max_velocity=0.10 m/s`，`timeout=15.0s`（adaptive ≈8s）。

用户当天先要 60mm，后来改成 60cm。

```python
DEFAULT_DIST_MM        = 600.0    # 正值=前进, 负值=后退
DEFAULT_TIMEOUT_S      = 15.0
DEFAULT_MAX_VELOCITY_MS = 0.10

# 调用方式 (与 task5/dipan.py / task4/target4.py 同款)
job = client.http.execute_car_action(
    "move_for",
    [dist_m, 0.0, 0.0],   # 纯 x 直线
    max_velocities=[v, v, 0.0],
    sync=True,            # 阻塞等闭环完成 (默认 False 异步会被 next step 抢跑)
    timeout=timeout,
)
```

**CLI**：
```bash
python main/arm/each_task/task7/dipan.py                 # 默认前进 600mm
python main/arm/each_task/task7/dipan.py --dist -600     # 后退 60cm
python main/arm/each_task/task7/dipan.py --vel 0.08      # 限速更稳
python main/arm/each_task/task7/dipan.py --timeout 30    # 大位移兜底
```

### 2.4 `position2.py` —— 位置 2 序列（5 步）

```python
POS_Y_UP_MM     = -190.0    # step 1: y 抬高 (完全出保护区)
POS_HAND_UP_DEG = -90.0     # step 2: 手爪 UP (init 位置)
POS_Y_DOWN_MM   = -70.0     # step 3: y 降回工作深度 (与 target.py setup 不同!)
POS_X_TO_MM     = -260.0    # step 4: x 滑到位置 2
POS_X_RETURN_MM = 0.0       # step 5: x 回 0 位 (撞墙 calibrate)
ANGLE_SPEED     = 80
```

**完整 5 步**：

```
[1/5] move_y(-190mm)              # y 抬高远出保护区, 给 step 2 留余地
[2/5] set_hand_angle(-90°, ...)   # ⚠️ 走 client (ArmRunner 没 set_hand_angle)
[3/5] move_y(-70mm)               # 手爪 OUT 后安全降回工作深度
[4/5] move_x_with_split(-260mm)   # 距 -320 下界还有 60mm 余量, 别调更负
[5/5] move_x_with_split(0mm)      # 撞墙 calibrate 清编码器漂移
```

**CLI**：
```bash
python main/arm/each_task/task7/position2.py                  # 默认全套
python main/arm/each_task/task7/position2.py --y-up -200      # 想抬更狠
```

**改动轨迹**（用户当天微调）：
```
x_to : -200 → -220 → -260            (横向逐步远离墙)
y_down: -80 → -70                     (位置 2 货物比位置 1 高 10mm, y 抬高 10mm 适配)
```

### 2.5 `position1.py` —— 位置 1 三阶段序列（底盘包围 + 5 步臂）

**设计**：位置 1 是第一个投递点，用户额外加底盘"后退→前进"动作（13cm=130mm 包围式访问）。

```
Phase 1: 底盘后退 130mm           ← 摆位前置, 给臂留更大 x 工作空间
Phase 2: 5 步臂 (与 position2 完全同款)
         2.1 move_y(-190mm)
         2.2 set_hand_angle(-90°)
         2.3 move_y(-70mm)
         2.4 move_x_with_split(-260mm)
         2.5 move_x_with_split(0mm)
Phase 3: 底盘前进 130mm           ← 摆位还原, 回原位
```

**关键常量**：

```python
DEFAULT_BACK_MM         = 130.0   # Phase 1
DEFAULT_FORWARD_MM      = 130.0   # Phase 3
DEFAULT_CHASSIS_VELOCITY_MS = 0.10
DEFAULT_CHASSIS_TIMEOUT_S   = 10.0
POS_Y_UP_MM             = -190.0  # 臂与 position2 完全同款
POS_HAND_UP_DEG         = -90.0
POS_Y_DOWN_MM           = -70.0
POS_X_TO_MM             = -260.0
POS_X_RETURN_MM         = 0.0
ANGLE_SPEED             = 80
```

**符号强制约定**（防误传撞墙）：
- CLI `--back` 接收正 mm → 内部 `-abs(back_mm)` 转负 = 后退
- CLI `--forward` 接收正 mm → 内部 `abs(forward_mm)` 转正 = 前进
- 即便用户传错符号（`--back -50`）也会被 `abs()` 强制取正再转换，避免车往前窜撞墙。

**底盘调用 pattern**（与 `task7/dipan.py` 同款 `_run`，但内联避免 import task7 包内模块）：
```python
client.http.execute_car_action(
    "move_for",
    [dist_m, 0.0, 0.0],
    max_velocities=[v, v, 0.0],
    sync=True,         # ⚠️ 必须阻塞等闭环, 后续臂动作不能抢跑
    timeout=timeout,
)
```

**CLI**：
```bash
python main/arm/each_task/task7/position1.py                       # 默认全套
python main/arm/each_task/task7/position1.py --back 200 --forward 200   # 改底盘距离
python main/arm/each_task/task7/position1.py --vel 0.05            # 更稳限速
```

**为什么不 import position2.py**：task7 包内模块曾被外部清空（同 [[task5-rebuild-2026-07-22]] 教训），按"自包含"约定本脚本独立实现 Phase 2 全部 5 步（用户原话"自己写"）。常量值与 position2 完全相同（y_up=-190/hand=-90/y_down=-70/x=-260/x=0）。

### 2.6 `position3.py` —— 位置 3 三阶段序列（底盘方向与 position1 相反）

**设计**：位置 3 是第三个投递点，用户给的底盘动作顺序与 position1 **相反**：
- position1 = "先后退 → 再前进"（Phase 1=back, Phase 3=forward）
- position3 = "先前进 → 再后退"（Phase 1=forward, Phase 3=back）

```
Phase 1: 底盘前进 130mm           ← 先前进, 给臂留更大 x 工作空间
Phase 2: 5 步臂 (与 position2 完全同款, x_to=-260)
         2.1 move_y(-190mm)
         2.2 set_hand_angle(-90°)
         2.3 move_y(-70mm)
         2.4 move_x_with_split(-260mm)   ← ⚠️ 与 position1 不同 (position1=-230)
         2.5 move_x_with_split(0mm)
Phase 3: 底盘后退 130mm           ← 再后退, 还原摆位
```

**关键常量**：

```python
DEFAULT_FORWARD_MM         = 130.0   # Phase 1 前进
DEFAULT_BACK_MM            = 130.0   # Phase 3 后退
DEFAULT_CHASSIS_VELOCITY_MS = 0.10
DEFAULT_CHASSIS_TIMEOUT_S   = 10.0
POS_Y_UP_MM             = -190.0    # 臂与 position2 完全同款
POS_HAND_UP_DEG         = -90.0
POS_Y_DOWN_MM           = -70.0
POS_X_TO_MM             = -260.0    # ⚠️ 与 position1 不同 (position1=-230)
POS_X_RETURN_MM         = 0.0
ANGLE_SPEED             = 80
```

**x_to 三档位对比**：

| 位置 | x_to_mm | 备注 |
| --- | --- | --- |
| position1 | **-230** | 浅 30mm（用户微调，与 position2 故意不同） |
| position2 | **-260** | 标准深（用户基线） |
| position3 | **-260** | 与 position2 同款（位置 2/3 横向距离一致） |

**CLI 与 position1 完全一致**（`--forward` / `--back` 接收正 mm），但 phase 顺序相反：

```bash
python main/arm/each_task/task7/position3.py                          # 默认全套
python main/arm/each_task/task7/position3.py --forward 200 --back 200   # 改底盘距离
python main/arm/each_task/task7/position3.py --vel 0.05                # 更稳限速
```

**底盘方向对照表**：

| 文件 | Phase 1 (pre-arm) | Phase 3 (post-arm) | 命名 |
| --- | --- | --- | --- |
| `position1.py` | **back** 130mm | **forward** 130mm | 先退后进 |
| `position3.py` | **forward** 130mm | **back** 130mm | 先进后退 |

**为什么不 import position1.py**：同 §2.5 自包含约束，Phase 2 的 5 步臂序列按用户"自己写"指示内联实现。

### 2.7 `position5.py` —— 位置 5 七步序列（**吸气 → 臂动作 → 放气**）

**v2 设计**（用户 2026-08-03 第二次修订）：从 6 步"y=0 触底再滑"改成 7 步"夹持式传送"——首尾各加一个 grasp 真空阀开关，中间改 hand 角度、改 y 工作深度、改 x 距离。

```
[1/7] grasp(on=True)             吸气 (vacuum on, 提前抓取)
[2/7] move_y(-190mm)             y 出保护区
[3/7] set_arm_angle(+90°)        大臂复位
[4/7] set_hand_angle(-45°)       手爪中间档 (不是 UP=-90)
[5/7] move_y(-50mm)              y 工作深度 (保护区外)
[6/7] move_x_with_split(-250mm)  x 最终位
[7/7] grasp(on=False)            放气 (vacuum off, 释放)
```

**关键常量**：

```python
POS_Y_UP_MM     = -190.0    # step 2
POS_ARM_DEG     =  90.0     # step 3
POS_HAND_DEG    = -45.0     # step 4 (中间档, 替代旧 -90 UP)
POS_Y_DOWN_MM   = -50.0     # step 5 (工作深度, 替代旧 0 触底)
POS_X_FINAL_MM  = -250.0    # step 6 (替代旧 -260)
ANGLE_SPEED     =  80
```

**与 v1 对比**：

| 维度 | v1 (旧 6 步) | v2 (新 7 步) |
| --- | --- | --- |
| grasp 真空阀 | 无 | **step 1/7 加吸气/放气** |
| 手爪角度 | -90° (UP) | **-45° (中间档)** |
| y 工作深度 | 0 (触底, 保护区上界) | **-50 (保护区外)** |
| x 最终位 | -260 | **-250** |
| x 中间位 | -190 (有) | **无** (单段) |
| 步数 | 6 步 | **7 步** |
| 隐患 | step 6 在 y=0 move_x RuntimeError | **已消除** (-50 出保护区) |

**grasp 调用 pattern**（见 [[arm-grasp-call-arm-base]]）：
```python
# ⚠️ 不能用 client.grasp() —— 走 _call_arm, kwargs 透传 TypeError 静默失败
# 必须 http.execute_arm_action 位置参 [bool(on)]
client.http.execute_arm_action(
    "grasp",
    [bool(on)],                # 位置参必须显式 bool
    sync=True,
    timeout=timeout,
)
```

**CLI**：
```bash
python main/arm/each_task/task7/position5.py                  # 默认 7 步
python main/arm/each_task/task7/position5.py --hand -60       # 现场微调手爪
python main/arm/each_task/task7/position5.py --y-down -80     # 工作深度微调
python main/arm/each_task/task7/position5.py --x-final -240   # 最终位微调
```

**与其它 position 文件区别**：

| 维度 | position1/2/3 | position5 v2 |
| --- | --- | --- |
| grasp 真空阀 | 无 | **有 (首尾)** |
| 底盘动作 | 有 (除 position2 外) | 无 |
| y 轴 | 单调出保护区 (升 → 降) | 升 → 降 (都在保护区外) |
| x 轴 | 单段 | 单段 (但 -250 比其它浅) |
| 步数 | 5 步 | 7 步 |
| 业务复杂度 | 低 | **中 (grasp + 5 步臂)** |

---

### 2.8 `get_position2.py` —— 位置 2 释放/归位的**纯臂序列**（v2，4 步收尾）

**设计**：与 `position2.py` 的区别是 **不动底盘**，只调机械臂。用于底盘归位后 / 跨位置切换时安全收尾。

**v1 vs v2 对比**：

| 维度 | v1 (5 步) | **v2 (4 步收尾, 当前)** |
| --- | --- | --- |
| y_up | -190 (离上限 10mm) | **-150 (留 50mm 余量)** |
| 大臂 | +90° (复位位) | **+85° (略低于复位位, 防撞)** |
| 手爪 | (不调) | **0° DOWN (显式调用)** |
| x | 0 (撞墙) | **0 (不变)** |
| y_down | -100 (终态) | **删除 (终态 = y_up = -150)** |
| 步数 | 5 步 | **4 步** |
| 终态 y | -100 | **-150 (保护区外)** |

**完整 4 步（v2）**：

```
[1/4] move_y(-150mm)                 # y 抬高 (出保护区)
[2/4] set_arm_angle(+85°, speed=80)  # 大臂到 +85° (走 runner.set_arm_angle)
[3/4] set_hand_angle(0°, speed=80, timeout=10)   # 手爪到 0° DOWN (走 client, ⚠️ runner 没有)
[4/4] move_x_with_split(0mm)         # x 归零 (撞墙 calibrate)
```

**关键常量**：

```python
POS_Y_MM       = -150.0    # step 1 (终态 y, 保护区外)
POS_ARM_DEG    =  85.0     # step 2 (略低于复位位 +90°, 业务硬限内)
POS_HAND_DEG   =   0.0     # step 3 (DOWN, 业务硬限下界)
POS_X_HOME_MM  =   0.0     # step 4 (撞墙 calibrate)
ANGLE_SPEED    =  80
HAND_TIMEOUT_S =  10.0     # 显式传 timeout, 必填位置参 (见 [[armrunner-set-hand-angle-gotcha]])
```

**业务硬限核对**（走前必查，见 ARM_API §1.1 / §7）：
- y=-150 ∈ [-200, 0] ✓ 且 -150 < -80 (保护区外) → 安全
- arm=+85° ∈ [+90, -150]° ✓ (上界附近, 比复位位 +90° 低 5° 防撞)
- hand=0° ∈ [-90, 0]° ✓ (DOWN)
- x=0 = 撞墙位, split 兜底 belt-slip (走满会被复位回 0, 见 ARM_API §9.1)

**顺序关键**：
- 第 1 步 y=-150 是为了让第 2-3 步 `set_arm_angle(+85°)` / `set_hand_angle(0°)` 在**保护区外**完成。保护区 y ∈ [0,-80] 内: set_arm_angle(非 MID/0) / set_hand_angle(非 -90) 会被 `_check_safe` 拦截。
- 第 4 步 x 归零在臂/手爪都设完之后做, 此时机械臂完全离开工作区, x 走满整个行程不会撞到大臂/手爪 → 安全。

**为什么是 v2**：v1 用了 -190 (离上限 10mm) + y_down=-100 双段 y 移动, 终态是 -100 但 v1 实际只把臂放在 -100 准备后续工作, 整体流程偏复杂。v2 简化: 直接 -150 终态 (保护区外), 删除冗余的 y_down 步, 同时显式加 hand=0° 让"释放"序列语义更明确 (臂/手爪都收尾)。

**CLI**：

```bash
python main/arm/each_task/task7/get_position2.py                # 默认全套
python main/arm/each_task/task7/get_position2.py --y -130       # 抬低一点
```

**与 `position2.py` 区别**：

| 维度 | position2.py | get_position2.py v2 |
| --- | --- | --- |
| 动底盘 | 否 | **否** |
| 动臂 | 是 (5 步) | **是 (4 步, 简化版)** |
| 用途 | 投递到位置 2 的完整流程 (含 x=-260 滑过去) | **底盘归位 / 跨位置切换的纯臂收尾** |
| y 终态 | -70 (工作深度) | **-150 (保护区外)** |
| 手爪 | 0° DOWN (抓取位) | **0° DOWN (显式调用)** |

### 2.9 `pingcang.py` —— 储存仓角度 raw 直传（默认 +90°）

**用途**：单独抽出储存仓舵机控制（"pingcang" = "拼仓"/"平仓" 拼音），供 task7 各位置按需调；绕开标准 LEFT/RIGHT 两档写死，支持任意合法角度。

**API 选择**（见 ARM_API §3.2 / §6）：

| 入口 | 用途 | 90° 是否支持 |
| --- | --- | --- |
| `set_storage(side)` | LEFT=-42° / RIGHT=165° 两档写死 | ❌ |
| **`set_storage_angle(angle, speed)`** | **raw 协议值直传, 任意角** | **✅** |

90° 不在 LEFT/RIGHT 两档里 → **必须用 `set_storage_angle`**。

**关键常量**：

```python
DEFAULT_ANGLE_DEG : 90.0     # 用户 2026-08-03 指定
DEFAULT_SPEED     : 100      # 舵机速度 1-100
DEFAULT_TIMEOUT_S : 10.0
```

**协议层校验**（运行时 enforce）：

```python
if not (-128 <= angle_deg <= 127):
    raise ValueError(...)     # mc602 servo_pwm signed byte 合法区间
```

90° ∈ [-128, 127] ✓。

**底层调用 pattern**（与 `task7/dipan.py` 同款，**注意 target 是 car 不是 arm**）：

```python
job = client.http.execute_car_action(
    "set_storage_angle",
    angle_deg,                  # ⚠️ 位置参 (单值, 不能 wrap 成 [angle, speed]!)
    speed=speed,                # ⚠️ 关键字参, 不要走位置参!
    sync=True,                  # 必须阻塞, 舵机到位后才算完
    timeout=timeout,
)
```

⚠️ **execute_car_action args pattern 不能照搬 dipan.py**（2026-08-03 现场踩坑）：

| action | 签名 | args 该怎么传 |
| --- | --- | --- |
| `move_for` | `move_for(position_offset, ...)` —— **单参 list** `[x, y, theta]` | 包成 `[x, y, z]` ✓ (dipan.py 模式) |
| **`set_storage_angle`** | **`set_storage_angle(angle, speed=100)` —— 双独立参** | **angle 位置参 + speed kwarg** ✗ |
| `set_shoot_state` | `set_shoot_state(value)` —— 单值 | 直接传 value |
| `lane_dis_offset` | `lane_dis_offset(distance, offset=...)` —— 双参 (distance 位置 + offset 缺省) | 包成 `[distance, offset]` 还是 `distance, offset=offset`? 见 ARM_API §3.2 |

`execute_car_action(name, *args, ...)` 把 `*args` 整个 `list(args)` 后塞 JSON `args` 字段；runtime `_dispatch_car` 再 `*args` 反开包。所以：
- **单参 list 签名**（如 `move_for`）：包成 list 后变成 `[[x, y, z]]`，runtime 反开包为 `[[x, y, z]]`，lambda `*args` 再展开 → `car.move_for([x, y, z])` ✓
- **多参签名**（如 `set_storage_angle`）：传 `[angle, speed]` 变 `[[angle, speed]]`，runtime 展开 → `car.set_storage_angle([angle, speed])`，**`angle` 收到 list**，`int(angle)` 在 `ServoPwm_2.set_angle` 报 "int() argument must be ... not 'list'"。

**正解**（与 `ArmClient.set_storage_angle` 内部 `self._call_car("set_storage_angle", ..., angle=angle, speed=speed, sync=True)` 一致）：

```python
client.http.execute_car_action(
    "set_storage_angle",
    angle_deg,        # 位置参
    speed=speed,      # kwarg, 避免被 list 包裹
    sync=True,
    timeout=timeout,
)
```

**业务警告**（ARM_API §6.2）：
- **无业务软限制**（2026-07-17 取消 y 安全门），物理碰撞 caller 自负。
- **跑前确认位姿**：建议 y 在保护区 [0, -80] 外 (y ≤ -80) + 大臂 +90° + 手爪 -90° UP, 防转动期间撞车。
- **调完后 `get_storage()` 返回 `"UNKNOWN"`**（任意角度不属于 LEFT/RIGHT 两档）—— **预期行为**, 不是 bug。
- **跑比赛前必须现场标定**：90° 是用户当前调试值, 不一定是比赛最终角度。舵机机械结构会随校准变化, 不要假设旧角度常量还有效。

**CLI**：

```bash
python main/arm/each_task/task7/pingcang.py                # 默认 +90°
python main/arm/each_task/task7/pingcang.py --angle 60     # 调成 60°
python main/arm/each_task/task7/pingcang.py --angle -42    # 等效 LEFT 档
python main/arm/each_task/task7/pingcang.py --speed 50     # 舵机慢一点
```

**为什么不用 ArmClient.set_storage_angle**：ArmClient.set_storage_angle 内部走 `_call_car(...)`，但默认 sync=False 异步；显式走 `client.http.execute_car_action(..., sync=True)` 与 task7 其它脚本同款, 阻塞语义更明确。

## 3. 我做的改动汇总（按时间序）

| 时间 | 改动 | 原因 |
| --- | --- | --- |
| 早上 | 新建 `task7/target.py` 1.0: 4 步位姿（y/arm/hand/x = -80/90/-90/-80） | 用户最初指定 |
| 早上 | 修 `runner.set_hand_angle` AttributeError → 改用 `client.set_hand_angle(angle, speed, timeout=runner.default_timeout_s)` | [[armrunner-set-hand-angle-gotcha]] 写到 memory |
| 中午 | v2: 在 target.py 末尾追加 `ocr_text_with_ernie()` → 5 步 | 用户要文字识别大模型 |
| 中午 | 扩 6 位置扫描：`POSITIONS = [(id, label, x_mm), ...]` × 6 | "1上左 2上中 3上右 4下左 5下中 6下右" |
| 中午 | 修错：6×扫描重复 OCR → 改成 1×OCR + `_parse_grid()` + `_flatten_grid_to_positions()` 映射 | 用户报错"不是我要的效果" |
| 下午 | 新建 `task7/dipan.py` 1.0: 底盘前进 60mm | "让底盘前进60mm, 命名dipan" |
| 下午 | 改 dipan.py 默认 dist=60→600mm（60mm→60cm） | "是移动60cm, 改一下" |
| 晚上 | 新建 `task7/position2.py`: 5 步序列（y_up=-190, hand=-90, y_down=-80, x=-200, x=0） | "现在是位置2, 命名position2" |
| 晚上 | 改 position2.py x_to=-200→-220→-260 | "x轴" 微调 |
| 晚上 | 改 position2.py y_down=-80→-70 | "y轴的-80改成-70" |
| 夜+compact | /compact 重新拉起 session, 新建 `task7/position1.py`: 后退 13cm → 5 步臂 (与 position2 同款) → 前进 13cm | "现在新建position1, 先让底盘后退13cm, 然后机械臂零件的参数和调用参数和position2一样, 自己写, 然后底盘再前进13cm" |
| 夜+compact | 外部调整 `position1.py:POS_X_TO_MM = -230.0` (用户手动改, 比 position2 浅 30mm) | 位置 1 横向距离微调 |
| 夜+compact | 新建 `task7/position3.py`: 前进 13cm → 5 步臂 (与 position2 同款, **x_to=-260**) → 后退 13cm | "现在position3是先前进13cm, 然后机械臂参数和position2一致, 然后再后退13cm" |
| 夜+compact | 新建 `main/arm/test/test_y_minus_30.py`: y 轴单点测试 (保护区下边界) | "现在在这里建一个让y轴降到-30的代码" |
| 夜+compact | 修 `test_y_minus_30.py` import: `main.arm.test._runtime_guard` → `main.arm.runtime_guard._runtime_guard` | 模板 `test_y_minus_150.py` 写错, 跑现场暴露 ModuleNotFoundError |
| 夜+compact | 新建 `task7/position5.py`: 6 步 (y up=-190 → arm 90° → hand -90° → x mid=-190 → y down=0 → x final=-260); **⚠️ y-down=0 在保护区上界, step 6 move_x 可能 RuntimeError** | "现在新建position5, 逻辑是先让y轴升到-190, 然后大臂转到90°, 然后手爪转到-90°, 然后x轴运动到-190, 然后y轴降到0, x轴再运动到-260" |
| 夜+compact | **重写 `task7/position5.py` v2 (7 步)**: 吸气 → y=-190 → arm 90° → hand -45° → y=-50 → x=-250 → 放气; **y=0 触底隐患消除** (改 -50 出保护区); 加 grasp 真空阀开关 | "现在让position5一开始就吸气, 然后y轴升到-190, 然后大臂为90°, 然后手爪舵机为-45°, 然后y轴降到-50, 然后x轴移动到-250, 然后放气" |
| 夜+compact | **重写 `task7/get_position2.py` v2 (5→4 步)**: 删 y_down=-100, y_up=-190→-150 (离上限 10→50mm), arm=+90°→+85° (防撞), 显式加 hand=0° DOWN (client.set_hand_angle, runner 没有这方法); 终态 y=-150 (保护区外) | "现在get position2稍微改一下, 改成y轴升到-150, 大臂为+85°, 手爪舵机为0°, x为0, 按顺序操作即可" |
| 夜+compact | 新建 `task7/pingcang.py`: 储存仓角度 raw 直传 (`set_storage_angle`), 默认 +90°; signed byte 校验 [-128, 127]; target=car, 走 client.http.execute_car_action(..., sync=True) | "现在在task7里面写一个文件, 命名为pingcang, 让储存仓的角度为90°" |
| 夜+compact | **修 pingcang.py v2 (execute_car_action args pattern)**: `[angle, speed]` 双层包裹报错 (`int() argument must be ... not 'list'`), 改成 `angle` 位置参 + `speed=speed` kwarg (与 `ArmClient.set_storage_angle` 内部 `_call_car(..., angle=angle, speed=speed, sync=True)` 一致) | 现场跑失败: "set_storage_angle 失败" + int(speed) list 错误, 根因是 dipan.py 的 `[dist_m, 0, 0]` wrap pattern 只对 `move_for` 单参 list 签名有效, 对 set_storage_angle 双参签名会双层包裹 |

**memory 加了 1 条**：
- `armrunner-set-hand-angle-gotcha.md` （2026-08-03 新建，写入 MEMORY.md 索引）

---

## 4. 面临的问题 / 待办

### 4.1 已知问题（已记录）

1. **task7/__init__.py 没改**：仍是 1 行（"产品配送 - 与 task6 强绑定"）。等用户给完整流程描述后再扩。
2. **业务层只能改 main/**（[[arm-business-layer-only]]）：任何需要改 runtime / smartcar SDK 的事都拒绝做（包括 reset_x fix、composite_run 扩展、x_stop_check 阈值等）。
3. **task7 还没建 4/6 共 2 个位置文件**：等用户给每位置的 (id, label, x_mm, y_mm) 才写。position1/2/3/5 都已建。

### 4.2 待办（按优先级）

| 序 | 事项 | 优先级 |
| --- | --- | --- |
| 1 | 用户现场给 6 位置的 x_mm / y_mm 后，建 position4/6.py（position1/2/3/5 已建） | 高 |
| 2 | 现场给 ERNIE token（如果 `main/misc/llm_config.yml` 里的 `47bcf191...` 失效） | 中 |
| 3 | 现场给 6 个位置的实际物理坐标 → 填到 POSITIONS（如要走 6 次扫描方案，但当前 v3 是 1×OCR） | 中 |
| 4 | 如果位置间需要按特定顺序扫（不是默认 1-2-3-4-5-6），改 `POSITIONS` 列表顺序 | 低 |
| 5 | 如果 `target.py` setup 的 y=-80 跟某位置 y 不一致，按现场实测统一 | 中 |

### 4.3 还没解决的疑问

- **每个位置是否真的需要 OCR 一次**？目前是 1×OCR（cam2 一帧拍全）。如果现场发现一帧拍不全（视角遮挡），可能要 6×OCR，每个位置 cam1 单独取。
- **target.py 的 y=-80 跟 position2.py 的 y=-70 是否要统一**？目前不一致，按用户意图保持不一致是合理的（位置 2 货物比位置 1 高 10mm）。
- **dipan.py 的 60cm 是否要分段**？60cm 一次 `move_for` 比 6×100mm 风险高（中段出问题整段重来），目前默认一次走完。现场可以测一次再决定要不要改成 N 段。
- **position5 v1 (旧 6 步) 的 y=0 已被 v2 修订消除**：v2 改成 y=-50 工作深度, 保护区外, 隐患不再。当前 position5.py 是 v2 (7 步: 吸气→臂→放气)。
- **grasp 调用走 http.execute_arm_action 位置参 [bool(on)]** ([[arm-grasp-call-arm-base]]), 不能用 client.grasp() —— 业务层脚本统一遵循。

---

## 5. 业务层硬约束（[[arm-business-layer-only]]）

**严禁改动**：
- `smartcar/whalesbot/**`
- `runtime/**`
- `car_wrap_2026.py`
- `car_start_2026.py`
- `car_task_function.py`

**可以改**：
- `main/**`（业务层）
- 包括 `main/arm/each_task/task7/*` ← 当前工作面

## 6. 续接指南（下次会话第一件事）

1. **读本文档**（你已经做了）
2. **读 [[arm-business-layer-only]]** 确认业务层边界
3. **读 [[armrunner-set-hand-angle-gotcha]]**：`ArmRunner` 没 `set_hand_angle`，必须 `client.set_hand_angle(angle, speed, timeout=runner.default_timeout_s)`
4. **读 [[x-axis-rollout-session]]** / [[x-axis-belt-slip]]：x 默认 `move_x_with_split`，60mm+ 都走 belt-slip 安全模式
5. **检查 task7 目录当前状态**：`ls main/arm/each_task/task7/` 看是否多了其它文件（用户中途可能改）
6. **跑 `python main/arm/each_task/task7/target.py` 验证 OCR 链路**：连 runtime + cam2 + ERNIE token 是否就绪

## 7. 关键文件路径速查

```
repo root: C:\Users\29368\Desktop\智能车\rak-car\
工作面:   main/arm/each_task/task7/  (target.py / dipan.py / position2.py)
共享:    main/arm/each_task/common.py:move_x_with_split
         main/arm/api/setters.py:ArmClient.set_hand_angle
         main/arm/loops/runner.py:ArmRunner.set_arm_angle / move_y
配置:    main/misc/llm_config.yml  (ERNIE token)
         config_car.yml (ernie_access_token 字段, fallback)
         main/settings.py:9  (DEFAULT_SERVER_ORIGIN = "http://192.168.5.230" ← 当前 Jetson IP)
```

## 8. 相关 memory 索引

- [[arm-api-reference]] —— ARM_API.md 速查
- [[arm-business-layer-only]] —— 业务层边界硬约束
- [[armrunner-set-hand-angle-gotcha]] —— **本会话踩的新坑**
- [[arm-grasp-call-arm-base]] —— grasp 调用坑
- [[x-axis-rollout-session]] —— x 轴全天 session
- [[x-axis-belt-slip]] —— x 轴 belt-slip 现象
- [[x-get-position-vs-realtime]] —— x_get_position 坏, 走 realtime
- [[x-split-mode-trial]] —— split 模式选择
- [[stream-cam-id-mapping]] —— cam2 = 侧摄 (config 与 stream URL 编号不一致)
- [[jetson-current-ip]] —— 当前 Jetson IP 192.168.5.230
- [[task4-rebuild-2026-07-22]] / [[task5-rebuild-2026-07-22]] —— 历史任务压缩文档参考格式

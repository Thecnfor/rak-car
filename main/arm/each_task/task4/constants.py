"""task4 业务常量 —— 单点真相, 不要在 a/b/c 文件里硬编。

来源:
  - bin 坐标: 用户口头拍板 (蓝=0, 黄=-65)
  - bin 姿态: 沿用 test_blue.py / test_yellow.py 步骤 5-7 (已实测)
  - y/hand/arm 区间: ARM_API.md §7.1 + §6.3
  - 球场几何: 用户口头 (左手边=放球, x 负=向左)
"""
from __future__ import annotations

# ---------- Bin 坐标 ----------
BLUE_BIN_X_MM: float = 0.0
YELLOW_BIN_X_MM: float = -65.0

# ---------- Y 位置 ----------
SAFE_Y_TRANSIT_MM: float = -190.0
"""持球转移 + idle 时的安全 y。必须出 y 保护区 [0, -80]。"""

BIN_OPEN_Y_MM: float = -150.0
"""开仓 y gate 上界（STORAGE_OPEN_Y_MAX_MM, 闭区间 inclusive）。
下界 -205. 在区间 [-205, -145] 内放过，区间外→raise。

⚠️ 实际下界比上界更优 (更远离触底，门位余量更大)，但当前物理位置实测
    y=-150 命中上界已可。开仓见 ARMClient.set_storage_angle 75° 的 Round 13/15 gate。
"""

PICK_Y_MM: float = -160.0
"""球心对应 y (4cm 球, 任务模型顶面)。
⚠️ **待现场校准** —— 见 plan §风险 1；plan 建议在 b2 加 move_y 试探循环
   从 -30 起每次 -5mm 找到命中点。先默认值 -160, 校准后改这里。
"""

# ---------- 姿态角度 ----------
ARM_INIT_DEG: float = 90.0
"""大臂 MID / 复位位 (+90°，2026-07-27 后 reset_position 默认角度，业务硬限上界)。"""

HAND_DOWN_DEG: float = 0.0
"""手爪朝下（朝地板）。set 时需要先 arm ≤ -30° 出联动保护区（api.py:583）。"""

HAND_INIT_DEG: float = -75.0
"""手爪"正前方" init 姿态。

⚠️ **实测偏差**：ARM_API.md §1.1 写 -90° 是 init, 但用户实测舵机实际识别角度是 -75°
（2026-07-22）。原因猜测: 舵机出厂标定与 SDK 写值差 15°。业务层用本常量, 不要再写 -90°.
ARMClient.set_hand_angle 业务硬限仍然是 [-90, 0]°, -75° 在范围内合法.
"""

# ---------- 舵机 ----------
STORAGE_OPEN_ANGLE_DEG: int = 75
"""开仓物理位。Round 13 y gate 触发。"""

STORAGE_CLOSE_ANGLE_DEG: int = 98
"""关仓物理位（任意角度都过，不触发 y gate）。"""

STORAGE_OPEN_SPEED: int = 5
"""开/关仓舵机速度。ARM_API §6.1 2026-07-21 user 改 10→5。"""

# ---------- X 轴限速 ----------
MOVE_X_V_MAX_MMS: float = 40.0
"""业务层默认限速（ARM_API §1.1 2026-07-22 修复后真生效）。
之前一直走 yaml 默认 400mm/s。"""

# ---------- 抓取 ----------
GRASP_HOLD_S: float = 1.0
"""吸气保持时长（grasp.py 当前 5s 太长，比赛节奏不允许）。
⚠️ 真空吸力验证缺失（plan §风险 3），建议先用 1.0s 然后现场调到
   grasp(True)+y 微抬 5mm 验证球跟上来。
"""

GRASP_TIMEOUT_S: float = 10.0

# ---------- 颜色 ----------
COLOR_BLUE: str = "blue"
COLOR_YELLOW: str = "yellow"
COLOR_UNKNOWN: str = "unknown"

# ---------- 球过滤（侧摄） ----------
# 2026-07-28 校准:基于 5 次 target2.py 实测黄色球数据 (cx~0.124, cy~-0.636,
# w~0.424, h~0.605, score~0.937, area~0.257)。旧值 0.5/0.6/0.003/0.60 太松,
# 噪声框都能过; 收紧到基于实测 ± 安全余量。
TARGET_SCORE_MIN: float = 0.85
"""最低置信度 (实测 0.927-0.941, 留 0.08 余量过滤噪声框)。"""

TARGET_ASPECT_TOL: float = 0.8
"""宽高比容差 (实测 w/h = 0.42/0.60 = 0.70, 留 ±0.10 容差)。
|aspect - 1.0| ≤ TARGET_ASPECT_TOL 即合格, 适配球比场景里略宽/略高的形变。"""

TARGET_AREA_MIN: float = 0.15
"""最小归一化面积。
- 历史 baseline 0.246-0.265 是 target1 y=-150 校准的, 留 0.04 buffer
- 2026-07-30: 现场 1↔2 球间歇, 小球 (被切边/压扁) area=0.158~0.183
  ↓ 0.20 会拒; 放宽到 0.15 (留 0.008-0.033 buffer)
- 0.15 已接近帧噪声框范围, 现场如有误检再考虑 aspect_tol 收紧
"""

TARGET_AREA_MAX: float = 0.60
"""最大归一化面积。
- 历史 baseline 0.246-0.265 是 target1 y=-150 校准的, 留 0.04 buffer
- 2026-07-30: 现场 2 球实测, 右球 area=0.457 完全可见, 0.30 会拒掉右球
  → 放宽到 0.50 (留 0.043 buffer 兼容近/远)
- 2026-08-03 P 姿态 (y=-100/x=-270) 实测: 球框 area=0.529, 0.50 拒掉
  → 放宽到 0.60 (留 0.07 buffer 兼容 P 姿态下的大球)
- 0.60 已接近帧噪声框范围, 现场如有误检再考虑 aspect_tol 收紧
"""

# ---------- 检测 / 选择策略 ----------

# ---------- 检测 / 选择策略 ----------
TARGET_DEDUP_NORM_MIN: float = 0.05
"""选 next 时, 跳过 cx_norm 距 0 < 此值的球 (上次画面中心已采的去重)。"""

TARGET_POLL_S: float = 0.2
"""b1 侧摄轮询 task_feed 间隔 (10Hz 守护线程)。"""

# ---------- belt-slip 兜底 ----------
X_SPEED_SAFETY_V_FALLBACK_MMS: float = 30.0
"""b2/b3 belt-slip fallback 用 x_speed_with_safety 的速度 (mm/s)。"""

X_SPEED_SAFETY_STALE_S: float = 2.0
"""x_speed_with_safety watchdog 超时 (s), 见 ARM_API §10。"""

# ---------- 任务总控 ----------
DEFAULT_MAX_ROUNDS: int = 8
"""一轮循环采一个球, 比赛正常 6-8 个球, 给点 buffer。"""

LOG_PREFIX_TASK4: str = "[task4]"

# ---------- 路径 ----------
import os
TASK4_LOGS_DIR: str = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".remember", "logs")
)
TASK4_TARGET_CACHE: str = os.path.join(TASK4_LOGS_DIR, "task4_target_latest.json")
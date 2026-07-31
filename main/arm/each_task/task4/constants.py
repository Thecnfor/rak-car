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

TARGET_AREA_MAX: float = 0.50
"""最大归一化面积。
- 历史 baseline 0.246-0.265 是 target1 y=-150 校准的, 留 0.04 buffer
- 2026-07-30: 现场 2 球实测, 右球 area=0.457 完全可见, 0.30 会拒掉右球
  → 放宽到 0.50 (留 0.043 buffer 兼容近/远)
- 2026-07-29 之前 0.50 → 2026-07-30 退到 0.30 → 现场拒大球 → 改回 0.50
"""

# ---------- Ball 验证基线 (target1.py 位姿下) ----------
# 2026-07-30 加测 (现场跑出**新最佳**黄球): cx=+0.026  cy=-0.748  w=0.534  h=0.505
#   score=0.939  area=0.270  aspect=1.057。
#   跟 2026-07-29 run 8 (cx=0.120, aspect=0.923) 差很多:
#   **cx 显著更居中** (0.026 vs 0.120, 差 0.094)
#   **aspect 跨过 1.0** (1.057 vs 0.923) — 球从"纵高>横宽"翻到"横宽>纵高"
#   **h 略矮** (0.505 vs 0.562) — 球被压成扁圆
#   可能 (a) 摄像头角度/位置变了 (b) 球场上球位置不同 (c) 球本身姿态变化。
#   **取 UNION 校准** (历史 + 7-29 + 7-30 + buffer), 噪声框仍会被 aspect_max/area_min 拦掉。
# 2026-07-29 加测 (历史新最佳): cx=+0.120  cy=-0.719  w=0.519  h=0.562
#   score=0.953  area=0.292  aspect=0.923。target1.py (y=-133, arm=+90°, hand=0°, x=-260) 已跑。
#   跟 2026-07-28 run 7 (cx=0.050, cy=-0.620, w=0.418, h=0.596, area=0.249, score=0.935, aspect=0.701)
#   差非常多: cy 更负 (球更远) + aspect 近正方 (历史 0.70 → 新 0.92) + w 更大 + h 略小。
# 2026-07-28 校准 (历史): target1.py (y=-133 现场实测, arm=+90°, hand=0°, x=-260)
#   跑完后, 侧摄识别球的**期望范围** (蓝黄共用, 几何一致, 仅 color 字段不同)。
#   给 target4 / 单元测试用 —— 跑完 target1 后立刻 target2, 球的检测应该
#   落在这些区间内; 不在就说明 target1 位姿偏移或侧摄装错。
# 9 次黄色球实测 (蓝球几何与黄球一致, 2026-07-28 user 确认):
#   [历史 y=-150, 1-6 次]  cx 0.058~0.173 (mean=0.124)  cy -0.675~-0.584 (mean=-0.636)
#     几何 w/h/aspect/area 全在 [0.40, 0.44] / [0.59, 0.62] / [0.24, 0.27] / [0.60, 0.80] 区间。
#   [历史 y=-133, 第 7 次]  cx=+0.050  cy=-0.620  w=0.418  h=0.596  score=0.935  aspect=0.701
#     ↑ cx 0.050 撑爆旧 CX_MIN=0.05, 加宽到 0.04 留 buffer
#   [7-29 新最佳 y=-133, 第 8 次]  cx=+0.120  cy=-0.719  w=0.519  h=0.562  score=0.953  area=0.292  aspect=0.923
#     ↑ cx/cy/w/h/area/aspect 全部超出 7 次基线区间, 取 UNION + buffer
#   [7-30 新最佳 y=-133, 第 9 次]  cx=+0.026  cy=-0.748  w=0.534  h=0.505  score=0.939  area=0.270  aspect=1.057
#     ↑ cx 显著更居中 + aspect 跨 1.0 (横宽>纵高) + h 略矮, 取 UNION + buffer
# 用 min/max + buffer 表达, 比 mean ± 1σ 更贴近实测两端 (1σ 太紧, 5 次里
# 2 次 (r1/r4) 落在边界外)。
# ⚠️ 2026-07-30 范围放宽警告: h_min=0.48 / aspect_max=1.10 已接近噪声框阈值
#   (h≤0.5 已是噪声范围, aspect>1.0 几何上不太圆), 现场如有误检请用
#   `--aspect-tol` / `--score-min` 临时收紧, 而非动 constants。
BALL_VERIFIED_CX_MIN: float = 0.02        # 2026-07-29: 新最佳 cx=0.120 + 历史 cx=0.050 → UNION 加 buffer [0.04→0.02, 0.18→0.20]
BALL_VERIFIED_CX_MAX: float = 0.20
BALL_VERIFIED_CY_MIN: float = -0.78       # 2026-07-29: 新最佳 cy=-0.719 → 加宽 [-0.68→-0.78, -0.58→-0.55]
BALL_VERIFIED_CY_MAX: float = -0.55
BALL_VERIFIED_W_MIN: float = 0.35         # 2026-07-29: 新最佳 w=0.519 → 加宽 [0.40→0.35, 0.44→0.56]
BALL_VERIFIED_W_MAX: float = 0.56
BALL_VERIFIED_H_MIN: float = 0.48         # 2026-07-30: 新最佳 h=0.505 (aspect=1.057 横宽>纵高) → 加宽 [0.55→0.48]; 0.48 已是噪声框边界
BALL_VERIFIED_H_MAX: float = 0.65
BALL_VERIFIED_AREA_MIN_VERIFY: float = 0.20       # 2026-07-29: 新最佳 area=0.292 → 加宽 [0.24→0.20, 0.27→0.35]
BALL_VERIFIED_AREA_MAX_VERIFY: float = 0.35
BALL_VERIFIED_SCORE_MIN_VERIFY: float = 0.80       # 2026-07-28: 旧 0.92 临界擦线 (实测 0.924), 放宽到 0.80; 2026-07-29/30 不动
BALL_VERIFIED_ASPECT_MIN: float = 0.55
"""2026-07-30: 新加测 aspect=1.057 (横宽>纵高, 反 7-29 0.923 趋势) → 加宽 [0.95→1.10]。
实测 7 历史 + 7-29 + 7-30 共 9 次都覆盖, 蓝黄共用。"""
BALL_VERIFIED_ASPECT_MAX: float = 1.10

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
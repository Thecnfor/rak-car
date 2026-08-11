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
YELLOW_BIN_X_MM: float = -45.0

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
"""大臂 MID / 复位位 (+90°，2026-07-27 后 reset_position 默认角度；业务硬限上界 +150°，2026-08-05 放宽)。"""

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
TARGET_SCORE_MIN: float = 0.6
"""最低置信度 (2026-08-05 用户: 0.85→0.72→0.6。
实测真球 0.85, 鬼影 0.62, 留 ~0.1 边际。creep 段前移运动模糊 score 进一步下降,
0.85 边缘卡阈值; 0.6 落在真球/鬼影中间, 推进留余量, track_chassis 按 label 独立选目标不受影响。"""

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
- 2026-07-29 之前 0.50 → 2026-07-30 退到 0.30 → 现场拒大球 → 改回 0.50
- 历史 baseline 0.246-0.265 是 target1 y=-150 校准的, 留 0.04 buffer
- 2026-07-30: 现场 2 球实测, 右球 area=0.457 完全可见, 0.30 会拒掉右球
  → 放宽到 0.50 (留 0.043 buffer 兼容近/远)
- 2026-08-03 P 姿态 (y=-100/x=-270) 实测: 球框 area=0.529, 0.50 拒掉
  → 放宽到 0.60 (留 0.07 buffer 兼容 P 姿态下的大球)
- 0.60 已接近帧噪声框范围, 现场如有误检再考虑 aspect_tol 收紧

⚠️ 2026-08-02 取消 BALL_VERIFIED_* 7 项位置验证 (用户要求 "识别到球就抓, 不用在
最佳抓取位置"): TARGET_* 4 项基础过滤 (score / aspect / area) 仍生效, 但
target2.fetch_balls 不再调用 _verify_ball_in_target1_pose。球检测只要过
TARGET_* 4 项 + color 是蓝/黄就视为有效候选。
"""

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


# ============================================================================
# target4 —— 慢速前移搜索 + 底盘视觉定位 (从 target4.py 下沉, 2026-08-10 拆分)
# ============================================================================

# ---- 预算 (2026-08-10 用户拍板: 冻结为模块默认值, 不再经 step_target4 参数化) ----
DEFAULT_MAX_PICKS: int = 1000
"""最多抓取数 (距离优先模式下设为极大值, 实际不限制)。"""

DEFAULT_MAX_CREEP_M: float = 0.58
"""累计前移距离预算 (m, 开环 速度×时间 记账)。唯一实际生效的终止条件。"""

DEFAULT_MAX_SECONDS: float = 9999.0
"""任务总时长预算 (s) (距离优先模式下设为极大值, 实际不限制)。"""

DEFAULT_CREEP_SPEED_MPS: float = 0.05
"""creep 搜索前移速度 (m/s)。2026-08-10 用户: 0.12 → 0.05 (慢扫, 减运动模糊)。"""

CREEP_POLL_HZ: float = 20.0
"""creep 期间 fetch_balls 轮询频率。"""

CREEP_MAX_SECONDS_S: float = 30.0
"""主循环 wait_for_ball 的单轮等待兜底 (s): 见球/走满距离预算都不满足时的保险。
2026-08-10 删除了 creep 线程内部的墙钟上限, 本常量只作调用方的等待耐心。"""

DEFAULT_TRACK_MAX_SECONDS: float = 4.0
"""单球底盘视觉伺服收敛预算 (s)。2026-08-10 用户: 6 → 4。"""

DEFAULT_MAX_CONSECUTIVE_PICK_FAILURES: int = 1000
"""连续 pick 失败超过此数 → 退出 (距离优先模式下设为极大值, 实际不限制)。"""

DEFAULT_MAX_CONSECUTIVE_TRACK_FAILURES: int = 2
"""连续 track 失败超过此数 → 退出。"""

DEFAULT_PICK_TIMEOUT_S: float = 60.0
"""pick_by_vision 总超时 (s)。"""

DEFAULT_TRACK_SOFT_DEADBAND: float = 0.15
"""track_chassis 软死区 (cx_err/cy_err 绝对值 < 此值视为"接近对齐")。"""

DEFAULT_TRACK_RETRY_SECONDS: float = 1.0
"""软收敛额外 time budget (s). near_arrived 时再给 <1s 用更大 deadband 重试. """

DEFAULT_TRACK_WIDE_DEADBAND: float = 0.35
"""track_chassis 宽死区 (cx_err/cy_err 绝对值 < 此值视为"近似对齐可以一试")。"""

from main.arm.each_task.common import POSE_P_X_MM  # noqa: E402
DEFAULT_RETURN_X_MM = POSE_P_X_MM
"""放 bin 后 x 回的目标位置 (mm)。默认 = POSE_P_X (P 姿态 x), None = 不回。"""

# ---- P 姿态参数 (可由外部覆盖) ----
# 2026-08-11 用户拍板: y -180 → -150 (同时生效开场摆位 + 每球摆位)
TASK4_POSE_P_Y_MM: float = -150.0
# 2026-08-10 用户拍板: -295 → -250 (远离物理墙 -295, 留余量; 同时生效开场摆位 + 每球摆位)
TASK4_POSE_P_X_MM: float = -250.0
TASK4_POSE_P_ARM_DEG: float = 90.0
# 2026-08-11 用户: 初始姿势 hand 10 → -10 (末端 -10); 放球角单独固定, 不跟这里
TASK4_POSE_P_HAND_DEG: float = -10.0

# ---- 开始阶段 (2026-08-10 用户拍板: 触发任务点后三步并发) ----
#    三步 = 四轴联动到 P 姿态 (已到位则跳过) ∥ lane 前进 0.1m ∥ 开仓 75°。
START_LANE_FORWARD_M: float = 0.1
"""触发任务点后沿车道前进距离 (m)。<=0 时跳过前进。"""
START_LANE_FORWARD_VX_MPS: float = 0.05
"""lane-follow 前进速度 (m/s)。2026-08-11 用户: 0.15 → 0.05 (慢)。"""

# 四轴联动"已在 P 姿态就跳过"的容差 (mm / °)。
P_POSE_SKIP_TOL_X_MM: float = 10.0
P_POSE_SKIP_TOL_Y_MM: float = 10.0
P_POSE_SKIP_TOL_ARM_DEG: float = 5.0
P_POSE_SKIP_TOL_HAND_DEG: float = 5.0

# ---- 临时调试开关 (2026-08-11 用户) ----
ALIGN_ONLY: bool = False
"""True = 只对准底盘/机械臂、不抓取 (调试用); False = 正常抓取流程。"""

# ============================================================================
# 新版流程 (2026-08-11 用户确认): 启动三步并发 → 第一球(蠕动→底盘对齐→臂伺服→抓放)
# → 后续球(0.1m 前进∥臂回初始 → 找球 → 臂伺服 → 抓放) → 左IR>0.75 退出 (8 球封顶)
# ============================================================================

# ---- 第一球: 蠕动 (占位终止, TODO 待讨论) ----
FIRST_CREEP_MAX_M: float = 5.0
"""第一球蠕动距离占位上限 (m)。TODO: 真正终止条件待用户后续讨论,
先给大值 + 主循环 40s 等待兜底 (无球 → 占位扫空收工)。"""

# ---- 底盘对齐 (4s → 超时加时 3s → 失败也继续) ----
TRACK_EXTEND_SECONDS: float = 3.0
"""底盘对齐超时后的加时 (s): 总上限 = DEFAULT_TRACK_MAX_SECONDS(4) + 3 = 7s。"""

# ---- 机械臂视觉伺服 (runtime run_arm_servo, 只动 x 十字 + 大臂) ----
ARM_SERVO_SETPOINT_CX: float = 0.045
"""机械臂伺服 cx 目标 (归一化, 大臂纠正 cx 到该值)。"""
ARM_SERVO_SETPOINT_CY: float = -0.083
"""机械臂伺服 cy 目标 (归一化, x 十字纠正 cy 到该值)。"""
ARM_SERVO_GAIN_ARM: float = 0.2
ARM_SERVO_GAIN_X: float = 0.2
ARM_SERVO_DEADZONE: float = 0.05
ARM_SERVO_RETRY_DEADZONE: float = 0.075
"""超时重试死区 = 0.05 × 1.5 (放大 0.5 倍, 更容易锁上)。"""
ARM_SERVO_MAX_VEL: float = 0.05
ARM_SERVO_TIMEOUT_S: float = 8.0
ARM_SERVO_RETRY_TIMEOUT_S: float = 4.0
"""超时加时 4s, 总上限 12s。"""
ARM_SERVO_SETTLE_HITS: int = 3
ARM_SERVO_SIGN_ARM: float = 1.0
ARM_SERVO_SIGN_X: float = 1.0
"""sign_arm / sign_x: 方向符号, 现场标定 (反了改 -1)。"""
ARM_SERVO_ARM_START: float = 90.0
ARM_SERVO_ARM_MIN: float = 60.0
ARM_SERVO_ARM_MAX: float = 130.0
ARM_SERVO_HZ: float = 20.0

# ---- 抓放序列 ----
PICK_LOWER_Y_MM: float = -40.0
"""伺服对齐后纵臂下降的吸附高度 (mm)。"""
PICK_SUCK_HAND_DEG: float = 0.0
"""盲降吸取时手爪角度 (0 = 吸嘴朝下贴球顶面, 同 task2 descend_hand_deg)。"""
PICK_HOLD_S: float = 0.5
"""吸住后保持时长 (s)。"""
PICK_LIFT_Y_MM: float = -150.0
"""吸住后抬升 y (mm)。"""
PICK_BIN_ARM_DEG: float = 95.0
"""放 bin 时大臂回的角度 (跟 x 轴一起转)。"""
PICK_RELEASE_Y_MM: float = -130.0
"""放球 y (mm)。2026-08-11 用户: -140 → -130。"""
PICK_RELEASE_HAND_DEG: float = 10.0
"""放球手爪角 (蓝/黄都 10, 原值, 不跟 P hand)。"""

# ---- 后续球扫描 ----
SCAN_ADVANCE_M: float = 0.08
"""每抓完一颗沿车道线前进距离 (m)。2026-08-11 用户: 0.10 → 0.08, 且改用 lane_follow。"""
SCAN_LOOK_S: float = 3.0
"""每停一站找球上限 (s)。"""
SCAN_GRAB_CX_HALF: float = 0.4
"""可抓窗口半宽 (前段): |cx - ARM_SERVO_SETPOINT_CX| ≤ 此值才尝试抓。
2026-08-11 用户: 0.2 → 0.4 (实车连续"未见可抓球"空轮, 放宽)。"""
SCAN_GRAB_CX_HALF_LATE: float = 0.6
"""可抓窗口半宽 (后段, 前 SCAN_GRAB_TIER_ADVANCES 次前进之后用): 放宽到 0.6
(2026-08-11 用户: 梯度窗口 —— 前段球正用窄窗精确, 后段球散布宽用大窗多抓)。"""
SCAN_GRAB_TIER_ADVANCES: int = 4
"""前 N 次前进用 SCAN_GRAB_CX_HALF (窄窗), 之后的用 SCAN_GRAB_CX_HALF_LATE (宽窗)。"""
SCAN_MAX_PICKS: int = 8
"""DRY-RUN 专用抓球数封顶 (占位球抓满即停, 防空转); 实车不封顶 (2026-08-11 用户)。"""
MIN_SCAN_ADVANCES: int = 7
"""扫描退出门槛: 至少前进这么多轮后, 才允许按"连续无球"条件退出 (2026-08-11 用户)。"""
SCAN_EMPTY_ROUNDS: int = 2
"""连续找球为空轮数达到此值 → 判定采区扫完收工 (2026-08-11 用户, 不依赖 IR)。"""

# ---- 抓取 / 中转位姿 ----
X_PICK_MM: float = -240.0
"""盲降前横移 x (mm)。"""

Y_PICK_MM: float = -55.0
"""抓球 y (吸盘贴近球面)。"""

Y_TRANSIT_MM: float = -140.0
"""中转 y (放仓位之前的过渡位)。"""

X_TRANSIT_MM: float = -220.0
"""中转 x (车体中线附近, 两次小位移降低 belt-slip 风险)。"""

# ---- 放 bin 参数 ----
Y_PUT_MM: float = -140.0
"""放球 y (再深 10mm 防脱落)。"""

BIN_X_MM = {COLOR_BLUE: 0.0, COLOR_YELLOW: -60.0}
"""蓝 bin x=0, 黄 bin x=-70。"""

BIN_Y_MM = {COLOR_BLUE: -140.0}
"""蓝 bin y=-135; 黄沿用 Y_PUT_MM。"""

BIN_HAND_DEG = {COLOR_BLUE: 10.0, COLOR_YELLOW: 10.0}
"""放球手爪角: 蓝/黄都 10° (2026-08-11 用户: 沿用原值, 不跟 P hand -10)。"""

# ---- 其他 ----
Y_FINAL_MM: float = -140.0
"""最终 y (识别位姿, 历史值)。"""

BALL_LABELS = ["ball_blue", "ball_yellow"]
"""track_chassis 目标集 (PaddleDet 模型标签)。"""

# ---- 时间戳辅助 (跨 target4 各子模块共享) ----
import time as _time  # noqa: E402
_TASK4_T0 = None


def reset_ts(t0) -> None:
    """把 task4 时间戳起点重置为 t0 (step_target4 启动时调用)。"""
    global _TASK4_T0
    _TASK4_T0 = t0


def _ts_str() -> str:
    """距 task4 启动的秒数, 打在每个动作前定位每步延迟。"""
    global _TASK4_T0
    if _TASK4_T0 is None:
        _TASK4_T0 = _time.monotonic()
    return f"t=+{_time.monotonic() - _TASK4_T0:.1f}s"


LOG_PREFIX_TARGET4: str = LOG_PREFIX_TASK4 + "/target4"
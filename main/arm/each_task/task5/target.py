"""task5 / target —— 把机械臂摆到指定目标位姿, 然后识别高仓示意颜色。

按用户指定顺序 (2026-07-30 改: move_y 放第一步, 出保护区):
  1. move_y(-100mm)       y 抬到 -100mm (move_y 走 y 步进电机, 允许保护区 [0, -30] 内调)
  2. move_x(0mm)         x 轴回中 (belt-slip 分段 + realtime 校验; y 已 -100, 保护区外)
  3. set_arm_angle(+90°)  大臂到复位位 (init 例外位, 保护区允许)
  4. set_hand_angle(-45°) 手爪到 -45° (中间位, 业务硬限内 [-90, 0] 正常位置)
  5. detect_high_tower_color()  抓侧摄 JPEG → HSV 阈值 → 蓝/黄/未知

⚠️ **前提条件变更 (2026-07-30)**:
   - 之前: y 必须 < -30 (move_x 第一步会拦)
   - 现在: 任意 y 都可以 (move_y 放第一步会自己出保护区)
   - 如果调用时 y 在 [0, -30] 保护区内, 第 1 步 move_y(-100) 会先把它抬出去, 后续动作都安全。

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner) + cv2/numpy,
   不 import task5 包内其它模块 (constants / grasp_5 / *_tower)。原因: task5
   目录里的辅助文件曾被外部动作清空过 (见会话记录), 自包含可保证
   `python target.py` 直接跑不受影响。
⚠️ x 位置一律走 _read_x_mm_realtime() 校验 (x_get_position 坏, ARM_API §11)。

跑法 (两种都行):
    python main/arm/each_task/task5/target.py
    python -m main.arm.each_task.task5.target
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.common import move_x_with_split  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402


# ---------- 目标位姿常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/target]"

TARGET_X_MM: float = 0.0
"""x 目标位 0mm (滑轨中段, belt-slip 风险低)。"""

TARGET_ARM_DEG: float = 90.0
"""大臂复位位 +90° (业务硬限上界, 2026-07-27 v3 重定义)。"""

TARGET_HAND_DEG: float = -45.0
"""手爪 -45° (中间位, 业务硬限 [-90, 0] 范围内)。
2026-07-30 从 -90 (UP) 改为 -45 (mid), 因为色标识别时手爪要略放平避开画面底部。
不是 init 例外位 (init 是 -90), 也不是业务硬限边界。"""

TARGET_Y_MM: float = -100.0
"""y 抬到 -100mm (不再触底)。
2026-07-30 从 0 (触底归零) 改为 -100 (抬 100mm, 给色标识别一个稳定视角, 避免
手爪吸盘遮画面下沿)。仍然在 y 保护区外 [-200, -30] 范围, move_y 允许。
注意: 改这个后, 如再调回 0 (触底) 需确认车端 y_limit 仍报 True (磁感 home 位)。"""

# belt-slip 安全 move_x 参数 (走 high_tower.py / test_x_to_150.py 模式)
MOVE_X_TOL_MM: float = 5.0          # 到位容差 (realtime 抖动 <1mm, 放宽给 PID 余量)
MOVE_X_V_MAX_MMS: float = 30.0      # 业务限速 (2026-07-22 限速透传 bug 修复后定档 30)
MOVE_X_MAX_ROUNDS: int = 12         # 最多尝试轮数
MOVE_X_STALL_MM: float = 3.0        # 本轮位移 < 此值视为卡住 (疑似打滑)
MOVE_X_MAX_STALL_ROUNDS: int = 3    # 连续卡住这么多轮 → 放弃
MOVE_X_KICK_SLEEP_S: float = 0.2    # kick: 停一下让同步带齿重新咬合

# 2026-07-30: _move_x_with_split 已抽离到 main.arm.each_task.common
# (本地保留别名, 兼容 run() 调用 + 历史 log 习惯)
# 见 main/arm/each_task/common.py:move_x_with_split 完整 docstring + wall/overshoot 检测


# ---------- 高仓示意颜色识别 (HSV 阈值, 自包含) ----------

# OpenCV HSV 范围 (H ∈ [0, 179])
HSV_BLUE_LO = (100, 50, 50)
"""蓝色下界 (H, S, V) — 蓝球 / 蓝色标 通用"""
HSV_BLUE_HI = (130, 255, 255)
"""蓝色上界"""
HSV_YELLOW_LO = (20, 50, 50)
"""黄色下界"""
HSV_YELLOW_HI = (35, 255, 255)
"""黄色上界"""

COLOR_DECISION_MARGIN: float = 0.10
"""主色判定: max_ratio - other_ratio > 此值才认, 否则 unknown"""
COLOR_MIN_RATIO: float = 0.05
"""主色最低像素占比 (< 此值判 unknown, 防噪声)"""

DEFAULT_CAM: str = "cam2"
"""默认相机: cam2 = 侧摄 (stream 服务 URL 命名)。

⚠️ **cam_id 两套编号不一致**:
  - config_car.yml: front=2 / side=1 (视频设备号, OpenCV /dev/videoN)
  - runtime/stream 服务 URL: /stream/frame/cam{N}.jpg 中的 cam2 = side camera
  - 本字段用 **stream URL 命名**, 不是 config_car.yml 的设备号
  - 现场验证: 看 cam1.jpg / cam2.jpg 哪张是 side 视角
"""
JPEG_FETCH_TIMEOUT_S: float = 5.0
"""单帧 JPEG 抓取 HTTP 超时"""

# ⚠️ 2026-07-30 v2: 默认 ROI 改为 **None (auto findContours)**。
# 旧 ROI (40, 50, 110, 100) 在 7-30 现场测过色标在左上; 但用户提供的**新截图**
# 显示色标在画面**中央偏上** (x≈440-540, y≈180-280), 旧 ROI 只框到蓝金属框
# → blue=25%, yellow=0% → 误判 blue (用户报告: "明明是黄色, 识别成蓝色")。
#
# **新策略**: 默认走 auto_find_roi=True → 全图 HSV mask → findContours 找
# 中等面积 + 长宽比 ~1 + 占比 <5% 的色标候选, 取最显著那个 bbox 当 ROI。
# 不再依赖固定坐标。DEFAULT_ROI 仍保留 (现场实测过的左上坐标), 可用 --roi 显式传。

DEFAULT_ROI: Optional[tuple] = None
"""默认 ROI = None → 走 auto_find_roi 自动找色标位置。

如果现场 auto 不稳 (例如色标被遮挡 / 反光干扰), 可用 --roi X Y W H 显式传。
参考值 (7-30 现场 cam2 实测): (40, 50, 110, 100) = 左上色标。
**仅当色标稳定在左上时**才用; 色标位置一变就失效。
"""


def _fetch_camera_jpeg(client: ArmClient, cam: str, timeout: float) -> bytes:
    """从 runtime 抓一帧 JPEG。URL: {api_base}/stream/frame/{cam}.jpg (runtime/api/routes.py:606)。

    ⚠️ **不能走 client.http.get()**:
      - api_client.py:47 _request 末尾会调 response.json() (返回 dict),
        对 image/jpeg 二进制直接 JSONDecodeError 炸。
      - 所以这里直接用 requests 拉 raw bytes。

    ⚠️ **路由在 root, 不在 /v1**:
      - runtime/api/routes.py:606 @router.get("/stream/frame/{cam_id}.jpg")
        是 root router (create_runtime_router 行 581), 不带 /v1 prefix。
      - 所以 URL = {api_base}/stream/frame/{cam}.jpg, 不能拼 api_prefix。

    ⚠️ **api_base 是 @property** (api_client.py:22-24), 无括号。

    Returns:
        JPEG bytes; 失败抛 RuntimeError。
    """
    import requests  # 局部 import, 不污染模块顶部 (业务层不一定需要 requests)
    base = client.http.api_base  # @property, 无括号
    url = f"{base}/stream/frame/{cam}.jpg"
    try:
        resp = requests.get(url, timeout=timeout)
        resp.raise_for_status()
    except Exception as e:
        raise RuntimeError(f"GET {url} 失败: {type(e).__name__}: {e}") from e
    jpeg = resp.content
    if len(jpeg) < 100:
        raise RuntimeError(f"GET {url} 返回 bytes 过短 ({len(jpeg)}B), 可能是错误页")
    return jpeg


# ============================================================================
# 色标自动定位 (2026-07-30 v2: 不再依赖固定 ROI)
# ============================================================================
#
# 旧问题: 写死 ROI (40, 50, 110, 100) 在 7-30 现场测过色标在画面左上, 但用户给的新
# 截图 (Image #7) 显示色标在画面**中央偏上** (x≈440-540, y≈180-280), 旧 ROI
# 只框到蓝金属框, blue 比例 25%, yellow 0%, 误判 blue。
#
# 解决: 全图 HSV mask → findContours → 过滤掉大球/金属框(大+占比高)/噪声(小)/
# 长条(长宽比大), 找最显著的小色块。

AUTO_TAG_MIN_AREA: int = 300
"""色标候选最小面积 (px²)。色标 ~100x100=10000, 留 300 容错。"""
AUTO_TAG_MAX_AREA: int = 30000
"""色标候选最大面积 (px²)。

⚠️ 2026-07-30 v3 改: 8000 → 30000。
实测 (Image #9/#10) 蓝金属框/大球可能 15000-70000px, 但**真色标也经常 12000-20000px**
(色标 100x100=10000 + 边缘反光/阴影让 cv2.contourArea 算大 1.5-2x)。
原 8000 把真色标在 Image #9 (area=16214) 过滤掉, 选了黄色噪声 (area=5034)
→ 误判 yellow。
新 30000 留够空间给色标反光, 同时 5% 占比过滤仍挡大球/大框(~60000px 占 5% 画面)
"""
AUTO_TAG_MAX_ASPECT: float = 2.5
"""色标候选最大长宽比。色标是方块, 长条/线条被排除。"""
AUTO_TAG_MAX_AREA_RATIO: float = 0.05
"""色标候选最大占画面比例 (< 5%)。大球/金属框占画面 10%+ 被排除。"""
AUTO_TAG_MIN_RATIO: float = 0.50
"""⚠️ 2026-07-30 v3 新增: 候选 bbox 内主色最低占比 (< 50% 排除)。

为什么需要:
  - 单独用面积 (v1) → 选大框/球
  - 单独用 ratio (v2) → 小候选天然 ratio 高, 盖过色标
  - 单独用绝对像素数 (v3 早期) → 大框像素数仍可能盖过色标
  - **ratio 50% 过滤 + 像素数选** → 真色标 ratio 60-70% 全过, 金属框/球
    ratio 20-50% 全部排除, 像素数选时真色标胜

实测 (Image #7/9/10):
  - 真色标:  ratio 60-70% (过)
  - 金属框:  ratio 20-50% (排除)
  - 小候选:  ratio 80%+ 但面积小, 像素数 < 色标 (被自然压)
  - 球:      ratio 50-65% + 5% 面积过滤 → 通常排除
"""


AUTO_TAG_MIN_RATIO: float = 0.50
"""⚠️ 2026-07-30 v3 新增: 候选 bbox 内主色最低占比 (< 50% 排除)。

实际写在 _find_color_tag_bboxes() 内部 rank 阶段使用, 减少误选。
"""


def _find_color_tag_bboxes(mask: np.ndarray, _min_ratio: float = AUTO_TAG_MIN_RATIO) -> list:
    """在 HSV mask 上找**所有**色标候选 bbox (按面积降序)。

    过滤规则 (排除噪声 / 金属框 / 大球 / 线条):
      - area ∈ [AUTO_TAG_MIN_AREA, AUTO_TAG_MAX_AREA]
      - 长宽比 < AUTO_TAG_MAX_ASPECT
      - 占画面 < AUTO_TAG_MAX_AREA_RATIO
      - bbox 内主色占比 ≥ _min_ratio  (v3 新增, 默认 50%)

    Returns:
        list of (x, y, w, h) 按面积降序; 没找到返回 []。
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    H, W = mask.shape
    total = H * W
    candidates: list = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        if area < AUTO_TAG_MIN_AREA or area > AUTO_TAG_MAX_AREA:
            continue
        ratio = max(w, h) / max(min(w, h), 1)
        if ratio > AUTO_TAG_MAX_ASPECT:
            continue
        if total > 0 and area / total > AUTO_TAG_MAX_AREA_RATIO:
            continue
        # v3 新增: bbox 内主色占比 < _min_ratio → 排除
        # 选元组里同时算 ratio, 避免外面再算一遍
        bbox_area = w * h
        if bbox_area > 0:
            crop = mask[y:y + h, x:x + w]
            mask_pixels = float(cv2.countNonZero(crop))
            in_ratio = mask_pixels / bbox_area
        else:
            in_ratio = 0.0
        if in_ratio < _min_ratio:
            continue
        candidates.append((area, int(x), int(y), int(w), int(h)))
    candidates.sort(reverse=True)  # 最大面积在前
    return [(x, y, w, h) for _, x, y, w, h in candidates]


def _find_color_tag_bbox(mask: np.ndarray) -> Optional[tuple]:
    """单选助手: 返回最大面积候选 (向后兼容)。**新代码请用 _find_color_tag_bboxes**。

    v3 起 detect_high_tower_color 改用 _find_color_tag_bboxes + 按 score 选,
    单选 API 仅保留供其他模块调用。
    """
    candidates = _find_color_tag_bboxes(mask)
    return candidates[0] if candidates else None


def detect_high_tower_color(
    client: ArmClient,
    *,
    cam: str = DEFAULT_CAM,
    roi: Optional[tuple] = DEFAULT_ROI,
    auto_find_roi: bool = True,
    timeout: float = JPEG_FETCH_TIMEOUT_S,
) -> dict:
    """高仓示意颜色识别: 抓侧摄 JPEG → HSV 阈值 → 判定主色 (蓝/黄/未知)。

    物理假设 (PPT Slide 10):
      - 高仓有"颜色标签" (蓝或黄), 单色块
      - 侧摄 cam2 能看到高仓标签
      - 色标位置随场地/tower/相机角度变化, 不可写死 ROI

    算法 (2026-07-30 v3 改进: 默认 auto find 色标位置, 选所有候选中 score 最高的):
      1. GET /stream/frame/{cam}.jpg 拿单帧 JPEG
      2. cv2.imdecode → BGR
      3. BGR → HSV (全图)
      4. 蓝掩码 (H∈[100,130], S,V≥50) + 黄掩码 (H∈[20,35], S,V≥50)
      5. **决定 ROI**:
         - 如果显式传 roi → 用 roi
         - 否则如果 auto_find_roi=True → 全图 findContours 拿**所有**色标候选
           (过滤: 面积 300-30000 / 长宽比 <2.5 / 占比 <5%), **每个候选算 bbox 内主色占比
           (=score)**, 选**全局 score 最高的**那个 bbox
         - 否则 → 全图
      6. 按 ROI 裁剪 mask → 算 blue_ratio / yellow_ratio
      7. 判定:
         - max(r_b, r_y) < COLOR_MIN_RATIO              → "unknown"
         - |r_b - r_y| < COLOR_DECISION_MARGIN            → "unknown" (难以区分)
         - r_b > r_y                                     → "blue"
         - else                                          → "yellow"

    Args:
        client: ArmClient (用 .http 抓 JPEG)
        cam: "cam1" (side) 或 "cam2" (front)
        roi: (x, y, w, h) 像素坐标, None = 走 auto_find_roi
        auto_find_roi: 当 roi=None 时, 是否全图自动找色标 (默认 True)
        timeout: JPEG 抓取超时 (秒)

    Returns:
        {
            "color": "blue" | "yellow" | "unknown",
            "blue_ratio": float,
            "yellow_ratio": float,
            "blue_pixels": int,
            "yellow_pixels": int,
            "total_pixels": int,
            "frame_shape": (h, w, c),
            "roi": (x, y, w, h) | None,            # 入参 (没改)
            "roi_used": (x, y, w, h) | None,       # 实际使用的 ROI (auto 或显式)
            "auto_find": bool,                       # 是否走了 auto 流程
            # v3 新增: 所有候选 + score (调试用)
            "auto_blue_bboxes": list[tuple],         # 所有蓝色候选 (按面积降序)
            "auto_yellow_bboxes": list[tuple],       # 所有黄色候选
            "auto_blue_scores": list[float],         # 蓝色候选 score (=bbox 内主色占比)
            "auto_yellow_scores": list[float],
            "winner_color": "blue" | "yellow" | None,  # 赢的候选属于哪个颜色
            "winner_score": float,                   # 赢的分数
            "winner_bbox": tuple | None,             # 赢的 bbox
            # v2 兼容字段: 单数 (取每个颜色最大候选)
            "auto_blue_bbox": tuple | None,
            "auto_yellow_bbox": tuple | None,
            "auto_blue_score": float,                # 最大候选的 score
            "auto_yellow_score": float,
            "cam": str,
            "elapsed_ms": float,
            "raw_jpeg_bytes": int,
        }
    """
    t0 = time.perf_counter()

    # 1. 抓 JPEG
    jpeg_bytes = _fetch_camera_jpeg(client, cam, timeout)
    raw_len = len(jpeg_bytes)

    # 2. 解码
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"cv2.imdecode 失败, JPEG bytes={raw_len}")

    H, W = frame.shape[:2]

    # 3. 全图 HSV + 蓝黄 mask (auto find 需要全图扫)
    hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue_mask_full = cv2.inRange(hsv_full, HSV_BLUE_LO, HSV_BLUE_HI)
    yellow_mask_full = cv2.inRange(hsv_full, HSV_YELLOW_LO, HSV_YELLOW_HI)

    # 4. 决定实际 ROI
    # 优先级: 显式 roi > auto_find > None(全图)
    auto_blue_bboxes: list = []         # 所有蓝色候选 (按面积降序)
    auto_yellow_bboxes: list = []       # 所有黄色候选
    auto_blue_scores: list = []         # 蓝色候选的 score (=bbox 内蓝色**绝对像素数**)
    auto_yellow_scores: list = []       # 黄色候选的 score
    auto_blue_ratios: list = []         # 蓝色候选的 bbox 内主色占比 (备查)
    auto_yellow_ratios: list = []       # 黄色候选的 bbox 内主色占比
    winner_bbox: Optional[tuple] = None
    winner_color: Optional[str] = None  # 赢的那个候选属于哪个颜色
    winner_score: float = 0.0
    roi_used: Optional[tuple] = None
    auto_find_applied = False

    if roi is not None:
        # 4a. 调用方显式传 ROI → 直接用
        roi_used = roi
    elif auto_find_roi:
        # 4b. auto 模式: findContours 找蓝/黄**所有**色标候选
        # ⚠️ 2026-07-30 v3: 遍历所有候选, 用**主色绝对像素数**做 score 选最高的。
        # v2 用 "ratio" 选时失灵: 小候选 (28x16) ratio=78% 盖过真色标 (87x87) ratio=66%,
        # 因为小候选主色占比天然高。**绝对像素数**让 "中等面积 + 高占比" 的真色标胜出,
        # 同时压住 "小+高占比" 和 "大+低占比" 两类噪声。
        auto_blue_bboxes = _find_color_tag_bboxes(blue_mask_full)
        auto_yellow_bboxes = _find_color_tag_bboxes(yellow_mask_full)
        auto_find_applied = True

        # 算每个候选的 score (=bbox 内主色**绝对像素数**, 不是 ratio)
        for bx, by, bw, bh in auto_blue_bboxes:
            bx = max(0, min(int(bx), W - 1)); by = max(0, min(int(by), H - 1))
            bw = max(1, min(int(bw), W - bx)); bh = max(1, min(int(bh), H - by))
            crop = blue_mask_full[by:by + bh, bx:bx + bw]
            if crop.size > 0:
                pixels = float(cv2.countNonZero(crop))
                ratio = pixels / crop.size
            else:
                pixels = 0.0
                ratio = 0.0
            auto_blue_scores.append(pixels)
            auto_blue_ratios.append(ratio)
            if pixels > winner_score:
                winner_score = pixels
                winner_bbox = (bx, by, bw, bh)
                winner_color = "blue"

        for yx, yy, yw, yh in auto_yellow_bboxes:
            yx = max(0, min(int(yx), W - 1)); yy = max(0, min(int(yy), H - 1))
            yw = max(1, min(int(yw), W - yx)); yh = max(1, min(int(yh), H - yy))
            crop = yellow_mask_full[yy:yy + yh, yx:yx + yw]
            if crop.size > 0:
                pixels = float(cv2.countNonZero(crop))
                ratio = pixels / crop.size
            else:
                pixels = 0.0
                ratio = 0.0
            auto_yellow_scores.append(pixels)
            auto_yellow_ratios.append(ratio)
            if pixels > winner_score:
                winner_score = pixels
                winner_bbox = (yx, yy, yw, yh)
                winner_color = "yellow"

        roi_used = winner_bbox
    # else: roi=None 且 auto_find_roi=False → 全图

    # 5. 裁剪 mask 到 ROI
    if roi_used is not None:
        x, y, w, h = roi_used
        x = max(0, min(int(x), W - 1))
        y = max(0, min(int(y), H - 1))
        w = max(1, min(int(w), W - x))
        h = max(1, min(int(h), H - y))
        blue_mask = blue_mask_full[y:y + h, x:x + w]
        yellow_mask = yellow_mask_full[y:y + h, x:x + w]
    else:
        blue_mask = blue_mask_full
        yellow_mask = yellow_mask_full

    # 6. 比例
    total_pixels = int(blue_mask.size)
    blue_pixels = int(cv2.countNonZero(blue_mask))
    yellow_pixels = int(cv2.countNonZero(yellow_mask))
    blue_ratio = blue_pixels / total_pixels if total_pixels > 0 else 0.0
    yellow_ratio = yellow_pixels / total_pixels if total_pixels > 0 else 0.0

    # 7. 判定 (裁剪到色标区域后, 主色比例高, 蓝色金属框已被排除)
    max_ratio = max(blue_ratio, yellow_ratio)
    other_ratio = min(blue_ratio, yellow_ratio)
    if max_ratio < COLOR_MIN_RATIO:
        color = "unknown"
    elif (max_ratio - other_ratio) < COLOR_DECISION_MARGIN:
        color = "unknown"  # 蓝黄太接近, 难以判定
    elif blue_ratio > yellow_ratio:
        color = "blue"
    else:
        color = "yellow"

    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    # 向后兼容: auto_blue_bbox / auto_yellow_bbox 单数字段保留
    # (返回该色最大候选 bbox, 如果有; 没有则 None)
    auto_blue_bbox = auto_blue_bboxes[0] if auto_blue_bboxes else None
    auto_yellow_bbox = auto_yellow_bboxes[0] if auto_yellow_bboxes else None
    # v3: 单数 score 保留 = 最大候选的**绝对像素数** (跟 score 列表一致)
    auto_blue_score = auto_blue_scores[0] if auto_blue_scores else 0.0
    auto_yellow_score = auto_yellow_scores[0] if auto_yellow_scores else 0.0
    auto_blue_ratio = auto_blue_ratios[0] if auto_blue_ratios else 0.0
    auto_yellow_ratio = auto_yellow_ratios[0] if auto_yellow_ratios else 0.0

    return {
        "color": color,
        "blue_ratio": blue_ratio,
        "yellow_ratio": yellow_ratio,
        "blue_pixels": blue_pixels,
        "yellow_pixels": yellow_pixels,
        "total_pixels": total_pixels,
        "frame_shape": tuple(frame.shape),
        "roi": tuple(roi) if roi is not None else None,
        "roi_used": tuple(roi_used) if roi_used is not None else None,
        "auto_find": auto_find_applied,
        # v3 新增: 所有候选 + score (调试用)
        # score 现在是**绝对像素数** (=bbox 内主色像素数, 不是 ratio)
        # ratio 是**占比** (备查)
        "auto_blue_bboxes": [tuple(b) for b in auto_blue_bboxes],
        "auto_yellow_bboxes": [tuple(b) for b in auto_yellow_bboxes],
        "auto_blue_scores": auto_blue_scores,
        "auto_yellow_scores": auto_yellow_scores,
        "auto_blue_ratios": auto_blue_ratios,
        "auto_yellow_ratios": auto_yellow_ratios,
        "winner_color": winner_color,
        "winner_score": winner_score,  # 赢家绝对像素数
        "winner_bbox": tuple(winner_bbox) if winner_bbox else None,
        # v2 兼容字段: 单数 (取每个色最大候选)
        "auto_blue_bbox": tuple(auto_blue_bbox) if auto_blue_bbox else None,
        "auto_yellow_bbox": tuple(auto_yellow_bbox) if auto_yellow_bbox else None,
        "auto_blue_score": auto_blue_score,  # 像素数 (新语义)
        "auto_yellow_score": auto_yellow_score,
        "auto_blue_ratio": auto_blue_ratio,  # 占比 (备查)
        "auto_yellow_ratio": auto_yellow_ratio,
        "cam": cam,
        "elapsed_ms": elapsed_ms,
        "raw_jpeg_bytes": raw_len,
    }


# ---------- belt-slip 安全 move_x (抽离到 main.arm.each_task.common) ----------

# 2026-07-30: 之前 3 个 task5 文件 (high_tower / low_tower / target) 各拷贝一份
# _move_x_with_split, 改一处要同步 3 处容易漏。现抽到 main.arm.each_task.common,
# 3 个文件 import 即可。target 用的是旧版 (无 wall_hit / overshoot), 抽离后
# 自动获得这些增强能力 (来自 low_tower 2026-07-30 现场 case 的加强)。
# 本地保留 _move_x_with_split 别名 → 兼容 run() 内部调用 + 历史 log 习惯
def _move_x_with_split(client: ArmClient, runner: ArmRunner,
                       target_x_mm: float) -> dict:
    """薄 wrapper: 透传 common.move_x_with_split, 注入 LOG_PREFIX。

    见 main/arm/each_task/common.py:move_x_with_split 完整 docstring。
    """
    return move_x_with_split(
        client, runner, target_x_mm,
        log_prefix=LOG_PREFIX,
    )


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        x_mm: float = TARGET_X_MM,
        arm_deg: float = TARGET_ARM_DEG,
        hand_deg: float = TARGET_HAND_DEG,
        y_mm: float = TARGET_Y_MM,
        *,
        detect_color: bool = True,
        cam: str = DEFAULT_CAM,
        roi: Optional[tuple] = DEFAULT_ROI,
        auto_find_roi: bool = True,
        color_timeout: float = JPEG_FETCH_TIMEOUT_S) -> dict:
    """按指定顺序: y → x → arm → hand, 然后 [5/5] 识别高仓色标。

    前置条件: 调用时 y 任意 (move_y 第一步会自己出保护区 [0, -30])。

    Args:
        detect_color: 是否在 [5/5] 抓侧摄做色标识别 (默认 True)
        cam: 识别用的相机, "cam1"=side / "cam2"=front
        roi: 识别 ROI (x, y, w, h) 像素, None=走 auto_find_roi
        auto_find_roi: 当 roi=None 时, 是否全图自动找色标 (默认 True, 推荐)
        color_timeout: 抓 JPEG HTTP 超时

    Returns:
        {
            "ok": True,
            "x_info": dict, "arm_deg": float, "hand_deg": float, "y_mm": float,
            "color_info": dict | None,  # 5/5 识别结果, detect_color=False 时 None
        }
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标顺序: y={y_mm}mm → x={x_mm}mm → arm={arm_deg}° → hand={hand_deg}°"
          f" → [识别色标 cam={cam} roi={roi} auto={auto_find_roi}]")
    print(f"  2026-07-30 改: y 第一步, 自动出保护区 [0, -30]; 任意起点 y 都行")
    print(f"  ⚠️ hand={hand_deg}° 是 mid 位 (不是 UP=-90), arm 必须先到 {arm_deg}° "
          f"(safe band 外, 允许非 UP 手爪动作)")

    # 1. y 抬到 -100mm (move_y 走 y 步进电机, 允许保护区 [0, -30] 内调, 出保护区)
    print(f"\n  [1/5] move_y({y_mm}mm)  抬到 -100mm (允许保护区起步, 走 y 步进电机)")
    y_job = runner.move_y(y_mm, timeout=30.0)
    print(f"        y_job.status={y_job.get('status') if isinstance(y_job, dict) else y_job}")

    # 2. x 回中 (belt-slip 分段; y 已 -100, 保护区外)
    print(f"\n  [2/5] move_x({x_mm}mm)  belt-slip 分段 (common.move_x_with_split, y 已出保护区)")
    x_info = _move_x_with_split(client, runner, x_mm)

    # 2026-07-30 改: 结构化打印 x_info 新字段 (跟 high_tower / low_tower 一致)
    result = x_info.get("result", "unknown")
    final_x = x_info.get("final_x", x_info.get("actual_x"))
    residual = x_info.get("residual_mm", 0.0)
    wall_hit = x_info.get("wall_hit", False)
    overshoot_mm = x_info.get("overshoot_mm", 0.0)
    print(f"        result       = {result}")
    print(f"        final_x      = {final_x:+.1f}mm  (target={x_info.get('target_x', x_mm):+.0f}mm, "
          f"residual={residual:+.1f}mm)")
    print(f"        wall_hit     = {wall_hit}")
    print(f"        overshoot_mm = {overshoot_mm:+.1f}mm")
    if result != "success":
        print(f"        [WARN]  x 未到位 ({result}), 后续 placement 可能撞车, 请人工介入")

    x_ok = (result in ("success", "already_in_range"))
    # 2026-07-30 v2: 加 already_in_range 兜底 (起点已在容差内也算 ok)

    # 3. 大臂 +90° 复位位 (init 例外位, 保护区允许)
    print(f"\n  [3/5] set_arm_angle({arm_deg}°)  复位位 (init 例外位, 保护区允许)")
    arm_job = client.set_arm_angle(arm_deg, speed=80, timeout=10.0)
    print(f"        arm_job.status={arm_job.get('status') if isinstance(arm_job, dict) else arm_job}")

    # 4. 手爪 -45° (mid 位, 业务硬限内 [-90, 0] 正常位置; 不是 UP/init 例外位)
    print(f"\n  [4/5] set_hand_angle({hand_deg}°)  mid (业务硬限内 [-90, 0], 正常位置)")
    hand_job = client.set_hand_angle(hand_deg, speed=80, timeout=10.0)
    print(f"        hand_job.status={hand_job.get('status') if isinstance(hand_job, dict) else hand_job}")

    # 5. 高仓示意颜色识别
    color_info = None
    if detect_color:
        print(f"\n  [5/5] detect_high_tower_color  cam={cam} roi={roi} auto_find={auto_find_roi} "
              f"timeout={color_timeout}s")
        try:
            color_info = detect_high_tower_color(
                client, cam=cam, roi=roi, auto_find_roi=auto_find_roi, timeout=color_timeout,
            )
            print(f"        color     = {color_info['color']}")
            print(f"        blue_pix  = {color_info['blue_pixels']:>7d} "
                  f"({color_info['blue_ratio']*100:5.1f}%)")
            print(f"        yellow_pix= {color_info['yellow_pixels']:>7d} "
                  f"({color_info['yellow_ratio']*100:5.1f}%)")
            print(f"        total_pix = {color_info['total_pixels']:>7d}  "
                  f"frame={color_info['frame_shape']}  "
                  f"jpeg={color_info['raw_jpeg_bytes']}B  "
                  f"elapsed={color_info['elapsed_ms']:.0f}ms")
            # 2026-07-30 v2: 打印 auto 找到的 bbox, 方便现场调试
            if color_info.get("auto_find"):
                print(f"        auto_blue_bbox   = {color_info.get('auto_blue_bbox')}")
                print(f"        auto_yellow_bbox = {color_info.get('auto_yellow_bbox')}")
                print(f"        roi_used         = {color_info.get('roi_used')}  "
                      f"(auto 选中的色标区域)")
        except Exception as e:
            print(f"        [FAIL] {type(e).__name__}: {e}")
            color_info = {"color": "unknown", "error": str(e)}
    else:
        print(f"\n  [5/5] detect_high_tower_color  SKIPPED (--no-detect)")

    print(f"\n========== {LOG_PREFIX} 完成 ==========\n")
    # 2026-07-30 改: ok 反映 x 实际结果
    # (color_info 独立, 不影响 ok —— caller 自己决定 color 是否参与最终判定)
    return {
        "ok": x_ok,
        "x_info": x_info,
        "arm_deg": arm_deg,
        "hand_deg": hand_deg,
        "y_mm": y_mm,
        "color_info": color_info,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 target: y→-100mm → x→0 → arm→+90° → hand→-45°(mid) → 色标识别",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x", type=float, default=TARGET_X_MM, help="x (mm)")
    p.add_argument("--arm", type=float, default=TARGET_ARM_DEG, help="大臂角度 (°)")
    p.add_argument("--hand", type=float, default=TARGET_HAND_DEG, help="手爪角度 (°), 默认 -45=mid")
    p.add_argument("--y", type=float, default=TARGET_Y_MM, help="y (mm), 默认 -100=抬 100mm")
    p.add_argument("--no-detect", dest="detect_color", action="store_false",
                   help="跳过 [5/5] 高仓色标识别")
    p.add_argument("--cam", default=DEFAULT_CAM,
                   help='识别用相机 (默认 cam2=side; ⚠️ 与 config_car.yml 设备号不同)')
    p.add_argument("--roi", type=int, nargs=4, default=None, metavar=("X", "Y", "W", "H"),
                   help="识别 ROI 像素坐标 (x y w h); 传了则不走 auto, 用显式 ROI")
    p.add_argument("--no-auto-roi", dest="auto_find_roi", action="store_false",
                   help="关闭 auto findContours (默认开); 关闭后没传 --roi 时走全图")
    p.add_argument("--color-timeout", type=float, default=JPEG_FETCH_TIMEOUT_S,
                   help="抓 JPEG HTTP 超时 (秒)")
    p.set_defaults(detect_color=True, auto_find_roi=True)
    return p


def main(argv=None) -> int:
    t_total_start = time.perf_counter()
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    # CLI --roi 不传时 None → 用 DEFAULT_ROI (=None, 走 auto_find_roi=True)
    roi = tuple(args.roi) if args.roi is not None else DEFAULT_ROI
    run(client, runner,
        x_mm=args.x, arm_deg=args.arm, hand_deg=args.hand, y_mm=args.y,
        detect_color=args.detect_color,
        cam=args.cam, roi=roi, auto_find_roi=args.auto_find_roi,
        color_timeout=args.color_timeout)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())

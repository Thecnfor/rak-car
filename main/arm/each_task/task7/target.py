"""task7 / target —— 6 位置扫描 + ERNIE 大模型文字识别 (按位置编号输出 + 落盘记录)

2026-08-03 v2 重构: 从"1 位置 5 步"升级为"6 位置扫描"。
2026-08-06 v3 改版: 公共 setup 从 4 步串行改成 1 步 4 机联动 composite_run
                    (仿 ``main/task/task1_seeding.py`` _init_step2_s_pose 同款)。

按用户编号约定:
    上左 (top-left)     = 1     [1] [2] [3]      ← 上排左→右扫描
    上中 (top-mid)      = 2
    上右 (top-right)    = 3
    下左 (bottom-left)  = 4     [4] [5] [6]      ← 下排左→右扫描
    下中 (bottom-mid)   = 5
    下右 (bottom-right) = 6

流程:
  公共 setup (一次, 所有位置共享):
    [1 步 4 机联动] composite_run(arm=+90°, x=0mm, y=-120mm, hand=-76°)
                    → 4 轴并发到位 (不走 y 保护区 pre-check, composite_run 本身不查)
  OCR + 网格解析:
    调一次 ERNIE 多模态 VL, cam2 一帧拍全 6 个名字 → row-major 切 6 名
  按 POSITIONS 映射输出 + 落盘:
    → 写 JSON 记录到 ~/.remember/logs/task7_ocr_<timestamp>.json
    → 控制台打印汇总 (6 条结果一行一条)

⚠️ **v2 → v3 改版差异** (2026-08-06 用户改):
  - **公共 setup 4 步 → 1 步 composite_run**: 原 4 步串行 (move_y → set_arm →
    set_hand → move_x_with_split) 改成 1 步 4 机联动 composite_run,
    与 ``get_position1.py`` / ``task1_seeding.py`` 同款。
  - **去掉 move_x_with_split**: composite_run 内部走 move_x_position (SDK 层),
    **不带 belt-slip retry**。target.py 是"切到观察位", 是状态过渡不是精密抓取,
    不需要 split 兜底 (与 task1_seeding / get_position1 同款取舍)。
  - **去掉 y 保护区 pre-check**: composite_run 内部**不调用** _check_y_protected
    (composite.py:60 注释 "23:31 用户拍板: 不怕撞车! _check_y_protected 去掉! 要速度!"),
    所以无论当前 y 在不在保护区, 4 轴并发都不会被 hand 校验拦截。pre-check 冗余, 删。
  - **耗时**: 4 步串行 ~6-8s → 1 步并发 ~2-3s。

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner)
   + stdlib + requests/base64, 不 import task7 包内其它模块。
   (本版不再 import move_x_with_split — 改用 composite_run 后不需要。)
   原因: task5 目录里的辅助文件曾被外部动作清空过 (见 [[task5-rebuild-2026-07-22]]),
   自包含可保证 `python target.py` 直接跑不受影响。

⚠️ **业务硬限** (走前要核对, 见 ARM_API §1.1 / §7 + setters.py):
  - y=-120 ≤ soft_y_max=-200 ✓ (距上限 80mm, 充裕)
  - x=0 ∈ [-320, +220] mm ✓ (中位)
  - arm=+90 ∈ [-150, +150]° ✓ (init 位置, 保护区允许)
  - hand=-76 ∈ [-90, +10]° ✓ (非 init, 但 y=-120 出保护区后允许)
  - y=-120 ≤ -80 ✓ (保护区外 40mm)

⚠️ OCR/LLM step 需要:
  - runtime 在线 (cam2 流在跑)
  - ERNIE access token 在 `main/misc/llm_config.yml` 或 env `ERNIE_ACCESS_TOKEN`
    或 `config_car.yml → ernie_access_token` (优先级与 test_order_read.py 一致)

⚠️ POSITIONS 当前 x_mm 全为 -80 (占位)。要 6 个位置看到 6 个不同画面, 必须把
   POSITIONS 里的 x_mm 改成实际坐标 (按上排 → 下排, 左 → 右分布)。改完直接重跑。

跑法 (两种都行):
    python main/arm/each_task/task7/target.py
    python -m main.arm.each_task.task7.target
"""
from __future__ import annotations

import argparse
import base64
import datetime
import json
import os
import sys
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# ---------- 目标位姿常量 (v3 改 1 步 composite_run) ----------

LOG_PREFIX: str = "[task7/target]"

TARGET_Y_MM: float = -120
"""composite_run 的 y_mm 终态 (-120mm, 出 y 保护区 [0, -80] 40mm)。

⚠️ 改这个值需注意:
   - 必须 ≤ -80 (保护区边界), 否则 set_*_angle 和 move_x 会被 _check_safe 拦截
   - 必须在 soft_y_max_mm (默认 -200) 范围内
"""

TARGET_ARM_DEG: float = 90.0
"""composite_run 的 arm 角度终态 (+90°, 复位位, 业务硬限 [-150, +150]° init 位置)。"""

TARGET_HAND_DEG: float = -76
"""composite_run 的 hand 角度终态 (-76°, 非 init)。

⚠️ [-90, +10]° 业务硬限内; -76 < -90 不成立, -76 > +10 不成立, 合法。
⚠️ 非 init (非 -90): composite_run 本身**不调用** _check_y_protected
   (composite.py:60 拍板"不怕撞车"), 所以 hand=-76 不会被 y 保护区拦截。
   终态 y=-120 仍出保护区, 后续动作仍安全。"""

TARGET_X_MM: float = 0
"""composite_run 的 x_mm 终态 (0mm, 中位)。

⚠️ 必须 ∈ [-320, +220] 软限位 ✓; 0 是中位。
⚠️ v3 不再走 move_x_with_split (composite_run 内部走 move_x_position, 不带 split)。
⚠️ POSITIONS 里的每行 x_mm (位置扫描用) 与本常量无关, 是另一组坐标。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task5/target.py 一致。"""

COMPOSITE_TIMEOUT_S: float = 30.0
"""composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底
(含网络 + job_queue + SDK 内部 4 路 as_completed)。"""

# ---------- 6 位置扫描配置 ----------

# 用户 2026-08-03 指定编号:
#   上左 (top-left)     = 1
#   上中 (top-mid)      = 2     [1] [2] [3]      ← 上排左→右扫
#   上右 (top-right)    = 3
#   下左 (bottom-left)  = 4     [4] [5] [6]      ← 下排左→右扫
#   下中 (bottom-mid)   = 5
#   下右 (bottom-right) = 6
#
# 当前 POSITIONS 的 x_mm 全占位 -80.0, 上排下排共享 y/arm/hand。要让 6 个
# 位置看到 6 个不同画面, 现场给 x_mm 后填进去就行 (上排左→右建议 x 依次增大,
# 下排同样分布)。

POSITIONS: list = [
    (1, "上左"),
    (2, "上中"),
    (3, "上右"),
    (4, "下左"),
    (5, "下中"),
    (6, "下右"),
]
"""6 位置定义: `(id, 中文标签)`。

- id 是输出前缀和 JSON 记录的 key, **不可改**
- label 只是给 `[1] [上左] 张三` 输出好看, 现场可改 (例如换成 "1号房" 等)
- ocr 用 cam2 一帧拍全, 不再做 6 次物理扫描
"""

RECORD_DIRNAME: str = ".remember/logs"
"""JSON 记录落盘目录 (相对家目录 $HOME)。绝对路径 = $HOME/RECORD_DIRNAME/task7_ocr_<ts>.json。

复用了 main/arm/each_task 已有约定: 业务侧自定义日志落在 ~/.remember/logs/,
不污染 runtime / config_car.yml。"""

RECORD_PREFIX: str = "task7_ocr_"
"""JSON 文件名前缀 (含下划线)。"""

# ---------- OCR / ERNIE 大模型常量 ----------

# ERNIE 多模态 chat endpoint (aistudio 官方, 与 main/misc/test_order_read.py 对齐)
ERNIE_CHAT_URL: str = "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions"
"""百度 aistudio 文心 ERNIE 多模态 chat 端点 (test_order_read.py 用过的, 同样可用 VL 模型)。"""

DEFAULT_OCR_MODEL: str = "ernie-4.5-turbo-vl"
"""默认 ERNIE 多模态模型 (vision-language, 支持图像 + 文本 → 文本)。"""

DEFAULT_OCR_PROMPT: str = (
    "你是一个 OCR 文字识别助手。请仔细观察图片,识别图中所有可见的文字,"
    "**严格按从左到右、从上到下顺序**,**每行 3 个名字用半角空格分隔** (2×3 网格布局: 上排 3 个 / 下排 3 个),"
    "输出**两行原文** (共 6 个名字)。不要解释、不要翻译、不要添加任何额外内容。"
    "如果图中没有可识别的文字, 请只输出 [NO_TEXT]。"
)
"""OCR 通用 prompt (不限定业务, 适合任意图)。要换成任务专用识别 (e.g. 订单牌/门牌/蔬菜)
可改用 main/misc/llm_config.yml 里的 order_read/delivery_detect/veggie_detect prompt,
或外部传入。"""

OCR_CAM: str = "cam2"
"""OCR 取帧相机。cam2 = 侧摄 (side), 与 runtime OCR 模型一致 (VISION_API.md §"模型总览")。

⚠️ 见 [[stream-cam-id-mapping]]: config_car.yml 写 side:1 是视频设备号,
   stream URL /stream/frame/cam{N}.jpg 里的 cam2 才是侧摄。"""

JPEG_FETCH_TIMEOUT_S: float = 10.0
"""抓单帧 JPEG 的 HTTP 超时。"""

ERNIE_TIMEOUT_S: float = 15.0
"""单次 ERNIE 调用超时 (与 llm_config.yml default 一致)。"""

ERNIE_TOP_P: float = 0.1
"""ERNIE top_p (低 → 输出更确定, OCR 场景希望稳定)。"""

ERNIE_TEMPERATURE: float = 0.1
"""ERNIE temperature (低 → 输出更确定)。"""


# ============================================================================
# OCR / ERNIE 大模型调用 (step 5)
# ============================================================================
#
# 完全自包含: 模仿 main/misc/test_order_read.py 已经过现场验证的 pattern,
# 不 import main.misc / main.task, 不依赖 runtime 暴露的 /v1/vision/ocr (那个是
# PaddleOCR CNN, 不是大模型)。这里直接走 ERNIE 多模态 VL, 严格按用户 "文字识别
# 大模型" 的要求做。
#
# 流程:
#   cam2.jpg → base64 → POST 百度 aistudio ERNIE → 取首个 choice.message.content
#             → 清理 markdown 包裹 → 输出原文

def _load_ernie_token() -> str:
    """读 ERNIE access token。优先级 (与 test_order_read.py 一致):
      1. env `ERNIE_ACCESS_TOKEN`
      2. `main/misc/llm_config.yml` 的 `ernie.access_token`
      3. `config_car.yml` 的 `ernie_access_token`
    找不到抛 RuntimeError。
    """
    # 1. env
    t = os.getenv("ERNIE_ACCESS_TOKEN", "").strip()
    if t:
        return t

    # 2 / 3. yaml 文件 (容错: yaml 库可能没装, 退化到最简字符串解析)
    def _parse_simple_yaml_kv(path: str, key_candidates: list[str]) -> str:
        if not os.path.exists(path):
            return ""
        try:
            with open(path, encoding="utf-8") as f:
                for line in f:
                    # 匹配 "key: value" 或 "  key: value"
                    for k in key_candidates:
                        # 容忍前导空格 + 字段名 + 冒号
                        prefix = f"{k}:"
                        stripped = line.lstrip()
                        if stripped.startswith(prefix):
                            v = stripped[len(prefix):].strip().strip('"').strip("'")
                            return v
        except Exception:
            return ""
        return ""

    # main/misc/llm_config.yml 嵌套在 ernie: 块下, 但我们的简单解析会匹配第一个 "access_token:"
    # 出现在顶层的 key; llm_config.yml 中只在这里用一次, 不会冲突。
    llm_cfg_path = os.path.join(_ROOT, "main", "misc", "llm_config.yml")
    t = _parse_simple_yaml_kv(llm_cfg_path, ["access_token"])
    if t and t != "REPLACE_YOUR_ACCESS_TOKEN_HERE":
        return t

    car_cfg_path = os.path.join(_ROOT, "config_car.yml")
    t = _parse_simple_yaml_kv(car_cfg_path, ["ernie_access_token"])
    if t and t != "REPLACE_YOUR_ACCESS_TOKEN_HERE":
        return t

    raise RuntimeError(
        "未找到 ERNIE access token. 在以下任一处填入:\n"
        "  (1) export ERNIE_ACCESS_TOKEN=...\n"
        "  (2) main/misc/llm_config.yml → ernie.access_token\n"
        "  (3) repo root config_car.yml → ernie_access_token\n"
        f"  当前检查路径: llm_cfg={llm_cfg_path!r}, car_cfg={car_cfg_path!r}"
    )


def _fetch_camera_jpeg(client: ArmClient, cam: str, timeout: float) -> bytes:
    """从 runtime stream 服务抓一帧 JPEG。URL = {api_base}/stream/frame/{cam}.jpg。

    ⚠️ 跟 task5/target.py:_fetch_camera_jpeg 同构 ([:125])。**不能走 client.http.get()**:
      api_client.py:33 _request 末尾会调 response.json() 对 JPEG bytes 直接 JSONDecodeError。
      所以这里直接用 requests 拉 raw bytes。
    ⚠️ 路由在 root 不在 /v1: runtime/api/routes.py:606 是 root router, URL 不拼 api_prefix。
    ⚠️ api_base 是 @property (api_client.py:23), 无括号。

    Returns:
        JPEG bytes; 失败抛 RuntimeError。
    """
    import requests
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


def _strip_md_fence(text: str) -> str:
    """如果 LLM 返回 ```json ... ``` 之类的 markdown 包裹, 去掉 fence 返回内部正文。

    OCR 场景下 prompt 要求纯文本输出, 但 ERNIE 偶尔会自作主张加 ``` 包裹, 这里兜底。
    """
    s = text.strip()
    if not s.startswith("```"):
        return s
    lines = s.splitlines()
    # 跳过首行 ```xxx
    if lines and lines[0].lstrip().startswith("```"):
        lines = lines[1:]
    # 跳过末行 ```
    if lines and lines[-1].lstrip().startswith("```"):
        lines = lines[:-1]
    return "\n".join(lines).strip()


def _call_ernie_ocr(
    jpeg_bytes: bytes,
    token: str,
    *,
    prompt: str = DEFAULT_OCR_PROMPT,
    model: str = DEFAULT_OCR_MODEL,
    timeout: float = ERNIE_TIMEOUT_S,
) -> dict:
    """POST 一帧 JPEG 到 ERNIE 多模态 VL, 返回 {ok, text, raw} 或 {ok: False, error}。

    Args:
        jpeg_bytes: 单帧 JPEG (来自 _fetch_camera_jpeg)
        token: ERNIE access token (走 _load_ernie_token)
        prompt: OCR prompt, 默认走 DEFAULT_OCR_PROMPT (通用 OCR)
        model: ERNIE 模型名, 默认 ernie-4.5-turbo-vl
        timeout: 单次 POST 超时

    Returns:
        成功: {"ok": True, "text": "<识别原文>", "raw": <完整 JSON 响应>}
        失败: {"ok": False, "error": "<可定位的错误>"}
    """
    import requests
    img_b64 = base64.b64encode(jpeg_bytes).decode()
    body = {
        "model": model,
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}},
        ]}],
        "top_p": ERNIE_TOP_P,
        "temperature": ERNIE_TEMPERATURE,
    }
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
        "x-bce-date": datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }

    try:
        resp = requests.post(ERNIE_CHAT_URL, headers=headers, json=body, timeout=timeout)
    except requests.Timeout:
        return {"ok": False, "error": f"ERNIE timeout ({timeout}s)"}
    except requests.RequestException as e:
        return {"ok": False, "error": f"ERNIE network {type(e).__name__}: {e}"}

    if resp.status_code in (401, 403):
        return {"ok": False, "error": f"ERNIE auth fail {resp.status_code} — token 无效/过期"}
    if not resp.ok:
        return {"ok": False, "error": f"ERNIE HTTP {resp.status_code}: {resp.text[:200]}"}

    try:
        content = resp.json()["choices"][0]["message"]["content"]
    except Exception as e:
        return {"ok": False, "error": f"ERNIE response parse fail: {type(e).__name__}: {e}"}

    text = _strip_md_fence(content)
    return {"ok": True, "text": text, "raw": resp.json()}


def ocr_text_with_ernie(
    client: ArmClient,
    *,
    cam: str = OCR_CAM,
    prompt: str = DEFAULT_OCR_PROMPT,
    model: str = DEFAULT_OCR_MODEL,
    timeout: float = ERNIE_TIMEOUT_S,
) -> dict:
    """一步到位: 抓 cam 帧 → 调 ERNIE → 输出文字。失败不抛 (返回 dict 含 ok=False)。

    Args:
        client: ArmClient (用 .http.api_base 取 stream URL)
        cam: 相机 id, 默认 cam2 (侧摄)
        prompt: 传给 ERNIE 的 prompt, 默认通用 OCR
        model: ERNIE 模型名
        timeout: ERNIE 超时 (秒)

    Returns:
        {
            "ok": bool,
            "text": "<识别原文>" | None,
            "model": str,
            "cam": str,
            "jpeg_bytes": int,
            "elapsed_ms": float,
            "error": str | None,         # 失败时存在
            "raw_response": dict | None, # ok=True 时存在 (ERNIE 完整返回)
        }
    """
    import time
    t0 = time.perf_counter()
    try:
        token = _load_ernie_token()
    except RuntimeError as e:
        return {"ok": False, "error": str(e), "model": model, "cam": cam,
                "text": None, "jpeg_bytes": 0, "elapsed_ms": 0.0,
                "raw_response": None}

    try:
        jpeg = _fetch_camera_jpeg(client, cam, JPEG_FETCH_TIMEOUT_S)
    except RuntimeError as e:
        return {"ok": False, "error": str(e), "model": model, "cam": cam,
                "text": None, "jpeg_bytes": 0,
                "elapsed_ms": (time.perf_counter() - t0) * 1000,
                "raw_response": None}

    result = _call_ernie_ocr(jpeg, token, prompt=prompt, model=model, timeout=timeout)
    elapsed_ms = (time.perf_counter() - t0) * 1000

    if not result.get("ok"):
        return {
            "ok": False,
            "error": result["error"],
            "text": None,
            "model": model,
            "cam": cam,
            "jpeg_bytes": len(jpeg),
            "elapsed_ms": elapsed_ms,
            "raw_response": None,
        }

    return {
        "ok": True,
        "text": result["text"],
        "model": model,
        "cam": cam,
        "jpeg_bytes": len(jpeg),
        "elapsed_ms": elapsed_ms,
        "raw_response": result.get("raw"),
    }


# ============================================================================
# 2×3 网格识别 + JSON 落盘
# ============================================================================
#
# 设计变更 (2026-08-03 用户反馈):
#   之前: 6 个位置每个都做 move_x + ocr, 6 次 OCR 6 张相同图片 → 输出重复 36 行
#   现在: 调一次 ERNIE OCR, 拿到 "张三 熊九 孙八\n田一 孟三 白七" 文本,
#         row-major 切成 6 个名字, 按 id=1..6 映射到 6 个位置。
#
# 为什么不再做 6 次扫描?
#   cam2 侧摄一帧就拍全 6 个名字, 移动 x 看到的图片是同一张。
#   物理扫描是后续动作的事 (e.g. 移到 n 号位投递), 不在 OCR 阶段。
#
# OCR prompt 必须强制 "按从左到右、从上到下顺序逐行输出" — 默认 DEFAULT_OCR_PROMPT
# 已经按这个写, row-major 解析才稳。
#
# ⚠️ 2026-08-06 修: prompt 强化为 "每行 3 个名字空格分隔" (2 行 × 3 列 = 6 名)。
#    旧 prompt 只说"逐行"没说每行几个, ERNIE-VL 实际输出 1 列 6 行,
#    导致 _parse_grid 把每行当 1 row → 6 row, len(out) >= 2 break 后只留 2 行
#    → 4 个名字丢失 (flat=[name, None, None, name, None, None])。
#    修法: prompt 显式要求 2×3 布局 + _parse_grid 加 fallback (1 列 N 行自动 row-major 切)

GRID_COLS: int = 3
"""2×3 网格列数 (左/中/右 = 3 列)。"""

GRID_ROWS: int = 2
"""2×3 网格行数 (上/下 = 2 行)。"""

ROWS_PER_POSITION: list[str] = ["上", "下"]
"""行的中文标签: row 0 = "上", row 1 = "下" (与"上排/下排"对齐)。"""

COLS_PER_POSITION: list[str] = ["左", "中", "右"]
"""列的中文标签: col 0 = "左", col 1 = "中", col 2 = "右"。"""


def _parse_grid(text: str, rows: int = GRID_ROWS, cols: int = GRID_COLS) -> list[list[str]]:
    """把 OCR 多行文字解析成 rows×cols 网格 (row-major flatten 后端上能填位置)。

    假设 (与 DEFAULT_OCR_PROMPT 一致):
      - OCR 按 "从左到右、从上到下" 输出, 共 2 行 (上排 / 下排)
      - 每行 3 个名字用半角空格分隔
      - 名字不会跨行

    鲁棒 fallback (2026-08-06 修):
      - ERNIE-VL 偶尔输出 1 列 6 行 ("张三\\n熊九\\n..."), 旧代码直接当 6 row 处理
        → len(out) >= 2 break → 只剩前 2 个名字,后面 4 个丢
      - 现在如果行数 > rows, 把多出的行依次切到当前 row,row 满 (cols 个) 换下一行

    Args:
        text: OCR 识别原文 (e.g. "张三 熊九 孙八\\n田一 孟三 白七"
                              或 1-列 6-行 "张三\\n熊九\\n孙八\\n田一\\n孟三\\n白七")
        rows: 期望行数 (默认 2, 上/下)
        cols: 期望列数 (默认 3, 左/中/右)

    Returns:
        二维 list, grid[r][c] = 名字。长度允许 < rows*cols (不足位置返回 None)。
        e.g. [["张三","熊九","孙八"], ["田一","孟三","白七"]]
    """
    out: list[list[str]] = []
    if not text:
        return out
    # 把全部 OCR 文字 token 化 (按行 → 每行空格切)
    all_names: list[str] = []
    for line in text.splitlines():
        line = (line or "").strip()
        if not line:
            continue
        tokens = line.split()
        all_names.extend(tokens)
    if not all_names:
        return out
    # row-major 切分: 第一个名字进 grid[0][0], 第 cols 个换行 grid[1][0]
    for idx, name in enumerate(all_names):
        r, c = divmod(idx, cols)
        if r >= rows:
            break  # 已超过期望行数, 丢弃 (防御 OCR 多吐一行)
        while len(out) <= r:
            out.append([])
        out[r].append(name)
    return out


def _flatten_grid_to_positions(grid: list[list[str]]) -> list[Optional[str]]:
    """把 rows×cols 网格按 row-major 拉平成 6 槽位列表。

    长度永远 = GRID_ROWS × GRID_COLS (= 6)。
    不足的槽位用 None 占位 (方便上层判 `未识别`).
    """
    flat: list[Optional[str]] = [None] * (GRID_ROWS * GRID_COLS)
    for r, row in enumerate(grid):
        if r >= GRID_ROWS:
            break
        for c, name in enumerate(row):
            if c >= GRID_COLS:
                break
            flat[r * GRID_COLS + c] = name
    return flat


def _save_grid_record(
    grid: list[list[str]],
    flat: list[Optional[str]],
    raw_text: str,
    *,
    ocr_result: dict,
    record_dir: Optional[str] = None,
) -> str:
    """把 2×3 网格识别结果写成 JSON 落盘。

    Args:
        grid: _parse_grid 返回
        flat: _flatten_grid_to_positions 返回 (长度 6)
        raw_text: OCR 原始文本
        ocr_result: ocr_text_with_ernie 返回 (含 jpeg_bytes / elapsed_ms 等元数据)
        record_dir: 落盘目录 (None = $HOME/.remember/logs)

    Returns:
        写入文件绝对路径。
    """
    base_dir = record_dir or os.path.join(os.path.expanduser("~"), RECORD_DIRNAME)
    os.makedirs(base_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(base_dir, f"{RECORD_PREFIX}{ts}.json")

    payload = {
        "schema_version": 2,
        "script": "main/arm/each_task/task7/target.py",
        "timestamp": datetime.datetime.now().isoformat(),
        "shared_pose": {
            "y_mm": TARGET_Y_MM,
            "arm_deg": TARGET_ARM_DEG,
            "hand_deg": TARGET_HAND_DEG,
            "angle_speed": ANGLE_SPEED,
        },
        "ocr_model": DEFAULT_OCR_MODEL,
        "ocr_cam": OCR_CAM,
        "raw_text": raw_text,
        "ocr_ok": bool(ocr_result.get("ok")),
        "ocr_elapsed_ms": float(ocr_result.get("elapsed_ms", 0.0)),
        "ocr_jpeg_bytes": int(ocr_result.get("jpeg_bytes", 0)),
        "grid": grid,                        # 2×3 原始
        "flat": flat,                        # row-major flatten (长度 6)
        "grid_size": {"rows": len(grid), "cols": (len(grid[0]) if grid else 0)},
        "positions": [
            {
                "id": pid,
                "label": plabel,
                "name": flat[pid - 1] if pid - 1 < len(flat) else None,
            }
            for pid, plabel in POSITIONS
        ],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return path


# ============================================================================
# 入口 (公共 setup + 1×OCR + 2×3 网格解析 → 6 位置映射 → 落盘)
# ============================================================================


def run(client: ArmClient, runner: ArmRunner, *, record_dir: Optional[str] = None) -> dict:
    """执行 1×OCR + 2×3 网格解析, 把 6 个名字按编号映射到 6 个位置。

    Args:
        client: ArmClient
        runner: ArmRunner
        record_dir: JSON 落盘目录 (None = $HOME/.remember/logs)

    Returns:
        {
            "ok": bool,                  # OCR 成功 + 解析出至少 1 个名字
            "shared_pose": {...},
            "composite_result": dict,   # v3 新增: 1 步 composite_run 原始 job dict
            "ocr_result": dict,
            "grid": list[list[str]],     # 2×3 原始网格
            "flat": list[str|None],      # 长度 6, row-major 顺序
            "results": [                 # 6 位置结果
                {"id": 1, "label": "上左", "name": "张三"},
                ...
            ],
            "record_path": str,
        }
    """
    print(f"\n========== {LOG_PREFIX} run (1×OCR + 2×3 网格) ==========")

    # 1. 公共姿态 (移到观察位) —— v3 改 1 步 4 机联动 composite_run
    #    ⚠️ **不再做 y 保护区 pre-check** (2026-08-06 用户拍板):
    #    composite_run 内部**不调用** _check_y_protected (composite.py:60 注释
    #    "23:31 用户拍板: 不怕撞车! _check_y_protected 去掉! 要速度!"),
    #    所以无论当前 y 在不在保护区, 4 轴并发都不会被 hand 校验拦截。
    #    旧版 pre-check 仿 task1_seeding 是冗余的, 删掉。
    print(f"  [setup] composite_run: arm={TARGET_ARM_DEG:+.0f}° x={TARGET_X_MM:.0f}mm "
          f"y={TARGET_Y_MM:.0f}mm hand={TARGET_HAND_DEG:+.0f}°  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    composite_result = client.composite_run(
        arm=TARGET_ARM_DEG,
        x_mm=TARGET_X_MM,
        y_mm=TARGET_Y_MM,
        hand=TARGET_HAND_DEG,
        speed=ANGLE_SPEED,
        timeout=COMPOSITE_TIMEOUT_S,
    )
    ok_setup = (
        isinstance(composite_result, dict)
        and composite_result.get("status") == "succeeded"
        and isinstance(composite_result.get("result"), dict)
        and composite_result["result"].get("ok", False)
    )
    if not ok_setup:
        print(f"  [setup] ❌ composite_run 失败: {composite_result}")
        return {
            "ok": False,
            "shared_pose": {
                "y_mm": TARGET_Y_MM,
                "arm_deg": TARGET_ARM_DEG,
                "hand_deg": TARGET_HAND_DEG,
                "angle_speed": ANGLE_SPEED,
            },
            "composite_result": composite_result,
            "ocr_result": None,
            "grid": [],
            "flat": [None] * (GRID_ROWS * GRID_COLS),
            "results": [{"id": pid, "label": plabel, "name": None} for pid, plabel in POSITIONS],
            "record_path": None,
        }

    # 2. 单次 OCR (一帧图片拍全 6 个名字)
    print(f"\n  [ocr] ocr_text_with_ernie(cam={OCR_CAM}, model={DEFAULT_OCR_MODEL})  "
          f"取 1 帧 cam2 → ERNIE VL")
    ocr = ocr_text_with_ernie(client)
    raw_text = ocr.get("text") or ""

    if ocr.get("ok"):
        print(f"  [ocr] ✅ ({ocr['elapsed_ms']:.0f}ms, {ocr['jpeg_bytes']}B 帧)")
        print(f"  [ocr] ── 原始文本 ──")
        for ln in raw_text.splitlines():
            print(f"  [ocr] │ {ln}")
        print(f"  [ocr] ── END ──")
    else:
        print(f"  [ocr] ❌ {ocr.get('error', '?')}")

    # 3. 解析 2×3 网格
    grid = _parse_grid(raw_text)
    flat = _flatten_grid_to_positions(grid)

    print(f"\n  ──── 2×3 网格解析 ────")
    print(f"  grid (rows={len(grid)}):")
    for r, row in enumerate(grid):
        rlabel = ROWS_PER_POSITION[r] if r < len(ROWS_PER_POSITION) else f"r{r}"
        print(f"    {rlabel}排: {' '.join(row)}")
    if len(grid) < GRID_ROWS:
        print(f"  ⚠️ OCR 只回了 {len(grid)} 行 (期望 {GRID_ROWS})")

    # 4. 按位置 1..6 映射输出
    results: list = []
    print(f"\n  ──── 6 位置映射 (编号 → 名字) ────")
    for pid, plabel in POSITIONS:
        name = flat[pid - 1] if pid - 1 < len(flat) else None
        if name:
            print(f"  [{pid}] [{plabel}] {name}")
        else:
            print(f"  [{pid}] [{plabel}] (未识别)")
        results.append({"id": pid, "label": plabel, "name": name})

    # 5. 落盘
    record_path = _save_grid_record(
        grid=grid, flat=flat, raw_text=raw_text,
        ocr_result=ocr, record_dir=record_dir,
    )
    print(f"\n  ──── 记录已落盘 ────")
    print(f"  {record_path}")

    ok = bool(ocr.get("ok")) and any(n is not None for n in flat)
    print(f"========== {LOG_PREFIX} 完成 (ok={ok}) ==========\n")

    return {
        "ok": ok,
        "shared_pose": {
            "y_mm": TARGET_Y_MM,
            "arm_deg": TARGET_ARM_DEG,
            "hand_deg": TARGET_HAND_DEG,
            "angle_speed": ANGLE_SPEED,
        },
        "composite_result": composite_result,
        "ocr_result": ocr,
        "grid": grid,
        "flat": flat,
        "results": results,
        "record_path": record_path,
    }


def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 公共 setup + 1×OCR + 2×3 网格解析 + JSON 落盘。"""
    default_record_dir = os.path.join(os.path.expanduser("~"), RECORD_DIRNAME)
    p = argparse.ArgumentParser(
        description=(
            "task7 target v3: 1 步 4 机联动 composite_run (arm=+90° x=0 y=-120 hand=-76°)\n"
            "+ 1×OCR + 2×3 网格解析 → 6 位置映射 → JSON 落盘\n"
            "  编号: 上左=1 上中=2 上右=3 下左=4 下中=5 下右=6\n"
            "  cam2 一帧拍全 6 个名字, 按 row-major 切 6 名映射到位置"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--record-dir", type=str, default=default_record_dir,
        help="JSON 落盘目录 (默认 $HOME/" + RECORD_DIRNAME + ")",
    )
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner, record_dir=args.record_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())

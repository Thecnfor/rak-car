#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/misc/test_pest_llm_shoot.py - 害虫识别工具集(恢复版,2026-08-04).

历史: 历史上以"4 板害虫 → LLM 判别 + 射击"为主线,本模块曾包含
`_call_llm / _check_token_health / _load_token / _mask_token / crop_bbox`。
合并过程中丢失, 现重建以恢复 21 个 task3 历史脚本的 import 链路。

== 现状 ==
- `crop_bbox` 是真实工具函数(OpenCV 裁 bbox → JPEG bytes),所有 task3 脚本都依赖
- `_call_llm / _load_token / _check_token_health / _mask_token` 是占位,
  真正的 ERNIE 调用已迁移到 `main/task/task3/llm_ernie.py`
- 新代码请直接 `from main.task.task3.llm_ernie import call_vision, ...`,
  不要继续 import 这里的桩符号

== ponytail ==
桩符号仅供历史脚本 import 不再断;一旦 21 个死代码被 sed 改路径后清理,
即可删除本文件。
"""
from __future__ import annotations

import sys
from typing import Optional

# ponytail: 仅在 cv2/numpy 缺失时降级为纯字节(历史脚本单独跑已多年未触)
try:
    import cv2
    import numpy as np
    _HAS_CV2 = True
except ImportError:  # noqa: BLE001
    cv2 = None
    np = None
    _HAS_CV2 = False


# -------- crop 配置常量(历史签名,所有旧脚本共用) --------
DEFAULT_CROP_PADDING = 0.10
DEFAULT_MIN_SCORE = 0.50
NO_ANIMAL_KEYWORDS = ("no animal", "no pest", "no_target")


def crop_bbox(frame_bytes: bytes, det_list, padding: float = DEFAULT_CROP_PADDING):
    """按 bbox (xc, yc, w, h) 归一化坐标裁 JPEG 帧, 返回 (crop_bytes, (x, y, w, h))。

    det_list = [cls_id, det_id, label, score, xc, yc, w, h]   # 长度 8
    像素坐标 = (xc, yc, w, h) 经 padding 放大后映射到帧 HxW
    """
    if not _HAS_CV2 or not frame_bytes or len(det_list) < 8:
        return None, (0, 0, 0, 0)
    try:
        img = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:  # noqa: BLE001
        return None, (0, 0, 0, 0)
    if img is None:
        return None, (0, 0, 0, 0)
    h, w = img.shape[:2]
    xc, yc, bw, bh = (float(x) for x in det_list[4:8])
    # bbox_norm 是 [-1, 1] 中心化坐标 (runtime 实测: norm=(px/dim)*2-1,
    # width_norm=(px_w/dim)*2), 先映射回像素再按 padding 外扩
    px_cx = (xc + 1.0) / 2.0 * w
    px_cy = (yc + 1.0) / 2.0 * h
    px_bw = bw / 2.0 * w
    px_bh = bh / 2.0 * h
    pad_w = px_bw * padding
    pad_h = px_bh * padding
    x0 = max(0, int(px_cx - px_bw / 2 - pad_w))
    y0 = max(0, int(px_cy - px_bh / 2 - pad_h))
    x1 = min(w, int(px_cx + px_bw / 2 + pad_w))
    y1 = min(h, int(px_cy + px_bh / 2 + pad_h))
    if x1 <= x0 or y1 <= y0:
        return None, (0, 0, 0, 0)
    crop = img[y0:y1, x0:x1]
    ok, buf = cv2.imencode(".jpg", crop)
    if not ok:
        return None, (0, 0, 0, 0)
    return bytes(buf), (x0, y0, x1 - x0, y1 - y0)


# -------- 桩符号:历史脚本 import 用 --------
# 新代码请走 main.task.task3.llm_ernie; 此处仅占位不抛错。

def _call_llm(token: str, image_b64_or_url: str, prompt: str, *args, **kwargs) -> dict:
    """桩:转发到 llm_ernie.call_vision,签名兼容旧调用方。"""
    try:
        from main.task.task3.llm_ernie import call_vision
        return call_vision(token, image_b64_or_url, prompt, **kwargs)
    except Exception as exc:  # noqa: BLE001
        return {"result": None, "analysis": f"stub_call_llm: {exc}"}


def _load_token(cli_token: Optional[str] = None) -> str:
    try:
        from main.task.task3.llm_ernie import load_token
        return load_token(cli_token)
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        print(f"[fatal] stub_load_token: {exc}", file=sys.stderr)
        sys.exit(2)


def _check_token_health(token: str, timeout: float = 12.0) -> None:
    try:
        from main.task.task3.llm_ernie import check_health
        check_health(token, timeout=timeout)
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        print(f"[fatal] stub_check_token_health: {exc}", file=sys.stderr)
        sys.exit(2)


def _mask_token(token: str) -> str:
    try:
        from main.task.task3.llm_ernie import mask_token
        return mask_token(token)
    except Exception:  # noqa: BLE001
        return "***"


__all__ = [
    "DEFAULT_CROP_PADDING",
    "DEFAULT_MIN_SCORE",
    "NO_ANIMAL_KEYWORDS",
    "crop_bbox",
    "_call_llm",
    "_load_token",
    "_check_token_health",
    "_mask_token",
]


if __name__ == "__main__":
    print("main/misc/test_pest_llm_shoot.py: stub module (recovery)")
    print("exports:", __all__)
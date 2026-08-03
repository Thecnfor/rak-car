#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/misc/test_pest_detect.py — 害虫识别, 纯摄像头+LLM.
配置: main/misc/llm_config.yml (不碰 task_config.yml)
跑法: python main/misc/test_pest_detect.py
"""
from __future__ import annotations

import base64
import datetime
import io
import json
import os
import sys
import time
from pathlib import Path

import requests
import yaml
from PIL import Image

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.api_client import RuntimeApiClient
from main.settings import load_settings

ERNIE_CHAT_URL = "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions"
_LLM_CONFIG_PATH = Path(__file__).resolve().with_name("llm_config.yml")


# ── 配置加载 ────────────────────────────────────────────
def _load_cfg() -> dict:
    if not _LLM_CONFIG_PATH.exists():
        raise FileNotFoundError(f"LLM 配置缺失: {_LLM_CONFIG_PATH}")
    with _LLM_CONFIG_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _load_token(cfg: dict) -> str:
    t = os.getenv("ERNIE_ACCESS_TOKEN", "").strip()
    if t: return t
    t = (cfg.get("ernie") or {}).get("access_token", "")
    if t and t != "REPLACE_YOUR_ACCESS_TOKEN_HERE": return t
    cp = _REPO_ROOT / "config_car.yml"
    if cp.exists():
        d = yaml.safe_load(cp.open("r", encoding="utf-8")) or {}
        t = d.get("ernie_access_token", "")
        if t and t != "REPLACE_YOUR_ACCESS_TOKEN_HERE": return t
    print("[fatal] 未找到 ERNIE access token", file=sys.stderr)
    sys.exit(2)


# ── LLM 调用 ────────────────────────────────────────────
def _call_llm(token: str, image_b64: str, prompt: str, ernie: dict) -> dict:
    body = {
        "model": ernie.get("model", "ernie-4.5-turbo-vl"),
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
        ]}],
        "top_p": ernie.get("top_p", 0.1),
        "temperature": ernie.get("temperature", 0.1),
    }
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
        "x-bce-date": datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    to = ernie.get("timeout_s", 15)
    last_err = ""
    for _ in range(2):
        try:
            resp = requests.post(ERNIE_CHAT_URL, headers=headers, json=body, timeout=to)
        except requests.Timeout:
            last_err = f"timeout after {to}s"; time.sleep(1.5); continue
        except requests.RequestException as exc:
            last_err = f"request error: {exc}"; time.sleep(1.5); continue
        if resp.status_code in (401, 403):
            print(f"[fatal] ERNIE {resp.status_code}", file=sys.stderr); sys.exit(2)
        if resp.status_code == 429 or resp.status_code >= 500:
            last_err = f"HTTP {resp.status_code}"; time.sleep(1.5); continue
        if not resp.ok: return {"result": None, "analysis": f"HTTP {resp.status_code}"}
        try:
            content = resp.json()["choices"][0]["message"]["content"]
        except Exception as exc:
            return {"result": None, "analysis": str(exc)}
        text = content.strip()
        if text.startswith("```"):
            ls = text.splitlines()
            ls = ls[1:] if ls[0].lstrip().startswith("```") else ls
            if ls and ls[-1].lstrip().startswith("```"): ls = ls[:-1]
            text = "\n".join(ls).strip()
        try:
            d = json.loads(text)
        except json.JSONDecodeError:
            return {"result": None, "analysis": f"unparseable: {content[:200]}"}
        if isinstance(d, dict) and "result" in d and "analysis" in d:
            return {"result": d["result"], "analysis": d["analysis"]}
        return {"result": None, "analysis": f"missing fields: {content[:200]}"}
    return {"result": None, "analysis": last_err or "unknown"}


def crop_bbox(jpeg_bytes: bytes, bbox_norm: dict, padding: float = 0.10):
    if not jpeg_bytes or not bbox_norm: return None
    img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
    iw, ih = img.size
    xc, yc = float(bbox_norm.get("x_center", 0)), float(bbox_norm.get("y_center", 0))
    wn, hn = float(bbox_norm.get("width", 0)), float(bbox_norm.get("height", 0))
    cx, cy = int((xc + 1) / 2 * iw), int((yc + 1) / 2 * ih)
    bw, bh = int(wn * iw / 2), int(hn * ih / 2)
    bwp, bhp = int(bw * (1.0 + padding)), int(bh * (1.0 + padding))
    x1, y1 = max(0, cx - bwp // 2), max(0, cy - bhp // 2)
    x2, y2 = min(iw, cx + bwp // 2), min(ih, cy + bhp // 2)
    if x2 <= x1 or y2 <= y1: return None
    buf = io.BytesIO(); img.crop((x1, y1, x2, y2)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def fetch_frame(session, streamer_url, timeout=10.0):
    try:
        r = session.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status(); return r.content
    except Exception as exc:
        print(f"[warn] cam2 帧失败: {exc}", file=sys.stderr); return None


# ── run ─────────────────────────────────────────────────
def run():
    cfg = _load_cfg()
    token = _load_token(cfg)
    settings = load_settings()
    ernie = cfg.get("ernie", {})
    pest_cfg = cfg.get("pest_detect", {})
    prompt = pest_cfg.get("system_prompt", "")
    max_rounds = pest_cfg.get("max_rounds", 3)
    poll = pest_cfg.get("poll_interval_s", 2.0)
    padding = pest_cfg.get("crop_padding", 0.10)

    print(f"API={settings.api_base}  token={token[:4]}***{token[-4:]}")
    client = RuntimeApiClient(settings=settings)
    client.wait_until_ready()
    session = requests.Session()

    pests, goods, judged = [], [], 0
    for rnd in range(1, max_rounds + 1):
        print(f"\n--- 第 {rnd}/{max_rounds} 轮 ---")
        try:
            ts = client.get_task_state().get("task_state") or {}
        except Exception as exc:
            print(f"[warn] task_state 失败: {exc}"); time.sleep(poll); continue
        dets = ts.get("detections") or []
        print(f"  active={ts.get('active')} detections={len(dets)}")
        if not dets: time.sleep(poll); continue
        frame = fetch_frame(session, settings.streamer_url)
        if not frame: time.sleep(poll); continue
        print(f"  frame: {len(frame)} bytes")
        for d in dets:
            judged += 1
            label = d.get("label", "?")
            crop = crop_bbox(frame, d.get("bbox_norm") or {}, padding)
            if not crop: print(f"  [{judged}] {label} 空裁剪"); continue
            img = base64.b64encode(crop).decode()
            v = _call_llm(token, img, prompt, ernie)
            res = v.get("result")
            a = v.get("analysis", "")[:100]
            print(f"  [{judged}] {label} -> {'害虫' if res == 0 else '益虫' if res == 1 else '未知'} | {a}")
            (pests if res == 0 else goods).append({"idx": judged, "label": label, "analysis": v.get("analysis", "")})

    print(f"\n害虫({len(pests)}): {[p['label'] for p in pests]}")
    print(f"益虫({len(goods)}): {[g['label'] for g in goods]}")
    return {"pests": pests, "beneficial": goods, "judged": judged, "pest_order": [p["label"] for p in pests]}


if __name__ == "__main__":
    r = run(); print("\nJSON:", json.dumps(r, ensure_ascii=False, indent=2))

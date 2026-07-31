#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/misc/test_order_read.py — 订单识别, 纯摄像头+LLM.
配置: main/misc/llm_config.yml (不碰 task_config.yml)
跑法: python main/misc/test_order_read.py
"""
from __future__ import annotations

import base64
import datetime
import json
import os
import sys
import time
from pathlib import Path

import requests
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.api_client import RuntimeApiClient
from main.settings import load_settings

ERNIE_CHAT_URL = "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions"
_LLM_CONFIG_PATH = Path(__file__).resolve().with_name("llm_config.yml")


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
            last_err = f"timeout"; time.sleep(1.5); continue
        except requests.RequestException as exc:
            last_err = str(exc); time.sleep(1.5); continue
        if resp.status_code in (401, 403):
            print(f"[fatal] ERNIE {resp.status_code}", file=sys.stderr); sys.exit(2)
        if resp.status_code == 429 or resp.status_code >= 500:
            last_err = f"HTTP {resp.status_code}"; time.sleep(1.5); continue
        if not resp.ok: return {"error": f"HTTP {resp.status_code}"}
        try:
            content = resp.json()["choices"][0]["message"]["content"]
        except Exception as exc:
            return {"error": str(exc)}
        text = content.strip()
        if text.startswith("```"):
            ls = text.splitlines()
            ls = ls[1:] if ls[0].lstrip().startswith("```") else ls
            if ls and ls[-1].lstrip().startswith("```"): ls = ls[:-1]
            text = "\n".join(ls).strip()
        try:
            d = json.loads(text)
        except json.JSONDecodeError:
            return {"error": f"unparseable: {content[:200]}"}
        if not isinstance(d, dict): return {"error": f"not dict: {content[:200]}"}
        return d
    return {"error": last_err}


def fetch_frame(session, streamer_url, timeout=10.0):
    try:
        r = session.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status(); return r.content
    except Exception as exc:
        print(f"[warn] cam2 帧失败: {exc}", file=sys.stderr); return None


def run():
    cfg = _load_cfg()
    token = _load_token(cfg)
    settings = load_settings()
    ernie = cfg.get("ernie", {})
    order_cfg = cfg.get("order_read", {})
    prompt = order_cfg.get("system_prompt", "")
    valid_goods = order_cfg.get("valid_goods", [])
    max_attempts = order_cfg.get("max_attempts", 5)
    poll = order_cfg.get("poll_interval_s", 2.0)

    print(f"API={settings.api_base}  token={token[:4]}***{token[-4:]}")
    client = RuntimeApiClient(settings=settings)
    client.wait_until_ready()
    session = requests.Session()
    orders = []

    for n in range(1, max_attempts + 1):
        print(f"\n--- 第 {n}/{max_attempts} 次 ---")
        frame = fetch_frame(session, settings.streamer_url)
        if not frame: time.sleep(poll); continue
        print(f"  frame: {len(frame)} bytes")
        img = base64.b64encode(frame).decode()
        t0 = time.time()
        d = _call_llm(token, img, prompt, ernie)
        dt = time.time() - t0
        if "error" in d:
            print(f"  ❌ ({dt:.1f}s): {d['error']}"); time.sleep(poll); continue
        name, goods, addr = d.get("name", ""), d.get("goods", ""), d.get("address")
        if not name or goods not in valid_goods or addr not in (1, 2):
            print(f"  ❌ 校验失败: name={name} goods={goods} addr={addr}"); continue
        print(f"  ✅ ({dt:.1f}s): {name} ← {goods} → {addr}号楼")
        orders.append({"name": name, "goods": goods, "address": addr})
        break

    print(f"\n共 {len(orders)} 个订单")
    for o in orders: print(f"  {o['name']} ← {o['goods']} → {o['address']}号楼")
    return {"ok": len(orders) > 0, "orders": orders}


if __name__ == "__main__":
    r = run(); print("\nJSON:", json.dumps(r, ensure_ascii=False, indent=2))

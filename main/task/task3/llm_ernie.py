#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/task/task3/llm_ernie.py - Baidu ERNIE VL adapter (self-contained)

适配层:对外暴露与 llm_minimax.py 相同的接口:
  - call_vision(token, image_url, prompt, timeout, system=..., use_json_response_format=...)
  - check_health(token, timeout)
  - mask_token(token)
  - load_token(cli_token=None)

实现:自包含 ERNIE HTTP 调用 + token 加载 + 健康检查,不依赖 main.misc.test_pest_llm_shoot
(那个文件已在合并时丢失,本模块替代它)。

ERNIE 文档:
  - endpoint: https://aistudio.baidu.com/llm/lmapi/v3/chat/completions
  - model: ernie-4.5-turbo-vl
  - 必须 header: Authorization: Bearer <token>
                x-bce-date: <RFC3339 UTC>(千帆 BCE 兼容 API 必需,否则 401)

== 用法 ==
    from main.task.task3.llm_ernie import call_vision, load_token
    token = load_token()
    verdict = call_vision(token, image_url, prompt, timeout=15)
    # → {"result": 0/1, "analysis": "..."} 或 {"result": None, "analysis": 错误描述}
"""
from __future__ import annotations

import datetime
import json
import os
import re
import sys
import time
from pathlib import Path
from typing import Optional

import requests


# -------- 配置 --------
ERNIE_CHAT_URL = os.getenv(
    "ERNIE_CHAT_URL",
    "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions",
)
ERNIE_VL_MODEL = os.getenv("ERNIE_VL_MODEL", "ernie-4.5-turbo-vl")
ERNIE_TIMEOUT_DEFAULT = 15.0

# config_car.yml:ernie_access_token 占位符(原 test_veggie_detect 也用这个判断)
_TOKEN_PLACEHOLDER = "REPLACE_YOUR_ACCESS_TOKEN_HERE"


def _repo_root() -> Path:
    """main/task/task3/llm_ernie.py → 仓库根(rak-car/)。

    parents[3] = rak-car (parents[2] 是 main/, 读到 main/config_car.yml 不存在)。
    """
    return Path(__file__).resolve().parents[3]


def _sanitize_key(k: str) -> str:
    """去 token 前后空白/CR/LF/tab/NUL(复制粘贴常粘到换行)。"""
    if not k:
        return k
    return k.strip().strip("\r\n\t\0")


def load_token(cli_token: Optional[str] = None) -> str:
    """优先级: CLI > env ERNIE_ACCESS_TOKEN > config_car.yml:ernie_access_token。"""
    if cli_token:
        return _sanitize_key(cli_token)
    t = os.getenv("ERNIE_ACCESS_TOKEN", "").strip()
    if t:
        return t
    cfg_path = _repo_root() / "config_car.yml"
    if cfg_path.exists():
        try:
            import yaml  # 局部导入,容忍无 yaml 环境

            d = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:  # noqa: BLE001
            print(f"[warn] read config_car.yml failed: {exc}", file=sys.stderr)
            d = {}
        t = (d.get("ernie_access_token") or "").strip()
        if t and t != _TOKEN_PLACEHOLDER:
            return t
    print("[fatal] 未找到 ERNIE access token (CLI/env/config_car.yml 均无)", file=sys.stderr)
    sys.exit(2)


def mask_token(token: str) -> str:
    """打码:前 4 + ... + 后 4,脱敏后用于日志。"""
    t = _sanitize_key(token)
    if len(t) <= 10:
        return "***"
    return f"{t[:4]}...{t[-4:]}"


def _bce_date_header() -> str:
    """千帆 BCE 兼容 API 必需的 x-bce-date(UTC RFC3339)。"""
    return datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ")


def _auth_header(token: str) -> dict[str, str]:
    return {
        "Authorization": f"Bearer {_sanitize_key(token)}",
        "Content-Type": "application/json",
        "x-bce-date": _bce_date_header(),
    }


def _strip_code_fence(text: str) -> str:
    """去 ```json ... ``` 包裹(模型偶尔会包)。"""
    s = text.strip()
    if not s.startswith("```"):
        return s
    lines = s.splitlines()
    if lines and lines[0].lstrip().startswith("```"):
        lines = lines[1:]
    if lines and lines[-1].lstrip().startswith("```"):
        lines = lines[:-1]
    return "\n".join(lines).strip()


def _parse_verdict(content: str) -> dict:
    """从模型 content 中提取 result(0/1) + analysis。失败 → {result:None, analysis:原因}。"""
    text = _strip_code_fence(content)
    # 优先尝试严格 JSON
    try:
        d = json.loads(text)
        if isinstance(d, dict):
            result = d.get("result")
            try:
                result = int(result) if result is not None else None
            except (TypeError, ValueError):
                result = None
            if result not in (0, 1):
                result = None
            return {
                "name": str(d.get("name") or d.get("species") or "unknown"),
                "result": result,
                "analysis": str(d.get("analysis") or ""),
            }
    except json.JSONDecodeError:
        pass
    # 退化:正则抓 result + analysis
    m_result = re.search(r'"result"\s*:\s*([01])', text)
    m_analysis = re.search(r'"analysis"\s*:\s*"([^"]*)"', text)
    if m_result:
        return {
            "name": "unknown",
            "result": int(m_result.group(1)),
            "analysis": m_analysis.group(1) if m_analysis else "",
        }
    return {"name": "unknown", "result": None, "analysis": f"unparseable: {content[:200]}"}


def _is_rate_limited(text):
    """aistudio 限流时返回 403 + errorCode 500 / “访问过于频繁” 等提示。"""
    return any(
        k in (text or "")
        for k in ("过于频繁", "频繁", "稍候再试", "rate limit", "too many", "限流")
    )


def call_vision(
    token: str,
    image_url: str,
    prompt: str,
    timeout: float = ERNIE_TIMEOUT_DEFAULT,
    system: Optional[str] = None,
    use_json_response_format: bool = False,  # ERNIE 不支持,保留接口
) -> dict:
    """调 ERNIE 多模态,返回 {name, result, analysis}。

    - image_url: "data:image/jpeg;base64,..." 或 https URL
    - system / use_json_response_format: 接口兼容参数,ERNIE 忽略
    """
    user_content = [
        {"type": "image_url", "image_url": {"url": image_url}},
        {"type": "text", "text": prompt},
    ]
    messages: list[dict] = []
    if system:
        messages.append({"role": "system", "content": system})
    messages.append({"role": "user", "content": user_content})

    body = {
        "model": ERNIE_VL_MODEL,
        "messages": messages,
        "temperature": 0.1,
        "top_p": 0.1,
    }
    headers = _auth_header(token)

    last_err = ""
    for attempt in range(3):
        try:
            resp = requests.post(ERNIE_CHAT_URL, headers=headers, json=body, timeout=timeout)
        except requests.Timeout:
            last_err = "timeout"
            time.sleep(1.5 * (attempt + 1))
            continue
        except requests.RequestException as exc:
            last_err = str(exc)
            time.sleep(1.5 * (attempt + 1))
            continue

        if resp.status_code == 401:
            # 401 = 鉴权真失败（token 无效/过期），重试无意义
            print(
                f"[fatal] ERNIE 401 - token invalid or expired\n"
                f"  endpoint: {ERNIE_CHAT_URL}\n  body: {resp.text[:200]}",
                file=sys.stderr,
            )
            sys.exit(2)
        if resp.status_code == 403:
            # aistudio 限流也返回 403；启动期 health 已验过 token，
            # 中途 403 基本是限流 → 退避重试，重试耗尽返回 unknown 继续流程
            reason = "rate-limited" if _is_rate_limited(resp.text) else "forbidden"
            last_err = f"HTTP 403 {reason} ({resp.text[:100]})"
            time.sleep(2.0 * (attempt + 1))
            continue
        if resp.status_code == 429 or resp.status_code >= 500:
            last_err = f"HTTP {resp.status_code}"
            time.sleep(1.5 * (attempt + 1))
            continue
        if not resp.ok:
            return {
                "name": "unknown",
                "result": None,
                "analysis": f"HTTP {resp.status_code}: {resp.text[:200]}",
            }
        try:
            content = resp.json()["choices"][0]["message"]["content"]
        except Exception as exc:  # noqa: BLE001
            return {"name": "unknown", "result": None, "analysis": f"parse error: {exc}"}
        return _parse_verdict(content)

    return {"name": "unknown", "result": None, "analysis": last_err or "no response"}


def check_health(token: str, timeout: float = 12.0) -> None:
    """启动期 PONG 探活:401/403 → sys.exit(2)。"""
    headers = _auth_header(token)
    body = {
        "model": ERNIE_VL_MODEL,
        "messages": [{"role": "user", "content": "reply PONG"}],
    }
    print(f"[health] {ERNIE_VL_MODEL} PONG via {ERNIE_CHAT_URL} ...", flush=True)
    print(
        f"[health] token len={len(_sanitize_key(token))} "
        f"first4={(_sanitize_key(token) or '')[:4]!r}",
        flush=True,
    )
    try:
        resp = requests.post(ERNIE_CHAT_URL, headers=headers, json=body, timeout=timeout)
    except Exception as exc:  # noqa: BLE001
        print(f"[fatal] network error: {exc}", file=sys.stderr)
        sys.exit(2)
    if resp.status_code in (401, 403):
        print(
            f"[fatal] ERNIE {resp.status_code} - token invalid or expired\n"
            f"  body: {resp.text[:200]}",
            file=sys.stderr,
        )
        sys.exit(2)
    if not resp.ok:
        print(f"[warn] ERNIE health HTTP {resp.status_code}: {resp.text[:200]}", file=sys.stderr)
        return
    try:
        content = resp.json()["choices"][0]["message"]["content"]
        if "PONG" in (content or "").upper():
            print(f"[health] OK ({resp.status_code}) - token valid, can start main loop")
        else:
            print(f"[warn] health unexpected content: {content[:100]!r}", file=sys.stderr)
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] parse health response: {exc}", file=sys.stderr)


if __name__ == "__main__":
    tok = os.getenv("ERNIE_ACCESS_TOKEN")
    if not tok:
        print("set ERNIE_ACCESS_TOKEN env first", file=sys.stderr)
        sys.exit(2)
    check_health(tok, timeout=10.0)
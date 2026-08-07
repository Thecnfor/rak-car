#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/task333/llm_minimax.py

MiniMax API 适配层(OpenAI 兼容 chat/completions + vision)。

== 用途 ==
替换 main/misc/test_pest_llm_shoot.py 的 _call_llm(百度 ERNIE),
保持下游调用方式不变:
    from main.tasks.task333.llm_minimax import call_vision, check_health
    verdict = call_vision(token, image_url, prompt, timeout=15)
    # → {"result": 0/1, "analysis": "..."}
    # → {"result": None, "analysis": "错误描述"}

== 可配置 ==
- MINIMAX_BASE       (默认 https://api.minimax.chat/v1)
- MINIMAX_VL_MODEL   (默认 MiniMax-VL-01)
- 都可用环境变量 MINIMAX_BASE / MINIMAX_VL_MODEL 覆盖。

== 鉴权 ==
Header: Authorization: Bearer <token>
不需要 x-bce-date(那是百度 BCE 兼容 API 的事)。
"""
from __future__ import annotations

import datetime
import json
import os
import re
import sys
import time
from typing import Any, Optional

import requests


# -------- 可配置 --------
MINIMAX_BASE = os.getenv("MINIMAX_BASE", "https://api.minimax.chat/v1").rstrip("/")
MINIMAX_VL_MODEL = os.getenv("MINIMAX_VL_MODEL", "MiniMax-M2.7")
MINIMAX_TIMEOUT_DEFAULT = 15.0


def _endpoint() -> str:
    return f"{MINIMAX_BASE}/chat/completions"


def _sanitize_key(k: str) -> str:
    """去掉 key 前后所有空白 / CR / LF / tab / NUL。

    这些字符在 HTTP header value 里是非法的(Go net/http 服务端
    会以 "Invalid leading whitespace, reserved character(s), or return
    character(s) in header" 直接 400 拒掉请求)。
    PowerShell $env:X="..." 复制粘贴常粘到尾部换行/空格,这里挡掉。
    """
    if not k:
        return k
    return k.strip().strip("\r\n\t\0")


def _auth_header(token: str) -> str:
    """生成 'Bearer <sanitized_key>'。key 经过清洗后再拼。"""
    return f"Bearer {_sanitize_key(token)}"


# -------- 健康检查 --------
def check_health(token: str, timeout: float = 12.0) -> None:
    """启动期 PONG 探活:发一个最小文本请求,401/403 → sys.exit(2)。"""
    body = {
        "model": MINIMAX_VL_MODEL,
        "messages": [{"role": "user", "content": "reply PONG"}],
    }
    headers = {
        "Authorization": _auth_header(token),
        "Content-Type": "application/json",
    }
    print(f"[health] {MINIMAX_VL_MODEL} PONG via {MINIMAX_BASE} ...", flush=True)
    print(f"[health] token len={len(_sanitize_key(token))} first4={(_sanitize_key(token) or '')[:4]!r}", flush=True)
    try:
        resp = requests.post(_endpoint(), headers=headers, json=body, timeout=timeout)
    except Exception as exc:  # noqa: BLE001
        print(f"[fatal] network error: {exc}", file=sys.stderr)
        sys.exit(2)

    if resp.status_code in (401, 403):
        print(
            f"[fatal] MiniMax {resp.status_code} - token invalid or expired\n"
            f"  endpoint: {_endpoint()}\n"
            f"  model:    {MINIMAX_VL_MODEL}\n"
            f"  body:     {resp.text[:200]}",
            file=sys.stderr,
        )
        sys.exit(2)

    if not resp.ok:
        print(
            f"[warn] MiniMax health HTTP {resp.status_code}: {resp.text[:200]}",
            file=sys.stderr,
        )
        return

    try:
        payload = resp.json()
        content = (payload.get("choices") or [{}])[0].get("message", {}).get("content", "")
    except (json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
        print(f"[warn] parse health response: {exc}", file=sys.stderr)
        return

    if "PONG" in content.upper():
        print(f"[health] OK ({resp.status_code}) - token valid, can start main loop")
    else:
        print(f"[warn] health unexpected content: {content[:100]!r}", file=sys.stderr)


# -------- 主调用:vision --------
# 强制 JSON-only 的 system 提示(防 MiniMax 推理模型只输出思考不输出 JSON)
MINIMAX_JSON_ONLY_SYSTEM = (
    "你是一个严格的 JSON-only 助手。"
    "用户会给你一张图和一个任务。"
    "无论中间如何思考,你**最终必须**只输出一个合法的 JSON 对象,不要输出 Markdown 代码块标记,不要输出任何其他文字。"
    "JSON 格式严格遵守用户要求的 schema。"
)

def call_vision(
    token: str,
    image_url: str,
    prompt: str,
    timeout: float = MINIMAX_TIMEOUT_DEFAULT,
    system: Optional[str] = MINIMAX_JSON_ONLY_SYSTEM,
    use_json_response_format: bool = True,
) -> dict:
    """调 MiniMax 视觉模型,返回 {result, analysis} 或 {result: None, analysis: 错误}。

    OpenAI 兼容格式:
      POST {base}/chat/completions
      Body: {model, messages:[{role:user, content:[
            {type:text, text:prompt},
            {type:image_url, image_url:{url:data:image/jpeg;base64,...}}
      ]}]}
    """
    messages: list[dict] = []
    if system:
        messages.append({"role": "system", "content": system})
    messages.append({
        "role": "user",
        "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": image_url}},
        ],
    })
    body: dict[str, Any] = {"model": MINIMAX_VL_MODEL, "messages": messages}
    if use_json_response_format:
        # 兼容 OpenAI 的 response_format={"type":"json_object"} —— 部分 provider 支持
        body["response_format"] = {"type": "json_object"}
    headers = {
        "Authorization": _auth_header(token),
        "Content-Type": "application/json",
    }

    def _post_once() -> Optional[requests.Response]:
        return requests.post(_endpoint(), headers=headers, json=body, timeout=timeout)

    def _parse(content: str) -> Optional[dict]:
        text = content.strip()
        # 1) 剥 <think>...</think> 思考块(MiniMax 推理模型常带;含嵌套)
        #    非贪婪 + 多次,处理多个思考块 + 不闭合的情况
        text = re.sub(r"<think>\b[\s\S]*?(?:</think>|$)", "", text, flags=re.IGNORECASE).strip()
        text = re.sub(r"</think>", "", text, flags=re.IGNORECASE).strip()
        # 2) 剥 ```json ... ``` / ``` ... ``` 围栏(可能多个)
        def _strip_fences(t: str) -> str:
            out = []
            i = 0
            while i < len(t):
                if t[i:].lstrip().startswith("```"):
                    # 找到下一个 ```
                    end = t.find("```", i + 3)
                    if end < 0:
                        # 没有结束符,丢掉剩下
                        break
                    i = end + 3
                    continue
                out.append(t[i])
                i += 1
            return "".join(out).strip()
        text = _strip_fences(text)
        # 3) 抠 JSON object: 取**最后一个**平衡的 {...}
        candidates = []
        for m in re.finditer(r"\{", text):
            start = m.start()
            depth = 0
            for j in range(start, len(text)):
                c = text[j]
                if c == "{":
                    depth += 1
                elif c == "}":
                    depth -= 1
                    if depth == 0:
                        candidates.append(text[start:j + 1])
                        break
        # 优先尝试完整文本,再尝试最后一个平衡块
        for cand in [text] + list(reversed(candidates)):
            try:
                data = json.loads(cand)
            except json.JSONDecodeError:
                continue
            if isinstance(data, dict):
                result = data.get("result")
                analysis = data.get("analysis")
                if isinstance(analysis, str) and result in (0, 1):
                    return {"result": int(result), "analysis": analysis}
        return None

    last_err = ""
    for attempt in range(2):
        try:
            resp = _post_once()
        except requests.Timeout as exc:
            last_err = f"timeout after {timeout}s"
            time.sleep(1.5)
            continue
        except requests.RequestException as exc:
            last_err = f"request error: {exc}"
            time.sleep(1.5)
            continue

        if resp.status_code in (401, 403):
            print(f"[fatal] MiniMax {resp.status_code} - token invalid during main loop", file=sys.stderr)
            sys.exit(2)
        if resp.status_code == 429 or resp.status_code >= 500:
            last_err = f"HTTP {resp.status_code}: {resp.text[:200]}"
            time.sleep(1.5)
            continue
        if not resp.ok:
            # 把尽可能多的诊断信息塞进 reason:URL + model + masked token + 响应体
            # + 响应 Content-Type + 响应头(防"header v"这种截断错误)
            try:
                body_text = resp.text
            except Exception:
                body_text = "<unreadable body>"
            masked_tok = mask_token(_sanitize_key(token))
            try:
                hdr_dump = "\n".join(
                    f"    {k}: {v}" for k, v in resp.headers.items()
                )
            except Exception:
                hdr_dump = "    <unreadable headers>"
            return {
                "result": None,
                "analysis": (
                    f"HTTP {resp.status_code} from {_endpoint()} "
                    f"(model={MINIMAX_VL_MODEL}, token={masked_tok})\n"
                    f"  Content-Type: {resp.headers.get('Content-Type', '?')}\n"
                    f"  Response headers:\n{hdr_dump}\n"
                    f"  Body: {body_text[:400]}"
                ),
            }

        try:
            payload = resp.json()
            content = payload["choices"][0]["message"]["content"]
        except (json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
            return {"result": None, "analysis": f"bad response shape: {exc}"}

        parsed = _parse(content)
        if parsed is not None:
            return parsed
        return {"result": None, "analysis": f"unparseable LLM output: {content[:200]}"}

    return {"result": None, "analysis": last_err or "unknown error"}


def mask_token(token: str) -> str:
    if len(token) <= 8:
        return "***"
    return f"{token[:4]}***{token[-4:]}"


if __name__ == "__main__":
    # 简单的自检入口
    tok = os.getenv("MINIMAX_API_KEY") or os.getenv("ERNIE_ACCESS_TOKEN")
    if not tok:
        print("set MINIMAX_API_KEY or ERNIE_ACCESS_TOKEN env first", file=sys.stderr)
        sys.exit(2)
    check_health(tok, timeout=10.0)
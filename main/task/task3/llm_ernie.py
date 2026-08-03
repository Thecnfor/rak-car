#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/llm_ernie.py - Baidu PaddlePaddle ERNIE VL adapter

适配层:对外暴露和 llm_minimax.py 完全相同的接口:
  - call_vision(token, image_url, prompt, timeout, system=..., use_json_response_format=...)
  - check_health(token, timeout)
  - mask_token(token)

实现:直接复用 main/misc/test_pest_llm_shoot.py 已验证的百度 _call_llm + _check_token_health。

ERNIE 文档:
  - endpoint: https://aistudio.baidu.com/llm/lmapi/v3/chat/completions
  - model: ernie-4.5-turbo-vl
  - 必须 header: Authorization: Bearer <token>
              x-bce-date: <RFC3339 UTC>(千帆 BCE 兼容 API 必需,否则 401)
"""
from __future__ import annotations

import datetime
import os
import re
import sys
import time
from typing import Optional

import requests


# -------- 配置 --------
ERNIE_CHAT_URL = os.getenv(
    "ERNIE_CHAT_URL",
    "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions",
)
ERNIE_VL_MODEL = os.getenv("ERNIE_VL_MODEL", "ernie-4.5-turbo-vl")
ERNIE_TIMEOUT_DEFAULT = 15.0

# 复用 test_pest_llm_shoot 已验证的 _call_llm + token 健康检查
from main.misc.test_pest_llm_shoot import (   # noqa: E402
    _call_llm as _ernie_call_llm_impl,
    _check_token_health as _ernie_check_token_health_impl,
    _mask_token as _mask_token_impl,
    _load_token as _ernie_load_token_impl,
)


def call_vision(
    token: str,
    image_url: str,
    prompt: str,
    timeout: float = ERNIE_TIMEOUT_DEFAULT,
    system: Optional[str] = None,
    use_json_response_format: bool = False,  # ERNIE 不支持此参数(保留接口兼容)
) -> dict:
    """调 ERNIE 多模态,返回 {result, analysis} 或 {result: None, analysis: 错误}。

    system / use_json_response_format 参数保留接口兼容(对 MiniMax 适配层有意义,
    对 ERNIE 忽略)。
    """
    return _ernie_call_llm_impl(token, image_url, prompt, timeout)


def check_health(token: str, timeout: float = 12.0) -> None:
    """启动期 PONG 探活:401/403 → sys.exit(2)。"""
    _ernie_check_token_health_impl(token, timeout=timeout)


def mask_token(token: str) -> str:
    return _mask_token_impl(token)


def load_token(cli_token: Optional[str] = None) -> str:
    """CLI > env ERNIE_ACCESS_TOKEN > config_car.yml。"""
    return _ernie_load_token_impl(cli_token)


if __name__ == "__main__":
    tok = os.getenv("ERNIE_ACCESS_TOKEN")
    if not tok:
        print("set ERNIE_ACCESS_TOKEN env first", file=sys.stderr)
        sys.exit(2)
    check_health(tok, timeout=10.0)
#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/inspect_backend.py

诊断脚本:确认 drive_then_analyze.py 等实际用的是哪个 LLM 后端 + endpoint +
model + key。不连硬件,不调 Paddle,纯环境探测;加 --ping 才会真发请求。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    $env:MINIMAX_API_KEY = "..."
    python -m main.tasks.task333.inspect_backend            # 只查配置
    python -m main.tasks.task333.inspect_backend --ping     # 再加 PONG 探活
    # 临时换 URL / model 看一眼(不会落到文件)
    python -m main.tasks.task333.inspect_backend --base "https://other/v1" --model "other-vl"
"""
from __future__ import annotations

import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser(
        description="Inspect which LLM backend drive_then_analyze.py will use")
    ap.add_argument("--ping", action="store_true",
                    help="send a PONG health-check request after printing config")
    ap.add_argument("--base", default=None,
                    help="临时覆盖 MINIMAX_BASE(只在本进程生效)")
    ap.add_argument("--model", default=None, dest="vl_model",
                    help="临时覆盖 MINIMAX_VL_MODEL(只在本进程生效)")
    args = ap.parse_args()

    print("=" * 64)
    print(" LLM 后端诊断 (drive_then_analyze.py / scan_* / judge_* 等)")
    print("=" * 64)

    # ---- 1. token 来源 + sanitize 前后对比 ----
    ernie_tok = os.getenv("ERNIE_ACCESS_TOKEN")
    minimax_tok = os.getenv("MINIMAX_API_KEY")

    if ernie_tok:
        token_raw, source, backend_guess = ernie_tok, "ERNIE_ACCESS_TOKEN", "ERNIE"
    elif minimax_tok:
        token_raw, source, backend_guess = minimax_tok, "MINIMAX_API_KEY", "MiniMax(fallback)"
    else:
        print("[fatal] 没找到任何 token。请先设以下任一环境变量:")
        print("  $env:ERNIE_ACCESS_TOKEN = '...'   (首选)")
        print("  $env:MINIMAX_API_KEY    = '...'   (兜底)")
        sys.exit(2)

    try:
        from main.tasks.task333.llm_minimax import _sanitize_key, mask_token
    except Exception as exc:
        print(f"[fatal] 加载 llm_minimax 失败(只为复用 sanitize 工具函数): {exc}")
        sys.exit(2)

    clean = _sanitize_key(token_raw)
    print(f"\n[token]")
    print(f"  来源 env var:       {source}")
    print(f"  原始长度:           {len(token_raw)}")
    print(f"  sanitize 后长度:    {len(clean)}")
    print(f"  原始  前8:          {token_raw[:8]!r}")
    print(f"  sanitize 前8:      {clean[:8]!r}")
    print(f"  原始  后8:          {token_raw[-8:]!r}")
    print(f"  sanitize 后8:      {clean[-8:]!r}")
    if token_raw != clean:
        diff = len(token_raw) - len(clean)
        print(f"  ⚠ sanitize 剥掉 {diff} 字符(空白/\\r\\n\\t\\0)")
    else:
        print(f"  ✓ sanitize 没动 key(干净)")
    print(f"  mask_token:         {mask_token(clean)}")

    # ---- 2. adapter 当前生效配置 ----
    print(f"\n[backend: {backend_guess}]")
    if backend_guess == "ERNIE":
        try:
            import main.misc.test_pest_llm_shoot as ernie_mod
            print(f"  ERNIE_CHAT_URL:     {ernie_mod.ERNIE_CHAT_URL}")
            print(f"  ERNIE_VL_MODEL:     {ernie_mod.ERNIE_VL_MODEL}")
            print(f"  Temperature:        {ernie_mod.ERNIE_TEMPERATURE}")
            print(f"  Top-P:              {ernie_mod.ERNIE_TOP_P}")
        except Exception as exc:
            print(f"  [warn] 读不到 ERNIE 常量: {exc}")
    else:
        try:
            import main.tasks.task333.llm_minimax as llm
            base = (args.base.rstrip("/") if args.base else llm.MINIMAX_BASE)
            model = (args.vl_model if args.vl_model else llm.MINIMAX_VL_MODEL)
            # 本进程覆盖(只影响本次 inspect,不落盘)
            if args.base:
                llm.MINIMAX_BASE = base
            if args.vl_model:
                llm.MINIMAX_VL_MODEL = model
            print(f"  MINIMAX_BASE:       {llm.MINIMAX_BASE}")
            print(f"  MINIMAX_VL_MODEL:   {llm.MINIMAX_VL_MODEL}")
            print(f"  实际请求 URL:       {llm.MINIMAX_BASE}/chat/completions")
            print(f"  POST body.model:    {llm.MINIMAX_VL_MODEL}")
            print(f"  POST response_fmt:  {{\"type\":\"json_object\"}}  (OpenAI 兼容)")
            print(f"  Timeout default:    {llm.MINIMAX_TIMEOUT_DEFAULT}s")
        except Exception as exc:
            print(f"  [warn] 读不到 MiniMax adapter: {exc}")

    # ---- 3. 模拟 drive_then_analyze.py 的 backend 选择 ----
    print(f"\n[drive_then_analyze.py 会怎么选 backend]")
    if ernie_tok:
        sel = "ERNIE  (ERNIE_ACCESS_TOKEN 已设,优先用)"
    elif minimax_tok:
        sel = "MiniMax(fallback)  (没设 ERNIE_ACCESS_TOKEN,降级)"
    else:
        sel = "?"
    print(f"  -> {sel}")

    # ---- 4. 可选 PONG 探活 ----
    if args.ping:
        print(f"\n[ping] 真发一次最小请求到 {backend_guess} ...")
        if backend_guess == "ERNIE":
            try:
                from main.misc.test_pest_llm_shoot import _check_token_health
                _check_token_health(clean, timeout=10.0)
            except SystemExit as exc:
                print(f"  _check_token_health sys.exit({exc.code})")
        else:
            try:
                from main.tasks.task333.llm_minimax import check_health
                check_health(clean, timeout=10.0)
            except SystemExit as exc:
                print(f"  check_health sys.exit({exc.code})")
    else:
        print(f"\n[hint] 上面全是本地配置,没发任何网络请求。")
        print(f"       要真验证 token 是否可用,加 --ping 重跑:")

    print("\n" + "=" * 64)
    print(" 诊断完成")
    print("=" * 64)


if __name__ == "__main__":
    main()
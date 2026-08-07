#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/test_shoot_one.py - 裸射击:不检测、不瞄准,直接发 1 发子弹

用途:验证枪机械/电控是否正常 — 不依赖 yolo、不调车、不转 yaw。
如果这都不响 → 硬件或 runtime/shooting action 问题,跟识别逻辑无关。

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.test_shoot_one
"""
from __future__ import annotations

import sys

from main.api_client import RuntimeApiClient


def car_call(client, name, *args, timeout=10.0, **kwargs):
    job = client.execute_car_action(name, *args, timeout=timeout,
                                    sync=True, **kwargs)
    if job.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {job.get('error')}")
    return job.get("result")


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()
    print("[shoot] ready, send 1 bullet...", flush=True)
    try:
        car_call(client, "shooting", timeout=5)
        print("[shoot] OK", flush=True)
    except Exception as e:
        print(f"[shoot err] {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
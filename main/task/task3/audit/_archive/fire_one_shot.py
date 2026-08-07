#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/fire_one_shot.py - single shot

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.fire_one_shot
"""
from __future__ import annotations

import time

from main.api_client import RuntimeApiClient


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()

    t0 = time.time()
    job = client.execute_car_action("shooting", timeout=8, sync=False)
    done = client.wait_job(job["id"], timeout=15)
    dt = time.time() - t0

    print(f"[shot] status={done.get('status')}  err={done.get('error')}  took={dt:.2f}s")
    if done.get("status") != "succeeded":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
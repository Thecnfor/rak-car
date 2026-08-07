#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/task333/fire.py — 射击控制台(可调参数)

用法:
    python -m main.tasks.task333.fire
    python -m main.tasks.task333.fire --count 5
    python -m main.tasks.task333.fire --count 3 --interval 1.0 --beep
"""
from __future__ import annotations

import argparse
import time

from main.api_client import RuntimeApiClient


def car_call(client, name, *args, timeout=15.0, **kw):
    job = client.execute_car_action(name, *args, timeout=timeout, sync=False, **kw)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {done.get('error')}")
    return done.get("result")


def main():
    ap = argparse.ArgumentParser(description="射击控制台")
    ap.add_argument("--count", type=int, default=1, help="连发次数(默认 1)")
    ap.add_argument("--interval", type=float, default=1.0, help="两次射击间隔秒数(默认 1.0)")
    ap.add_argument("--timeout", type=float, default=8.0, help="单次 shooting HTTP 超时秒数(默认 8)")
    ap.add_argument("--beep", action="store_true", help="打前/打后 beep 提示")
    args = ap.parse_args()

    if args.count < 1:
        ap.error("--count 必须 >= 1")
    if args.interval < 0:
        ap.error("--interval 必须 >= 0")

    client = RuntimeApiClient()
    client.wait_until_ready()

    print(f"[fire] count={args.count} interval={args.interval}s timeout={args.timeout}s beep={args.beep}")

    if args.beep:
        car_call(client, "beep", timeout=5)

    for i in range(args.count):
        t0 = time.time()
        try:
            car_call(client, "shooting", timeout=args.timeout)
            print(f"[shot {i+1}/{args.count}] ok in {time.time()-t0:.2f}s")
        except Exception as exc:
            print(f"[shot {i+1}/{args.count}] FAIL: {exc}", flush=True)
            raise
        if i < args.count - 1 and args.interval > 0:
            time.sleep(args.interval)

    if args.beep:
        car_call(client, "beep", timeout=5)

    print(f"[done] fired {args.count} shots")


if __name__ == "__main__":
    main()
#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/arm_y_minus150.py - move arm y to -150mm

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.arm_y_minus150
    python -m main.tasks.task333.arm_y_minus150 --target -0.150
"""
from __future__ import annotations

import argparse
import time

from main.api_client import RuntimeApiClient


def arm_call(client, name, *a, timeout=20.0, **k):
    job = client.execute_arm_action(name, *a, timeout=timeout, sync=False, **k)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"arm.{name} failed: {done.get('error')}")
    return done.get("result")


def main():
    ap = argparse.ArgumentParser(description="move arm y to target")
    ap.add_argument("--target", type=float, default=-0.150,
                    help="target y position in meters (default -0.150 = -150mm)")
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    before = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"[before] y_m={before.get('y_m')} ref_encoder={before.get('ref_encoder')}",
          flush=True)

    print(f"[move] arm.move_y_position({args.target}) ...", flush=True)
    arm_call(client, "move_y_position", float(args.target), timeout=args.timeout)
    time.sleep(0.3)

    after = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"[after]  y_m={after.get('y_m')} ref_encoder={after.get('ref_encoder')}",
          flush=True)
    print(f"[done] arm y -> {after.get('y_m')}", flush=True)


if __name__ == "__main__":
    main()
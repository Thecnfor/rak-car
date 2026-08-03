#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/arm_y_reset.py - reset arm y-axis to 0

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.arm_y_reset
    python -m main.tasks.task333.arm_y_reset --target 0.0 --timeout 20
"""
from __future__ import annotations

import argparse
import time

from main.api_client import RuntimeApiClient


def main():
    ap = argparse.ArgumentParser(description="reset arm y-axis to target")
    ap.add_argument("--target", type=float, default=0.0,
                    help="target y position in meters (default 0.0)")
    ap.add_argument("--timeout", type=float, default=20.0,
                    help="HTTP job timeout sec (default 20)")
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    before = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"[before] y_m={before.get('y_m')} x_m={before.get('x_m')} "
          f"ref_encoder={before.get('ref_encoder')} active={before.get('active')}")

    job = client.execute_arm_action("move_y_position", float(args.target),
                                    timeout=args.timeout, sync=False)
    done = client.wait_job(job["id"], timeout=args.timeout + 10)

    if done.get("status") != "succeeded":
        print(f"[err] move_y_position failed: status={done.get('status')} "
              f"error={done.get('error')}", flush=True)
        raise SystemExit(1)

    time.sleep(0.3)
    after = (client.get_arm_state() or {}).get("arm_state") or {}
    print(f"[after]  y_m={after.get('y_m')} x_m={after.get('x_m')} "
          f"ref_encoder={after.get('ref_encoder')} active={after.get('active')}")
    print(f"[done] arm y moved to {after.get('y_m')}")


if __name__ == "__main__":
    main()
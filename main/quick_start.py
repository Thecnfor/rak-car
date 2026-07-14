#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import os

try:
    from main.api_client import RuntimeApiClient
    from main.settings import load_settings
except ImportError:  # pragma: no cover
    from api_client import RuntimeApiClient
    from settings import load_settings


def main():
    settings = load_settings()
    client = RuntimeApiClient()
    run_beep = os.getenv("RAK_CAR_QUICK_START_BEEP", "0") == "1"
    print("=== config ===")
    print("SERVER_ORIGIN =", settings.server_origin)
    print("API_BASE      =", client.api_base)
    print("STREAMER_URL  =", settings.streamer_url)
    print("API_PREFIX    =", client.api_prefix)
    print("\n=== health ===")
    print(json.dumps(client.get_health(), ensure_ascii=False, indent=2))
    print("\n=== actions sample ===")
    actions_response = client.get_actions()
    actions = (
        actions_response.get("actions")
        or (actions_response.get("data") or {}).get("actions")
        or {}
    )
    def _sample_names(value):
        if isinstance(value, dict):
            return sorted(list(value.keys()))[:8]
        if isinstance(value, list):
            return sorted([str(item) for item in value])[:8]
        return []
    sample = {
        "car": _sample_names(actions.get("car")),
        "arm": _sample_names(actions.get("arm")),
        "task": _sample_names(actions.get("task")),
    }
    print(json.dumps(sample, ensure_ascii=False, indent=2))

    if run_beep:
        print("\n=== beep ===")
        print(
            json.dumps(
                client.call("car", "beep", timeout=40),
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        print("\n=== beep ===")
        print("已跳过。需要主动蜂鸣时执行: RAK_CAR_QUICK_START_BEEP=1 python3 quick_start.py")

    run_realtime = os.getenv("RAK_CAR_QUICK_START_REALTIME", "0") == "1"
    if run_realtime:
        print("\n=== realtime bus-servo read (opt-in, requires mc602 hardware) ===")
        try:
            print(
                json.dumps(
                    client.realtime_bus_servo_read(1),
                    ensure_ascii=False,
                    indent=2,
                )
            )
        except Exception as exc:
            print(f"realtime bus-servo read failed (likely no hardware): {exc}")
    else:
        print("\n=== realtime ===")
        print(
            "已跳过。需要主动测实时硬件路径时执行:"
            " RAK_CAR_QUICK_START_REALTIME=1 python3 quick_start.py"
        )


if __name__ == "__main__":
    main()

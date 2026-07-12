#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json

from main.api_client import RuntimeApiClient


def main():
    client = RuntimeApiClient()
    print("API_BASE =", client.api_base)
    print("API_PREFIX =", client.api_prefix)
    print("\n=== health ===")
    print(json.dumps(client.get_health(), ensure_ascii=False, indent=2))
    print("\n=== beep ===")
    print(
        json.dumps(
            client.call("car", "beep", timeout=40),
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()

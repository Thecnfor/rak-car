#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from dataclasses import dataclass

DEFAULT_API_BASE = "http://127.0.0.1:5050"
DEFAULT_API_PREFIX = "/v1"
DEFAULT_REQUEST_TIMEOUT = 10.0
DEFAULT_WAIT_TIMEOUT = 300.0
DEFAULT_POLL_INTERVAL = 0.5
DEFAULT_STREAMER_URL = "http://127.0.0.1:5000/"


@dataclass(frozen=True)
class BusinessSettings:
    api_base: str
    api_prefix: str
    request_timeout: float
    wait_timeout: float
    poll_interval: float
    streamer_url: str


def load_settings():
    return BusinessSettings(
        api_base=os.getenv("RAK_CAR_API_BASE", DEFAULT_API_BASE).rstrip("/"),
        api_prefix=os.getenv("RAK_CAR_API_PREFIX", DEFAULT_API_PREFIX),
        request_timeout=float(
            os.getenv("RAK_CAR_REQUEST_TIMEOUT", str(DEFAULT_REQUEST_TIMEOUT))
        ),
        wait_timeout=float(
            os.getenv(
                "RAK_CAR_WAIT_TIMEOUT",
                str(DEFAULT_WAIT_TIMEOUT),
            )
        ),
        poll_interval=float(
            os.getenv(
                "RAK_CAR_POLL_INTERVAL",
                str(DEFAULT_POLL_INTERVAL),
            )
        ),
        streamer_url=os.getenv(
            "RAK_CAR_STREAMER_URL",
            DEFAULT_STREAMER_URL,
        ),
    )

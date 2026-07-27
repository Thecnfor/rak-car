#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from dataclasses import dataclass

DEFAULT_SERVER_ORIGIN = "http://192.168.6.231"
DEFAULT_API_PORT = 5050
DEFAULT_STREAM_PORT = DEFAULT_API_PORT
DEFAULT_STREAM_PATH = "/stream/"
DEFAULT_API_PREFIX = "/v1"
DEFAULT_REQUEST_TIMEOUT = 10.0
DEFAULT_WAIT_TIMEOUT = 300.0
DEFAULT_POLL_INTERVAL = 0.5


def _strip_trailing_slash(value):
    return str(value).rstrip("/")


def _build_http_url(origin, port):
    return f"{_strip_trailing_slash(origin)}:{int(port)}"


def _build_streamer_url(origin, port, path):
    stream_path = "/" + str(path or "").strip("/")
    if stream_path != "/":
        stream_path += "/"
    return f"{_build_http_url(origin, port)}{stream_path}"


@dataclass(frozen=True)
class BusinessSettings:
    server_origin: str
    api_base: str
    api_port: int
    api_prefix: str
    request_timeout: float
    wait_timeout: float
    poll_interval: float
    streamer_url: str
    stream_port: int


def load_settings():
    server_origin = _strip_trailing_slash(
        os.getenv("RAK_CAR_SERVER_ORIGIN", DEFAULT_SERVER_ORIGIN)
    )
    api_port = int(os.getenv("RAK_CAR_API_PORT", str(DEFAULT_API_PORT)))
    stream_port = int(os.getenv("RAK_CAR_STREAM_PORT", str(DEFAULT_STREAM_PORT)))
    stream_path = os.getenv("RAK_CAR_STREAM_PATH", DEFAULT_STREAM_PATH)
    api_base = _strip_trailing_slash(
        os.getenv(
            "RAK_CAR_API_BASE",
            _build_http_url(server_origin, api_port),
        )
    )
    streamer_url = os.getenv(
        "RAK_CAR_STREAMER_URL",
        _build_streamer_url(server_origin, stream_port, stream_path),
    )
    return BusinessSettings(
        server_origin=server_origin,
        api_base=api_base,
        api_port=api_port,
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
        streamer_url=streamer_url,
        stream_port=stream_port,
    )

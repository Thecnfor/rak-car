#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""runtime.api.routers — 按资源拆分的 APIRouter 子包。

- _helpers:   共享 payload/execute/WS dispatch 辅助
- stream:     /stream/*、/video_feed/*
- keypress:   /keypress
- system:     /v1/health、/v1/runtime、/v1/actions、/v1/config、/v1/infer/*、/v1/estop*
- vision:     /v1/vision/*
- realtime:   /v1/realtime/*
- jobs:       /v1/jobs*、/v1/execute、/v1/control/*
- ws:         /v1/ws
- legacy:     /api/*（旧前缀兼容面）
"""

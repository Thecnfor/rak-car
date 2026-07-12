#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
runtime 服务集中配置

多人协作时，优先改这里，不要到处改脚本里的 IP/端口。
"""
import os


# 对外给局域网同事访问的默认地址
PUBLIC_HOST = "192.168.3.60"
PUBLIC_STREAM_PORT = 5000

# 本机监听地址
BIND_HOST = "0.0.0.0"
BIND_PORT = 5050

# API 路由前缀
API_V1_PREFIX = "/v1"
LEGACY_API_PREFIX = "/api"

# 初始化行为
AUTO_INIT_ON_START = True
RESET_ARM_ON_AUTO_INIT = False
RESET_POSITION_ON_INIT = True
STOP_AFTER_ACTION_DEFAULT = False
AUTO_INIT_RETRY_INTERVAL = 3.0
ACTION_READY_TIMEOUT = 30.0
ACTION_READY_POLL_INTERVAL = 0.5

# 任务队列
JOB_HISTORY_LIMIT = 100
DEFAULT_JOB_WAIT_TIMEOUT = 300.0
DEFAULT_POLL_INTERVAL = 0.5


def _bool_env(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def get_bind_host():
    return os.getenv("RAK_CAR_BIND_HOST", BIND_HOST)


def get_bind_port():
    return int(os.getenv("RAK_CAR_BIND_PORT", str(BIND_PORT)))


def get_public_host():
    return os.getenv("RAK_CAR_PUBLIC_HOST", PUBLIC_HOST)


def get_public_api_host():
    return get_public_host()


def get_public_api_port():
    return get_bind_port()


def get_public_stream_host():
    return get_public_host()


def get_public_stream_port():
    return int(os.getenv("RAK_CAR_PUBLIC_STREAM_PORT", str(PUBLIC_STREAM_PORT)))


def get_api_v1_prefix():
    return os.getenv("RAK_CAR_API_PREFIX", API_V1_PREFIX)


def get_legacy_api_prefix():
    return os.getenv("RAK_CAR_LEGACY_API_PREFIX", LEGACY_API_PREFIX)


def get_auto_init_on_start():
    return _bool_env("RAK_CAR_AUTO_INIT", AUTO_INIT_ON_START)


def get_reset_arm_on_auto_init():
    return _bool_env("RAK_CAR_RESET_ARM", RESET_ARM_ON_AUTO_INIT)


def get_reset_position_on_init():
    return _bool_env("RAK_CAR_RESET_POSITION_ON_INIT", RESET_POSITION_ON_INIT)


def get_stop_after_action_default():
    return _bool_env("RAK_CAR_STOP_AFTER_ACTION", STOP_AFTER_ACTION_DEFAULT)


def get_auto_init_retry_interval():
    return float(
        os.getenv(
            "RAK_CAR_AUTO_INIT_RETRY_INTERVAL",
            str(AUTO_INIT_RETRY_INTERVAL),
        )
    )


def get_action_ready_timeout():
    return float(
        os.getenv(
            "RAK_CAR_ACTION_READY_TIMEOUT",
            str(ACTION_READY_TIMEOUT),
        )
    )


def get_action_ready_poll_interval():
    return float(
        os.getenv(
            "RAK_CAR_ACTION_READY_POLL_INTERVAL",
            str(ACTION_READY_POLL_INTERVAL),
        )
    )


def get_public_api_base():
    return f"http://{get_public_api_host()}:{get_public_api_port()}"


def get_public_stream_base():
    return f"http://{get_public_stream_host()}:{get_public_stream_port()}/"


def get_bind_base():
    return f"http://{get_bind_host()}:{get_bind_port()}"


def get_runtime_settings():
    return {
        "bind_host": get_bind_host(),
        "bind_port": get_bind_port(),
        "public_host": get_public_host(),
        "public_api_base": get_public_api_base(),
        "public_stream_base": get_public_stream_base(),
        "api_v1_prefix": get_api_v1_prefix(),
        "legacy_api_prefix": get_legacy_api_prefix(),
        "auto_init_on_start": get_auto_init_on_start(),
        "reset_arm_on_auto_init": get_reset_arm_on_auto_init(),
        "reset_position_on_init": get_reset_position_on_init(),
        "stop_after_action_default": get_stop_after_action_default(),
        "auto_init_retry_interval": get_auto_init_retry_interval(),
        "action_ready_timeout": get_action_ready_timeout(),
        "action_ready_poll_interval": get_action_ready_poll_interval(),
        "job_history_limit": JOB_HISTORY_LIMIT,
    }

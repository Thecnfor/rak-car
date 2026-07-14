#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
runtime 服务集中配置

多人协作时，优先改这里，不要到处改脚本里的 IP/端口。
"""
import os


# 对外给局域网同事访问的默认地址
PUBLIC_HOST = "192.168.3.60"
PUBLIC_STREAM_PORT = 5050
PUBLIC_STREAM_PATH = "/stream/"

# 本机监听地址
BIND_HOST = "0.0.0.0"
BIND_PORT = 5050

# API 路由前缀
API_V1_PREFIX = "/v1"
LEGACY_API_PREFIX = "/api"

# 初始化行为
# 默认开启后台自愈：API 进程启动后会在后台持续尝试拉起小车，
# 下位机掉电恢复后也能自动重建整车对象。需要时可通过
# 环境变量 RAK_CAR_AUTO_INIT=0 关闭。
AUTO_INIT_ON_START = True
RESET_ARM_ON_AUTO_INIT = False
RESET_POSITION_ON_INIT = True
STOP_AFTER_ACTION_DEFAULT = False
AUTO_INIT_RETRY_INTERVAL = 3.0
ACTION_READY_TIMEOUT = 30.0
ACTION_READY_POLL_INTERVAL = 0.5
AUTO_DOWNLOAD_ON_BOOTLOADER = False
INFER_AUTO_START = True
INFER_POLL_INTERVAL = 1.0
INFER_READY_TIMEOUT = 45.0
INFER_HEALTH_TIMEOUT = 2.0
INFER_BACKEND_SCRIPT = "/home/jetson/workspace/rak-car/smartcar/paddlebaidu/infer_cs/base/infer_back_end.py"

# 任务队列
JOB_HISTORY_LIMIT = 100
DEFAULT_JOB_WAIT_TIMEOUT = 300.0
DEFAULT_POLL_INTERVAL = 0.5


def _bool_env(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _normalize_path(value):
    text = "/" + str(value or "").strip("/")
    if text == "/":
        return text
    return text + "/"


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
    value = os.getenv("RAK_CAR_PUBLIC_STREAM_PORT")
    if value is None:
        return get_bind_port()
    return int(value)


def get_public_stream_path():
    return _normalize_path(
        os.getenv("RAK_CAR_PUBLIC_STREAM_PATH", PUBLIC_STREAM_PATH)
    )


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


def get_auto_download_on_bootloader():
    return _bool_env(
        "RAK_CAR_AUTO_DOWNLOAD_ON_BOOTLOADER",
        AUTO_DOWNLOAD_ON_BOOTLOADER,
    )


def get_infer_auto_start():
    return _bool_env("RAK_CAR_INFER_AUTO_START", INFER_AUTO_START)


def get_infer_poll_interval():
    return float(
        os.getenv(
            "RAK_CAR_INFER_POLL_INTERVAL",
            str(INFER_POLL_INTERVAL),
        )
    )


def get_infer_ready_timeout():
    return float(
        os.getenv(
            "RAK_CAR_INFER_READY_TIMEOUT",
            str(INFER_READY_TIMEOUT),
        )
    )


def get_infer_health_timeout():
    return float(
        os.getenv(
            "RAK_CAR_INFER_HEALTH_TIMEOUT",
            str(INFER_HEALTH_TIMEOUT),
        )
    )


def get_infer_backend_script():
    return os.getenv("RAK_CAR_INFER_BACKEND_SCRIPT", INFER_BACKEND_SCRIPT)


def get_public_api_base():
    return f"http://{get_public_api_host()}:{get_public_api_port()}"


def get_public_stream_base():
    return (
        f"http://{get_public_stream_host()}:{get_public_stream_port()}"
        f"{get_public_stream_path()}"
    )


def get_bind_base():
    return f"http://{get_bind_host()}:{get_bind_port()}"


def get_runtime_settings():
    return {
        "bind_host": get_bind_host(),
        "bind_port": get_bind_port(),
        "public_host": get_public_host(),
        "public_api_base": get_public_api_base(),
        "public_stream_base": get_public_stream_base(),
        "public_stream_path": get_public_stream_path(),
        "api_v1_prefix": get_api_v1_prefix(),
        "legacy_api_prefix": get_legacy_api_prefix(),
        "auto_init_on_start": get_auto_init_on_start(),
        "reset_arm_on_auto_init": get_reset_arm_on_auto_init(),
        "reset_position_on_init": get_reset_position_on_init(),
        "stop_after_action_default": get_stop_after_action_default(),
        "auto_init_retry_interval": get_auto_init_retry_interval(),
        "action_ready_timeout": get_action_ready_timeout(),
        "action_ready_poll_interval": get_action_ready_poll_interval(),
        "auto_download_on_bootloader": get_auto_download_on_bootloader(),
        "infer_auto_start": get_infer_auto_start(),
        "infer_poll_interval": get_infer_poll_interval(),
        "infer_ready_timeout": get_infer_ready_timeout(),
        "infer_health_timeout": get_infer_health_timeout(),
        "infer_backend_script": get_infer_backend_script(),
        "job_history_limit": JOB_HISTORY_LIMIT,
    }

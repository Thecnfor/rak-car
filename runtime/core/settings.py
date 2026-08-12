#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
runtime 服务集中配置

多人协作时，优先改这里，不要到处改脚本里的 IP/端口。
"""
import os


# 对外给局域网同事访问的默认地址
PUBLIC_HOST = "192.168.6.231"
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
# 2026-07-16：默认 True 让 init 调 reset_position（hand=UP + arm=MID + y 触底）。
# 旧版 False 只跑 reset_y，hand/arm 不归 init 位置 → 比赛阶段舵机位置不可控。
# 设为 False（环境变量 RAK_CAR_RESET_ARM=0）跳过 reset_position。
RESET_ARM_ON_AUTO_INIT = True
RESET_POSITION_ON_INIT = True
# 2026-07-27：x 自动撞墙归零只放在真正 init（创建新 car 实例）路径。
# 不再放在 ensure_initialized 的复用路径，避免执行任务 / 健康检查时被自动补 reset_x。
# 默认 True（env RAK_CAR_RESET_X_ON_INIT=0 关掉）。
# reset_x 内部 try/except 兜底（撞墙失败也不阻塞 init 整体）。
RESET_X_ON_INIT = True
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

# 2026-08-01：OOM 韧性相关默认值（详见 .trae/specs/system-arch-optimization/spec.md）
# 推理后端启动时只预热 INFER_EAGER_MODELS（默认 lane）；其余走懒加载。
INFER_EAGER_MODELS_DEFAULT = "lane"
# 闲置超过该秒数且不在 INFER_EAGER_MODELS 内的模型会被后台 tick 自动卸载。
INFER_IDLE_UNLOAD_SECONDS_DEFAULT = 300.0
# 推理进程内单帧推理硬超时：超过返回 [] 而不阻塞后续（防 EFSM 雪崩）。
INFER_FRAME_TIMEOUT_S_DEFAULT = 5.0
# 推理进程 RSS 软限；连续 2 个 probe 周期超限按 OOM_POLICY 卸载。
INFER_RSS_LIMIT_MB_DEFAULT = 1200
# OOM 卸载策略：drop_oldest / drop_ocr / none。
INFER_OOM_POLICY_DEFAULT = "drop_oldest"
# runtime 进程内存压力阈值；超过则 feeds 按 ir→odom→arm→task 顺序降档。
CAR_MEMORY_PRESSURE_MB_DEFAULT = 1500
# runtime 进程 RSS 硬软限；超过只 warn + 上报 /v1/health；95% 主动 gc + drop_oldest。
CAR_RSS_LIMIT_MB_DEFAULT = 1800

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


def get_web_console_enabled():
    """是否挂载 /console/ 工程化控制台静态站（web/ 前端：monitor + teach）。

    默认开启；车端要省内存时用 env RAK_CAR_DISABLE_WEB_CONSOLE=1 关闭
    （不挂 StaticFiles、不注册 /console 路由）。
    """
    return not _bool_env("RAK_CAR_DISABLE_WEB_CONSOLE", False)


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


def get_reset_x_on_init():
    """真正 init 路径是否触发 reset_x 撞墙（env: RAK_CAR_RESET_X_ON_INIT，默认 1）。"""
    return _bool_env("RAK_CAR_RESET_X_ON_INIT", RESET_X_ON_INIT)


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


def get_infer_eager_models():
    """启动时预热的模型名（逗号分隔）。默认仅 lane；其余走懒加载。"""
    raw = os.getenv("RAK_INFER_EAGER_MODELS", INFER_EAGER_MODELS_DEFAULT)
    return [m.strip() for m in raw.split(",") if m.strip()]


def get_infer_idle_unload_seconds():
    return float(
        os.getenv(
            "RAK_INFER_IDLE_UNLOAD_SECONDS",
            str(INFER_IDLE_UNLOAD_SECONDS_DEFAULT),
        )
    )


def get_infer_frame_timeout_s():
    return float(
        os.getenv(
            "RAK_INFER_FRAME_TIMEOUT_S",
            str(INFER_FRAME_TIMEOUT_S_DEFAULT),
        )
    )


def get_infer_rss_limit_mb():
    return int(
        os.getenv(
            "RAK_INFER_RSS_LIMIT_MB",
            str(INFER_RSS_LIMIT_MB_DEFAULT),
        )
    )


def get_infer_oom_policy():
    raw = os.getenv("RAK_INFER_OOM_POLICY", INFER_OOM_POLICY_DEFAULT)
    return str(raw).strip().lower() or INFER_OOM_POLICY_DEFAULT


def get_car_memory_pressure_mb():
    return int(
        os.getenv(
            "RAK_CAR_MEMORY_PRESSURE_MB",
            str(CAR_MEMORY_PRESSURE_MB_DEFAULT),
        )
    )


def get_car_rss_limit_mb():
    return int(
        os.getenv(
            "RAK_CAR_RSS_LIMIT_MB",
            str(CAR_RSS_LIMIT_MB_DEFAULT),
        )
    )


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
        "reset_x_on_init": get_reset_x_on_init(),
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
        "infer_eager_models": get_infer_eager_models(),
        "infer_idle_unload_seconds": get_infer_idle_unload_seconds(),
        "infer_frame_timeout_s": get_infer_frame_timeout_s(),
        "infer_rss_limit_mb": get_infer_rss_limit_mb(),
        "infer_oom_policy": get_infer_oom_policy(),
        "car_memory_pressure_mb": get_car_memory_pressure_mb(),
        "car_rss_limit_mb": get_car_rss_limit_mb(),
        "job_history_limit": JOB_HISTORY_LIMIT,
    }

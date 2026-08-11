#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/chassis/cli/run_lane_cfg.py
仅巡线入口：连 runtime，沿车道中心线前进/后退，配置读仓库根 config_car.yml。

闭环在 **runtime 进程内**（``MyCar.lane_dis_offset`` / ``lane_time`` → ``lane_base``，
每帧零网络往返），复用 ``main/chassis/controllers/move_along_lane``。本脚本只
POST 一次 ``/v1/execute`` 同步等结果。

**config_car.yml 里的巡线配置**：
  - ``lane_pid``：runtime ``MyCar`` 初始化时读（motion_mixin.py:39）构建
    ``self.lane_pid``，巡线闭环（lane_base）就用这套 y/angle 双 PID。
    改它 + 重启 runtime（pm2 restart rak-car-api）即生效，本脚本会打印出来核对。
  - ``speed`` 段是 WhalesBot 模板残留（x=横向/左右），runtime 根本不读，
    不映射到 vx，避免方向错。

用法：
    # 需要 runtime 可达：export RAK_CAR_SERVER_ORIGIN=http://<Jetson IP>:5050
    python3 main/chassis/cli/run_lane_cfg.py --distance 5      # 走 5 米停
    python3 main/chassis/cli/run_lane_cfg.py --seconds 5       # 跑 5 秒停
    python3 main/chassis/cli/run_lane_cfg.py --vx 0.3 --distance 10
    python3 main/chassis/cli/run_lane_cfg.py --dry-run         # 只读配置不下发
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.chassis.controllers.move_along_lane import move_along_lane  # noqa: E402

_DEFAULT_VX = 0.20  # 与 move_along_lane 一致，实车验证过的安全巡航速度


def _load_cfg(path: Optional[Path] = None) -> dict:
    """读 config_car.yml → dict；缺失/解析失败/非 dict → {}（回退默认）。"""
    cfg_path = path or (_REPO_ROOT / "config_car.yml")
    try:
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"⚠ config_car.yml 读取失败: {exc}（回退默认配置）")
        return {}
    return cfg if isinstance(cfg, dict) else {}


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.run_lane_cfg",
        description="仅巡线入口：连 runtime 沿车道中心线前进/后退，配置读 config_car.yml。",
    )
    parser.add_argument(
        "--config", type=str, default=None,
        help="config_car.yml 路径；默认仓库根目录（runtime MyCar 读的就是它）",
    )
    parser.add_argument(
        "--vx", type=float, default=None,
        help="带符号前向速度 (m/s)：正=前进，负=后退。默认 0.20",
    )
    parser.add_argument(
        "--distance", type=float, default=None,
        help="目标行驶距离 (m)。设了以距离为准停（lane_dis_offset）",
    )
    parser.add_argument(
        "--seconds", type=float, default=None,
        help="运行时长 (s)。纯时间模式默认 5.0；--distance 下作兜底",
    )
    parser.add_argument("--dry-run", action="store_true", help="只读配置不下发")
    args = parser.parse_args(argv)

    cfg = _load_cfg(Path(args.config) if args.config else None)
    vx = _DEFAULT_VX if args.vx is None else args.vx

    lane_pid = cfg.get("lane_pid") or {}
    print(f"[run_lane_cfg] vx={vx:+.2f} m/s"
          + ("（--vx 覆盖）" if args.vx is not None else "（默认）"))
    if lane_pid:
        print("  config_car.yml lane_pid（runtime 巡线闭环在用，改它需重启 runtime）:")
        print(f"    cfg_pid_y:     {lane_pid.get('cfg_pid_y')}")
        print(f"    cfg_pid_angle: {lane_pid.get('cfg_pid_angle')}")
    else:
        print("  ⚠ config_car.yml 缺 lane_pid 段")

    if args.distance is None and args.seconds is None:
        args.seconds = 5.0
    move_along_lane(vx=vx, distance_m=args.distance, max_seconds=args.seconds,
                    dry_run=args.dry_run)
    print("[run_lane_cfg] 结束")


if __name__ == "__main__":
    main()

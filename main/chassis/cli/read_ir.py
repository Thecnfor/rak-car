"""main/chassis/cli/read_ir.py
实时读取红外距离传感器，TUI 刷新显示。

用法：
    python3 -m main.chassis.cli.read_ir
    python3 -m main.chassis.cli.read_ir --side left
    python3 -m main.chassis.cli.read_ir --hz 10
"""
from __future__ import annotations

import argparse
import time

from main.chassis import ChassisClient
from main.chassis.tasks.read_ir import read_ir


def _tui_print(header: str, left: float, right: float, width: int = 40) -> None:
    """在同一个位置刷新显示，不滚屏。"""
    bar_l = min(int(left * 10), width)
    bar_r = min(int(right * 10), width)

    lines = [
        f"  IR 距离传感器 — {header}",
        f"  {'─' * (width + 30)}",
        f"  Left  │ {'█' * bar_l}{' ' * (width - bar_l)} │ {left:.3f} m",
        f"  Right │ {'█' * bar_r}{' ' * (width - bar_r)} │ {right:.3f} m",
        f"  {'─' * (width + 30)}",
        "  Ctrl-C 退出",
    ]
    print("\n".join(f"{line}\033[K" for line in lines))


def _tui_print_single(side: str, value: float, width: int = 40) -> None:
    """单侧 TUI 显示。"""
    bar = min(int(value * 10), width)
    label = side.capitalize()

    lines = [
        f"  IR 距离传感器 — {label} only",
        f"  {'─' * (width + 30)}",
        f"  {label:<5} │ {'█' * bar}{' ' * (width - bar)} │ {value:.3f} m",
        f"  {'─' * (width + 30)}",
        " ",
        "  Ctrl-C 退出",
    ]
    print("\n".join(f"{line}\033[K" for line in lines))


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.read_ir",
        description="实时读取 IR 红外距离传感器（TUI 显示）。",
    )
    parser.add_argument(
        "--side",
        choices=["left", "right"],
        default=None,
        help="只读单侧（默认两侧都读）",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="读取频率，默认 10Hz",
    )
    args = parser.parse_args(argv)

    api = ChassisClient.connect()
    dt = 1.0 / max(args.hz, 0.1)

    # 先撑出足够行数避免初次滚屏
    for _ in range(6):
        print()

    lines_printed = 6
    print(f"\033[{lines_printed}A", end="", flush=True)  # 光标上移

    header = f"{args.hz:.0f}Hz  {args.side or 'both'}"
    last_left: float = 0.0
    last_right: float = 0.0

    try:
        while True:
            t0 = time.monotonic()
            try:
                if args.side:
                    val: float = read_ir(api, side=args.side)  # type: ignore[assignment]
                    if args.side == "left":
                        last_left = val
                        last_right = 0.0
                    else:
                        last_right = val
                        last_left = 0.0
                    _tui_print_single(args.side, val)
                else:
                    result = read_ir(api)
                    last_left = float(result.get("left", 0.0))
                    last_right = float(result.get("right", 0.0))
                    _tui_print(header, last_left, last_right)

            except Exception as e:
                _tui_print_err(str(e), last_left, last_right)

            # 光标回到第一行覆盖刷新
            print(f"\033[{lines_printed}A", end="", flush=True)

            elapsed = time.monotonic() - t0
            sleep_s = dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n" * lines_printed + "退出。")
        api.close()


def _tui_print_err(msg: str, left: float, right: float, width: int = 40) -> None:
    """异常时覆盖显示：保留上一帧读数 + 错误提示。"""
    bar_l = min(int(left * 10), width)
    bar_r = min(int(right * 10), width)
    lines = [
        f"  IR 距离传感器 — 读取异常",
        f"  {'─' * (width + 30)}",
        f"  Left  │ {'█' * bar_l}{' ' * (width - bar_l)} │ {left:.3f} m",
        f"  Right │ {'█' * bar_r}{' ' * (width - bar_r)} │ {right:.3f} m",
        f"  {'─' * (width + 30)}",
        f"  错误: {msg[:50]}",
    ]
    print("\n".join(f"{line}\033[K" for line in lines))


if __name__ == "__main__":
    main()

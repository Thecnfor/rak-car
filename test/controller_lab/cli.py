"""统一 CLI 入口。"""

from __future__ import annotations

import argparse
import json

from . import actions
from .bootloader import boot_ping, recover_to_program
from .devices import DeviceCommand
from .downloader import download_file, ensure_bootloader, ping_control
from .probe import probe_controller, probe_port
from .protocol import exchange_program_payload
from .serial_utils import hex_bytes, list_candidate_ports, open_serial


def _print_json(data):
    print(json.dumps(data, ensure_ascii=False, indent=2))


def _require_dangerous(args):
    if not getattr(args, "dangerous", False):
        raise SystemExit("该命令会触发真实硬件动作，请显式加上 --dangerous")


def _resolve_port(args):
    if getattr(args, "port", None):
        return args.port
    probe = probe_controller()
    if probe.get("ready") and probe.get("port"):
        return probe["port"]
    ports = list_candidate_ports()
    if len(ports) == 1:
        return ports[0]["device"]
    raise SystemExit("未指定 --port，且无法唯一推断控制器串口")


def cmd_ports(_args):
    _print_json({"ports": list_candidate_ports()})


def cmd_probe(args):
    if args.port:
        _print_json(probe_port(args.port))
        return
    _print_json(probe_controller())


def cmd_boot_ping(args):
    port = _resolve_port(args)
    with open_serial(port) as serial_obj:
        ok, frame = boot_ping(serial_obj)
    _print_json({"port": port, "ok": ok, "reply_hex": hex_bytes(frame)})


def cmd_recover(args):
    port = _resolve_port(args)
    _print_json(recover_to_program(port))


def cmd_raw(args):
    port = _resolve_port(args)
    payload = bytes.fromhex(args.payload)
    with open_serial(port) as serial_obj:
        reply = exchange_program_payload(serial_obj, payload, timeout_s=args.timeout)
    _print_json({"port": port, "payload_hex": hex_bytes(payload), "reply_hex": hex_bytes(reply)})


def cmd_device_set(args):
    _require_dangerous(args)
    port = _resolve_port(args)
    values = [int(value, 0) for value in args.values]
    with open_serial(port) as serial_obj:
        result = DeviceCommand(args.device, port_id=args.device_port).set(serial_obj, *values)
    _print_json({"port": port, "device": args.device, "result": result})


def cmd_device_get(args):
    port = _resolve_port(args)
    values = [int(value, 0) for value in args.values]
    with open_serial(port) as serial_obj:
        result = DeviceCommand(args.device, port_id=args.device_port).get(serial_obj, *values)
    _print_json({"port": port, "device": args.device, "result": result})


def cmd_beep(args):
    _require_dangerous(args)
    port = _resolve_port(args)
    with open_serial(port) as serial_obj:
        result = actions.beep(serial_obj, freq=args.freq, duration=args.duration)
    _print_json({"port": port, "result": result})


def cmd_chassis(args):
    _require_dangerous(args)
    port = _resolve_port(args)
    speeds = [args.lf, args.rf, args.lr, args.rr]
    with open_serial(port) as serial_obj:
        result = actions.set_motor4(serial_obj, speeds)
    _print_json({"port": port, "speeds": speeds, "result": result})


def cmd_chassis_stop(args):
    _require_dangerous(args)
    port = _resolve_port(args)
    with open_serial(port) as serial_obj:
        result = actions.stop_chassis(serial_obj)
    _print_json({"port": port, "result": result})


def cmd_servo(args):
    _require_dangerous(args)
    port = _resolve_port(args)
    with open_serial(port) as serial_obj:
        if args.servo_type == "pwm":
            result = actions.set_servo_pwm(serial_obj, port=args.device_port, angle=args.angle, speed=args.speed)
        else:
            result = actions.set_servo_bus(serial_obj, port=args.device_port, angle=args.angle, speed=args.speed)
    _print_json({"port": port, "result": result})


def cmd_stepper(args):
    _require_dangerous(args)
    port = _resolve_port(args)
    with open_serial(port) as serial_obj:
        result = actions.set_stepper(serial_obj, port=args.device_port, velocity=args.velocity, position=args.position)
    _print_json({"port": port, "result": result})


def cmd_dout(args):
    _require_dangerous(args)
    port = _resolve_port(args)
    with open_serial(port) as serial_obj:
        result = actions.set_dout(serial_obj, port=args.device_port, value=args.value)
    _print_json({"port": port, "result": result})


def cmd_sensor(args):
    port = _resolve_port(args)
    with open_serial(port) as serial_obj:
        if args.sensor_type == "infrared":
            result = actions.read_infrared(serial_obj, port=args.device_port)
        elif args.sensor_type == "board-key":
            result = actions.read_board_key(serial_obj)
        else:
            result = actions.read_bluetooth_pad(serial_obj)
    _print_json({"port": port, "result": result})


def cmd_download(args):
    _require_dangerous(args)
    if not args.yes_download:
        raise SystemExit("下载/烧录必须显式加上 --yes-download")
    port = _resolve_port(args)
    boot_status = ensure_bootloader(port)
    if not boot_status["ok"]:
        raise SystemExit(json.dumps(boot_status, ensure_ascii=False, indent=2))
    result = download_file(
        port_name=port,
        file_path=args.file,
        slot=args.slot,
        run_after_download=args.run_after_download,
        write_name=not args.skip_name,
    )
    _print_json(result)


def cmd_ping_control(args):
    port = _resolve_port(args)
    _print_json(ping_control(port))


def build_parser():
    common_parser = argparse.ArgumentParser(add_help=False)
    common_parser.add_argument("--port", help="控制器串口，例如 /dev/ttyUSB0")

    parser = argparse.ArgumentParser(
        description="独立下位机通信与操作测试工具",
        parents=[common_parser],
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    ports_parser = subparsers.add_parser("ports", help="列候选串口", parents=[common_parser])
    ports_parser.set_defaults(func=cmd_ports)

    probe_parser = subparsers.add_parser("probe", help="探测 program 模式", parents=[common_parser])
    probe_parser.set_defaults(func=cmd_probe)

    boot_parser = subparsers.add_parser("boot-ping", help="探测 bootloader", parents=[common_parser])
    boot_parser.set_defaults(func=cmd_boot_ping)

    recover_parser = subparsers.add_parser("recover", help="RUNCODE 拉起 program", parents=[common_parser])
    recover_parser.set_defaults(func=cmd_recover)

    raw_parser = subparsers.add_parser("raw", help="发送原始 program payload", parents=[common_parser])
    raw_parser.add_argument("payload", help="十六进制 payload，例如 '02 01 10'")
    raw_parser.add_argument("--timeout", type=float, default=0.2)
    raw_parser.set_defaults(func=cmd_raw)

    dev_set = subparsers.add_parser("device-set", help="通用设备写命令", parents=[common_parser])
    dev_set.add_argument("device")
    dev_set.add_argument("values", nargs="*")
    dev_set.add_argument("--device-port", type=int)
    dev_set.add_argument("--dangerous", action="store_true")
    dev_set.set_defaults(func=cmd_device_set)

    dev_get = subparsers.add_parser("device-get", help="通用设备读命令", parents=[common_parser])
    dev_get.add_argument("device")
    dev_get.add_argument("values", nargs="*")
    dev_get.add_argument("--device-port", type=int)
    dev_get.set_defaults(func=cmd_device_get)

    beep_parser = subparsers.add_parser("beep", help="蜂鸣器测试", parents=[common_parser])
    beep_parser.add_argument("--freq", type=int, default=262)
    beep_parser.add_argument("--duration", type=float, default=0.2)
    beep_parser.add_argument("--dangerous", action="store_true")
    beep_parser.set_defaults(func=cmd_beep)

    chassis_parser = subparsers.add_parser("chassis", help="四轮速度测试", parents=[common_parser])
    chassis_parser.add_argument("--lf", type=int, required=True)
    chassis_parser.add_argument("--rf", type=int, required=True)
    chassis_parser.add_argument("--lr", type=int, required=True)
    chassis_parser.add_argument("--rr", type=int, required=True)
    chassis_parser.add_argument("--dangerous", action="store_true")
    chassis_parser.set_defaults(func=cmd_chassis)

    stop_parser = subparsers.add_parser("chassis-stop", help="底盘急停", parents=[common_parser])
    stop_parser.add_argument("--dangerous", action="store_true")
    stop_parser.set_defaults(func=cmd_chassis_stop)

    servo_parser = subparsers.add_parser("servo", help="舵机测试", parents=[common_parser])
    servo_parser.add_argument("servo_type", choices=["pwm", "bus"])
    servo_parser.add_argument("--device-port", type=int, required=True)
    servo_parser.add_argument("--angle", type=int, required=True)
    servo_parser.add_argument("--speed", type=int, default=80)
    servo_parser.add_argument("--dangerous", action="store_true")
    servo_parser.set_defaults(func=cmd_servo)

    stepper_parser = subparsers.add_parser("stepper", help="步进测试", parents=[common_parser])
    stepper_parser.add_argument("--device-port", type=int, required=True)
    stepper_parser.add_argument("--velocity", type=int, required=True)
    stepper_parser.add_argument("--position", type=int, required=True)
    stepper_parser.add_argument("--dangerous", action="store_true")
    stepper_parser.set_defaults(func=cmd_stepper)

    dout_parser = subparsers.add_parser("dout", help="数字输出测试", parents=[common_parser])
    dout_parser.add_argument("--device-port", type=int, required=True)
    dout_parser.add_argument("--value", type=int, required=True)
    dout_parser.add_argument("--dangerous", action="store_true")
    dout_parser.set_defaults(func=cmd_dout)

    sensor_parser = subparsers.add_parser("sensor", help="读取传感器", parents=[common_parser])
    sensor_parser.add_argument("sensor_type", choices=["infrared", "board-key", "bluetooth"])
    sensor_parser.add_argument("--device-port", type=int)
    sensor_parser.set_defaults(func=cmd_sensor)

    download_parser = subparsers.add_parser("download", help="下载/烧录", parents=[common_parser])
    download_parser.add_argument("--file", required=True)
    download_parser.add_argument("--slot", default="RunA", choices=["RunA", "RunB", "RunC", "RunD", "RunE", "RunF"])
    download_parser.add_argument("--run-after-download", action="store_true")
    download_parser.add_argument("--skip-name", action="store_true")
    download_parser.add_argument("--yes-download", action="store_true")
    download_parser.add_argument("--dangerous", action="store_true")
    download_parser.set_defaults(func=cmd_download)

    ping_parser = subparsers.add_parser("ping-control", help="bootloader PING 控制器", parents=[common_parser])
    ping_parser.set_defaults(func=cmd_ping_control)

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    args.func(args)

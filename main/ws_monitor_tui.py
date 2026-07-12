#!/usr/bin/python3
# -*- coding: utf-8 -*-
import curses
import json
import os
import sys
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from main.ws_client import RuntimeWsClient


class MonitorState:
    def __init__(self):
        self.last_error = None
        self.last_health = None
        self.last_runtime = None
        self.last_battery = None
        self.last_ir = None
        self.last_arm = None
        self.last_update_at = None
        self.linear_speed = 0.10
        self.angular_speed = 0.80
        self.pulse_duration = 0.18
        self.last_command = "无"
        self.last_command_at = None
        self.drive_mode = "continuous"
        self.active_motion = None
        self.last_motion_input_at = None
        self.last_motion_send_at = 0.0
        self.motion_send_interval = 0.08
        self.motion_hold_timeout = 0.28
        self.motion_stop_sent = True


def _fmt_json_line(value):
    if value is None:
        return "-"
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def _format_float(value, digits=3, suffix=""):
    if value is None:
        return "-"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except Exception:
        return str(value)


def _extract_job_result(response, default="-"):
    data = (response or {}).get("data") or {}
    job = data.get("job") or {}
    if job.get("status") == "succeeded":
        return job.get("result", default)
    return default


def _draw_line(stdscr, row, text, width, attr=0):
    height, real_width = stdscr.getmaxyx()
    if row < 0 or row >= height:
        return
    usable_width = min(width, real_width)
    if usable_width <= 1:
        return
    # curses 在右下角落笔很容易报 ERR，这里永远少写 1 列。
    max_chars = usable_width - 1
    content = str(text).replace("\n", " ")
    if len(content) > max_chars:
        if max_chars >= 2:
            content = content[: max_chars - 1] + "…"
        else:
            content = content[:max_chars]
    try:
        stdscr.addnstr(row, 0, content.ljust(max_chars), max_chars, attr)
    except curses.error:
        pass


def _draw_lines(stdscr, start_row, lines, width, title=None):
    row = start_row
    if title:
        _draw_line(stdscr, row, title, width, curses.A_BOLD)
        row += 1
    for line in lines:
        _draw_line(stdscr, row, line, width)
        row += 1
    return row


def poll_once(client, state):
    health = client.health(timeout=3.0)
    state.last_health = health
    state.last_error = None
    state.last_update_at = time.time()
    health_data = health.get("data", {})
    status = health_data.get("state", {})
    if status.get("initialized"):
        state.last_runtime = client.runtime(timeout=3.0)
        state.last_battery = client.execute(
            "car", "get_battery_voltage", timeout=3.0
        )
        state.last_ir = client.execute(
            "car", "get_all_ir_distance", timeout=3.0
        )
        state.last_arm = client.execute(
            "car", "get_arm_state", timeout=3.0
        )


def send_drive_command(client, state, x=0.0, y=0.0, z=0.0, label="移动", duration=None):
    duration = state.pulse_duration if duration is None else float(duration)
    client.execute(
        "car",
        "set_chassis_velocity",
        timeout=2.0,
        kwargs={
            "x": round(float(x), 4),
            "y": round(float(y), 4),
            "z": round(float(z), 4),
            "duration": duration,
        },
    )
    state.last_command = (
        f"{label} x={_format_float(x)} y={_format_float(y)} "
        f"z={_format_float(z)} dur={_format_float(duration, 2, 's')}"
    )
    state.last_command_at = time.time()


def set_active_motion(state, x, y, z, label):
    state.active_motion = {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "label": label,
    }
    state.last_motion_input_at = time.time()
    state.motion_stop_sent = False


def stop_active_motion(client, state, reason="停止"):
    state.active_motion = None
    state.last_motion_input_at = None
    if not state.motion_stop_sent:
        send_drive_command(client, state, 0.0, 0.0, 0.0, label=reason, duration=0.10)
    else:
        state.last_command = reason
        state.last_command_at = time.time()
    state.motion_stop_sent = True


def update_continuous_motion(client, state):
    if state.drive_mode != "continuous":
        return
    now = time.time()
    motion = state.active_motion
    if motion is None:
        return
    if (
        state.last_motion_input_at is not None
        and now - state.last_motion_input_at <= state.motion_hold_timeout
    ):
        if now - state.last_motion_send_at >= state.motion_send_interval:
            send_drive_command(
                client,
                state,
                motion["x"],
                motion["y"],
                motion["z"],
                label=f"{motion['label']}[连动]",
                duration=max(state.pulse_duration * 1.8, 0.22),
            )
            state.last_motion_send_at = now
    else:
        stop_active_motion(client, state, reason="连动停止")


def handle_key(client, state, key):
    if key in (ord("+"), ord("=")):
        state.linear_speed = min(0.60, state.linear_speed + 0.02)
        state.last_command = f"线速度调高到 {state.linear_speed:.2f} m/s"
        return
    if key in (ord("-"), ord("_")):
        state.linear_speed = max(0.02, state.linear_speed - 0.02)
        state.last_command = f"线速度调低到 {state.linear_speed:.2f} m/s"
        return
    if key == ord("]"):
        state.angular_speed = min(3.00, state.angular_speed + 0.10)
        state.last_command = f"角速度调高到 {state.angular_speed:.2f} rad/s"
        return
    if key == ord("["):
        state.angular_speed = max(0.20, state.angular_speed - 0.10)
        state.last_command = f"角速度调低到 {state.angular_speed:.2f} rad/s"
        return
    if key == ord(">"):
        state.pulse_duration = min(0.60, state.pulse_duration + 0.02)
        state.last_command = f"脉冲时长调高到 {state.pulse_duration:.2f} s"
        return
    if key == ord("<"):
        state.pulse_duration = max(0.06, state.pulse_duration - 0.02)
        state.last_command = f"脉冲时长调低到 {state.pulse_duration:.2f} s"
        return
    if key in (ord("m"), ord("M")):
        state.drive_mode = (
            "pulse" if state.drive_mode == "continuous" else "continuous"
        )
        state.last_command = (
            "已切到连动模式" if state.drive_mode == "continuous" else "已切到点动模式"
        )
        state.last_command_at = time.time()
        state.active_motion = None
        state.motion_stop_sent = True
        return

    move_map = {
        curses.KEY_UP: (state.linear_speed, 0.0, 0.0, "前进"),
        ord("w"): (state.linear_speed, 0.0, 0.0, "前进"),
        ord("W"): (state.linear_speed, 0.0, 0.0, "前进"),
        curses.KEY_DOWN: (-state.linear_speed, 0.0, 0.0, "后退"),
        ord("s"): (-state.linear_speed, 0.0, 0.0, "后退"),
        ord("S"): (-state.linear_speed, 0.0, 0.0, "后退"),
        curses.KEY_LEFT: (0.0, state.linear_speed, 0.0, "左移"),
        ord("a"): (0.0, state.linear_speed, 0.0, "左移"),
        ord("A"): (0.0, state.linear_speed, 0.0, "左移"),
        curses.KEY_RIGHT: (0.0, -state.linear_speed, 0.0, "右移"),
        ord("d"): (0.0, -state.linear_speed, 0.0, "右移"),
        ord("D"): (0.0, -state.linear_speed, 0.0, "右移"),
        ord("j"): (0.0, 0.0, state.angular_speed, "左转"),
        ord("J"): (0.0, 0.0, state.angular_speed, "左转"),
        ord("k"): (0.0, 0.0, -state.angular_speed, "右转"),
        ord("K"): (0.0, 0.0, -state.angular_speed, "右转"),
        ord(" "): (0.0, 0.0, 0.0, "急停"),
        ord("x"): (0.0, 0.0, 0.0, "急停"),
        ord("X"): (0.0, 0.0, 0.0, "急停"),
    }
    if key in move_map:
        x, y, z, label = move_map[key]
        if label == "急停":
            stop_active_motion(client, state, reason="急停")
            return
        if state.drive_mode == "continuous":
            set_active_motion(state, x, y, z, label)
            state.last_command = f"{label}[连动待发]"
            state.last_command_at = time.time()
            return
        send_drive_command(client, state, x=x, y=y, z=z, label=label)


def draw(stdscr, client, state):
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    welcome = client.welcome or {}
    health_data = (state.last_health or {}).get("data", {})
    runtime_data = (state.last_runtime or {}).get("data", {})
    state_data = health_data.get("state", {})
    controller = state_data.get("controller_session", {})
    runtime_snapshot = runtime_data.get("runtime") or {}
    battery = _extract_job_result(state.last_battery, default=None)
    ir = _extract_job_result(state.last_ir, default={})
    arm = _extract_job_result(state.last_arm, default={})
    odometry = runtime_snapshot.get("odometry") or ["-", "-", "-"]
    distance = runtime_snapshot.get("distance")

    row = 0
    row = _draw_lines(
        stdscr,
        row,
        [
            f"RAK-CAR WS MONITOR    {'已连接' if welcome else '未连接'}    "
            f"刷新 {time.strftime('%H:%M:%S', time.localtime(state.last_update_at)) if state.last_update_at else '-'}",
            f"地址: {client.ws_url}",
        ],
        width,
    )
    row += 1
    row = _draw_lines(
        stdscr,
        row,
        [
            f"初始化: {state_data.get('initialized')}    初始化中: {state_data.get('initializing')}    当前任务: {state_data.get('current_job_id') or '-'}",
            f"控制器: {controller.get('state') or '-'}    代际: {controller.get('generation') or '-'}    失败计数: {controller.get('failure_count') or 0}",
            f"控制器详情: {controller.get('detail') or '-'}",
            f"停止标志: {state_data.get('stop_flag')}    队列: {state_data.get('queued_jobs')}    推流: {state_data.get('streamer_url') or '-'}",
        ],
        width,
        title="状态",
    )
    row += 1
    row = _draw_lines(
        stdscr,
        row,
        [
            f"电池: {_format_float(battery, 2, ' V')}",
            f"IR: 左 {_format_float((ir or {}).get('left'), 3, ' m')}    右 {_format_float((ir or {}).get('right'), 3, ' m')}",
            f"机械臂: x {_format_float((arm or {}).get('x'), 3, ' m')}    y {_format_float((arm or {}).get('y'), 3, ' m')}    side {(arm or {}).get('side', '-')}",
            f"机械臂角度: arm {(arm or {}).get('arm_angle', '-')}    hand {(arm or {}).get('hand_angle', '-')}    y_limit {(arm or {}).get('y_limit', '-')}",
            f"位姿: x {_format_float(odometry[0], 3)}    y {_format_float(odometry[1], 3)}    theta {_format_float(odometry[2], 3)}",
            f"距离: {_format_float(distance, 3, ' m')}",
        ],
        width,
        title="传感器",
    )
    row += 1
    compact = height < 20
    control_lines = [
        f"模式 M: {'连动' if state.drive_mode == 'continuous' else '点动'}    线速度 +/-: {_format_float(state.linear_speed, 2, ' m/s')}    角速度 [/]: {_format_float(state.angular_speed, 2, ' rad/s')}",
        f"脉冲 </>: {_format_float(state.pulse_duration, 2, ' s')}    连动刷新: {_format_float(state.motion_send_interval, 2, ' s')}    松手超时: {_format_float(state.motion_hold_timeout, 2, ' s')}",
        "移动: ↑/W 前进  ↓/S 后退  ←/A 左移  →/D 右移  J 左转  K 右转  Space/X 急停",
        "系统: R 刷新  C 重连  Q 退出",
        f"最近命令: {state.last_command}",
    ]
    if not compact:
        control_lines.append(
            f"欢迎包: {_fmt_json_line({'links': welcome.get('links'), 'usage': welcome.get('usage')}) if welcome else '-'}"
        )
    row = _draw_lines(stdscr, row, control_lines, width, title="控制")
    row += 1
    _draw_lines(
        stdscr,
        row,
        [str(state.last_error or "-")],
        width,
        title="最近错误",
    )
    stdscr.refresh()


def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(60)
    stdscr.keypad(True)

    client = RuntimeWsClient()
    state = MonitorState()
    next_poll = 0.0

    while True:
        key = stdscr.getch()
        if key == ord("q"):
            break
        if key == ord("c"):
            client.close()
            state.last_command = "已手动重连"
        if key == ord("r"):
            next_poll = 0.0
            state.last_command = "已手动刷新"
        if key not in (-1, ord("q"), ord("c"), ord("r")):
            try:
                handle_key(client, state, key)
            except Exception as exc:
                state.last_error = f"控制失败: {exc}"
                client.close()

        try:
            update_continuous_motion(client, state)
        except Exception as exc:
            state.last_error = f"连动失败: {exc}"
            client.close()
            state.active_motion = None
            state.motion_stop_sent = True

        if time.time() >= next_poll:
            try:
                poll_once(client, state)
            except Exception as exc:
                state.last_error = str(exc)
                client.close()
            next_poll = time.time() + 0.8

        draw(stdscr, client, state)


if __name__ == "__main__":
    curses.wrapper(main)

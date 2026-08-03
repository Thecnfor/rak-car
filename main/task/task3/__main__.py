#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/task333/__main__.py

任务3 搜索-居中工具:底盘 + 机械臂 联动,把害虫 animal 目标送到侧摄 cam2
视野中心 (bbox_norm 中心 ≈ (0.5, 0.5)),并记录对齐时的机械臂位置
(y/x/ref_encoder),落盘到 audit 目录 + 打印。

== 控制策略(自上而下三层) ==
1. SCAN 扫描:无检测时底盘小幅自转 / arm 拉距离,在 0~SCAN_RADIUS 范围内找目标。
2. ALIGN 居中:有检测时,bbox 中心 (xc, yc) 偏离 (0.5, 0.5):
   - 横向偏差 |dx| = |xc - 0.5| → 优先用 arm.move_y_position 推(机械臂 y 改变 = 视野横移);
     偏大时叠加底盘小角度自转 car.move_for([0,0,±Δθ])。
   - 纵向偏差 |dy| = |yc - 0.5| → arm.move_x_position 推(机械臂 x 改变 = 视野纵移)。
   步长按偏差大小缩放,逐步逼近,达到 tolerance 视为对齐。
3. RECORD 记录:对齐后立刻读 /v1/realtime/arm/state 拿 y_mm/x_mm/ref_encoder,
   写入 main/tasks/task333/audit/found.json + 打印。

== 硬件动作 ==
底盘 car.move_for(直走/横移/转角),机械臂 arm.move_y_position / arm.move_x_position。
全部走「异步提交 + 轮询 job」,规避 192.168.6.231 网关的 504 timeout。

== 约束 ==
- 不修改 runtime/、smartcar/、config_car.yml、main/misc/*、main/arm/*、main/chassis/*。
- 不调 emergency_stop / reset_stop_flag(避免误清 stop 状态)。
- 机械臂/底盘都只在本脚本内通过 HTTP action 调用。

== 跑前 ==
- runtime 已起,/v1/infer/state 里 task 模型 ready=true。
- 侧摄 cam2 视野里有"动物/害虫"目标(打印图/卡片均可)。
- 机械臂已上电,arm_feed 守护线程正常(arm_state.active=true)。

== 跑法 ==
    python -m main.tasks.task333                                    # 默认参数
    python -m main.tasks.task333 --tolerance 0.04 --max-scan-steps 12
    python -m main.tasks.task333 --dry-run                          # 只看检测/计算步长,不动硬件
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Optional

from main.api_client import RuntimeApiClient
from main.settings import load_settings


# --- 默认参数 ---
DEFAULT_TOLERANCE = 0.06                # bbox 中心到 (0.5, 0.5) 的可接受半径(归一化)
DEFAULT_MAX_SCAN_STEPS = 10             # 扫描阶段最大底盘自转步数
DEFAULT_MAX_ALIGN_ITERS = 30            # 居中阶段最大迭代次数
DEFAULT_SCAN_DZ = 0.20                  # 扫描阶段每次底盘自转弧度(≈11.5°)
DEFAULT_SCAN_ARMY_STEP = 0.005          # 扫描阶段 arm.y 拉距离步长(米)
DEFAULT_ARM_X_CENTER = 0.30             # arm.x 目标居中值(米,给到视野中央目标常用)
DEFAULT_ARM_Y_CENTER = 0.0              # arm.y 目标居中值(米)
DEFAULT_ALIGN_DZ = 0.05                 # 居中阶段底盘小角度步长(弧度,≈2.9°)
DEFAULT_ALIGN_ARMY_STEP = 0.02          # 居中阶段 arm.y 步长上限(米)
DEFAULT_ALIGN_ARMX_STEP = 0.015         # 居中阶段 arm.x 步长上限(米)
DEFAULT_DETECT_TIMEOUT = 5.0
DEFAULT_JOB_TIMEOUT = 30.0
DEFAULT_ARM_MOVE_TIMEOUT = 15.0
DEFAULT_AUDIT_DIR = "audit/"

CENTER_X = 0.0                         # task_feed bbox_norm 是 [-1, 1] 中心化坐标,中心 = (0, 0)
CENTER_Y = 0.0
TARGET_LABEL = "animal"                 # 任务检测里害虫的 label


def _resolve_audit_dir(save_dir: str) -> Path:
    """相对路径按本包(main/tasks/task333/)解析,绝对路径直接用。"""
    p = Path(save_dir)
    if p.is_absolute():
        return p
    return Path(__file__).resolve().parent / save_dir


# ----------------------------------------------------------------------
# 异步 car 动作(规避网关 504)
# ----------------------------------------------------------------------
def wait_car(client, name, *a, timeout=None, **k):
    job = client.execute_car_action(name, *a, timeout=timeout, sync=False, **k)
    done = client.wait_job(job["id"], timeout=(timeout or 60.0) + 10.0)
    if done.get("status") != "succeeded":
        raise RuntimeError(
            f"car.{name} 失败: status={done.get('status')} error={done.get('error')}"
        )
    return done.get("result")


def wait_arm(client, name, *a, timeout=None, **k):
    job = client.execute_arm_action(name, *a, timeout=timeout, sync=False, **k)
    done = client.wait_job(job["id"], timeout=(timeout or 60.0) + 10.0)
    if done.get("status") != "succeeded":
        raise RuntimeError(
            f"arm.{name} 失败: status={done.get('status')} error={done.get('error')}"
        )
    return done.get("result")


def try_car(client, name, *a, timeout=None, **k):
    try:
        return wait_car(client, name, *a, timeout=timeout, **k)
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] car.{name} 失败: {exc}", file=sys.stderr)
        return None


def try_arm(client, name, *a, timeout=None, **k):
    try:
        return wait_arm(client, name, *a, timeout=timeout, **k)
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] arm.{name} 失败: {exc}", file=sys.stderr)
        return None


# ----------------------------------------------------------------------
# 检测/状态读取
# ----------------------------------------------------------------------
def read_detections(client) -> list[dict]:
    """读 task_feed 守护线程缓存,返回 list[dict](原样,方便访问 bbox_norm.x_center 等)。"""
    try:
        resp = client.get_task_state()
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] get_task_state 异常: {exc}", file=sys.stderr)
        return []
    ts = (resp or {}).get("task_state") or {}
    return list(ts.get("detections") or [])


def pick_target(detections: list[dict], label: str = TARGET_LABEL) -> Optional[dict]:
    """挑一个最高分的指定 label 目标(没有就返回 None)。"""
    best = None
    best_score = -1.0
    for d in detections:
        if d.get("label") != label:
            continue
        sc = d.get("score") or 0.0
        if sc > best_score:
            best_score = sc
            best = d
    return best


def read_arm_state(client) -> dict:
    try:
        resp = client.get_arm_state()
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] get_arm_state 异常: {exc}", file=sys.stderr)
        return {}
    return (resp or {}).get("arm_state") or {}


# ----------------------------------------------------------------------
# 步长计算
# ----------------------------------------------------------------------
def step_for_offset(off: float, step_max: float, k: float = 4.0) -> float:
    """按偏差 off 算步长:偏差大步长接近 step_max,偏差小步长按比例缩小。
    k 是灵敏度(每 0.1 偏差对应多少 step_max 比例)。
    """
    if step_max <= 0:
        return 0.0
    mag = min(abs(off) * k, 1.0)  # 0~1
    return float(step_max) * mag * (1.0 if off >= 0 else -1.0)


# ----------------------------------------------------------------------
# 扫描阶段:无目标时,底盘小幅自转扫描
# ----------------------------------------------------------------------
def scan_for_target(
    client: RuntimeApiClient,
    args: argparse.Namespace,
) -> bool:
    """底盘自转扫描,直到找到目标或步数用完。返回是否找到。"""
    print("[scan] 阶段开始,底盘自转扫描 + 逐步拉近 arm.x")
    for step_i in range(1, args.max_scan_steps + 1):
        # 底盘小幅自转(左右交替)
        dz = args.scan_dz if step_i % 2 == 1 else -args.scan_dz
        print(f"[scan {step_i}/{args.max_scan_steps}] 底盘自转 {dz:+.2f} rad")
        if not args.dry_run:
            try_car(client, "move_for", [0.0, 0.0, float(dz)], timeout=args.job_timeout)
        time.sleep(0.3)
        dets = read_detections(client)
        if pick_target(dets):
            print(f"[scan {step_i}] [OK] found target, entering align")
            return True
        # 同时拉一下 arm.x(扫描不同距离)
        try:
            cur_arm = read_arm_state(client)
            cur_x = cur_arm.get("x_m") or 0.0
        except Exception:  # noqa: BLE001
            cur_x = 0.0
        new_x = max(0.10, min(0.45, cur_x + args.scan_armx_step))
        if not args.dry_run:
            try_arm(client, "move_x_position", float(new_x), timeout=args.arm_move_timeout)
    print("[scan] [FAIL] scan steps exhausted, target not found")
    return False


# ----------------------------------------------------------------------
# 对齐阶段:小步逼近,直到 bbox 中心到 (0.5, 0.5) 在 tolerance 内
# ----------------------------------------------------------------------
def align_to_center(
    client: RuntimeApiClient,
    args: argparse.Namespace,
) -> tuple[Optional[dict], list[dict]]:
    """迭代居中。返回 (对齐时的 target dict, history)。
    history 每个元素记录一步: {iter, xc, yc, dx, dy, action, arm_after}。
    """
    history: list[dict] = []
    target: Optional[dict] = None

    for it in range(1, args.max_align_iters + 1):
        dets = read_detections(client)
        target = pick_target(dets)
        if target is None:
            print(f"[align {it}] [FAIL] target lost, abort")
            history.append({"iter": it, "lost": True})
            return None, history

        bbox = target.get("bbox_norm") or {}
        xc = float(bbox.get("x_center", 0.5))
        yc = float(bbox.get("y_center", 0.5))
        dx = xc - CENTER_X
        dy = yc - CENTER_Y
        dist = (dx * dx + dy * dy) ** 0.5
        print(f"[align {it}] xc={xc:.3f} yc={yc:.3f} dx={dx:+.3f} dy={dy:+.3f} dist={dist:.3f}")

        rec: dict[str, Any] = {"iter": it, "xc": xc, "yc": yc, "dx": dx, "dy": dy, "dist": dist}

        if dist <= args.tolerance:
            print(f"[align {it}] [OK] aligned (dist={dist:.3f} <= tol={args.tolerance})")
            rec["action"] = "stop-aligned"
            history.append(rec)
            return target, history

        # ---- 算步长并执行 ----
        # 横向偏差 dx 优先用 arm.y(更精细),偏大叠加底盘小角度
        action_log = []
        cur_arm = read_arm_state(client)
        cur_ax = cur_arm.get("x_m")
        cur_ay = cur_arm.get("y_m")

        # 1) arm.y 推横向
        army_step = step_for_offset(dx, args.align_army_step, k=4.0)
        if abs(army_step) > 1e-4 and cur_ay is not None:
            new_ay = cur_ay + army_step
            print(f"[align {it}]   arm.y {cur_ay:+.4f} → {new_ay:+.4f} (step {army_step:+.4f})")
            if not args.dry_run:
                try_arm(client, "move_y_position", float(new_ay), timeout=args.arm_move_timeout)
            action_log.append(f"arm.y {cur_ay:+.4f}->{new_ay:+.4f}")

        # 2) 偏大时叠加底盘小角度
        if abs(dx) > 0.15:
            dz = args.align_dz if dx > 0 else -args.align_dz
            print(f"[align {it}]   chassis z {dz:+.2f} rad (dx={dx:+.3f} 偏大叠加)")
            if not args.dry_run:
                try_car(client, "move_for", [0.0, 0.0, float(dz)], timeout=args.job_timeout)
            action_log.append(f"chassis dz {dz:+.2f}")

        # 3) arm.x 推纵向(yc 越小越靠上,所以 dy>0 表示目标在画面下方 → arm.x 加大往前伸)
        armx_step = step_for_offset(-dy, args.align_armx_step, k=4.0)  # 符号相反
        if abs(armx_step) > 1e-4 and cur_ax is not None:
            new_ax = cur_ax + armx_step
            print(f"[align {it}]   arm.x {cur_ax:+.4f} → {new_ax:+.4f} (step {armx_step:+.4f})")
            if not args.dry_run:
                try_arm(client, "move_x_position", float(new_ax), timeout=args.arm_move_timeout)
            action_log.append(f"arm.x {cur_ax:+.4f}->{new_ax:+.4f}")

        rec["action"] = "; ".join(action_log) if action_log else "noop"
        rec["arm_after"] = read_arm_state(client)
        history.append(rec)

        time.sleep(0.25)

    print(f"[align] [FAIL] max iters {args.max_align_iters} reached, not within tolerance={args.tolerance}")
    return target, history


# ----------------------------------------------------------------------
# 记录阶段
# ----------------------------------------------------------------------
def record_found(
    target: dict,
    arm_state: dict,
    history: list[dict],
    args: argparse.Namespace,
) -> None:
    audit_dir = _resolve_audit_dir(args.save_dir)
    audit_dir.mkdir(parents=True, exist_ok=True)
    out_path = audit_dir / "found.json"

    payload = {
        "timestamp": time.time(),
        "target": target,
        "arm_state_at_found": arm_state,
        "tolerance": args.tolerance,
        "history": history,
        "config": {
            "scan_dz": args.scan_dz,
            "scan_armx_step": args.scan_armx_step,
            "align_dz": args.align_dz,
            "align_army_step": args.align_army_step,
            "align_armx_step": args.align_armx_step,
        },
    }
    out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n========== FOUND ==========")
    print(f"  label        : {target.get('label')}")
    print(f"  score        : {target.get('score')}")
    bbox = target.get("bbox_norm") or {}
    print(f"  bbox center  : xc={bbox.get('x_center'):.3f}  yc={bbox.get('y_center'):.3f}  (centered coords, target is (0,0))")
    print(f"  arm y_mm     : {arm_state.get('y_mm')}")
    print(f"  arm x_mm     : {arm_state.get('x_mm')}")
    print(f"  arm y_m      : {arm_state.get('y_m')}")
    print(f"  arm x_m      : {arm_state.get('x_m')}")
    print(f"  ref_encoder  : {arm_state.get('ref_encoder')}")
    print(f"  arm active   : {arm_state.get('active')}")
    print(f"  history iters: {len(history)}")
    print(f"  audit        : {out_path}")
    print("============================\n")


# ----------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------
def run() -> None:
    parser = argparse.ArgumentParser(
        description="任务3 搜索-居中:底盘+机械臂联动,把 animal 送到 cam2 视野中心,记录 arm 位置",
    )
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                        help=f"bbox 中心到 (0.5,0.5) 的可接受半径(默认 {DEFAULT_TOLERANCE})")
    parser.add_argument("--max-scan-steps", type=int, default=DEFAULT_MAX_SCAN_STEPS, dest="max_scan_steps",
                        help=f"扫描阶段最大底盘自转步数(默认 {DEFAULT_MAX_SCAN_STEPS})")
    parser.add_argument("--max-align-iters", type=int, default=DEFAULT_MAX_ALIGN_ITERS, dest="max_align_iters",
                        help=f"居中阶段最大迭代次数(默认 {DEFAULT_MAX_ALIGN_ITERS})")
    parser.add_argument("--scan-dz", type=float, default=DEFAULT_SCAN_DZ, dest="scan_dz",
                        help=f"扫描底盘自转弧度(默认 {DEFAULT_SCAN_DZ})")
    parser.add_argument("--scan-armx-step", type=float, default=0.015, dest="scan_armx_step",
                        help="扫描阶段 arm.x 步长(米,默认 0.015)")
    parser.add_argument("--align-dz", type=float, default=DEFAULT_ALIGN_DZ, dest="align_dz",
                        help=f"居中底盘小角度弧度(默认 {DEFAULT_ALIGN_DZ})")
    parser.add_argument("--align-army-step", type=float, default=DEFAULT_ALIGN_ARMY_STEP, dest="align_army_step",
                        help=f"居中 arm.y 步长上限(默认 {DEFAULT_ALIGN_ARMY_STEP})")
    parser.add_argument("--align-armx-step", type=float, default=DEFAULT_ALIGN_ARMX_STEP, dest="align_armx_step",
                        help=f"居中 arm.x 步长上限(默认 {DEFAULT_ALIGN_ARMX_STEP})")
    parser.add_argument("--target-label", type=str, default=TARGET_LABEL, dest="target_label",
                        help=f"目标 label(默认 {TARGET_LABEL})")
    parser.add_argument("--save-dir", type=str, default=DEFAULT_AUDIT_DIR, dest="save_dir",
                        help=f"审计目录(默认 {DEFAULT_AUDIT_DIR})")
    parser.add_argument("--job-timeout", type=float, default=DEFAULT_JOB_TIMEOUT, dest="job_timeout",
                        help=f"底盘 job 超时秒数(默认 {DEFAULT_JOB_TIMEOUT})")
    parser.add_argument("--arm-move-timeout", type=float, default=DEFAULT_ARM_MOVE_TIMEOUT, dest="arm_move_timeout",
                        help=f"机械臂 job 超时秒数(默认 {DEFAULT_ARM_MOVE_TIMEOUT})")
    parser.add_argument("--detect-timeout", type=float, default=DEFAULT_DETECT_TIMEOUT, dest="detect_timeout",
                        help="read_detection 超时秒数(默认 5.0)")
    parser.add_argument("--dry-run", action="store_true", help="只计算/打印步长,不真移动硬件")
    args = parser.parse_args()

    if not 0 < args.tolerance < 1:
        parser.error("--tolerance 必须在 (0, 1)")
    if args.max_scan_steps < 1:
        parser.error("--max-scan-steps 必须 >= 1")
    if args.max_align_iters < 1:
        parser.error("--max-align-iters 必须 >= 1")

    settings = load_settings()
    print(f"API_BASE = {settings.api_base}")
    print(
        f"[ready] tolerance={args.tolerance} max_scan_steps={args.max_scan_steps} "
        f"max_align_iters={args.max_align_iters} target={args.target_label} dry_run={args.dry_run}"
    )

    client = RuntimeApiClient(settings=settings)
    client.wait_until_ready()

    # 起始:看一眼 arm 状态
    init_arm = read_arm_state(client)
    print(
        f"[init] arm y_m={init_arm.get('y_m')} x_m={init_arm.get('x_m')} "
        f"ref_encoder={init_arm.get('ref_encoder')} active={init_arm.get('active')}"
    )

    # 起始:看检测
    dets0 = read_detections(client)
    t0 = pick_target(dets0, args.target_label)
    if t0 is not None:
        bbox0 = t0.get("bbox_norm") or {}
        print(
            f"[init] 已检测到 {args.target_label}: xc={bbox0.get('x_center'):.3f} "
            f"yc={bbox0.get('y_center'):.3f} score={t0.get('score')}"
        )
    else:
        print(f"[init] 视野内暂无 {args.target_label},进入扫描")

    try:
        # 1) 如果没目标,扫描
        target = t0
        if target is None:
            if not scan_for_target(client, args):
                print("[abort] scan did not find target, exit")
                return
            target = pick_target(read_detections(client), args.target_label)
            if target is None:
                print("[abort] target lost after scan, exit")
                return

        # 2) 居中迭代
        target, history = align_to_center(client, args)

        # 3) 记录
        final_arm = read_arm_state(client)
        if target is not None:
            record_found(target, final_arm, history, args)
        else:
            print("[abort] target lost during align, cannot record")
            # 仍把 history 落盘,方便调试
            audit_dir = _resolve_audit_dir(args.save_dir)
            audit_dir.mkdir(parents=True, exist_ok=True)
            (audit_dir / "lost.json").write_text(
                json.dumps({"history": history, "arm_state": final_arm}, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

    except KeyboardInterrupt:
        print("\n[abort] KeyboardInterrupt")
    finally:
        # 收尾:停底盘(收尾动作都是"尽力而为",不抛)
        try_car(client, "stop", timeout=args.job_timeout)


if __name__ == "__main__":
    run()

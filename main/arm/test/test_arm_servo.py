#!/usr/bin/python3
"""test_arm_servo.py
机械臂 大臂(bus 舵机,port=2)测试。

测试项:set_side("LEFT" / "MID" / "RIGHT") 后读回 arm_angle
预期角度(arm_cfg.yaml:hand_cfg.hand.angle_list):
  LEFT  =  93
  MID   =   0
  RIGHT = -93

⚠️ 真实硬件,大臂会真的转动。先确认机械臂周围无障碍物。

运行:
  export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
  python3 main/arm/test/test_arm_servo.py
"""
import os
import sys
import time

# 把项目根目录(rak-car/)加到 sys.path,这样才能 import main.*
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.test._runtime_guard import preflight, postflight  # noqa: E402


# 期望角度表(与 arm_cfg.yaml:hand_cfg.hand.angle_list 对齐)
EXPECTED_ANGLE = {
    "LEFT": 93,
    "MID": 0,
    "RIGHT": -93,
}

# 测试序列:从当前方向出发 -> 走完全部 3 个目标 -> 回到 MID
SEQUENCE = ["LEFT", "MID", "RIGHT", "MID"]

# runtime 重建期间等待稳定的时长
WAIT_STABLE_S = 8.0
# 单次 set_side 后等舵机物理到位
SETTLE_S = 0.6
# 读回 side 字段的最多重试次数(auto-init 重建会重置 MyCar.side=MID,需要重读)
RETRY_READ = 4
RETRY_DELAY_S = 0.4

# 大臂总线舵机端口(arm_cfg.yaml:hand_cfg.hand.port=2)
ARM_BUS_PORT = 2


def read_bus_physical_angle(client) -> tuple[bool, int | None]:
    """通过 realtime 端点读物理舵机角度(真实编码器,非 car state 缓存)。

    Returns (ok, angle_degrees):
      ok=True  + angle=int  → 读到了
      ok=False              → 读失败(可能 runtime 卡 init / service 未注册)
    """
    try:
        r = client.http.realtime_bus_servo_read(ARM_BUS_PORT)
        if isinstance(r, dict) and r.get("ok") and "angle" in r:
            return True, int(r["angle"])
    except Exception:
        pass
    return False, None


def _snapshot_health(client) -> dict:
    """取 runtime health 的关键字段。HTTP 嵌套结构:
        h = {ok: bool, state: {initialized, initializing, ...}, links: {...}}
    返回 schema 平铺后用于判定:
        {"ok": bool, "initialized": bool, "initializing": bool,
         "ctrl_state": str|None, "raw": <原始 health dict>}
    """
    try:
        h = client.http.get_health()
    except Exception:
        return {"ok": False, "initialized": False, "initializing": False,
                "ctrl_state": None, "raw": None}
    if not isinstance(h, dict):
        return {"ok": False, "initialized": False, "initializing": False,
                "ctrl_state": None, "raw": None}
    s = h.get("state", {}) or {}
    cs = s.get("controller_session", {}) or {}
    return {
        "ok": bool(h.get("ok", False)),
        "initialized": bool(s.get("initialized", False)),
        "initializing": bool(s.get("initializing", False)),
        "ctrl_state": cs.get("state"),
        "raw": h,
    }


def wait_runtime_stable(client, timeout_s: float = WAIT_STABLE_S) -> bool:
    """等 runtime 不在 initializing 状态。返回 True=已稳定,False=超时。

    防 LEFT/RIGHT 命令被 auto-init rebuild 吞掉 —— 详情见 CLAUDE.md
    "debug-runtime-init-queue.md" H1 假设。
    """
    deadline = time.time() + timeout_s
    last_snap = None
    while time.time() < deadline:
        last_snap = _snapshot_health(client)
        if last_snap["ok"] and last_snap["initialized"] and not last_snap["initializing"]:
            return True
        time.sleep(0.2)
    if last_snap is not None:
        ctrl = last_snap.get("ctrl_state")
        init_fl = last_snap.get("initializing")
        ok_fl = last_snap.get("ok")
        ready = last_snap.get("initialized")
        print(
            f"  [WARN] runtime {timeout_s:.0f}s 内未稳定 — "
            f"ok={ok_fl} initialized={ready} initializing={init_fl} "
            f"ctrl={ctrl}"
        )
    return False


def emit_pre_set_health(client, label: str) -> None:
    """每站跑前打个 health 看一眼,方便事后看出 auto-init 时机。"""
    snap = _snapshot_health(client)
    if not snap["ok"]:
        print(f"  [NOHEALTH] pre {label}")
        return
    ip = "[REBUILD]" if snap["initializing"] else "[STABLE]"
    print(
        f"  {ip} pre {label}: initialized={snap['initialized']}  "
        f"initializing={snap['initializing']}  ctrl={snap['ctrl_state']}"
    )


def main() -> int:
    client = ArmClient.connect()
    if not preflight(client):
        sys.exit(1)
    print()

    runner = ArmRunner(client)

    st = client.get_state()
    print("=== 初始状态 ===")
    print(f"  side={st.side}  arm_angle={st.arm_angle}")
    print()

    print("=== 大臂 bus 舵机测试 ===")
    fails = 0
    for side in SEQUENCE:
        # ---- 跑前:runtime 必须稳定 + 设 side 前打 health ----
        if not wait_runtime_stable(client):
            print(f"  [FAIL] cmd={side:<6}  runtime 不稳定(在 rebuild) — 跳过本站")
            fails += 1
            continue
        emit_pre_set_health(client, side)

        # ---- 下发 ----
        # runner.set_side() 内部走 api_client.execute(),该函数在 job 入队后
        # 返回 queued/running dict。我们不需要等到终态 —— 业务流(manual debug)关心
        # 物理舵机是否转动,所以这里只 sleep 给舵机物理到位时间 + 健康检查,
        # 避免 wait_job 在 runtime 抖动时把测试挂死。如果运维需要严格 done,
        # 看 runtime_service.queued_jobs 是否清空即可。
        try:
            job = runner.set_side(side, timeout=10)
        except Exception as e:
            print(f"  [FAIL] cmd={side:<6}  {type(e).__name__}: {str(e)[:80]}")
            fails += 1
            continue

        if not isinstance(job, dict):
            print(f"  [FAIL] cmd={side:<6}  job 不是 dict: {type(job).__name__}: {str(job)[:60]}")
            fails += 1
            continue
        job_id = job.get("id")
        status = job.get("status")
        # 业务 API 入队 = 命令已被 runtime 接受。timeout=10s 等下发的硬时限已过
        # — 如果是 queued/running 不算"API 失败",我们只关心之后能不能看到物理反馈
        api_accepted = (status in {"succeeded", "queued", "running"})

        # ---- 等舵机物理到位 + 监听健康 ----
        rebuild_seen_during_settle = False
        deadline = time.time() + 3.0
        while time.time() < deadline:
            time.sleep(0.15)
            snap = _snapshot_health(client)
            if snap["initializing"]:
                rebuild_seen_during_settle = True
                # 等到重建结束再继续
                while snap["initializing"]:
                    time.sleep(0.2)
                    snap = _snapshot_health(client)

        # 现在读 job 终态(如果 runtime 已经在 background 处理好了;
        # 我们容忍它没在 3s 内完成 —— 看物理反馈是终极判据)
        try:
            final_job = client.http.get_job(job_id) if job_id else {}
        except Exception:
            final_job = {}
        status = final_job.get("status", status)
        api_ok = (status == "succeeded")

        # ---- 等舵机物理到位 ----
        time.sleep(SETTLE_S)

        # ---- 等 runtime 再次稳定(避免在 rebuild 中读)----
        if not wait_runtime_stable(client):
            print(f"  [WARN] cmd={side:<6}  runtime 跑后未稳定 — 结果可能不可信")

        # ---- 读回判定 (容忍 runtime 中途 rebuild)----
        # runtime auto-init 可能在 job done 之后、我们读 side 之前,把 MyCar 重建,
        # 新建的 arm 实例默认 side="MID" — 即使 set_side('LEFT') 物理成功,servo side
        # 也会被覆盖回 MID。这里重复读取 + 等稳定,直到读侧对上或重试上限。
        expect = EXPECTED_ANGLE[side]
        side_ok = False
        angle_ok = False
        cur = None
        rebuild_seen = rebuild_seen_during_settle
        for attempt in range(1, RETRY_READ + 1):
            cur = client.get_state()
            side_ok = (cur.side == side)
            angle_ok = (cur.arm_angle == expect)
            if side_ok:
                break
            # 等待 runtime 若正在 rebuild 跑完,再读一次
            time.sleep(RETRY_DELAY_S)
            wait_runtime_stable(client, timeout_s=WAIT_STABLE_S)
            # 抽空观察一次健康(若发现 initializing 走过,标记 rebuild_seen)
            snap = _snapshot_health(client)
            if snap["initializing"]:
                rebuild_seen = True

        # ---- 同时读物理舵机角度(realtime 端点,真实编码器)----
        phys_ok, phys_angle = read_bus_physical_angle(client)

        angle_display = "—" if cur.arm_angle is None else f"{cur.arm_angle}"
        phys_display = "—" if phys_angle is None else f"{phys_angle:+d}"
        # 三态判定:
        #   1) runtime side 字段对
        #   2) 物理 bus 舵机角度对(最严)
        #   3) api 成功
        if cur.arm_angle is None:
            label = f"api_accepted={api_accepted}  status={status!r}  angle=N/A"
        else:
            label = f"api_accepted={api_accepted}  status={status!r}  angle={angle_ok}"
        phys_match = phys_ok and phys_angle is not None and abs(phys_angle - expect) <= 3
        flag = "OK  " if (side_ok and api_accepted and phys_match) else "FAIL"
        extra = ""
        if not api_accepted:
            err = job.get("error")
            jid = job.get("id")
            extra = (
                f"  [diag] job.id={jid} status={status!r} "
                f"error={str(err)[:120] if err else None}"
            )
        elif not side_ok and rebuild_seen:
            extra = "  [NOTE] runtime 走过 auto-init rebuild,side 可能被重置"
        elif side_ok and not phys_match and phys_ok:
            extra = (
                f"  [NOTE] runtime side={cur.side} 但物理角度 {phys_angle}° ≠ expect {expect}° — "
                f"runtime state 与硬件不一致(硬件未真转 / 反向)"
            )
        elif not side_ok and phys_match:
            extra = (
                f"  [NOTE] 物理角度对({phys_angle}°)但 runtime side={cur.side} — "
                f"runtime state 没同步"
            )

        # 显示尝试次数
        tries_str = f"x{attempt}" if not side_ok else "x1"
        ok_count = sum(int(x) for x in (side_ok, api_accepted, phys_match))

        print(
            f"  [{flag}] cmd={side:<6}  state.side={cur.side:<6}  "
            f"phys={phys_display:>4}°/(exp {expect:+d}°)  tries={tries_str}  ok={ok_count}/3  {label}"
        )
        if extra:
            print(extra)
        # 判定标准:物理角度对 + API 入队成功(这是真正决定硬件好坏的判据)
        if not (api_accepted and phys_match):
            fails += 1

    print()
    total = len(SEQUENCE)
    postflight(client, "after")
    print(f"{'PASS' if fails == 0 else 'FAIL'}: {total - fails}/{total} ok")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

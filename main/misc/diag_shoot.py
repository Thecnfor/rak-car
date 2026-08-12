"""main/misc/diag_shoot.py — 射击供电诊断: 连发 N 发, 每发打印 射前/吸合瞬间/射后 电池电压 + 跌落.

回答一个问题: **"子弹疲软无力"到底是不是供电电流的问题?**

原理:
- runtime `shooting()` 现在会在继电器吸合瞬间读一次电池电压 (v_during),
  并与射前空闲 (v_idle) 对比, 把结果随 job result 返回.
- 判定:
  - **v_drop (v_idle - v_during) > ~1.5V** → 供电链路吃不住负载:
    电池带载跌落 / 线路细或接头氧化 / 继电器触点老化 / 阀线圈电流过大.
    修: 换粗线, 清洁接头, 查继电器触点, 查电池是否老化, 或补强供电.
  - **v_drop 很小但子弹仍弱** → 不是供电问题, 查:
    ① 气罐压力 (每发耗气, 后几发更弱 = 没补气/漏气);
    ② 机构 (弹丸配合/管壁摩擦/气路漏气).
- 对比 "第 1 发 vs 最后一发" 的 v_drop: 如果越射越弱 + v_drop 越大, 供电/电池问题;
  如果越射越弱但 v_drop 不变, 是气压问题.

跑前准备:
- 车停稳, 枪口朝安全方向, 弹装好
- 连发之间留装填/补气时间 (--interval 默认 2s)

用法:
    python3 -m main.misc.diag_shoot
    python3 -m main.misc.diag_shoot --count 5 --interval 3
    python3 -m main.misc.diag_shoot --hold 0.40     # 临时试更长激活时长 (env 透传)
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from main.api_client import RuntimeApiClient

DROP_WARN_V = 1.5          # v_drop 超过此值判供电不足 (经验阈值)
WARN_MIN_IDLE_V = 10.0     # 射前空闲电压低于此值先警告 (11.1V 3S 锂电低于 ~10V 基本没电)


def read_idle(client, samples=3, gap=0.3):
    """采 samples 次空闲电压, 返回中位数 (V); 失败返回 None."""
    vals = []
    for _ in range(samples):
        try:
            job = client.execute_car_action(
                "get_battery_voltage", sync=True, timeout=5)
            v = job.get("result")
            if isinstance(v, (int, float)) and v > 0:
                vals.append(float(v))
        except Exception:
            pass
        time.sleep(gap)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def one_shot(client, hold, interval):
    """打一发, 返回电压字典 (shooting() 返回), 失败返回 None."""
    print(f"\n[shot] firing (hold={hold if hold is not None else 'default'}s)...",
          flush=True)
    t0 = time.time()
    try:
        job = client.execute_car_action(
            "shooting", sync=True, timeout=10)
    except Exception as exc:
        print(f"[shot] 失败: {exc}", file=sys.stderr, flush=True)
        return None
    dt = time.time() - t0
    res = (job.get("result") or {}) if isinstance(job, dict) else {}
    print(f"[shot] ok in {dt:.2f}s result={res}", flush=True)
    return res


def verdict(v_idle, res):
    """根据空闲电压 + 单发电压遥测给判定."""
    if not res:
        return "no telemetry (runtime 未返回电压, 检查是否已更新代码)"
    v_during = res.get("v_during")
    v_drop = res.get("v_drop")
    if v_during is None:
        return "no during-voltage (电池读失败, 看 runtime 日志)"
    line = f"  v_drop = {v_drop:.2f}V"
    if v_idle is not None and v_idle < WARN_MIN_IDLE_V:
        line += f"  ⚠️ 空闲电压 {v_idle:.2f}V 偏低, 先充电"
    if v_drop > DROP_WARN_V:
        line += ("  → 供电不足! 查电池/线路/继电器触点/阀线圈电流 "
                 "(不是代码问题)")
    else:
        line += ("  → 供电正常, 子弹弱查 气罐压力/漏气/机构")
    return line


def main() -> int:
    ap = argparse.ArgumentParser(description="射击供电诊断")
    ap.add_argument("--count", type=int, default=3, help="射击次数 (默认 3)")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="两发间隔秒数 (默认 2.0, 留装填/补气时间)")
    ap.add_argument("--hold", type=float, default=None,
                    help="临时覆盖继电器激活时长 (s), 只对本进程生效")
    args = ap.parse_args()
    if args.count < 1:
        raise ValueError("--count 必须 >= 1")

    if args.hold is not None:
        if not 0.05 <= args.hold <= 1.0:
            raise ValueError("--hold 必须在 [0.05, 1.0] 内")
        os.environ["RAK_CAR_SHOOT_RELAY_HOLD_S"] = str(args.hold)

    client = RuntimeApiClient()
    client.wait_until_ready()

    v_idle = read_idle(client)
    print(f"[idle] 空闲电池电压 ≈ {v_idle:.2f}V" if v_idle else "[idle] 电压读取失败")

    for i in range(args.count):
        res = one_shot(client, args.hold, args.interval)
        if res:
            print(f"[{i + 1}/{args.count}] {verdict(v_idle, res)}", flush=True)
        if i < args.count - 1:
            time.sleep(args.interval)

    print("\n[done] 判定汇总: 看每发 v_drop。"
          "第 1 发 vs 最后一发对比: 越射越弱且 v_drop 变大 → 供电; "
          "v_drop 不变但越射越弱 → 气罐压力/漏气。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

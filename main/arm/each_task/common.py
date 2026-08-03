"""main/arm/each_task/common —— taskX 业务层共用的 arm 操作工具。

职责:
  - 抽离 task5 (high_tower / low_tower / target) 共用的 belt-slip 安全 move_x
    (之前 3 个文件各拷贝一份, 改一处要同步 3 处, 容易漏)。
  - 后续 task4 / task2 等如果用同样模式, 直接 import 即可。

⚠️ 本模块**只依赖 main.arm** (ArmClient/ArmRunner), 不 import task5 包内
   任何模块(防止 task5 自包含策略被破坏)。
⚠️ x 位置一律走 client._read_x_mm_realtime() 校验 (x_get_position 坏, ARM_API §11)。

2026-07-30: 抽离自 task5/{high_tower, low_tower, target}.py, 加 wall_hit
             + overshoot 检测 (low_tower 现场实测撞墙 -278.5 触发)。
"""
from __future__ import annotations

import time
from typing import Optional

# ============================================================================
# belt-slip 安全 move_x 常量
# ============================================================================

# 这些是 default 值, 调用方可以传 kwargs 覆盖 (例如调速度更慢 / 墙更近)。
DEFAULT_V_MAX_MMS: float = 30.0
"""业务限速 (mm/s)。2026-07-22 限速透传 bug 修复后定档 30。"""

DEFAULT_TOL_MM: float = 5.0
"""到位容差 (mm)。realtime 抖动 <1mm, 放宽给 PID 余量。"""

DEFAULT_MAX_ROUNDS: int = 12
"""最多尝试轮数。"""

DEFAULT_STALL_MM: float = 3.0
"""本轮位移 < 此值视为 stall (疑似打滑/堵转/撞墙)。"""

DEFAULT_MAX_STALL_ROUNDS: int = 3
"""连续 stall 多少轮 → 放弃。"""

DEFAULT_KICK_SLEEP_S: float = 0.2
"""stall 时停多久让同步带齿重咬合 (秒)。"""

# 2026-07-30 现场实测: low_tower.py 第一轮 move_x overshoot 100mm 撞到 -278.5。
# 留 20mm 余量定 -300。如下次换场地 / 换 arm, 重新撞墙反推。

DEFAULT_WALL_MM: float = -295.0
"""x 物理墙位置 (mm, 负值方向)。
2026-08-03 P 姿态 POSE_P_X_MM=-300 实测撞墙 (x=-290.2 距墙 -300 还有 9.8mm 即 wall_hit),
留 5mm 余量。"""

DEFAULT_WALL_TOL_MM: float = 60.0
"""距墙 < 此值视为 wall_hit, 立即 break (不等 stall 计数器)。
2026-08-03 P 姿态 POSE_P_X_MM=-300 实测撞墙到 -290.2mm, 距 wall_mm=-295 还有 9.8mm。
放宽容差到 60mm 容忍这种 '差一点点但到不了' 的情况, 让 move_x 走 wall_hit 后抛
RuntimeError 而不是反复 stall 探测。"""


# ============================================================================
# P 姿态 (Pose-P) —— task4 标准采收/检测姿态
# ============================================================================
# 2026-08-03 现场标定 (下位机 reset 后): 球在画面中心 + 侧摄能稳定看到球
# 区间内的统一姿态。所有 task4 寻路 / 检测 / 抓取之间都恢复到这里。

POSE_P_Y_MM: float = -200.0
"""P 姿态 y (mm)。"""

POSE_P_X_MM: float = -300.0
"""P 姿态 x (mm)。"""

POSE_P_ARM_DEG: float = 90.0
"""P 姿态大臂角度 (°, MID / 复位位, 业务硬限上界)。"""

POSE_P_HAND_DEG: float = 0.0
"""P 姿态手爪角度 (°, DOWN, 需先 arm >= +30° 出联动保护区)。"""

POSE_P_GRAB_Y_MM: float = -160.0
"""P 姿态下抓球 y (mm, 4cm 球球心高度)。"""

# ponytail: POSE_P 是 task4 单一真相源, 不要在 target4/target1 等其他文件里硬编
# 同一组数字, 一律 from main.arm.each_task.common import POSE_P_*


def goto_pose_p(client, runner, *, log_prefix: str = "[goto_pose_p]") -> dict:
    """恢复臂到 P 姿态 (Pose-P): y=POSE_P_Y → composite_run(arm/hand) → x=POSE_P_X。

    复用 move_x_hard_reach 的 split + reset_x 撞墙兜底 (belt-slip 修复后稳)。
    composite_run 走 4 电机 ThreadPoolExecutor 并行, arm/hand/y 同步摆位。

    Args:
        client: ArmClient
        runner: ArmRunner (保留参数兼容性, 当前未直接用)
        log_prefix: 打印前缀

    Returns:
        dict 含 ``actual_y_mm``, ``actual_x_mm`` (从 realtime 读真值)。
        任何内部异常向上抛 (业务层决定是否退化)。
    """
    print(f"\n========== {log_prefix} 恢复 P 姿态 "
          f"(composite_run 4 轴同步: y={POSE_P_Y_MM} → x={POSE_P_X_MM} "
          f"arm={POSE_P_ARM_DEG}°/hand={POSE_P_HAND_DEG}°) ==========")
    # 1. composite_run 4 轴同步到位姿 (arm/x/y/hand 一次下发, ThreadPoolExecutor 并行)
    #    不调 move_x_hard_reach / belt-slip / wall_hit —— SDK composite_run 自带 x/y PID 闭环,
    #    belt-slip 修复是 SDK 内部的事, 业务层不该自己再叠一层。
    client.composite_run(
        arm=POSE_P_ARM_DEG,
        x_mm=POSE_P_X_MM,
        y_mm=POSE_P_Y_MM,
        hand=POSE_P_HAND_DEG,
        speed=80,
        timeout=30.0,
    )
    actual_y = None
    try:
        st = client.http.get_arm_state()
        y_st = st.get("arm_state", {}) if isinstance(st, dict) else {}
        actual_y = y_st.get("y_mm")
    except Exception:
        actual_y = None
    actual_x = client._read_x_mm_realtime()
    print(f"========== {log_prefix} 完成 "
          f"(realtime y={actual_y}mm x={actual_x}mm) ==========\n")
    return {
        "actual_y_mm": actual_y,
        "actual_x_mm": actual_x,
    }

DEFAULT_OVERSHOOT_RATIO: float = 1.5
"""overshoot 判定: 单轮 step > |初始 delta| × 此值 → 标记 overshoot。"""


# ============================================================================
# trust 模式 move_x (2026-07-31 新增)
# ============================================================================
# 适用场景: realtime x 读数不可信 (实测 +285.8 ↔ -292.x 跳变 578mm)
#   - belt-slip 仍走 motor_280 PID 内部闭环 (准), 不依赖外部读数
#   - 业务层已知读数问题, 选择信任 move_x() 内部返回值
# 行为:
#   - 单次 client.move_x(target, v_max, timeout)
#   - 不循环, 不踢 stall, 不检查 wall, 不检查 overshoot
#   - 读一次 realtime 仅供日志 (actual_x 可能不准)
#   - 返回 reached=True 永远 (信任 move_x)
# 风险:
#   - 可能撞墙不报 (motor PID 内部有 stall 兜底, 但我们看不到)
#   - 可能打滑到位也不知道 (失去 stall 检测)
#   - 业务层需要接受这个 trade-off
# ============================================================================

def move_x_trust(
    client,
    runner,
    target_x_mm: float,
    *,
    log_prefix: str = "[move_x_trust]",
    v_max_mms: float = DEFAULT_V_MAX_MMS,
    timeout: float = 30.0,
) -> dict:
    """trust 模式 move_x: 单次 move_x() 调用, 不做 belt-slip/wall/overshoot 检测。

    适用 realtime 读数不可靠的场景 (move_x 内部 PID 用控制器内部位置, 准)。

    Args:
        client: ArmClient 实例 (调 .move_x + ._read_x_mm_realtime)。
        runner: ArmRunner 实例 (保留参数兼容性, 函数内未使用)。
        target_x_mm: 目标 x 位置 (mm)。
        log_prefix: 日志前缀, 例如 "[task5/high_tower]" / "[task5/low_tower]" / "[task5/target]"。
        v_max_mms: 业务限速 (mm/s)。
        timeout: 单次超时 (秒)。

    Returns:
        {
            "target_x":     float,   # 请求目标
            "actual_x":     float | None,  # realtime 参考值 (可能不准)
            "segments":     0,       # 永远是 0 (单次)
            "reached":      True,    # 永远 True (信任)
            "result":       str,     # "trust" / "trust_failed"
            "wall_hit":     False,
            "overshoot_mm": 0.0,
        }
    """
    print(f"  {log_prefix} move_x({target_x_mm:+.0f}mm)  TRUST 模式 "
          f"(不 stall/wall/overshoot 检测, 信任 move_x 内部 PID)")

    try:
        client.move_x(x_mm=target_x_mm, v_max_mms=v_max_mms, timeout=timeout)
    except Exception as e:
        print(f"  {log_prefix} [FAIL] move_x 异常: "
              f"{type(e).__name__}: {str(e)[:80]}")
        return {
            "target_x": target_x_mm,
            "actual_x": None,
            "segments": 0,
            "reached": True,   # 脚本继续 (trust 模式不抛)
            "result": "trust_failed",
            "wall_hit": False,
            "overshoot_mm": 0.0,
            "error": str(e),
        }

    # 读一次参考值, 不阻塞
    x_after = client._read_x_mm_realtime()
    print(f"  {log_prefix} move_x 完成, realtime x = {x_after} "
          f"(仅供参考, 已知读数可能不准)")

    return {
        "target_x": target_x_mm,
        "actual_x": x_after,
        "segments": 0,
        "reached": True,
        "result": "trust",
        "wall_hit": False,
        "overshoot_mm": 0.0,
    }


# ============================================================================
# belt-slip + wall + overshoot 安全 move_x
# ============================================================================

def move_x_with_split(
    client,
    runner,
    target_x_mm: float,
    *,
    log_prefix: str = "[move_x]",
    v_max_mms: float = DEFAULT_V_MAX_MMS,
    tol_mm: float = DEFAULT_TOL_MM,
    max_rounds: int = DEFAULT_MAX_ROUNDS,
    stall_mm: float = DEFAULT_STALL_MM,
    max_stall_rounds: int = DEFAULT_MAX_STALL_ROUNDS,
    kick_sleep_s: float = DEFAULT_KICK_SLEEP_S,
    wall_mm: float = DEFAULT_WALL_MM,
    wall_tol_mm: float = DEFAULT_WALL_TOL_MM,
    overshoot_ratio: float = DEFAULT_OVERSHOOT_RATIO,
) -> dict:
    """belt-slip + wall + overshoot 安全的 move_x。

    行为:
      1. 读 realtime x0 算 delta
      2. 已在容差内 → result="already_in_range", reached=True, 立即返回
      3. 每轮 move_x(target, v_max_mms) + realtime 读真值
      4. 判定 (按优先级):
         a. |x_now - target| < tol_mm          → result="success", reached=True
         b. |x_now - wall_mm| < wall_tol_mm    → wall_hit=True, 立即 break
         c. abs(step) > |initial_delta| * 1.5  → 记入 max_overshoot_mm (不 break)
         d. abs(step) < stall_mm              → stall 计数; 连续 N 轮 → break
      5. 综合 result 分类:
         - "success"             |residual| < tol
         - "overshoot_wall_hit"  撞墙 + 走了过头
         - "wall_hit"            平推到墙 (无 overshoot)
         - "overshoot"           走了过头但没撞墙
         - "stalled"             带打滑 / 堵转 / 其他

    Args:
        client: ArmClient 实例 (调 ._read_x_mm_realtime() + .move_x() + .stop_x_speed_safety())
        runner: ArmRunner 实例 (保留兼容性, 函数内未使用)
        target_x_mm: 目标 x 位置 (mm)
        log_prefix: 日志前缀, 例如 "[task5/high_tower]" / "[task5/low_tower]" / "[task5/target]"
        v_max_mms: 业务限速 (mm/s)
        tol_mm: 到位容差 (mm)
        max_rounds: 最多尝试轮数
        stall_mm: 单轮位移 < 此值视为 stall
        max_stall_rounds: 连续 stall 多少轮 → 放弃
        kick_sleep_s: stall 时停多久让带重咬合
        wall_mm: x 物理墙位置 (mm)
        wall_tol_mm: 距墙 < 此值 = wall_hit
        overshoot_ratio: overshoot 判定系数

    Returns:
        {
            "target_x":     float,   # 请求目标
            "actual_x":     float,   # 最终位置 (alias: final_x, 旧版兼容)
            "final_x":      float,   # 同 actual_x
            "residual_mm":  float,   # target - final (正=没到, 负=过头)
            "segments":     int,     # 实际走的轮数
            "reached":      bool,    # True = |residual| < tol_mm
            "result":       str,     # 见上方 5 种枚举
            "wall_hit":     bool,    # True = 撞墙
            "overshoot_mm": float,   # 单轮最大超调 (0 = 没超)
        }
    """
    # 读基准
    x0 = client._read_x_mm_realtime()
    if x0 is None:
        raise RuntimeError("realtime x_mm 读不到 (arm_feed 未启 / realtime 不可用)")
    delta = target_x_mm - x0
    initial_delta_abs = abs(delta)

    if initial_delta_abs <= tol_mm:
        print(f"  {log_prefix} move_x({target_x_mm}mm)  已在容差内 (x0={x0:+.1f}mm), 跳过")
        return {
            "target_x": target_x_mm, "actual_x": x0, "final_x": x0,
            "residual_mm": target_x_mm - x0, "segments": 0, "reached": True,
            "result": "already_in_range", "wall_hit": False, "overshoot_mm": 0.0,
        }

    print(f"  {log_prefix} move_x({target_x_mm}mm)  距 {delta:+.1f}mm  "
          f"v_max={v_max_mms:.0f}mm/s  TOL=±{tol_mm:.0f}mm  "
          f"墙={wall_mm:.0f}±{wall_tol_mm:.0f}mm  "
          f"reach+wall+stall 模式 (common.py 2026-07-30 抽离)")

    x_prev = x0
    stall_rounds = 0
    steps = 0
    reached = False
    x_final = x0
    wall_hit = False
    max_overshoot_mm = 0.0

    for rnd in range(1, max_rounds + 1):
        try:
            client.move_x(x_mm=target_x_mm, v_max_mms=v_max_mms)
            x_now = client._read_x_mm_realtime()
            if x_now is None:
                raise RuntimeError("realtime x_mm 读不到")
        except Exception as e:
            print(f"  {log_prefix} [FAIL] 轮{rnd:2d}  {type(e).__name__}: {str(e)[:80]}")
            break

        step = x_now - x_prev
        err = x_now - target_x_mm
        steps += 1
        x_prev = x_now
        x_final = x_now

        # overshoot 跟踪
        if initial_delta_abs > 0 and abs(step) > initial_delta_abs * overshoot_ratio:
            overshoot_this_round = abs(step) - initial_delta_abs
            if overshoot_this_round > max_overshoot_mm:
                max_overshoot_mm = overshoot_this_round

        print(f"  {log_prefix} 轮{rnd:2d}  x={x_now:+7.1f}mm  本轮走={step:+6.1f}mm  "
              f"距目标={err:+6.1f}mm")

        if abs(err) < tol_mm:
            reached = True
            break

        # wall_hit 检测: 距墙 < TOL → 立即 break (不等 stall)
        if abs(x_now - wall_mm) < wall_tol_mm:
            wall_hit = True
            print(f"         [WALL_HIT] x={x_now:+.1f}mm 距墙{wall_mm:+.0f}mm "
                  f"< {wall_tol_mm:.0f}mm, 立即 break (不等 stall)")
            break

        if abs(step) < stall_mm:
            stall_rounds += 1
            print(f"         [SLIP] 本轮几乎没动, kick 停 {kick_sleep_s}s 让同步带重咬合 "
                  f"(连续卡住 {stall_rounds}/{max_stall_rounds})")
            if stall_rounds >= max_stall_rounds:
                print(f"         [ABORT] 连续 {max_stall_rounds} 轮无进展, "
                      f"撞墙/带打滑治不动, 放弃")
                break
            client.stop_x_speed_safety()
            time.sleep(kick_sleep_s)
        else:
            stall_rounds = 0

    # 综合 result
    if reached:
        result = "success"
    elif wall_hit and max_overshoot_mm > 0:
        result = "overshoot_wall_hit"
    elif wall_hit:
        result = "wall_hit"
    elif max_overshoot_mm > 0:
        result = "overshoot"
    else:
        result = "stalled"

    return {
        "target_x": target_x_mm,
        "actual_x": x_final,
        "final_x": x_final,
        "residual_mm": target_x_mm - x_final,
        "segments": steps,
        "reached": reached,
        "result": result,
        "wall_hit": wall_hit,
        "overshoot_mm": max_overshoot_mm,
    }


# ============================================================================
# 硬到位 move_x (2026-07-31 新增, 方案 B)
# ============================================================================
# 适用场景: split 模式没到位 (stalled/wall_hit 后到不了 target), 用 reset_x
#   撞墙重置编码器零点 + 重新走 split 兜底。
# 行为:
#   1. 先走一次 move_x_with_split
#   2. 没 reached → reset_x 撞墙定原点 (清掉编码器漂移)
#   3. 再走一次 move_x_with_split (从新原点出发)
#   4. 还没 reached → 抛 RuntimeError (业务层看见, 不再静默)
# 风险:
#   - reset_x 撞墙 motor 走 ~120mm, 要 ~3-4s
#   - 撞墙过程可能撞坏同步带 (belt-slip 修复后应该没事, belt-slip 复发会加剧)
# 业务层用法:
#   - 默认 direction="right" (跟 task4 x_to_zero.py / task5 get_blue.py 一致)
#   - 撞错墙 → 业务层传 direction="left"
# ============================================================================

def move_x_hard_reach(
    client,
    runner,
    target_x_mm: float,
    *,
    log_prefix: str = "[move_x_hard_reach]",
    v_max_mms: float = DEFAULT_V_MAX_MMS,
    tol_mm: float = DEFAULT_TOL_MM,
    max_rounds: int = DEFAULT_MAX_ROUNDS,
    stall_mm: float = DEFAULT_STALL_MM,
    max_stall_rounds: int = DEFAULT_MAX_STALL_ROUNDS,
    kick_sleep_s: float = DEFAULT_KICK_SLEEP_S,
    wall_mm: float = DEFAULT_WALL_MM,
    wall_tol_mm: float = DEFAULT_WALL_TOL_MM,
    overshoot_ratio: float = DEFAULT_OVERSHOOT_RATIO,
    reset_direction: str = "right",
    reset_velocity_mms: float = 50.0,
    reset_probe_time: float = 0.3,
    reset_timeout: float = 30.0,
) -> dict:
    """方案 B: split 模式 + reset_x 撞墙重置兜底。

    行为:
      1. 调 move_x_with_split 尝试到位
      2. 没 reached → reset_x 撞墙重置 (清编码器漂移)
      3. 再调一次 move_x_with_split
      4. 还 reached=False → RuntimeError 抛给业务层 (不静默)

    Args:
        client: ArmClient 实例
        runner: ArmRunner 实例 (保留参数兼容性)
        target_x_mm: 目标 x 位置 (mm)
        ... (split 模式所有参数都透传给 move_x_with_split)
        reset_direction: reset_x 撞墙方向 ("right"/"left"), 跟 task4 x_to_zero.py 一致
        reset_velocity_mms: 撞墙速度 (mm/s, 默认 50 稳)
        reset_probe_time: 反向探针时间 (默认 0.3, 避免 stall 误判)
        reset_timeout: reset_x 超时

    Returns:
        {
            "target_x":     float,
            "actual_x":     float,
            "final_x":      float,
            "residual_mm":  float,
            "segments":     int,
            "reached":      bool,
            "result":       str,   # "success" / "stalled_after_reset" / "wall_hit_after_reset" / "reset_then_success"
            "wall_hit":     bool,
            "overshoot_mm": float,
            "reset_count":  int,   # 撞墙重置次数 (0 或 1)
        }

    Raises:
        RuntimeError: reset_x 后 split 还是没到位 (belt-slip 复发 / 编码器坏)
    """
    print(f"  {log_prefix} 第一次 split 模式尝试 {target_x_mm:+.0f}mm")
    res1 = move_x_with_split(
        client, runner,
        target_x_mm=target_x_mm,
        log_prefix=log_prefix + " [split#1]",
        v_max_mms=v_max_mms,
        tol_mm=tol_mm,
        max_rounds=max_rounds,
        stall_mm=stall_mm,
        max_stall_rounds=max_stall_rounds,
        kick_sleep_s=kick_sleep_s,
        wall_mm=wall_mm,
        wall_tol_mm=wall_tol_mm,
        overshoot_ratio=overshoot_ratio,
    )

    # ⚠️ 熔断 (2026-07-31): 第一轮 split 实时位置完全没变 → motor 可能死了
    #   之前会进入 stall 5 轮死循环,业务层看不见 motor 真死; 立刻抛错。
    # 2026-08-01: FUSE 之前先试 reset_x 一次 (last-ditch), reset_x 成功 + 编码器重置
    #   后可能让 motor 恢复响应 (硬件过流保护会自恢复; 编码器失同步也能 reset)。
    if res1["segments"] >= 1 and abs(res1["residual_mm"]) >= abs(target_x_mm) * 0.9:
        print(f"  [FUSE] {log_prefix} split#1 完全没动 (residual={res1['residual_mm']:+.1f}mm"
              f", 目标 {target_x_mm:+.0f}mm 的 {abs(res1['residual_mm'])/abs(target_x_mm)*100:.0f}%)"
              f"; **先试 reset_x 一次** (last-ditch 救 motor)")
        try:
            print(f"  {log_prefix} [FUSE-rescue] reset_x 撞墙重置编码器...")
            reset_job = client._call_arm(
                "reset_x", timeout=reset_timeout, sync=True,
                direction=reset_direction,
                reset_velocity=reset_velocity_mms / 1000.0,
                probe_time=reset_probe_time,
            )
            print(f"  {log_prefix} [FUSE-rescue] reset_x 完成, 再走一次 split...")
            res_rescue = move_x_with_split(
                client, runner,
                target_x_mm=target_x_mm,
                log_prefix=log_prefix + " [split#2 after FUSE-rescue]",
                v_max_mms=v_max_mms,
                tol_mm=tol_mm,
                max_rounds=max_rounds,
                stall_mm=stall_mm,
                max_stall_rounds=max_stall_rounds,
                kick_sleep_s=kick_sleep_s,
                wall_mm=wall_mm,
                wall_tol_mm=wall_tol_mm,
                overshoot_ratio=overshoot_ratio,
            )
            res_rescue["reset_count"] = 1
            if res_rescue["reached"]:
                # reset_x 救回来了!
                print(f"  [FUSE-RESCUED] {log_prefix} reset_x 后 split 成功到位 "
                      f"({res_rescue['final_x']:+.1f}mm), 标记为 reset_then_success")
                res_rescue["result"] = "reset_then_success"
                return res_rescue
            # reset_x 也没救回来, 抛错 (兜底失败)
            msg = (f"{log_prefix} motor 完全没响应: split#1 走了 {res1['segments']} 轮,"
                   f"FUSE-rescue reset_x 后 split#2 仍 residual={res_rescue['residual_mm']:+.1f}mm,"
                   f"目标={target_x_mm:+.0f}mm。可能 belt-slip 跳齿 / 编码器坏 / "
                   f"motor 驱动板过流保护。业务层不再静默 stall 循环,立即抛错请查硬件。")
            print(f"  [FUSE-RAISE] {msg}")
            raise RuntimeError(msg)
        except RuntimeError:
            raise
        except Exception as rescue_exc:
            # reset_x 本身挂了 (e.g. 控制器没响应) → 抛原 FUSE 错
            msg = (f"{log_prefix} motor 完全没响应: split#1 走了 {res1['segments']} 轮,"
                   f"residual={res1['residual_mm']:+.1f}mm,目标={target_x_mm:+.0f}mm。"
                   f"FUSE-rescue 救不了 (reset_x 抛 {type(rescue_exc).__name__}: "
                   f"{str(rescue_exc)[:80]})。可能 belt-slip 跳齿 / 编码器失同步 / "
                   f"motor 驱动板过流保护 / 控制器挂死。")
            print(f"  [FUSE-RAISE] {msg}")
            raise RuntimeError(msg) from rescue_exc

    if res1["reached"]:
        res1["reset_count"] = 0
        return res1

    # 没到位, reset_x 撞墙重置
    print(f"  {log_prefix} 第一次 split 没到位 (result={res1['result']!r}, "
          f"residual={res1['residual_mm']:+.1f}mm), reset_x 撞墙重置...")
    reset_job = client._call_arm(
        "reset_x", timeout=reset_timeout, sync=True,
        direction=reset_direction,
        reset_velocity=reset_velocity_mms / 1000.0,  # m/s
        probe_time=reset_probe_time,
    )
    print(f"  {log_prefix} reset_x 完成, 重新走 split...")

    # 再走一次 split (从新原点出发)
    res2 = move_x_with_split(
        client, runner,
        target_x_mm=target_x_mm,
        log_prefix=log_prefix + " [split#2 after reset]",
        v_max_mms=v_max_mms,
        tol_mm=tol_mm,
        max_rounds=max_rounds,
        stall_mm=stall_mm,
        max_stall_rounds=max_stall_rounds,
        kick_sleep_s=kick_sleep_s,
        wall_mm=wall_mm,
        wall_tol_mm=wall_tol_mm,
        overshoot_ratio=overshoot_ratio,
    )

    res2["reset_count"] = 1

    if not res2["reached"]:
        # belt-slip 复发 / 编码器坏, 不再静默
        msg = (f"{log_prefix} reset_x 后 split 还是没到位 "
               f"(result={res2['result']!r}, residual={res2['residual_mm']:+.1f}mm)。"
               f"belt-slip 复发 或 编码器坏了, 业务层需要看见。")
        print(f"  [RAISE] {msg}")
        raise RuntimeError(msg)

    # 标记重试成功
    if res2["result"] == "success":
        res2["result"] = "reset_then_success"
    else:
        # wall_hit / stalled 但 residual 在容差内 (极端情况)
        res2["result"] = f"reset_then_{res2['result']}"

    return res2

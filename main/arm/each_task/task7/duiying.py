"""task7 / duiying —— **对应** target 识别的 6 人名 ↔ task6 liebiao 两人名,输出匹配项。

按用户 2026-08-04 现场要求:

  流程:
    1. 调 ``task7/target.py:run()`` 跑 1×OCR + 2×3 网格解析,拿到 6 个位置 (1-6) 的人名
    2. 调 ``task6/liebiao.py`` 显式 ``_load_from_json()`` 加载磁盘上的历史数据
       (v3 进程隔离要求,默认 import 时为空)
    3. 把 liebiao1[0] / liaobiao2[0] 的 ``人名`` 字段跟 6 个名字比对
    4. 匹配成功 → 输出:
       ``✅ 符合 liaobiao1, 人名=王五, 蔬菜=土豆, target 位置 2 (上中)``
       (含位置 1/2 编号 + 人名 + 蔬菜 + target 识别的位置 1-6 + 中文标签)
    5. 未匹配 → **静默忽略** (不打印)

⚠️ **本文件编排 (orchestrator) 角色 — 与 task6/the_final.py 同款**:

  task7 自包含条款 (``target.py:24-27``、``position1.py:47-50``) 字面只禁止
  "不 import task7 包内任何模块"。本文件**编排跨包模块**, 故跨包 import
  ``task6/liebiao`` 破例允许 (参照 ``task6/the_final.py:23-26`` 跨模块编排先例)。

  安全前提: ``task6/liebiao.py:121-122`` 字面写出调用入口
  ``from main.arm.each_task.task6.liebiao import _load_from_json``,
  作者明示欢迎此类调用。

⚠️ **自包含** (与 task7 其他脚本同款):

  只依赖 ``main.arm`` + ``main.arm.each_task.task7.target`` (同包, 安全) +
  ``main.arm.each_task.task6.liebiao`` (跨包, 编排破例)。
  不引 task7 包内其它模块 (position*.py / dipan.py / pingcang.py 等)。
  原因: task5/目录曾被外部清空过 (见 [[task5-rebuild-2026-07-22]]),
  自包含保证 ``python duiying.py`` 直接跑不受影响。

⚠️ **名字比对** (``_normalize_name``):

  target.run() 的 ``results[i].name`` 直接来自 OCR 输出,可能含:
    - 前后空白 (``\\n`` / 空格)
    - 全角字符混入 (e.g. 全角空格 / 全角括号)
    - 常见 OCR 噪声 (空格、换行)
  liebiao 的人名是用户键入或 OCR 后人工确认过的,通常干净。
  归一化策略:
    1. ``str.strip()`` 去前后空白
    2. ``str.replace()`` 把全角空格 / 全角冒号等转半角
    3. **不**做语义级校验 (e.g. 同音字) — 那是 wenzishibie.py 的事,
       本脚本只做"字符串是否一致" 的轻校验。

⚠️ **边界行为**:

  - ``liaobiao1`` / ``liaobiao2`` 空 → 输出 ``⚠️ liaobiao1 空, 跳过``, 仍走另一个
  - target OCR 失败 → 默认软警告继续走 liebiao 比对; ``--strict`` 抛 RuntimeError
  - 6 个名字全 None → 输出 ``⚠️ target OCR 没识别到任何名字, 跳过全部比对``
  - liebiao 某项 ``人名`` 字段空 → 跳过该项, 不报错
  - 多个 target 位置匹配同一个人 (极少见, 同名) → 只报第一个 (按 position id 升序)

⚠️ **不动手的事**:

  - 不修改 ``target.py`` / ``liebiao.py`` / 任何已有文件
  - 不做 append 写入 liebiao (只读 + 比对)
  - 不引入 pytest 单测 (跟 task7 其他脚本同款)

跑法:
    python main/arm/each_task/task7/duiying.py
    python -m main.arm.each_task.task7.duiying
    python main/arm/each_task/task7/duiying.py --strict   # OCR 失败 → 报错
    python main/arm/each_task/task7/duiying.py --record-dir /tmp/ocr  # 自定义落盘目录
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

# 跨包 import 破例 (编排角色) — 见模块 docstring 顶部说明
from main.arm.each_task.task7 import target as task7_target  # noqa: E402
from main.arm.each_task.task6 import liebiao as task6_liebiao  # noqa: E402


# ---------- 常量 ----------

LOG_PREFIX: str = "[task7/duiying]"

# 默认 OCR 落盘目录 (与 target.py:RECORD_DIRNAME 一致)
DEFAULT_RECORD_DIR: str = os.path.join(os.path.expanduser("~"), ".remember/logs")


# ---------- 名字归一化 (轻校验,非语义级) ----------

def _normalize_name(raw: str) -> str:
    """轻量归一化 target OCR 输出的人名, 用于跟 liebiao 人名比对。

    策略:
      1. ``str.strip()`` 去前后空白 + 换行
      2. 全角空格 U+3000 → 半角空格
      3. 全角冒号 U+FF1A → 半角 ``:`` (防御 OCR 偶尔带括号/冒号)
      4. 全角逗号 U+FF0C → 半角 ``,``

    不做的:
      - 同音字消歧 (那是 wenzishibie 的事)
      - 中文繁简转换 (用户场景都是简体)
      - 标点删除 (保守起见保留标点)

    Args:
        raw: OCR 输出原始字符串 (可能含 ``None``, 此时返回 ``""``)

    Returns:
        归一化后字符串, 用于 ``==`` 比对。
    """
    if not raw:
        return ""
    s = str(raw).strip()
    s = s.replace("　", " ")   # 全角空格 → 半角
    s = s.replace("：", ":")   # 全角冒号 → 半角
    s = s.replace("，", ",")   # 全角逗号 → 半角
    return s.strip()


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner,
        record_dir: str = DEFAULT_RECORD_DIR,
        strict: bool = False) -> dict:
    """按用户语义: target 识别人名 → 跟 liebiao 两人名对应 → 命中输出。

    Args:
        client: ArmClient (传给 task7_target.run)
        runner: ArmRunner (传给 task7_target.run)
        record_dir: task7_target 的 OCR JSON 落盘目录
        strict: OCR 失败是否直接抛错 (默认 False 软警告继续)

    Returns:
        {
            "ok": bool,                              # 整体是否成功执行 (未必有匹配)
            "target_result": dict | None,            # task7_target.run() 原始返回
            "matches": list[dict],                   # 命中的 match 项
                                                    # [{source, list_no, name, goods,
                                                    #   target_id, target_label},
                                                    #   ...]
            "liebiao_loaded": bool,                  # _load_from_json 是否成功
            "liaobiao1_empty": bool,
            "liaobiao2_empty": bool,
        }

    Raises:
        RuntimeError: ``strict=True`` 且 target OCR 失败
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (target OCR + liebiao 比对) ==========")

    # ===== 1. 调 task7/target.run() 拿 6 个名字 =====
    print(f"\n  [step 1/3] 调 task7/target.run() 拿 6 个位置人名")
    try:
        target_result = task7_target.run(client, runner, record_dir=record_dir)
    except Exception as e:
        print(f"  {LOG_PREFIX} ❌ task7/target.run() 异常: {type(e).__name__}: {e}")
        if strict:
            raise RuntimeError(f"{LOG_PREFIX} strict 模式下 target.run() 失败: {e}") from e
        return {
            "ok": False,
            "target_result": None,
            "matches": [],
            "liebiao_loaded": False,
            "liaobiao1_empty": True,
            "liaobiao2_empty": True,
        }

    results = target_result.get("results", []) or []
    flat = target_result.get("flat", []) or []
    ocr_ok = bool(target_result.get("ok"))

    if not ocr_ok:
        print(f"  {LOG_PREFIX} ⚠️ target OCR 未识别到有效名字 (ok={ocr_ok})")
        if strict:
            raise RuntimeError(f"{LOG_PREFIX} strict 模式下 target OCR 失败")
    else:
        print(f"  {LOG_PREFIX} target OCR 成功, 6 位置识别:")
        for r in results:
            print(f"    [{r.get('id')}] [{r.get('label')}] {r.get('name')!r}")

    # ===== 2. 显式从 JSON 加载 liebiao (v3 进程隔离要求) =====
    print(f"\n  [step 2/3] task6/liebiao._load_from_json() 加载磁盘数据")
    loaded = task6_liebiao._load_from_json()
    print(f"  {LOG_PREFIX} liebiao 加载: ok={loaded}, "
          f"liaobiao1={len(task6_liebiao.liaobiao1)} 项, "
          f"liaobiao2={len(task6_liebiao.liaobiao2)} 项")

    liaobiao1_empty = len(task6_liebiao.liaobiao1) == 0
    liaobiao2_empty = len(task6_liebiao.liaobiao2) == 0

    if liaobiao1_empty:
        print(f"  {LOG_PREFIX} ⚠️ liaobiao1 空 (用户没跑 task6/target1.py 填数据), 跳过")
    if liaobiao2_empty:
        print(f"  {LOG_PREFIX} ⚠️ liaobiao2 空 (用户没跑 task6/target2.py 填数据), 跳过")

    # ===== 3. 比对 (按用户语义: 位置 1/2 + 人名 + 蔬菜 + target 位置 1-6) =====
    print(f"\n  [step 3/3] 比对 target 6 名字 ↔ liebiao 两人名")
    matches: list[dict] = []

    # 把 target results 按 id 升序排好, 保证多个匹配时只报第一个 (id 最小的)
    sorted_results = sorted(results, key=lambda r: (r.get("id") or 999))

    for source_label, source_list in (
        ("liaobiao1", task6_liebiao.liaobiao1),
        ("liaobiao2", task6_liebiao.liaobiao2),
    ):
        if not source_list:
            continue
        # 用户场景每条列表 1 项; 取 [0] 而不是遍历 (避免一对多歧义)
        entry = source_list[0]
        target_name = _normalize_name(entry.get("人名", ""))
        target_goods = str(entry.get("蔬菜", "")).strip()
        if not target_name:
            print(f"  {LOG_PREFIX} ⚠️ {source_label}[0] 人名为空, 跳过")
            continue
        # 在 6 个 target 名字里找 (按 id 升序, 第一个命中就 break)
        for r in sorted_results:
            r_name = _normalize_name(r.get("name") or "")
            if not r_name:
                continue
            if r_name == target_name:
                matches.append({
                    "source": source_label,
                    "list_no": 1 if source_label == "liaobiao1" else 2,
                    "name": target_name,
                    "goods": target_goods,
                    "target_id": r.get("id"),
                    "target_label": r.get("label"),
                })
                break  # 每个 liebiao 项最多匹配 1 个 target 位置

    # ===== 输出匹配结果 =====
    if not matches and (liaobiao1_empty and liaobiao2_empty):
        print(f"\n  {LOG_PREFIX} ⚠️ liebiao 双空 + 无匹配, 退出")
    elif not matches:
        # 检查是否因为 target OCR 没识别到
        any_name = any((r.get("name") for r in results))
        if not any_name:
            print(f"\n  {LOG_PREFIX} ⚠️ target OCR 没识别到任何名字, 跳过全部比对")
        else:
            print(f"\n  {LOG_PREFIX} (无匹配) — 6 个名字与 liebiao 两人名都不符")
    else:
        print(f"\n  ──── 匹配结果 ({len(matches)} 项) ────")
        for m in matches:
            print(
                f"  ✅ 符合 {m['source']}, "
                f"人名={m['name']}, 蔬菜={m['goods']}, "
                f"target 位置 {m['target_id']} ({m['target_label']})"
            )

    dt = time.time() - t0
    print(f"========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========\n")

    return {
        "ok": True,  # 整体流程跑通 (未必有匹配)
        "target_result": target_result,
        "matches": matches,
        "liebiao_loaded": loaded,
        "liaobiao1_empty": liaobiao1_empty,
        "liaobiao2_empty": liaobiao2_empty,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: --record-dir (覆盖 OCR JSON 落盘目录) + --strict (OCR 失败抛错)。"""
    p = argparse.ArgumentParser(
        description=(
            "task7 duiying: target 识别人名 ↔ task6 liebiao 两人名对应, "
            "命中输出位置 1/2 + 人名 + 蔬菜 + target 位置 1-6"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--record-dir", type=str, default=DEFAULT_RECORD_DIR,
        help="task7/target OCR JSON 落盘目录 (默认 $HOME/.remember/logs)",
    )
    p.add_argument(
        "--strict", action="store_true",
        help="严格模式: target OCR 失败 → 直接抛 RuntimeError (默认软警告继续走 liebiao 比对)",
    )
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner, record_dir=args.record_dir, strict=args.strict)
    return 0


if __name__ == "__main__":
    sys.exit(main())
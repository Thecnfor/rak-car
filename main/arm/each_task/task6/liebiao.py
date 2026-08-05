"""task6 / liebiao —— 位置 1 / 位置 2 的 (蔬菜, 人名) 列表数据 (v3 隔离版)。

数据结构:
  liaobiao1: List[dict] —— 位置 1 列表,每项 ``{"蔬菜": str, "人名": str}``
  liaobiao2: List[dict] —— 位置 2 列表,同上

⚠️ **v3 重大改动 (2026-08-04 用户现场要求)**: 不再 import 时自动从 JSON 加载!

  v2 行为 (废弃): 模块 import → 自动 ``_load_from_json()`` → 把磁盘上的历史数据塞进
                   内存列表。问题: 跨进程运行时, 上一次进程写入的数据会被下一次进程
                   自动加载, 出现 "target1 写的数据被 target2 看到" 这种串味。
  v3 行为 (现):
    1. **进程隔离**: ``import liebiao`` 后列表**永远是空的** (除非显式 ``--load``)。
                      每个进程跑完自己 append 的数据, 写到磁盘覆盖原文件,
                      不会污染下一次进程的 in-memory 状态。
    2. **写时直接覆盖**: ``append_liaobiao1/2`` / ``clear_*`` 都触发 ``_save_to_json()``,
                         用当前 in-memory 状态**整体覆盖**磁盘文件 (不 merge, 不 append)。
                         一次写入 = 一次 ``os.replace`` 原子覆盖。
    3. **显式加载**: CLI ``--load`` 标志才会从磁盘读 JSON 进 in-memory。
                      业务代码可以用 ``liebiao._load_from_json()`` 手动调。
    4. **JSON 文件 = 当前进程的快照**: 文件内容**仅反映最后写入它的进程**的
       in-memory 状态; 之后跑的进程不会"读到"它 (除非显式 ``--load``)。

  业务影响:
    - ``python target1.py`` → 内存 liaobiao1 增长 + 写盘覆盖。下一个 ``python liebiao.py``
                              不带 ``--load`` 看不到 (因为新进程空加载)。
    - ``python liebiao.py --load`` → 显式从文件读, 看到上一次 target1 写入的内容。
    - **target1/2 互不串味**: target2 进程 import 时列表为空, append_liaobiao2
                              只往 liaobiao2 加, 不读 liaobiao1。

⚠️ 持久化文件位置 (不变):
  - 默认: ``<task6_dir>/.liebiao.json`` (隐藏文件, 跟脚本同一目录)
  - 覆盖: 环境变量 ``LIEBIAO_JSON_PATH`` 或 CLI ``--json-path``

⚠️ 本文件本身**只存储和打印**, 不触发任何 OCR/机械臂流程。
   要填充数据, 必须跑 ``target1.py`` (→ liaobiao1) / ``target2.py`` (→ liaobiao2),
   或者跑 ``wenzishibie.py --target-list liaobiao1`` (单 OCR 调用直接入库)。
   CLI ``--fill-demo`` / ``--reset`` 提供演示数据和清空命令, 仅用于离线调试。

跑法:
    python liebiao.py                  # 打印当前进程内存列表 (默认空, 不读 JSON)
    python liebiao.py --load           # 先从 JSON 加载到内存, 再打印
    python liebiao.py --reset          # 清空两个列表 (写空 JSON)
    python liebiao.py --fill-demo      # 填入固定演示数据 (写 JSON)
    python liebiao.py --reset --fill-demo  # 先清空再填演示数据
    python liebiao.py --load --reset   # 先读 JSON 进来再清空 (罕见, 主要调试)
"""

import argparse
import json
import os
from pathlib import Path

# 位置 1 —— (蔬菜, 人名) 列表 (持久化到 .liebiao.json)
liaobiao1: list[dict] = []

# 位置 2 —— (蔬菜, 人名) 列表 (持久化到 .liebiao.json)
liaobiao2: list[dict] = []

# 持久化文件路径: <task6_dir>/.liebiao.json (默认)
# 隐藏文件, 跟脚本同目录, 跨进程自动共享
_LIEBIAO_JSON_FILENAME: str = ".liebiao.json"
_LIEBIAO_JSON_PATH: Path = Path(__file__).resolve().parent / _LIEBIAO_JSON_FILENAME
"""默认 JSON 路径, 可被环境变量 ``LIEBIAO_JSON_PATH`` 覆盖。"""

# 固定演示数据 (供 --fill-demo 使用, 与历史 OCR 实测校准一致)
_DEMO_DATA_LIAOBIAO1: list[tuple[str, str]] = [
    ("王五", "土豆"),       # 2026-08-04 用户实测 OCR (黑名单命中 + 软警告但接受)
    ("李四", "蘑菇"),       # 2026-08-04 用户实测 OCR
    ("诸葛亮", "番茄"),     # 2026-08-04 演示范例
]
_DEMO_DATA_LIAOBIAO2: list[tuple[str, str]] = [
    ("欧阳修", "西兰花"),   # 演示范例
    ("张三丰", "油菜"),     # 演范示例
]


# ---------- 持久化层 ----------

def _get_json_path() -> Path:
    """拿 JSON 文件路径, 优先 env LIEBIAO_JSON_PATH, fallback 默认 .liebiao.json。"""
    env_path = os.environ.get("LIEBIAO_JSON_PATH", "").strip()
    if env_path:
        return Path(env_path)
    return _LIEBIAO_JSON_PATH


def _save_to_json() -> None:
    """把 liaobiao1 + liaobiao2 写到 JSON 文件。原子写 (temp + replace)。"""
    path = _get_json_path()
    payload = {
        "liaobiao1": liaobiao1,
        "liaobiao2": liaobiao2,
        "version": 2,                       # v2 = 持久化版
    }
    tmp = path.with_suffix(path.suffix + ".tmp")
    try:
        tmp.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        # 原子替换 (POSIX 语义, Windows 也会尽量接近)
        os.replace(tmp, path)
    except Exception as exc:
        # 写盘失败不抛 (本地调试, 不阻塞业务流)
        print(f"  ⚠️ liebiao JSON 写盘失败 ({path}): {exc!r}")
        if tmp.exists():
            try:
                tmp.unlink()
            except Exception:
                pass


def _load_from_json() -> bool:
    """从 JSON 文件加载到 liaobiao1 + liaobiao2。返回是否成功加载。

    ⚠️ **v3 改动**: 此函数**不再**在模块 import 时自动调用!
       默认行为: ``import liebiao`` → 列表为空 (进程隔离)。
       显式调用方式:
         - CLI:  ``python liebiao.py --load`` (见 main())
         - 业务: ``from main.arm.each_task.task6.liebiao import _load_from_json``
                 然后手动 ``_load_from_json()`` (不推荐, 业务层一般不需要)

    - 文件不存在 → 跳过, 列表保持空, 返回 False
    - 文件存在但解析失败 / 不是 dict → 跳过, 列表保持空 (防御旧格式), 返回 False
    - 成功后用 ``liaobiao1.clear()`` + ``liaobiao1.extend(...)`` **整体覆盖**
      in-memory 内容 (clear 后 extend, 替换语义)
    """
    path = _get_json_path()
    if not path.exists():
        return False
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"  ⚠️ liebiao JSON 读盘失败 ({path}): {exc!r}")
        return False
    if not isinstance(raw, dict):
        return False
    l1 = raw.get("liaobiao1")
    l2 = raw.get("liaobiao2")
    if not isinstance(l1, list) or not isinstance(l2, list):
        return False
    # 用 clear + extend **整体覆盖** in-memory 内容 (v3 强调 "读取完直接覆盖掉原来的")
    liaobiao1.clear()
    liaobiao1.extend(l1)
    liaobiao2.clear()
    liaobiao2.extend(l2)
    return True


# ⚠️ v3 改动: **删除** 模块级自动加载。
# 旧逻辑: _LOADED_FROM_JSON: bool = _load_from_json()  ← 已删除
# 新逻辑: 进程 import 时列表永远是空的; 写时 ``_save_to_json()`` 直接覆盖磁盘文件,
#         不会被下一次进程自动读到 (除非显式 --load)。
# 副作用: 旧代码里有 ``_LOADED_FROM_JSON`` 引用的地方 (例如 main() 里的提示)
#         全部清理掉, 见下面 build_parser() / main()。


def append_liaobiao1(name: str, goods: str) -> dict:
    """追加一条 (人名, 蔬菜) 到 liaobiao1 (位置 1) + 自动写盘。

    Args:
        name: 客户姓名 (中文, 由 ``wenzishibie.run()`` 提取并校验过)
        goods: 食材名 (∈ 9 种白名单: 青椒/蘑菇/芹菜/番茄/油菜/豆角/西兰花/土豆/金针菇)

    Returns:
        添加的 record dict ``{"蔬菜": goods, "人名": name}``

    Side Effects:
        调用 ``_save_to_json()`` 把当前列表写到 ``.liebiao.json`` (持久化)。
    """
    record = {"蔬菜": goods, "人名": name}
    liaobiao1.append(record)
    _save_to_json()                                       # v2 新增: 持久化
    print(f"  ✅ liaobiao1.append[{len(liaobiao1)}]: 人名={name!r}  蔬菜={goods!r}")
    return record


def append_liaobiao2(name: str, goods: str) -> dict:
    """追加一条 (人名, 蔬菜) 到 liaobiao2 (位置 2) + 自动写盘。语义与 append_liaobiao1 完全一致。"""
    record = {"蔬菜": goods, "人名": name}
    liaobiao2.append(record)
    _save_to_json()                                       # v2 新增: 持久化
    print(f"  ✅ liaobiao2.append[{len(liaobiao2)}]: 人名={name!r}  蔬菜={goods!r}")
    return record


def clear_liaobiao1() -> None:
    """清空 liaobiao1 (默认不动 liaobiao2) + 自动写盘。"""
    liaobiao1.clear()
    _save_to_json()


def clear_liaobiao2() -> None:
    """清空 liaobiao2 (默认不动 liaobiao1) + 自动写盘。"""
    liaobiao2.clear()
    _save_to_json()


def clear_all() -> None:
    """清空 liaobiao1 + liaobiao2 + 自动写盘。"""
    liaobiao1.clear()
    liaobiao2.clear()
    _save_to_json()


def _print_list(name: str, data: list[dict]) -> None:
    """打印单个列表的 (蔬菜, 人名) 内容。"""
    print(f"=== {name} ({len(data)} 项) ===")
    if not data:
        print("  (空)")
        return
    for i, item in enumerate(data, 1):
        print(f"  [{i}] 蔬菜={item.get('蔬菜', '')!r}  人名={item.get('人名', '')!r}")


def _fill_demo() -> None:
    """把 _DEMO_DATA_* 固定演示数据 append 到两个列表 + 自动写盘。"""
    # 注意: 用替换语义 (clear 后 append), 不像真实 OCR 是 append-only
    liaobiao1.clear()
    liaobiao2.clear()
    print("⚠️ --fill-demo: 清空 + 填入固定演示数据 (王五/土豆 等) + 写盘")
    for name, goods in _DEMO_DATA_LIAOBIAO1:
        append_liaobiao1(name, goods)
    for name, goods in _DEMO_DATA_LIAOBIAO2:
        append_liaobiao2(name, goods)


def _reset() -> None:
    """清空两个列表 + 写盘。"""
    clear_all()
    print("⚠️ 已清空 liaobiao1 + liaobiao2 (写盘)")


def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: --load / --reset / --fill-demo。

    v3 行为:
      默认 (无参数): 仅打印**当前进程内存**的列表 (空, 不读 JSON)。
      ``--load``: 先从 ``.liebiao.json`` 显式加载到内存, 再走后续操作。
                  注意: ``--load`` 后任何 ``--reset`` / ``--fill-demo`` 都会作用在加载后的数据上。
    """
    p = argparse.ArgumentParser(
        description=(
            "task6 liebiao (v3 隔离版): 位置 1 / 位置 2 列表查看 / 清空 / 填演示数据; "
            f"默认不读 JSON, ``--load`` 显式加载; JSON 路径 {_get_json_path()}"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--load", action="store_true",
                   help=("v3 新增: 从 ``.liebiao.json`` 显式加载到内存 (默认不读, "
                         "进程隔离)。加载后, --reset / --fill-demo 作用在加载后的数据上。"))
    p.add_argument("--reset", action="store_true",
                   help="清空 liaobiao1 + liaobiao2")
    p.add_argument("--fill-demo", action="store_true",
                   dest="fill_demo",
                   help=("填入 _DEMO_DATA_* 固定演示数据 (王五/土豆 等), "
                         "会先清空当前列表"))
    p.add_argument("--json-path", default=None,
                   dest="json_path",
                   help=("覆盖 JSON 路径 (默认走 env LIEBIAO_JSON_PATH → "
                         f"模块默认 {_LIEBIAO_JSON_PATH})"))
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    # CLI --json-path 临时覆盖 (写到 env 让 _save_to_json 看到, 不污染默认)
    if args.json_path:
        os.environ["LIEBIAO_JSON_PATH"] = args.json_path

    # v3 新增: --load 显式从 JSON 加载到内存
    # 默认行为: 不加载 (进程隔离), 列表保持空。
    if args.load:
        ok = _load_from_json()
        if ok:
            total = len(liaobiao1) + len(liaobiao2)
            print(f"📂 从 {_get_json_path()} 加载了 {total} 条历史数据 (--load 显式)")
        else:
            print(f"📂 {_get_json_path()} 不存在或解析失败, 列表保持空")

    if args.reset and not args.fill_demo:
        _reset()
    elif args.fill_demo:
        _fill_demo()
    elif args.reset and args.fill_demo:
        _fill_demo()                       # fill_demo 内部先 clear, 等价于 reset

    _print_list("liaobiao1 (位置 1)", liaobiao1)
    print()
    _print_list("liaobiao2 (位置 2)", liaobiao2)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())

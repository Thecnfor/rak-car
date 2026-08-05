"""task6 / wenzishibie —— **任务六文字识别** (侧摄 cam2 + 文心大模型 ERNIE)。

按用户 2026-08-04 新建需求: 用 **侧摄 (cam2)** 拍订单牌, 调 **文心大模型 (ERNIE)**
直接提取想要的文字内容 (订单牌上的**客户名 + 想要的蔬菜**), 返回 JSON。

v2 (2026-08-04): 删掉 address/楼号字段 — 用户要求"不用是几号楼, 只需要人名和蔬菜"。

跟现有 ``main/misc/test_order_read.py`` 的区别:
  - test_order_read.py 是 **main/misc** 下的旧通用版, 依赖 ``main.misc.llm_config.yml``
    + ``main.api_client.RuntimeApiClient`` (走 runtime /v1 接口再转发到 cam2),
    是历史遗留路径。
  - 本 wenzishibie.py 是 **main/arm/each_task/task6/** 下的新版, 直接 HTTP 拉
    ``stream_url/frame/cam2.jpg`` + 直接 POST ERNIE, 不走 runtime 中转,
    跟 task6/tuigan.py 同款"自包含 + 直接调外部 HTTP"风格。

跑法 (按 car 默认 host=192.168.5.230):
    python main/arm/each_task/task6/wenzishibie.py
    python -m main.arm.each_task.task6.wenzishibie
    python main/arm/each_task/task6/wenzishibie.py --cam cam1          # 改用前摄
    python main/arm/each_task/task6/wenzishibie.py --retries 10        # 多试几次
    python main/arm/each_task/task6/wenzishibie.py --save-frame        # 帧存盘 (调试)

环境变量:
    RAK_CAR_SERVER_ORIGIN / RAK_CAR_STREAM_PORT / RAK_CAR_STREAM_PATH
    → 覆盖 streamer_url (默认 http://192.168.5.230:5050/stream/)
    ERNIE_ACCESS_TOKEN
    → 覆盖 yaml 里的 token (优先 yaml → env)

依赖:
    main.settings.load_settings  → streamer_url
    main/../config_car.yml        → ernie_access_token (旧版 token 兜底)
    main/misc/llm_config.yml      → ernie.access_token (新版 token 优先)

⚠️ **本文件自包含** (与 task6/tuigan.py、task7/{position1-6,get_position1/2}.py 同款):
   只依赖 ``main.settings`` + 标准库 + requests + yaml,
   不 import task6 包内任何模块。原因: task5 包曾被外部清空过一次
   (见 [[task5-rebuild-2026-07-22]]), 自包含可保证 ``python wenzishibie.py``
   直接跑不受影响。
"""
from __future__ import annotations

import argparse
import base64
import datetime
import json
import logging
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

# 让 main.settings 能被 import (项目根目录加入 sys.path)
_ROOT = Path(__file__).resolve().parents[4]   # .../main/arm/each_task/task6 → 项目根
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import re

import requests
import yaml

from main.settings import load_settings  # noqa: E402

# v6 加 (2026-08-04): OCR 成功后自动写进 liebiao, 让 wenzishibie 直接负责"识别 + 入库"
# liebiao.py 是 task6 同包姐妹文件, 无循环依赖 (liebiao 只依赖 stdlib)。
from main.arm.each_task.task6.liebiao import append_liaobiao1, append_liaobiao2  # noqa: E402


# ---------- v6: target_list 自动入库 helper ----------

def _maybe_append_to_target_list(
    target_list: Optional[str], name: str, goods: str,
) -> tuple[Optional[str], Optional[dict]]:
    """v6 加: ``target_list`` 指定时, 调对应 ``append_liaobiaoN(name, goods)`` 入库。

    Args:
        target_list: ``"liaobiao1"`` / ``"liaobiao2"`` / ``None``。
                    非法值 (非 None 非上述两个之一) 抛 ``ValueError``。
        name: 校验通过的姓名
        goods: 校验通过的食材

    Returns:
        ``(appended_to, appended_record)``
        - appended_to: ``"liaobiao1"`` / ``"liaobiao2"`` / ``None``
        - appended_record: appender 返回的 ``{"蔬菜": ..., "人名": ...}`` 或 ``None``

    Raises:
        ValueError: target_list 非 None 也非 ``"liaobiao1"`` / ``"liaobiao2"``。
    """
    if target_list is None:
        return None, None
    if target_list == "liaobiao1":
        record = append_liaobiao1(name, goods)
        return "liaobiao1", record
    if target_list == "liaobiao2":
        record = append_liaobiao2(name, goods)
        return "liaobiao2", record
    raise ValueError(
        f"target_list 必须是 None / 'liaobiao1' / 'liaobiao2', 收到 {target_list!r}"
    )


# ---------- 序列常量 (用户 2026-08-04 指定) ----------

LOG_PREFIX: str = "[task6/wenzishibie]"

ERNIE_CHAT_URL = "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions"
"""ERNIE 大模型 chat 端点 (aistudio v3 协议)。"""

# 可能的 token 源文件 (优先级: env > main/misc/llm_config.yml > config_car.yml)
_LLM_CONFIG_PATH: Path = _ROOT / "main" / "misc" / "llm_config.yml"
_CONFIG_CAR_PATH: Path = _ROOT / "config_car.yml"


DEFAULT_CAM: str = "cam2"
"""默认相机 = **cam2 = 侧摄** (用户 2026-08-04 指定)。

⚠️ 注意: config_car.yml 写 ``side:1`` 是 video device, 但 stream URL
   ``/stream/frame/cam{N}.jpg`` 里 ``side`` 对应 ``cam2``, 两套编号独立。
   task5/target.py 的 ``DEFAULT_CAM = "cam2"`` 同款约定。
   改用前摄用 ``--cam cam1``。"""

DEFAULT_MODEL: str = "ernie-4.5-turbo-vl"
"""默认 ERNIE 模型 (多模态, OCR 友好)。

⚠️ **2026-08-04 现场实测**: ``ernie-4.5-vl-28b-a3b`` (PPT Slide 64 推荐)
   报 ``errorCode:40405 暂不支持该模型`` (aistudio v3 端点不支持) →
   改成 ``ernie-4.5-turbo-vl`` (跟 main/misc/llm_config.yml:6 + test_order_read.py:54
   同款), 实测可正常调通。
⚠️ 脚本启动时还会从 ``main/misc/llm_config.yml:ernie.model`` 读覆盖,
   优先级: CLI ``--model`` > yml ``ernie.model`` > 本常量。"""

DEFAULT_TIMEOUT_S: float = 15.0
"""ERNIE 单次 HTTP POST 超时秒。"""

DEFAULT_MAX_RETRIES: int = 5
"""最大尝试次数 (含首次), 默认 5 次。"""

DEFAULT_POLL_INTERVAL_S: float = 2.0
"""两次尝试之间的等待秒。"""

DEFAULT_PROMPT: str = """你是订单识别助手, 任务是准确读取订单牌上的 **客户姓名** 和 **需要配送的食材**。

⚠️ **关键理解** (容易出错的地方):

- **客户姓名** (name): 通常是 **2-4 个汉字的中文人名**。例如: 张三、李四、王五、张三丰。
  - **不要** 把 "配送到 张三家" 中的 "家" 当成姓名的一部分 (家 = 收货地址标记, 不是人名)。
  - **不要** 把姓 + 名中间的标点 (顿号/逗号/空格) 当成姓名一部分。
  - **不要** 输出称呼 (先生/女士/老师/师傅 等)。
  - **不要** 输出数字 / 英文 / 邮箱 / 电话。
  - 如果订单牌上有多个名字 (主客户 + 备用), 只取 **第一个** 主客户名。

- **食材** (goods): **必须是以下 9 种之一, 不允许其他任何值**:
  ["青椒", "蘑菇", "芹菜", "番茄", "油菜", "豆角", "西兰花", "土豆", "金针菇"]
  - **番茄 = 西红柿**, 订单写 "西红柿" / "番茄" / "tomato" 都输出 "番茄"。
  - **不要** 输出复数 ("番茄" 不要写成 "番茄们"), 也不要输出量词 (个/只/根/把)。
  - 如果订单写多个菜 (例: "番茄和土豆"), 只取 **第一个** 主菜。

🚫 **绝对禁止: 编造 (hallucination)**:
- **看不清 / 没看清 / 图模糊** → 必须输出 `{"raw_text": "<所有可见文字>"}`, **不要编造姓名或食材**。
- **常见 LLM 幻觉陷阱** (绝不要默认输出): "张三" / "李四" / "王五" / "赵六" / "钱七" / "孙八" / "周九" / "吴十" / "小明" / "小红" / "客户" / "用户"。
  - 如果你"觉得"是这些名字, **说明你没看清**, 请返回 raw_text, 不要猜。
- **典型幻觉食材** (绝不要默认输出): "番茄" + 任意固定组合。这些是 LLM 默认猜测, 不是真实订单。
- 宁可返回 raw_text 让用户重拍, 也不要输出看起来合理但实际是编造的内容。

📋 **输出格式** (严格遵守, 错一个字段就 invalid):
- 必须输出 JSON 对象, **只含 2 个字段**: `name` + `goods`。
- **不要** 包含 address / 楼号 / 思考过程 / 解释 / Markdown 标记 / 任何其他字段。
- 如果图片没有订单信息 (模糊/错误), 返回: `{"raw_text": <所有可见文字>}`

📋 **示例** (格式参考, 不是真实订单):

示例 1 (标准 2 字名 + 蔬菜):
  图片: "张三 番茄 1号楼"
  输出: `{"name": "张三", "goods": "番茄"}`

示例 2 (3 字名 + 别名西红柿):
  图片: "欧阳修 西红柿 2号楼"
  输出: `{"name": "欧阳修", "goods": "番茄"}`

示例 3 (复杂 4 字名 + 多菜):
  图片: "诸葛亮 番茄和土豆 1号楼"
  输出: `{"name": "诸葛亮", "goods": "番茄"}`

示例 4 (图片模糊, **不准编造**):
  图片: 模糊看不清楚
  输出: `{"raw_text": "<所有可见但辨认不清的文字>"}`  ← 不要编一个"看起来对"的答案

现在请处理这张订单牌图片, **只输出 JSON, 不要其他任何内容**。
"""
"""默认 prompt (v3 加强语义理解 + v4 加强反幻觉, 2026-08-04 加): 提取订单牌文字。

⚠️ v4 (2026-08-04 晚上) 加强反幻觉:
   1. 加 🚫 绝对禁止编造 段
   2. 列出 LLM 默认猜测的常见人名 (张三/李四/赵六等), 警告 LLM 不要默认输出
   3. 强调"看不清必须返回 raw_text, 不要猜"
   4. 加示例 4 演示模糊图片的正确输出 (raw_text 而不是编答案)
⚠️ v3 改动: 加 few-shot 4 例 + 强调语义 (name 2-4 字中文, goods 9 种之一)
⚠️ v2 改动: 删 address 字段 (用户: "不用是几号楼, 只需要人名和蔬菜")
⚠️ goods 白名单 9 种 (跟 llm_config.yml:50 同款), 校验函数会在 ERNIE 返回后二次校验
"""

# 备用更严格 prompt (校验失败时第二次尝试用)
STRICT_PROMPT: str = """你是订单识别助手。这是一次**更严格的重新识别** (上次失败)。

🚫 **关键: 上次识别很可能是 LLM 幻觉** (编了一个看起来合理但实际看不清的答案)。
**这次必须更谨慎**: 看不清就返回 raw_text, **绝对不要**编造姓名或食材。

⚠️ **上次失败原因**: 你的回答不符合业务规范。请严格按以下要求重新输出:

1. **name 必须是 2-4 个汉字的中文人名** (例: 欧阳修、诸葛亮、张三丰)。
   - **绝对不要** 输出这些 LLM 默认猜测 (高度怀疑是幻觉): 张三、李四、王五、赵六、钱七、孙八、周九、吴十、小明、小红、客户、用户、姓名、名字、某人、某先生
   - 如果看不清是不是其中之一, 直接返回 raw_text
   - 错误示范 (不要输出): "张三先生" / "张三 1" / "张三家" / "Mr. Zhang" / "赵六"
   - 正确示范: `{"name": "欧阳修"}`

2. **goods 必须严格是这 9 个值之一** (不允许任何其他文字):
   青椒 / 蘑菇 / 芹菜 / 番茄 / 油菜 / 豆角 / 西兰花 / 土豆 / 金针菇
   - 错误示范 (不要输出): "番茄1个" / "西红柿" / "番茄土豆"
   - 正确示范: `{"goods": "番茄"}`  ← 西红柿也要写成番茄

3. **只输出 JSON, 2 个字段**: `{"name": "...", "goods": "..."}`
   - 不要任何解释 / 不要 Markdown / 不要换行 / 不要多余字段

4. 如果图片无法识别, 输出: `{"raw_text": "<所有可见文字>"}`  ← 宁可让用户重拍, 也不要编
"""
"""v3 (2026-08-04) 加: 校验失败时第二次尝试用的"更严格"prompt。

⚠️ v4 改动: 加强反幻觉警告
   1. 列出 LLM 默认猜测人名黑名单 (张三/李四/赵六等)
   2. 强调"上次失败很可能是幻觉, 这次必须更谨慎"
   3. 强调"看不清就 raw_text, 绝对不要编造"
⚠️ 触发条件: ``_validate_order_result`` 返回 invalid (含黑名单命中) → run() 自动用本 prompt 重试一次。
"""

# 食材白名单 (订单 goods 字段, 跟 llm_config.yml:50 valid_goods 同款)
DEFAULT_VALID_GOODS: tuple = (
    "青椒", "蘑菇", "芹菜", "番茄", "油菜", "豆角", "西兰花", "土豆", "金针菇",
)
"""订单 goods 字段白名单 (9 种食材)。

⚠️ 跟 main/misc/llm_config.yml:50 的 ``order_read.valid_goods`` 完全一致:
   ["青椒", "蘑菇", "芹菜", "番茄", "油菜", "豆角", "西兰花", "土豆", "金针菇"]
⚠️ 注意: 番茄 = 西红柿, 订单写"西红柿"时 goods 输出"番茄"。
⚠️ 校验逻辑: ERNIE 返回后, ``_validate_order_result`` 会校验 goods 必须 ∈ 此
   tuple, 否则 valid=False。
⚠️ CLI ``--valid-goods`` 可现场覆盖 (逗号分隔), 一般不需要改。"""

# v2 已删除: 订单只需要 name + goods, 不再需要楼号校验

# v4 (2026-08-04 晚上) 加: LLM 默认猜测人名黑名单
# 当 ERNIE 返回的 name 是这些之一时, 高度怀疑是 LLM 幻觉 (没看清图但编了个常见名字),
# 业务侧视为 invalid → 触发 STRICT_PROMPT 重试 + 下帧重试。
LLM_GUESS_NAMES: frozenset = frozenset({
    # 2 字名 (LLM 最常编)
    "张三", "李四", "王五", "赵六", "钱七", "孙八", "周九", "吴十",
    "郑一", "王二", "冯三", "陈四", "褚五", "卫六", "蒋七", "沈八",
    # 3 字常见 (LLM 第二常编)
    "张三丰", "李小白", "王小明", "张小明", "李小明",
    # 2 字 "小X" 模式 (常见编造)
    "小明", "小红", "小华", "小丽", "小芳", "小军", "小强", "小伟",
    # 通用占位词 (LLM 偶尔编)
    "客户", "用户", "顾客", "买家", "收货人",
    "姓名", "名字", "某人", "某先生", "某女士", "某人名",
})
"""v4 (2026-08-04 晚上) 加: LLM 默认猜测人名黑名单。

⚠️ 当 ERNIE 返回的 name 命中此 frozenset, 视为**低置信度 = LLM 幻觉**,
   ``_validate_order_result`` 会返回 valid=False, 触发 STRICT_PROMPT 重试。
⚠️ 这是 v4 加的"动态防御": 即使 prompt 写得再清楚, LLM 仍可能编这些常见名字,
   业务层必须有兜底识别机制。
⚠️ 现场如果 LLM 仍编出其他幻觉名字 (不在此集合), 可以直接加到本集合里。
⚠️ 注意: 这是**单方面黑名单**, 不影响正常识别 — 真实订单名字 (如 "诸葛亮", "欧阳修")
   不在黑名单里, 不会误判。
"""

# 帧保存
DEFAULT_SAVE_DIRNAME: str = "wenzishibie_frames"


# ---------- logger ----------

logger = logging.getLogger("task6.wenzishibie")


# ---------- token 加载 (env → llm_config.yml → config_car.yml) ----------

def _load_token_from_env() -> Optional[str]:
    """从环境变量 ERNIE_ACCESS_TOKEN 加载。"""
    t = os.getenv("ERNIE_ACCESS_TOKEN", "").strip()
    return t or None


def _load_token_from_yaml(path: Path) -> Optional[str]:
    """从 yaml 文件加载, 支持多种 key 路径:
      - llm.ernie.access_token  (main/misc/llm_config.yml)
      - ernie.access_token      (新版简化)
      - ernie_access_token      (config_car.yml 老格式)
    """
    if not path.exists():
        return None
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        logger.debug("读 yaml 失败 %s: %s", path, exc)
        return None
    candidates = [
        ("llm", "ernie", "access_token"),
        ("ernie", "access_token"),
        ("ernie_access_token",),
    ]
    for keys in candidates:
        cur = data
        for k in keys:
            if not isinstance(cur, dict) or k not in cur:
                cur = None
                break
            cur = cur[k]
        if cur and isinstance(cur, str) and cur.strip() and cur.strip() != "REPLACE_YOUR_ACCESS_TOKEN_HERE":
            return cur.strip()
    return None


def _load_ernie_cfg() -> Dict[str, Any]:
    """从 ``main/misc/llm_config.yml`` 读 ernie 段 (model / top_p / temperature / timeout_s)。

    缺字段 fallback 到 DEFAULT_* 常量; 整个文件缺 → 全 DEFAULT。

    Returns:
        {"model": str, "top_p": float, "temperature": float, "timeout_s": float, "source": str}
        其中 source = "llm_config.yml" / "defaults"
    """
    defaults: Dict[str, Any] = {
        "model": DEFAULT_MODEL,
        "top_p": 0.1,
        "temperature": 0.1,
        "timeout_s": DEFAULT_TIMEOUT_S,
        "source": "defaults",
    }
    if not _LLM_CONFIG_PATH.exists():
        return defaults
    try:
        data = yaml.safe_load(_LLM_CONFIG_PATH.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        logger.debug("读 llm_config.yml 失败: %s", exc)
        return defaults
    ernie_cfg = data.get("ernie") if isinstance(data, dict) else None
    if not isinstance(ernie_cfg, dict):
        return defaults
    for key in ("model", "top_p", "temperature", "timeout_s"):
        if key in ernie_cfg and ernie_cfg[key] is not None:
            defaults[key] = ernie_cfg[key]
    defaults["source"] = "llm_config.yml"
    return defaults


def _load_token() -> str:
    """加载 ERNIE access token, 优先级: env > llm_config.yml > config_car.yml。"""
    t = _load_token_from_env()
    if t:
        logger.info("token 来源: env ERNIE_ACCESS_TOKEN")
        return t
    t = _load_token_from_yaml(_LLM_CONFIG_PATH)
    if t:
        logger.info("token 来源: %s", _LLM_CONFIG_PATH.relative_to(_ROOT))
        return t
    t = _load_token_from_yaml(_CONFIG_CAR_PATH)
    if t:
        logger.info("token 来源: %s", _CONFIG_CAR_PATH.relative_to(_ROOT))
        return t
    raise RuntimeError(
        "未找到 ERNIE access token, 请设置 env ERNIE_ACCESS_TOKEN "
        "或在 main/misc/llm_config.yml (llm.ernie.access_token) 或 "
        "config_car.yml (ernie_access_token) 中填写"
    )


# ---------- 侧摄拉帧 ----------

def _fetch_side_frame(stream_url: str, cam: str = DEFAULT_CAM,
                      timeout: float = DEFAULT_TIMEOUT_S) -> Optional[bytes]:
    """从 streamer HTTP 端拉一帧 JPEG 字节。

    URL 形式: ``{stream_url.rstrip('/')}/frame/{cam}.jpg``
    失败 (网络/超时/非 200) 返回 None, 不抛。
    """
    url = f"{stream_url.rstrip('/')}/frame/{cam}.jpg"
    try:
        r = requests.get(url, timeout=timeout)
        r.raise_for_status()
        return r.content
    except requests.Timeout:
        print(f"  ⚠️ 拉帧超时 ({timeout}s): {url}", file=sys.stderr)
        return None
    except requests.RequestException as exc:
        print(f"  ⚠️ 拉帧失败: {exc}", file=sys.stderr)
        return None
    except Exception as exc:                              # noqa: BLE001
        print(f"  ⚠️ 拉帧未知错误: {exc}", file=sys.stderr)
        return None


# ---------- ERNIE 调用 ----------

def _strip_code_fence(text: str) -> str:
    """去除 ```json ... ``` 包裹, 返回裸文本。"""
    s = text.strip()
    if not s.startswith("```"):
        return s
    lines = s.splitlines()
    # 去掉首行 ```json / ```
    if lines and lines[0].lstrip().startswith("```"):
        lines = lines[1:]
    # 去掉末行 ```
    if lines and lines[-1].lstrip().startswith("```"):
        lines = lines[:-1]
    return "\n".join(lines).strip()


def _validate_order_result(
    parsed: Dict[str, Any],
    valid_goods: tuple = DEFAULT_VALID_GOODS,
    strict_llm_guess: bool = False,
) -> Dict[str, Any]:
    """校验 ERNIE 返回的订单 JSON, 强制 name/goods 字段合法。

    v2 (2026-08-04): 删掉 address 校验 — 用户决定"不用是几号楼, 只需要人名和蔬菜"。
    v4 (2026-08-04): 加 strict_llm_guess 黑名单 (默认 False, CLI opt-in)。
    v5 (2026-08-04): **拆 errors (硬) / warnings (软)** — 见下。

    校验规则:
      - name   : 非空字符串; 像中文人名 (2-6 纯汉字, 无称呼/后缀)
      - goods  : ∈ valid_goods (默认 9 种食材)
      - 黑名单 (默认模式):   命中 → warnings, valid 不变 (建议人工复核, 不阻塞)
      - 黑名单 (严格模式):   命中 → errors, valid=False (触发 STRICT retry)

    ⚠️ **v5 关键修复**: 之前默认模式软警告也进 errors → ``len(errors) > 0`` →
       ``valid=False`` → run() 触发不必要的 STRICT retry → STRICT 又给嵌套 raw_text
       → 结果更糟 (用户实测 2026-08-04 "王五/土豆" 识别正确但走 STRICT 后报废)。
       修复: 默认模式黑名单命中走 warnings 不进 errors, valid 保持 True。

    Args:
        parsed: ERNIE 返回的 JSON dict (已经 JSON 解析过)
        valid_goods: goods 字段白名单
        strict_llm_guess: 是否启用 LLM 幻觉黑名单 (默认 False, 避免误伤真实合法识别)

    Returns:
        {
            "valid":    bool,        # 全部字段硬性合法才 True (warnings 不影响)
            "name":     str | None,
            "goods":    str | None,
            "errors":   list[str],   # 硬错误 → valid=False, 触发 STRICT retry
            "warnings": list[str],   # 软警告 → valid 不变, 仅打印 (v5 新增)
        }
    """
    if not isinstance(parsed, dict):
        return {
            "valid": False, "name": None, "goods": None,
            "errors": [f"parsed 不是 dict: {type(parsed).__name__}"],
            "warnings": [],
        }

    name = parsed.get("name")
    goods = parsed.get("goods")

    errors: list = []
    warnings: list = []                            # v5 新增: 软警告, 不影响 valid

    # name: v3 加强语义校验 — 必须像中文人名 (2-6 纯汉字, 不带称呼/后缀)
    if not isinstance(name, str) or not name.strip():
        errors.append(f"name 缺失或非字符串: {name!r}")
        name_out = None
    else:
        raw_name = name.strip()
        # v4 加: LLM 默认猜测人名黑名单 (幻觉防御)
        # v5 修复: 默认模式软警告走 warnings, 不进 errors, valid 保持 True
        #         只有 strict_llm_guess=True 时才进 errors 判 invalid
        #         原因: 真实订单名有可能恰好等于这些名字, 不能误伤
        llm_guess_hit = raw_name in LLM_GUESS_NAMES
        if llm_guess_hit:
            guess_msg = (f"name {raw_name!r} 在 LLM 默认猜测黑名单中 "
                         f"(可能是幻觉, 也可能是真实合法名字)")
            if strict_llm_guess:
                errors.append(guess_msg + " — 严格模式视为 invalid")
                name_out = None
            else:
                # v5: 默认模式软警告 (从 errors 移到 warnings, 不影响 valid)
                warnings.append(guess_msg + " — 默认模式接受 (建议人工复核)")
                name_out = raw_name
        elif _is_chinese_name(raw_name):
            name_out = raw_name
        else:
            # 尝试归一化 (剥称呼/后缀)
            normalized = _normalize_name(raw_name)
            if normalized:
                # v4 加: 归一化结果也要查黑名单 (v5 同样拆 errors/warnings)
                if normalized in LLM_GUESS_NAMES:
                    gmsg = (f"name 归一化结果 {normalized!r} 在 LLM 默认猜测黑名单中 "
                            f"(可能是幻觉)")
                    if strict_llm_guess:
                        errors.append(gmsg + " — 严格模式视为 invalid")
                        name_out = None
                    else:
                        # v5: 软警告
                        warnings.append(gmsg + " — 默认模式接受 (建议人工复核)")
                        name_out = normalized
                else:
                    name_out = normalized
                    # v5: 归一化成功是软警告 (从 errors 移到 warnings)
                    warnings.append(
                        f"name 原始 {raw_name!r} 不像人名, 已归一化为 {normalized!r}"
                    )
            else:
                # 归一化失败 → 硬错误, valid=False
                errors.append(
                    f"name 不像中文人名 (需要 2-6 个纯汉字, 无称呼/后缀): {raw_name!r}"
                )
                name_out = None

    # goods: 白名单 (9 种食材) + v3 多菜归一化
    if not isinstance(goods, str) or goods not in valid_goods:
        # v2 (2026-08-04) 归一化兜底: 即使 LLM 输出 "番茄/金针菇(...)" 这种
        # 多菜混写 + 括号解释, 业务侧尝试按 "/" / "、" / "," / "或" 拆分取首项,
        # 再 strip 空白/中文括号/英文括号, 看子串里是否含 9 种菜之一。
        normalized = _normalize_goods_to_whitelist(goods, valid_goods) if isinstance(goods, str) else None
        if normalized:
            goods_out = normalized
            # v5: 归一化成功是软警告
            warnings.append(f"goods 归一化: {goods!r} → {normalized!r}")
        else:
            errors.append(f"goods 不在白名单 {list(valid_goods)}: {goods!r}")
            goods_out = None
    else:
        goods_out = goods

    return {
        "valid": len(errors) == 0,             # v5: valid 只看 errors, 不看 warnings
        "name": name_out,
        "goods": goods_out,
        "errors": errors,
        "warnings": warnings,                  # v5 新增: 软警告, 影响 print 不影响 valid
    }


def _call_ernie(
    token: str,
    image_b64: str,
    prompt: str,
    model: str = DEFAULT_MODEL,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    top_p: float = 0.1,
    temperature: float = 0.1,
) -> Dict[str, Any]:
    """调 ERNIE 多模态 chat 接口 (aistudio v3 协议)。

    top_p / temperature / timeout_s 都可由调用方传入 (run() 从 yml 读)。
    Returns:
        {"ok": True, "parsed": dict, "raw": str}      ← 成功
        {"error": str}                                ← 失败
    """
    body = {
        "model": model,
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
        ]}],
        "top_p": top_p,
        "temperature": temperature,
    }
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
        "x-bce-date": datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    try:
        resp = requests.post(ERNIE_CHAT_URL, headers=headers, json=body, timeout=timeout_s)
    except requests.Timeout:
        return {"error": f"timeout ({timeout_s}s)"}
    except requests.RequestException as exc:
        return {"error": f"network: {exc}"}

    if resp.status_code in (401, 403):
        return {"error": f"auth {resp.status_code} (token 无效?)"}
    if not resp.ok:
        return {"error": f"HTTP {resp.status_code}: {resp.text[:200]}"}

    try:
        content = resp.json()["choices"][0]["message"]["content"]
    except Exception as exc:                            # noqa: BLE001
        return {"error": f"parse response: {exc}"}

    text = _strip_code_fence(content)

    # v2 (2026-08-04) 改用 _extract_order_dict 多层 fallback:
    #   1) text 本身就是合规 JSON dict (含 name / goods)
    #   2) text 是 dict 但仅含嵌套 raw_text 字段, raw_text 内部再 parse
    #   3) text 里嵌套多个 JSON 块 (LLM thinking 后的多次尝试), 找含 name/goods 的那块
    #   4) 都失败 → {"raw_text": text} 当兜底 (继续走 _validate 走 raw_text 路径)
    parsed = _extract_order_dict(text)
    return {"ok": True, "parsed": parsed, "raw": content}


# v2 (2026-08-04) 加: LLM 经常把 effective JSON 嵌套在 raw_text 字符串里,
# 业务侧必须能自动剥出来。4 层 fallback 见下面 docstring。
def _extract_order_dict(text: str) -> Dict[str, Any]:
    """从 LLM 返回的 content 抽取 order dict, 多层 fallback 防止 raw_text 嵌套。

    LLM 现场实测会输出以下几种结构 (2026-08-04 5 次试跑抓到的样本):

      类型 A — 标准:        ``{"name": "钱七", "goods": "青椒"}``
      类型 B — 抽到 raw_text (LLM 拒绝识别图):
                               ``{"raw_text": "作为一个人工智能语言模型..."}``
      类型 C — **嵌套** (本次报错根因):
                               ``{"raw_text": "{\\"name\\": \\"赵六\\", \\"goods\\": \\"番茄\\" ...}"}``
      类型 D — 多块 JSON (LLM thinking 后输出多个 JSON):
                               ``{"raw_text_check_step": "..."} {"name": ..., "goods": ...} {"raw_text_final": "..."}``

    本函数把这 4 类都尽量归到最终 ``{"name": ..., "goods": ...}``。

    Args:
        text: 已 _strip_code_fence 处理过的 LLM 输出

    Returns:
        优先返回含 ``name`` 或 ``goods`` 的 dict; 都失败 → ``{"raw_text": text}``。
    """
    text = (text or "").strip()
    if not text:
        return {"raw_text": text}

    # 1) text 本身是合法 JSON dict 且含 name / goods → 直接返回
    try:
        d = json.loads(text)
        if isinstance(d, dict) and (d.get("name") or d.get("goods")):
            return d
        # 1b) text 是 dict 但只有 raw_text 字段, raw_text 是字符串
        #     嵌套了另一个 JSON dict → 二次 parse
        if isinstance(d, dict) and isinstance(d.get("raw_text"), str):
            inner_text = d["raw_text"].strip()
            try:
                d2 = json.loads(inner_text)
                if isinstance(d2, dict) and (d2.get("name") or d2.get("goods")):
                    return d2
            except (json.JSONDecodeError, ValueError):
                pass
    except json.JSONDecodeError:
        pass

    # 2) text 里嵌套多个 { ... } JSON 块 (LLM thinking 后输出多版本),
    #    找含 name / goods 的那个 — 用 regex 抽非嵌套块的候选
    candidates = re.findall(r"\{[^{}]*\"name\"[^{}]*\"goods\"[^{}]*\}", text)
    if not candidates:
        # 兜底: 抽所有 {...} 块, 不限内容
        candidates = re.findall(r"\{[^{}]*\}", text)
    for cand in reversed(candidates):  # 反向扫, 通常最后一个是最终答案
        try:
            d = json.loads(cand)
            if isinstance(d, dict) and (d.get("name") or d.get("goods")):
                return d
        except json.JSONDecodeError:
            continue

    # 3) 全失败 → 退回原行为 (兜底 raw_text)
    return {"raw_text": text}


def _normalize_goods_to_whitelist(goods: str, valid_goods: tuple) -> Optional[str]:
    """把 goods 字符串归一化到 valid_goods 白名单, 多层 fallback。

    当 LLM 输出 `"番茄/金针菇（题目要求只输出规定的9种食材之一...）"` 这种
    多菜混写 + 中文括号 + 解释 时, 业务尝试:

      1. 按 `/` / `、` / `,` / `或` 拆分 → 取第一项
      2. strip 中英文括号 + 空白
      3. 看首项是否 ∈ valid_goods
      4. 否则: 扫描整字符串 (子串匹配), 找到**任意**白名单成员 → 取那一个

    Returns:
        找到白名单成员 → 该成员名; 都没找到 → None
    """
    if not isinstance(goods, str):
        return None
    s = goods.strip()
    if not s:
        return None

    # 1) 取首段
    head = re.split(r"[/、,，\s]+", s, maxsplit=1)[0].strip()
    # 2) strip 中英文括号 + 周围空白
    head_clean = re.sub(r"[()（）\[\]【】]", "", head).strip()
    if head_clean in valid_goods:
        return head_clean

    # 3) 子串扫描 — 在整个 goods 字符串里找白名单成员
    for v in valid_goods:
        if v in s:
            return v
    return None


# v3 (2026-08-04) 加: name 语义校验 — 防止 LLM 输出 "张三先生" / "张三家" / "Mr. Zhang"
def _is_chinese_name(name: str) -> bool:
    """判断字符串是否像中文人名 (v3 加强语义校验)。

    规则:
      1. 必须是 2-6 个**纯汉字** (Unicode 范围 \\u4e00-\\u9fff)
      2. 不能含数字 / 英文 / 中文标点 / 空白 / 称呼词
      3. 不能以常见后缀结尾 ("家" / "先生" / "女士" / "老师" / "师傅" / "总" / "经理" / "主任")

    Args:
        name: 待校验字符串

    Returns:
        True  → 像中文人名
        False → 不像 (例如: "张三先生" / "张三家" / "Mr. Zhang" / "张三 1")

    Examples:
        >>> _is_chinese_name("张三")
        True
        >>> _is_chinese_name("欧阳修")
        True
        >>> _is_chinese_name("张三先生")
        False
        >>> _is_chinese_name("张三家")
        False
        >>> _is_chinese_name("Mr. Zhang")
        False
    """
    if not isinstance(name, str):
        return False
    s = name.strip()
    if not s:
        return False

    # 1) 长度 2-6
    if not (2 <= len(s) <= 6):
        return False

    # 2) 全部字符必须是汉字 (CJK Unified Ideographs)
    if not re.match(r"^[一-鿿]+$", s):
        return False

    # 3) 不能以这些后缀结尾 (LLM 把 "家" / "先生" 拼进姓名)
    bad_suffixes = ("家", "先生", "女士", "老师", "师傅", "总", "经理", "主任", "医生", "教授")
    for suf in bad_suffixes:
        if s.endswith(suf):
            return False

    return True


def _normalize_name(name: str) -> Optional[str]:
    """把 LLM 输出的人名归一化, 剥掉称呼/后缀/标点。

    当 LLM 输出 "张三先生" / "张三家" / "Mr. Zhang" 时, 业务尝试:
      1. strip 英文称呼前缀 (Mr./Ms./Dr. 等)
      2. 截掉常见后缀 ("先生" / "女士" / "老师" / "家" / "总" 等)
      3. strip 中英文标点 + 空白
      4. 看是否变成合法中文人名

    Returns:
        归一化后的人名; 归一化失败 → None (继续走原始校验)
    """
    if not isinstance(name, str):
        return None
    s = name.strip()
    if not s:
        return None

    # 1) 剥英文称呼前缀
    s = re.sub(r"^(Mr\.?|Ms\.?|Mrs\.?|Dr\.?|Prof\.?)\s*", "", s, flags=re.IGNORECASE)

    # 2) 截后缀 ("先生" / "女士" 等常见称呼 + 收货标记 "家")
    #    反复截断, 直到没有这些后缀 (最多 3 次, 防止无限循环)
    for _ in range(3):
        changed = False
        for suf in ("先生", "女士", "老师", "师傅", "医生", "教授", "经理", "主任", "总", "家"):
            if s.endswith(suf) and len(s) > len(suf):
                s = s[:-len(suf)]
                changed = True
                break
        if not changed:
            break

    # 3) strip 中英文标点 + 空白
    s = re.sub(r"[\s,。.!?!？\-—_:;；()（）【】\[\]]+", "", s)
    s = s.strip()

    # 4) 校验归一化结果是否像人名
    if _is_chinese_name(s):
        return s
    return None


# ---------- 主流程 ----------

def run(
    retries: int = DEFAULT_MAX_RETRIES,
    poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
    cam: str = DEFAULT_CAM,
    prompt: str = DEFAULT_PROMPT,
    model: Optional[str] = None,
    save_frame: bool = False,
    save_dir: Optional[Path] = None,
    stream_url: Optional[str] = None,
    valid_goods: tuple = DEFAULT_VALID_GOODS,
    use_strict_retry: bool = True,
    strict_llm_guess: bool = False,
    target_list: Optional[str] = None,
) -> Dict[str, Any]:
    """侧摄 + ERNIE 文字提取主流程: 拉 N 帧 → 调大模型 → 校验 → 重试至成功或耗尽。

    v3 加强 (2026-08-04):
      - DEFAULT_PROMPT 改为 few-shot 4 例 + 强调语义 (name 是 2-4 字中文, goods 是 9 种之一)
      - 校验失败时**自动换 STRICT_PROMPT 再试一次** (同帧, 不浪费一次重试)
      - name 加语义校验 (1-6 汉字, 无称呼/后缀), 失败时自动归一化 (剥 "先生"/"家" 等)
    v4 加强 (2026-08-04):
      - 加 LLM_GUESS_NAMES 黑名单 (37 个常见默认猜测名, 防幻觉)
      - 默认作软警告 (errors 里有提示, valid 仍 True), 不误伤真实合法识别
      - strict_llm_guess=True 时黑名单命中才判 invalid, 触发 STRICT_PROMPT 重试
    v6 加 (2026-08-04):
      - ``target_list`` 参数让 wenzishibie **直接负责"识别 + 入库"**, 不用再
        target1/2 调用方手动调 ``append_liaobiaoN(name, goods)``。
        - ``"liaobiao1"`` → 校验通过则 ``liaobiao1.append(...)``
        - ``"liaobiao2"`` → 校验通过则 ``liaobiao2.append(...)``
        - ``None`` (默认) → 不写, 保持 v5 行为 (调用方自己 append)

    Args:
        retries:           最大尝试次数 (含首次), 默认 5。
        poll_interval_s:   两次尝试间隔秒, 默认 2.0。
        cam:               相机 ID, 默认 "cam2" (侧摄)。
        prompt:            给大模型的 prompt, 默认 DEFAULT_PROMPT (订单牌提取)。
        model:             ERNIE 模型 ID, None 时从 llm_config.yml 读, fallback DEFAULT_MODEL。
        save_frame:        是否把每帧 JPEG 保存到磁盘 (调试用)。
        save_dir:          帧保存目录; None 时用 cwd/{DEFAULT_SAVE_DIRNAME}。
        stream_url:        覆盖 settings.streamer_url (None 时从 main.settings 读)。
        valid_goods:       goods 字段白名单 (默认 9 种食材)。
        use_strict_retry:  校验失败时是否自动换 STRICT_PROMPT 再试一次 (默认 True)。
        strict_llm_guess:  启用 LLM 默认猜测黑名单严格模式 (默认 False)。
                          False → 黑名单命中只警告, valid=True (避免误伤)
                          True  → 黑名单命中判 invalid, 触发 STRICT_PROMPT 重试
        target_list:       v6 新增。可选 ``"liaobiao1"`` / ``"liaobiao2"`` / ``None``。
                          指定后, ``validated.valid=True`` 时自动调
                          ``append_liaobiao1/2(name, goods)``; 指定 ``None`` 时
                          不写 (调用方自己 append, 与 v5 兼容)。
                          校验失败 / 拉帧失败 / ERNIE 调用失败 → **不写** (避免脏数据)。

    Returns:
        {
            "ok":            True / False,
            "cam":           str,            # 用的哪个 cam
            "model":         str,            # ERNIE 模型
            "attempts":      int,            # 实际尝试次数
            "result":        dict,           # 解析后的字段 (name/goods 或 raw_text)
            "raw_response":  str,            # ERNIE 原始 content
            "frames_bytes":  int,            # 最后一帧字节数
            "stream_url":    str,            # 实际用的 stream URL
            "appended_to":   str | None,     # v6 新增: 写进了哪个 list ("liaobiao1"/"liaobiao2"/None)
            "appended_record": dict | None,  # v6 新增: 写入的 record dict
            "error":         str,            # 仅 ok=False
        }
    """
    # v6 加: 提前校验 target_list, 防止 OCR 跑半天最后才抛 ValueError
    if target_list is not None and target_list not in ("liaobiao1", "liaobiao2"):
        raise ValueError(
            f"target_list 必须是 None / 'liaobiao1' / 'liaobiao2', 收到 {target_list!r}"
        )

    settings = load_settings()
    stream_url = stream_url or settings.streamer_url

    # 读 yml 里的 ernie 配置 (model / top_p / temperature / timeout_s)
    # 优先级: CLI 显式传 > yml > DEFAULT_*
    ernie_cfg = _load_ernie_cfg()
    if model is None:
        model = ernie_cfg["model"]
    yml_top_p = float(ernie_cfg.get("top_p", 0.1))
    yml_temperature = float(ernie_cfg.get("temperature", 0.1))
    yml_timeout_s = float(ernie_cfg.get("timeout_s", DEFAULT_TIMEOUT_S))

    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  stream_url: {stream_url}")
    print(f"  cam: {cam}")
    print(f"  model: {model}  (来源: {ernie_cfg['source']})")
    print(f"  retries: {retries}  poll: {poll_interval_s}s")
    print(f"  top_p: {yml_top_p}  temperature: {yml_temperature}  timeout_s: {yml_timeout_s}")
    print(f"  prompt (前 200 字): {prompt[:200]}{'...' if len(prompt) > 200 else ''}")
    print(f"  save_frame: {save_frame}")

    token = _load_token()
    print(f"  token: {token[:4]}***{token[-4:]} (len={len(token)})")

    if save_frame:
        save_dir = Path(save_dir) if save_dir else Path.cwd() / DEFAULT_SAVE_DIRNAME
        save_dir.mkdir(parents=True, exist_ok=True)
        print(f"  save_dir: {save_dir}")

    last_error = "no attempts"
    last_frame_size = 0
    raw_response = ""

    for attempt in range(1, max(1, retries) + 1):
        print(f"\n  ── 尝试 {attempt}/{retries} ──")
        # v3: 每帧重置 strict 标记, 防止跨帧污染
        strict_attempted_this_frame = False
        frame = _fetch_side_frame(stream_url, cam=cam)
        if frame is None:
            last_error = f"拉帧失败 (cam={cam})"
            print(f"  ❌ {last_error}")
            time.sleep(poll_interval_s)
            continue
        last_frame_size = len(frame)
        print(f"  帧: {last_frame_size} bytes")

        if save_frame and save_dir is not None:
            fp = save_dir / f"frame_{attempt:02d}_{cam}.jpg"
            fp.write_bytes(frame)
            print(f"  保存: {fp.relative_to(Path.cwd()) if fp.is_relative_to(Path.cwd()) else fp}")

        img_b64 = base64.b64encode(frame).decode()
        t1 = time.time()
        result = _call_ernie(
            token, img_b64, prompt,
            model=model, timeout_s=yml_timeout_s,
            top_p=yml_top_p, temperature=yml_temperature,
        )
        dt = time.time() - t1

        if "error" in result:
            last_error = result["error"]
            print(f"  ❌ ERNIE ({dt:.1f}s): {last_error}")
            time.sleep(poll_interval_s)
            continue

        raw_response = result["raw"]
        parsed = result["parsed"]
        print(f"  ✅ ERNIE ({dt:.1f}s):")
        print(f"     parsed: {json.dumps(parsed, ensure_ascii=False, indent=2)}")
        print(f"     raw: {raw_response[:200]}{'...' if len(raw_response) > 200 else ''}")

        # ===== 校验订单字段 (goods ∈ 9 种食材白名单 + name 中文语义 + v4 黑名单) =====
        validated = _validate_order_result(
            parsed, valid_goods=valid_goods,
            strict_llm_guess=strict_llm_guess,
        )
        if validated["valid"]:
            print(f"  ✅ 校验通过: name={validated['name']!r}  "
                  f"goods={validated['goods']!r}")
            # v5: 软警告 (黑名单命中/已归一化) 用独立 warnings 字段, 直接打印
            for w in validated.get("warnings") or []:
                print(f"  ⚠️ 软警告: {w}")
        else:
            print(f"  ⚠️ 校验失败:")
            for err in validated["errors"]:
                print(f"     - {err}")
            # v3 加强: 校验失败时, 如果 use_strict_retry=True 且还没 strict 过,
            # 立即用 STRICT_PROMPT 在同帧重试一次 (不浪费一帧)
            if use_strict_retry and not strict_attempted_this_frame:
                strict_attempted_this_frame = True
                print(f"  🔁 校验失败, 同帧改用 STRICT_PROMPT 再试一次...")
                t2 = time.time()
                result2 = _call_ernie(
                    token, img_b64, STRICT_PROMPT,
                    model=model, timeout_s=yml_timeout_s,
                    top_p=yml_top_p, temperature=yml_temperature,
                )
                dt2 = time.time() - t2
                if "error" in result2:
                    print(f"  ❌ STRICT ERNIE ({dt2:.1f}s): {result2['error']}")
                else:
                    raw_response2 = result2["raw"]
                    parsed2 = result2["parsed"]
                    print(f"  ✅ STRICT ERNIE ({dt2:.1f}s):")
                    print(f"     parsed: {json.dumps(parsed2, ensure_ascii=False, indent=2)}")
                    validated2 = _validate_order_result(
                        parsed2, valid_goods=valid_goods,
                        strict_llm_guess=strict_llm_guess,
                    )
                    if validated2["valid"]:
                        print(f"  ✅ STRICT 校验通过: name={validated2['name']!r}  "
                              f"goods={validated2['goods']!r}")
                        # v5: STRICT 路径也打印软警告
                        for w in validated2.get("warnings") or []:
                            print(f"  ⚠️ STRICT 软警告: {w}")
                        # 用 strict 结果覆盖
                        parsed = parsed2
                        raw_response = raw_response2
                        validated = validated2
                    else:
                        print(f"  ⚠️ STRICT 校验仍失败:")
                        for err in validated2["errors"]:
                            print(f"     - {err}")
                        last_error = "STRICT 校验失败: " + "; ".join(validated2["errors"])
                        time.sleep(poll_interval_s)
                        continue
                # strict 也走通了, 跳出 if 进入最终返回
                if validated["valid"]:
                    dt_total = time.time() - t0
                    print(f"\n========== {LOG_PREFIX} 完成 ({dt_total:.2f}s, 第 {attempt} 次成功 (含 STRICT 重试)) ==========\n")
                    # v6 加: STRICT 路径也自动写 liebiao
                    appended_to, appended_record = _maybe_append_to_target_list(
                        target_list, validated["name"], validated["goods"],
                    )
                    if appended_to:
                        print(f"  📥 自动写入 {appended_to}: {appended_record}")
                    return {
                        "ok": True,
                        "cam": cam,
                        "model": model,
                        "attempts": attempt,
                        "result": parsed,
                        "validated": validated,
                        "raw_response": raw_response,
                        "frames_bytes": last_frame_size,
                        "stream_url": stream_url,
                        "appended_to": appended_to,
                        "appended_record": appended_record,
                    }
            # 普通流程: 校验失败 → 当作这次尝试无效, 进入下一次重试
            last_error = "校验失败: " + "; ".join(validated["errors"])
            time.sleep(poll_interval_s)
            continue

        dt_total = time.time() - t0
        print(f"\n========== {LOG_PREFIX} 完成 ({dt_total:.2f}s, 第 {attempt} 次成功) ==========\n")

        # v6 加: target_list 自动写 liebiao (校验通过 → append, 否则 → 不写, 默认值 None)
        appended_to, appended_record = _maybe_append_to_target_list(
            target_list, validated["name"], validated["goods"],
        )
        if appended_to:
            print(f"  📥 自动写入 {appended_to}: {appended_record}")

        return {
            "ok": True,
            "cam": cam,
            "model": model,
            "attempts": attempt,
            "result": parsed,
            "validated": validated,             # {"valid", "name", "goods", "errors", "warnings"}
            "raw_response": raw_response,
            "frames_bytes": last_frame_size,
            "stream_url": stream_url,
            "appended_to": appended_to,         # v6 新增: "liaobiao1" / "liaobiao2" / None
            "appended_record": appended_record, # v6 新增: 写入的 record dict 或 None
        }

    dt_total = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 失败 ({dt_total:.2f}s, {retries} 次全部失败) ==========")
    print(f"  最后错误: {last_error}\n")
    return {
        "ok": False,
        "cam": cam,
        "model": model,
        "attempts": retries,
        "result": None,
        "validated": None,               # v6 加: 失败时也带 None 字段, 避免 KeyError
        "raw_response": raw_response,
        "frames_bytes": last_frame_size,
        "stream_url": stream_url,
        "appended_to": None,             # v6 新增: 失败一定不写
        "appended_record": None,         # v6 新增
        "error": last_error,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 覆盖 cam/model/retries/poll/prompt/save-frame。

    --model 默认从 llm_config.yml:ernie.model 读 (回退到模块常量)。
    token / stream_url 不暴露 CLI (前者走 env/yaml, 后者走 settings/env)。
    """
    # 从 yml 读默认 model, 让 --model 的 help 显示当前 yml 实际配的模型
    try:
        default_model_for_help = _load_ernie_cfg()["model"]
    except Exception:
        default_model_for_help = DEFAULT_MODEL

    p = argparse.ArgumentParser(
        description=(
            "task6 wenzishibie: 侧摄 (默认 cam2) + 文心大模型 ERNIE 提取订单牌文字"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--cam", default=DEFAULT_CAM,
                   help="相机 ID (cam1=前 / cam2=侧, 默认 cam2)")
    p.add_argument("--model", default=None,
                   help=f"ERNIE 模型 ID (默认从 llm_config.yml 读, 当前 yml 配置: {default_model_for_help})")
    p.add_argument("--retries", type=int, default=DEFAULT_MAX_RETRIES,
                   help="最大尝试次数 (含首次, 默认 5)")
    p.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL_S,
                   dest="poll_interval",
                   help="两次尝试间隔秒 (默认 2.0)")
    p.add_argument("--prompt", default=DEFAULT_PROMPT,
                   help="给大模型的 prompt (默认: 提取订单 + 文字)")
    p.add_argument("--save-frame", action="store_true",
                   dest="save_frame",
                   help=f"把每帧 JPEG 保存到 ./{DEFAULT_SAVE_DIRNAME}/ (调试用)")
    p.add_argument("--valid-goods", default=None,
                   dest="valid_goods",
                   help=(f"goods 字段白名单 (逗号分隔, 默认: "
                         f"{','.join(DEFAULT_VALID_GOODS)})"))
    p.add_argument("--strict-llm-guess", action="store_true",
                   dest="strict_llm_guess",
                   help=("启用 LLM 默认猜测黑名单严格模式 (默认关闭, 仅警告不阻塞)。"
                         "开启后, name 命中 LLM_GUESS_NAMES (37 个常见幻觉名) 视为 invalid, "
                         "触发 STRICT_PROMPT 重试。⚠️ 可能误伤真实合法识别, 仅在明确 LLM "
                         "幻觉频繁时开启"))
    p.add_argument("--target-list", default=None,
                   dest="target_list",
                   help=("v6 新增: 指定后, OCR 校验通过时自动 append 到对应列表。"
                         "取值: 'liaobiao1' / 'liaobiao2' / 'none' (默认不写, 与 v5 兼容)。"
                         "  liaobiao1 → 位置 1 / liaobiao2 → 位置 2"))
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    # 解析 --valid-goods (None 或 "" → 用默认)
    if args.valid_goods:
        valid_goods = tuple(s.strip() for s in args.valid_goods.split(",") if s.strip())
    else:
        valid_goods = DEFAULT_VALID_GOODS

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")

    # 解析 --target-list: 字符串 / 留空 → None
    if args.target_list in (None, "", "none", "None", "null"):
        target_list = None
    elif args.target_list in ("liaobiao1", "liaobiao2"):
        target_list = args.target_list
    else:
        raise SystemExit(
            f"--target-list 必须是 'liaobiao1' / 'liaobiao2' / 'none', 收到 {args.target_list!r}"
        )

    result = run(
        retries=args.retries,
        poll_interval_s=args.poll_interval,
        cam=args.cam,
        prompt=args.prompt,
        model=args.model,
        save_frame=args.save_frame,
        valid_goods=valid_goods,
        strict_llm_guess=args.strict_llm_guess,
        target_list=target_list,
    )
    print("\n最终结果 (JSON):")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    # 校验通过才算 ok=True (ok 是 ERNIE 调用成功, validated.valid 才是真正合规)
    return 0 if (result["ok"] and result.get("validated", {}).get("valid")) else 1


if __name__ == "__main__":
    sys.exit(main())
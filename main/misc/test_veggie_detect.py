#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/misc/test_veggie_detect.py — 蔬菜识别, 纯摄像头+LLM.
不依赖 task_feed 目标检测模型，直接拍 cam2 全图送 ERNIE VL 识别蔬菜种类和位置。
用于任务六（订单取菜）：检测模型不准时，用大模型兜底。

配置: main/misc/llm_config.yml
跑法: python main/misc/test_veggie_detect.py
"""
from __future__ import annotations

import base64
import datetime
import json
import os
import sys
import time
from pathlib import Path

import requests
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.api_client import RuntimeApiClient
from main.settings import load_settings

ERNIE_CHAT_URL = "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions"
_LLM_CONFIG_PATH = Path(__file__).resolve().with_name("llm_config.yml")

# ── 蔬菜识别 System Prompt ─────────────────────────────
VEGGIE_DETECT_PROMPT = """你是一个智慧农业机器人视觉系统。请仔细观察图片中的所有蔬菜，识别它们的种类和在画面中的位置。

识别规则：
- 蔬菜种类只能是以下之一：青椒、蘑菇、芹菜、番茄、油菜、豆角、西兰花、土豆、金针菇

⚠️ 高频混淆组① —— 绿色蔬菜（西兰花/油菜/芹菜/青椒/豆角），务必按以下特征严格区分：
- 西兰花：深绿色，顶部是密集的颗粒状小花球聚成伞状/圆顶状，茎粗壮。像一棵迷你的树。不要和油菜混淆——油菜是散开的叶片，西兰花是紧密的花球。
- 油菜：鲜绿色，散开的椭圆形叶片，叶柄细长，整体呈放射状展开，根部聚拢。像小白菜/上海青。不要和西兰花混淆——油菜没有花球。
- 芹菜：绿色（不是黄色！），茎秆扁平有纵向棱纹（凹槽），比青椒和豆角都粗。关键：通常一捆捆扎在一起，茎秆粗壮。
- 青椒：绿色或深绿色，细长形（牛角形/长圆锥形），不是灯笼形圆椒！表面光滑有蜡质光泽。关键：通常两个一起出现。
- 豆角：绿色极细长条状（线形），比青椒细很多！表面光滑。关键：通常好几根（3-5根）一起呈现。

🚫 青椒 vs 豆角 vs 芹菜 特别警示（三者最容易互相误检）：
- 按粗细：芹菜茎最粗（手指粗）→ 青椒中等（拇指粗，牛角形）→ 豆角最细（筷子/细线粗）。
- 按数量：好几根细线→豆角，两个牛角形→青椒，一捆粗茎→芹菜。
- 芹菜是绿色的！不要和黄色蔬菜混淆。
- 不确定时：成捆粗茎选芹菜，两个细长选青椒，好几根细线选豆角。

🚫 西兰花特别警示（最容易误检）：
- 西兰花有唯一特征"颗粒状小花球聚成的伞状/圆顶"，只有看到这个特征才能判定为西兰花。
- 如果没有看到明显的密集小花球结构，即使蔬菜是绿色的也不能判定为西兰花！
- 以下情况绝对不是西兰花，请根据实际特征选择对应蔬菜：
  绿色散开的叶片 → 油菜
  绿色细长的茎秆 → 芹菜
  绿色光滑灯笼形果实 → 青椒
  绿色细长豆荚（内有凸起） → 豆角
  绿叶蔬菜（叶片大而平展） → 油菜
- 如果不确定是不是西兰花，默认选你认为最可能的其他绿色蔬菜，不要选西兰花。

⚠️ 高频混淆组② —— 圆形个体（番茄/土豆/蘑菇），务必按以下特征严格区分：
- 番茄：红色或橙红色（偶尔青绿色），圆形或扁圆形，表面光滑有光泽，顶部可能有绿色蒂头。像红色的苹果/球。关键色：红。
- 土豆：棕色或黄褐色（土色），不规则椭圆形或长圆形，表面粗糙可能有芽眼（小凹坑），常带有泥土感。像不规则的土块。关键色：棕/褐。
- 蘑菇：白色或浅棕色（奶油色），由圆顶菌盖+细圆柱菌柄组成，像一把小伞。菌盖下可能有褶皱。不要和土豆混淆——蘑菇有菌盖菌柄结构，土豆是实心椭圆形。

其他蔬菜识别要点：
- 金针菇：白色细长菌柄+极小菌盖，成簇生长，像一束白色的细针。

通用规则：
- 如果画面中有多个蔬菜，请全部列出
- 如果图片中看不清或无法确定蔬菜种类，置信度填"低"，并在 analysis 中说明不确定的原因
- 如果图片中没有蔬菜或完全无法识别，items 返回 []

输出要求（严格遵守）：
- 货架有两排：右边第一排（右）、左边第二排（左）。每排从上到下编号 1~4。
- position 字段使用格式："右1"（右边第一排第1行/最上）、"右4"（右边第一排第4行/最下）、"左2"（左边第二排第2行）等。
- 必须返回一个 JSON 对象，不要包含任何 Markdown 标记或解释文字。
- JSON 包含以下字段：
1. "items": (数组) 每个元素包含：
   - "name": (字符串) 蔬菜名称（九选一）
   - "position": (字符串) "右1"~"右4" 或 "左1"~"左4"
   - "confidence": (字符串) "高"/"中"/"低"
2. "analysis": (字符串) 简述你识别到了什么、用了哪些区分特征来判断。

示例：
{
    "items": [
        {"name": "土豆", "position": "左4", "confidence": "高"},
        {"name": "番茄", "position": "右1", "confidence": "高"}
    ],
    "analysis": "左边第二排第4行有一个棕色不规则椭圆形的土豆（表面粗糙有芽眼），右边第一排第1行有一个红色光滑圆形的番茄（有绿色蒂头）。"
}
"""


def _load_cfg() -> dict:
    if not _LLM_CONFIG_PATH.exists():
        raise FileNotFoundError(f"LLM 配置缺失: {_LLM_CONFIG_PATH}")
    with _LLM_CONFIG_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _load_token(cfg: dict) -> str:
    t = os.getenv("ERNIE_ACCESS_TOKEN", "").strip()
    if t: return t
    t = (cfg.get("ernie") or {}).get("access_token", "")
    if t and t != "REPLACE_YOUR_ACCESS_TOKEN_HERE": return t
    cp = _REPO_ROOT / "config_car.yml"
    if cp.exists():
        d = yaml.safe_load(cp.open("r", encoding="utf-8")) or {}
        t = d.get("ernie_access_token", "")
        if t and t != "REPLACE_YOUR_ACCESS_TOKEN_HERE": return t
    print("[fatal] 未找到 ERNIE access token", file=sys.stderr)
    sys.exit(2)


def _call_llm(token: str, image_b64: str, prompt: str, ernie: dict) -> dict:
    body = {
        "model": ernie.get("model", "ernie-4.5-turbo-vl"),
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
        ]}],
        "top_p": ernie.get("top_p", 0.1),
        "temperature": ernie.get("temperature", 0.1),
    }
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
        "x-bce-date": datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    to = ernie.get("timeout_s", 15)
    last_err = ""
    for _ in range(2):
        try:
            resp = requests.post(ERNIE_CHAT_URL, headers=headers, json=body, timeout=to)
        except requests.Timeout:
            last_err = "timeout"; time.sleep(1.5); continue
        except requests.RequestException as exc:
            last_err = str(exc); time.sleep(1.5); continue
        if resp.status_code in (401, 403):
            print(f"[fatal] ERNIE {resp.status_code}", file=sys.stderr); sys.exit(2)
        if resp.status_code == 429 or resp.status_code >= 500:
            last_err = f"HTTP {resp.status_code}"; time.sleep(1.5); continue
        if not resp.ok:
            return {"items": [], "analysis": f"HTTP {resp.status_code}", "error": True}
        try:
            content = resp.json()["choices"][0]["message"]["content"]
        except Exception as exc:
            return {"items": [], "analysis": str(exc), "error": True}
        text = content.strip()
        if text.startswith("```"):
            ls = text.splitlines()
            ls = ls[1:] if ls[0].lstrip().startswith("```") else ls
            if ls and ls[-1].lstrip().startswith("```"): ls = ls[:-1]
            text = "\n".join(ls).strip()
        try:
            d = json.loads(text)
        except json.JSONDecodeError:
            return {"items": [], "analysis": f"unparseable: {content[:200]}", "error": True}
        if isinstance(d, dict) and "items" in d:
            return d
        return {"items": [], "analysis": f"missing items: {content[:200]}", "error": True}
    return {"items": [], "analysis": last_err, "error": True}


def fetch_frame(session, streamer_url, timeout=10.0):
    try:
        r = session.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status(); return r.content
    except Exception as exc:
        print(f"[warn] cam2 帧失败: {exc}", file=sys.stderr); return None


def run():
    cfg = _load_cfg()
    token = _load_token(cfg)
    settings = load_settings()
    ernie = cfg.get("ernie", {})
    prompt = (cfg.get("veggie_detect") or {}).get("system_prompt") or VEGGIE_DETECT_PROMPT

    print(f"API={settings.api_base}  token={token[:4]}***{token[-4:]}")
    client = RuntimeApiClient(settings=settings)
    client.wait_until_ready()
    session = requests.Session()

    for attempt in range(1, 6):
        print(f"\n--- 第 {attempt}/5 次检测 ---")
        frame = fetch_frame(session, settings.streamer_url)
        if not frame:
            time.sleep(2); continue
        print(f"  frame: {len(frame)} bytes")

        img = base64.b64encode(frame).decode()
        t0 = time.time()
        result = _call_llm(token, img, prompt, ernie)
        dt = time.time() - t0

        items = result.get("items") or []
        if result.get("error"):
            print(f"  ❌ ({dt:.1f}s): {result.get('analysis', '')[:100]}")
            time.sleep(2); continue

        print(f"  ✅ ({dt:.1f}s): 识别到 {len(items)} 个蔬菜")
        for it in items:
            print(f"    [{it.get('position', '?')}] {it.get('name', '?')} 置信度={it.get('confidence', '?')}")
        print(f"  analysis: {result.get('analysis', '')[:120]}")

        if items:
            print(f"\n蔬菜列表: {[it['name'] for it in items]}")
            print(f"位置映射: {[(it['name'], it['position']) for it in items]}")
            break
    else:
        print("\n❌ 5次均未识别到蔬菜")

    return {"ok": len(items) > 0, "items": items, "raw_analysis": result.get("analysis", "")}


if __name__ == "__main__":
    r = run(); print("\nJSON:", json.dumps(r, ensure_ascii=False, indent=2))

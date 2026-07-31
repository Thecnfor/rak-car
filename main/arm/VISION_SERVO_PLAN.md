# 机械臂视觉伺服 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `main/arm/` 叠加视觉伺服能力；末端摄像头（side cam）检测 20 类业务目标；4 电机（y / x / 大臂 / 末端爪）按检测反馈闭环；支持多目标选择与优先级。

**Architecture:**
- Layer 1 `RuntimeApiClient` 加 `request_vision_task` + `get_vision_task_cache`
- Layer 1.5 `main/arm/labels.py` 新文件：20 项 `Label` enum + `LABELS` + `LABEL_GROUPS`
- Layer 2 `main/arm/vision.py` 新文件：`ArmVisionClient` + `TargetSelector` + `find_target` 多目标
- Layer 2.5 `ArmClient.composite_run / composite_run_reset` 薄封装（消费现有 `arm.composite_run`）+ `vision` 懒属性
- Layer 3 `ArmRunner.move_to_vision_target / pick_by_vision` 高层组合

**Tech Stack:** Python 3.10+, dataclasses, runtime HTTP `:5050`, 已有 `arm.composite_run` (commit `a48839d`)。

**Spec 文档：** `main/arm/VISION_SERVO_DESIGN.md`（commit `6711ae6`）。

## Global Constraints

- 不改 `runtime/`、`smartcar/`、`config_car.yml` — 纯客户端封装
- 所有业务位姿单位 mm；车端边界处换算 m（`_mm_to_m` helper）
- 守住 CLAUDE.md 红线：业务层不绕过 `main.arm.*` 直调 `client.call("arm", ...)`
- `move_to_vision_target` / `pick_by_vision` 调用前置：`y < -30mm`（出保护区），由 `composite_run` 入口 `_check_y_protected` 兜底
- 视觉伺服默认参数：`mm_per_norm=30.0`、`settle_tol_norm=0.05`、`min_step_mm=1.0`、`timeout=10.0`
- 目标丢失策略：5 帧连续未命中 abort（`on_missing_track="abort"` 默认；可改 `wait`）
- 20 项 label catalog 是**硬约束**；不许新增 label 名
- 单测用 stdlib `unittest`（项目无 pytest，按 CLAUDE.md 不引入）
- 真机测试地址：`192.168.5.230:5050`（覆盖 .gitignore 默认）

---

## 文件清单（实施前锁定）

| 文件 | 状态 | 行数估算 |
|---|---|---|
| `main/api_client.py` | 编辑 +2 方法 | +50 |
| `main/arm/labels.py` | **新建** | +90 |
| `main/arm/tests/__init__.py` | **新建** | +1 |
| `main/arm/tests/test_labels.py` | **新建**（单测） | +80 |
| `main/arm/vision.py` | **新建** | +280 |
| `main/arm/tests/test_vision_parsers.py` | **新建**（单测） | +90 |
| `main/arm/tests/test_vision_selector.py` | **新建**（单测） | +120 |
| `main/arm/api.py` | 编辑 +3 方法 +1 属性 | +60 |
| `main/arm/loops/runner.py` | 编辑 +2 方法 | +60 |
| `main/arm/__init__.py` | 编辑 export | +12 |
| `main/arm/ARM_API.md` | 编辑 文档 | +220 |

**总计 ~+1060 行（含单测）**

---

### Task 1: RuntimeApiClient vision 调用方法

**Files:**
- Modify: `main/api_client.py:18-50` 区域（紧挨 `_request` helper 后面）
- Test: `main/arm/tests/test_api_vision.py`（新建）

**Interfaces:**
- Consumes: `self._request("GET"/"POST", path, payload=..., timeout=...)`（已存在）
- Produces:
  - `RuntimeApiClient.request_vision_task(*, sort_pos=(0,0), limit_x=1.0, limit_y=1.0, timeout=20.0) -> Dict`
  - `RuntimeApiClient.get_vision_task_cache() -> Dict`

- [ ] **Step 1: 写失败单测**

```python
# main/arm/tests/test_api_vision.py
"""RuntimeApiClient vision 调用方法的单测 —— 用 mock 替代真实 HTTP"""
import unittest
from unittest.mock import MagicMock, patch
from main.api_client import RuntimeApiClient


class TestRequestVisionTask(unittest.TestCase):
    def setUp(self):
        self.client = RuntimeApiClient.__new__(RuntimeApiClient)  # skip __init__
        self.client.api_prefix = "/v1"
        self.client.base_url = "http://test:5050"
        self.client.timeout = 30.0

    def test_request_vision_task_payload_shape(self):
        captured = {}
        def fake_request(method, path, payload=None, timeout=None):
            captured["method"] = method
            captured["path"] = path
            captured["payload"] = payload
            captured["timeout"] = timeout
            return {"ok": True, "detections": []}
        self.client._request = fake_request

        self.client.request_vision_task(
            sort_pos=(0.1, 0.2), limit_x=0.5, limit_y=0.8, timeout=15.0
        )

        self.assertEqual(captured["method"], "POST")
        self.assertEqual(captured["path"], "/v1/vision/task")
        self.assertEqual(captured["payload"]["sort_pos"], [0.1, 0.2])
        self.assertEqual(captured["payload"]["limit_x"], 0.5)
        self.assertEqual(captured["payload"]["limit_y"], 0.8)
        self.assertEqual(captured["payload"]["timeout"], 15.0)
        self.assertGreaterEqual(captured["timeout"], 15.0)  # outer timeout + 5s

    def test_get_vision_task_cache_calls_correct_path(self):
        captured = {}
        def fake_request(method, path, payload=None, timeout=None):
            captured["method"] = method
            captured["path"] = path
            return {"ok": True, "task_state": {"detections": []}}
        self.client._request = fake_request

        result = self.client.get_vision_task_cache()

        self.assertEqual(captured["method"], "GET")
        self.assertEqual(captured["path"], "/v1/realtime/vision/task")
        self.assertIn("task_state", result)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_api_vision -v
```

预期：`AttributeError: 'RuntimeApiClient' object has no attribute 'request_vision_task'`

- [ ] **Step 3: 在 api_client.py 加 2 个方法**（紧挨 `_request` helper 后面）

```python
def request_vision_task(
    self, *,
    sort_pos=(0.0, 0.0),
    limit_x: float = 1.0,
    limit_y: float = 1.0,
    timeout: float = 20.0,
):
    """POST /v1/vision/task — 同步单次推理（含 bbox_pixels）。

    返回 runtime JSON 原样 dict，由 vision.py 层负责解析。
    """
    return self._request(
        "POST",
        f"{self.api_prefix}/vision/task",
        payload={
            "sort_pos": [float(sort_pos[0]), float(sort_pos[1])],
            "limit_x": float(limit_x),
            "limit_y": float(limit_y),
            "timeout": float(timeout),
        },
        timeout=timeout + 5.0,
    )

def get_vision_task_cache(self):
    """GET /v1/realtime/vision/task — 读 task_feed 30Hz 缓存。"""
    return self._request("GET", f"{self.api_prefix}/realtime/vision/task")
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_api_vision -v
```

预期：`Ran 2 tests in ... OK`

- [ ] **Step 5: 真机 smoke test**

```bash
curl -sS --max-time 5 http://192.168.5.230:5050/v1/realtime/vision/task | python3 -m json.tool | head -20
```

预期：返回 `{"ok": true, "task_state": {...}}`，且 `detections` 是 list（可能空）。

- [ ] **Step 6: Commit**

```bash
git add main/api_client.py main/arm/tests/test_api_vision.py
git commit -m "feat(api): RuntimeApiClient 加 request_vision_task / get_vision_task_cache

- POST /v1/vision/task：单次同步推理（带 bbox_pixels + filter）
- GET /v1/realtime/vision/task：读 task_feed 30Hz 缓存（视觉伺服主路径）

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 2: main/arm/labels.py（20 项 catalog）

**Files:**
- Create: `main/arm/labels.py`
- Create: `main/arm/tests/__init__.py`（空文件）
- Create: `main/arm/tests/test_labels.py`

**Interfaces:**
- Consumes: 无（pure Python）
- Produces:
  - `class Label(str, Enum)` —— 20 项（`ANIMAL` / `BALL_BLUE` / `BALL_YELLOW` / `CYLINDER_1/2/3` / `CYLINDER_SET` / `H_*` 9 个 / `WATER` / `WATER_L1/L2/L3`）
  - `LabelInfo` dataclass: `(id: int, name: str, desc: str)`
  - `LABELS: Tuple[LabelInfo, ...]` —— 20 项
  - `LABEL_GROUPS: Dict[str, Tuple[Label, ...]]` —— 6 组
  - `get_label_info(name: str) -> LabelInfo`
  - `is_in_group(name: str, group: str) -> bool`

- [ ] **Step 1: 写失败单测**

```python
# main/arm/tests/test_labels.py
"""main/arm/labels.py 单测：20 项枚举 + 分组查询 + 工厂"""
import unittest
from main.arm.labels import (
    Label, LabelInfo, LABELS, LABEL_GROUPS,
    get_label_info, is_in_group,
)


class TestLabels(unittest.TestCase):
    def test_labels_count_is_twenty(self):
        self.assertEqual(len(LABELS), 20)

    def test_labels_ids_sequential_1_to_20(self):
        ids = [info.id for info in LABELS]
        self.assertEqual(ids, list(range(1, 21)))

    def test_label_enum_is_str_subclass(self):
        # Label.H_DOU_JIAO 应该是字符串 "h_dou_jiao"
        self.assertEqual(Label.H_DOU_JIAO, "h_dou_jiao")
        self.assertIsInstance(Label.H_DOU_JIAO, str)

    def test_get_label_info_returns_correct_entry(self):
        info = get_label_info("h_dou_jiao")
        self.assertEqual(info.id, 8)
        self.assertEqual(info.desc, "豆角")

    def test_get_label_info_unknown_raises(self):
        with self.assertRaises(ValueError) as ctx:
            get_label_info("not_a_real_label")
        self.assertIn("未知 label", str(ctx.exception))

    def test_label_groups_keys(self):
        expected_groups = {"animal", "ball", "cylinder", "cylinder_meta", "vegetable", "water"}
        self.assertEqual(set(LABEL_GROUPS.keys()), expected_groups)

    def test_vegetable_group_has_nine(self):
        self.assertEqual(len(LABEL_GROUPS["vegetable"]), 9)

    def test_water_group_has_four(self):
        self.assertEqual(len(LABEL_GROUPS["water"]), 4)

    def test_is_in_group(self):
        self.assertTrue(is_in_group("h_dou_jiao", "vegetable"))
        self.assertTrue(is_in_group("water_l1", "water"))
        self.assertFalse(is_in_group("h_dou_jiao", "ball"))
        self.assertFalse(is_in_group("not_a_label", "vegetable"))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_labels -v
```

预期：`ModuleNotFoundError: No module named 'main.arm.labels'`

- [ ] **Step 3: 创建 main/arm/tests/__init__.py（空文件）**

```bash
touch /home/xrak/Desktop/rak-car/main/arm/tests/__init__.py
```

- [ ] **Step 4: 写 main/arm/labels.py**（照 spec §4 完整内容）

```python
# main/arm/labels.py —— 把 spec §4 完整内容贴入
"""业务目标类别 catalog —— 对齐 task backend 模型输出 (20 项)。"""
from dataclasses import dataclass
from enum import Enum
from typing import Tuple, Dict


@dataclass(frozen=True)
class LabelInfo:
    id: int
    name: str
    desc: str

    def __str__(self) -> str: return f"Label({self.name})"


class Label(str, Enum):
    ANIMAL        = "animal"
    BALL_BLUE     = "ball_blue"
    BALL_YELLOW   = "ball_yellow"
    CYLINDER_1    = "cylinder_1"
    CYLINDER_2    = "cylinder_2"
    CYLINDER_3    = "cylinder_3"
    CYLINDER_SET  = "cylinder_set"
    H_DOU_JIAO    = "h_dou_jiao"
    H_FAN_QIE     = "h_fan_qie"
    H_JIN_ZHEN_GU = "h_jin_zhen_gu"
    H_MO_GU       = "h_mo_gu"
    H_QIN_CAI     = "h_qin_cai"
    H_QING_JIAO   = "h_qing_jiao"
    H_TU_DOU      = "h_tu_dou"
    H_XI_LAN_HUA  = "h_xi_lan_hua"
    H_YOU_CAI     = "h_you_cai"
    WATER         = "water"
    WATER_L1      = "water_l1"
    WATER_L2      = "water_l2"
    WATER_L3      = "water_l3"


LABELS: Tuple[LabelInfo, ...] = (
    LabelInfo(1,  "animal",        "动物"),
    LabelInfo(2,  "ball_blue",     "蓝色球"),
    LabelInfo(3,  "ball_yellow",   "黄色球"),
    LabelInfo(4,  "cylinder_1",    "圆柱体（1号）"),
    LabelInfo(5,  "cylinder_2",    "圆柱体（2号）"),
    LabelInfo(6,  "cylinder_3",    "圆柱体（3号）"),
    LabelInfo(7,  "cylinder_set",  "圆柱体组合"),
    LabelInfo(8,  "h_dou_jiao",    "豆角"),
    LabelInfo(9,  "h_fan_qie",     "番茄"),
    LabelInfo(10, "h_jin_zhen_gu", "金针菇"),
    LabelInfo(11, "h_mo_gu",       "蘑菇"),
    LabelInfo(12, "h_qin_cai",     "芹菜"),
    LabelInfo(13, "h_qing_jiao",   "青椒"),
    LabelInfo(14, "h_tu_dou",      "土豆"),
    LabelInfo(15, "h_xi_lan_hua",  "西兰花"),
    LabelInfo(16, "h_you_cai",     "油菜"),
    LabelInfo(17, "water",         "水容器"),
    LabelInfo(18, "water_l1",      "水容器（等级1）"),
    LabelInfo(19, "water_l2",      "水容器（等级2）"),
    LabelInfo(20, "water_l3",      "水容器（等级3）"),
)


LABEL_GROUPS: Dict[str, Tuple[Label, ...]] = {
    "animal":    (Label.ANIMAL,),
    "ball":      (Label.BALL_BLUE, Label.BALL_YELLOW),
    "cylinder":  (Label.CYLINDER_1, Label.CYLINDER_2, Label.CYLINDER_3),
    "cylinder_meta": (Label.CYLINDER_SET,),
    "vegetable": (Label.H_DOU_JIAO, Label.H_FAN_QIE, Label.H_JIN_ZHEN_GU,
                  Label.H_MO_GU, Label.H_QIN_CAI, Label.H_QING_JIAO,
                  Label.H_TU_DOU, Label.H_XI_LAN_HUA, Label.H_YOU_CAI),
    "water":     (Label.WATER, Label.WATER_L1, Label.WATER_L2, Label.WATER_L3),
}


def get_label_info(name: str) -> LabelInfo:
    for info in LABELS:
        if info.name == name:
            return info
    raise ValueError(f"未知 label: {name!r}（共 20 项，参考 LABELS）")


def is_in_group(name: str, group: str) -> bool:
    try:
        return Label(name) in LABEL_GROUPS.get(group, ())
    except ValueError:
        return False
```

- [ ] **Step 5: 跑测试确认通过**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_labels -v
```

预期：`Ran 9 tests in ... OK`

- [ ] **Step 6: Commit**

```bash
git add main/arm/labels.py main/arm/tests/__init__.py main/arm/tests/test_labels.py
git commit -m "feat(arm): main/arm/labels.py —— 20 项业务目标类别 catalog

- Label(str, Enum) 直接当字符串传给 runtime
- LABELS 按用户给定 (id, name, desc) 落库
- LABEL_GROUPS：animal / ball / cylinder / cylinder_meta / vegetable / water
- get_label_info / is_in_group 查询辅助

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 3: vision.py 数据类型 + 解析器（BBoxNorm / BBoxPixels / Detection / _parse_cache / _parse_sync）

**Files:**
- Create: `main/arm/vision.py`（先写最小版：只含数据类型 + 解析器）
- Create: `main/arm/tests/test_vision_parsers.py`

**Interfaces:**
- Consumes: `Label` / `LabelInfo`（Task 2 产出）
- Produces:
  - `BBoxNorm` dataclass + `is_centered(tol=0.05) -> bool` property
  - `BBoxPixels` dataclass
  - `Detection` dataclass: `(label, score, track_id, class_id, bbox_norm, bbox_pixels, fetched_at)`
  - `_parse_cache(raw: Dict) -> List[Detection]` —— `/v1/realtime/vision/task` 解析
  - `_parse_sync(raw: Dict) -> List[Detection]` —— `/v1/vision/task` 解析（含 bbox_pixels）

- [ ] **Step 1: 写失败单测**

```python
# main/arm/tests/test_vision_parsers.py
"""vision.py 数据类型 + 解析器单测"""
import time
import unittest
from main.arm.vision import (
    BBoxNorm, BBoxPixels, Detection,
    _parse_cache, _parse_sync,
)


CACHE_FIXTURE = {
    "ok": True,
    "task_state": {
        "active": True,
        "detections": [
            {
                "cls_id": 15,
                "det_id": 3,
                "label": "h_dou_jiao",
                "score": 0.88,
                "bbox_norm": {
                    "x_center": 0.10,
                    "y_center": -0.05,
                    "width": 0.22,
                    "height": 0.15,
                },
            },
        ],
        "count": 1,
        "updated_at": 1785488303.73,
    },
}


SYNC_FIXTURE = {
    "ok": True,
    "model": "task",
    "camera": "cam2",
    "detections": [
        {
            "index": 0,
            "class_id": 3,
            "track_id": 0,
            "label": "cylinder_1",
            "score": 0.95,
            "bbox_norm": {"x_center": 0.0, "y_center": 0.0, "width": 0.1, "height": 0.2},
            "bbox_pixels": {"x1": 320, "y1": 240, "x2": 384, "y2": 336, "width": 64, "height": 96},
        },
    ],
    "count": 1,
    "frame_shape": [480, 640, 3],
}


class TestBBoxNorm(unittest.TestCase):
    def test_is_centered_within_tol(self):
        b = BBoxNorm(x_center=0.02, y_center=-0.03, width=0.1, height=0.1)
        self.assertTrue(b.is_centered(tol=0.05))
        self.assertTrue(b.is_centered(tol=0.05))

    def test_is_centered_outside_tol(self):
        b = BBoxNorm(x_center=0.20, y_center=-0.30, width=0.1, height=0.1)
        self.assertFalse(b.is_centered(tol=0.05))


class TestParseCache(unittest.TestCase):
    def test_parse_returns_one_detection(self):
        dets = _parse_cache(CACHE_FIXTURE)
        self.assertEqual(len(dets), 1)
        d = dets[0]
        self.assertEqual(d.label, "h_dou_jiao")
        self.assertEqual(d.score, 0.88)
        self.assertEqual(d.track_id, 3)
        self.assertEqual(d.class_id, 15)
        self.assertIsNone(d.bbox_pixels)
        self.assertAlmostEqual(d.bbox_norm.x_center, 0.10)

    def test_parse_handles_empty_detections(self):
        dets = _parse_cache({"ok": True, "task_state": {"detections": []}})
        self.assertEqual(dets, [])


class TestParseSync(unittest.TestCase):
    def test_parse_returns_detection_with_pixels(self):
        dets = _parse_sync(SYNC_FIXTURE)
        self.assertEqual(len(dets), 1)
        d = dets[0]
        self.assertEqual(d.label, "cylinder_1")
        self.assertIsNotNone(d.bbox_pixels)
        self.assertEqual(d.bbox_pixels.x1, 320)
        self.assertEqual(d.bbox_pixels.width, 64)

    def test_parse_handles_missing_pixels(self):
        raw = {**SYNC_FIXTURE, "detections": [
            {**SYNC_FIXTURE["detections"][0]}
        ]}
        raw["detections"][0].pop("bbox_pixels")
        dets = _parse_sync(raw)
        self.assertIsNone(dets[0].bbox_pixels)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_vision_parsers -v
```

预期：`ModuleNotFoundError: No module named 'main.arm.vision'`

- [ ] **Step 3: 写 main/arm/vision.py**（最小版，只含数据类型 + 解析器）

```python
"""main/arm/vision.py —— 机械臂视觉伺服客户端（详见 VISION_SERVO_DESIGN.md）。"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from .labels import Label, LabelInfo, LABELS, LABEL_GROUPS  # noqa: F401


@dataclass(frozen=True)
class BBoxNorm:
    x_center: float
    y_center: float
    width: float
    height: float

    @property
    def is_centered(self) -> bool:
        return self.is_centered_at(0.05)

    def is_centered_at(self, tol: float) -> bool:
        return abs(self.x_center) <= tol and abs(self.y_center) <= tol


@dataclass(frozen=True)
class BBoxPixels:
    x1: int
    y1: int
    x2: int
    y2: int
    width: int
    height: int


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    track_id: Optional[int]
    class_id: Optional[int]
    bbox_norm: BBoxNorm
    bbox_pixels: Optional[BBoxPixels]
    fetched_at: float

    def __repr__(self) -> str:
        return (
            f"Detection({self.label}#{self.track_id} "
            f"score={self.score:.2f} cx={self.bbox_norm.x_center:+.2f})"
        )


def _parse_cache(raw: Dict[str, Any]) -> List[Detection]:
    """GET /v1/realtime/vision/task → List[Detection]（无 bbox_pixels）"""
    state = raw.get("task_state") or {}
    dets = state.get("detections") or []
    now = float(state.get("updated_at") or time.time())
    out: List[Detection] = []
    for d in dets:
        bn = d.get("bbox_norm") or {}
        out.append(Detection(
            label=str(d["label"]),
            score=float(d["score"]),
            track_id=d.get("det_id") or d.get("track_id"),
            class_id=d.get("cls_id") or d.get("class_id"),
            bbox_norm=BBoxNorm(
                float(bn["x_center"]), float(bn["y_center"]),
                float(bn.get("width", 0.0)), float(bn.get("height", 0.0)),
            ),
            bbox_pixels=None,
            fetched_at=now,
        ))
    return out


def _parse_sync(raw: Dict[str, Any]) -> List[Detection]:
    """POST /v1/vision/task → List[Detection]（含 bbox_pixels）"""
    dets = raw.get("detections") or []
    now = time.time()
    out: List[Detection] = []
    for d in dets:
        bn = d.get("bbox_norm") or {}
        bp = d.get("bbox_pixels") or None
        out.append(Detection(
            label=str(d["label"]),
            score=float(d["score"]),
            track_id=d.get("track_id"),
            class_id=d.get("class_id"),
            bbox_norm=BBoxNorm(
                float(bn["x_center"]), float(bn["y_center"]),
                float(bn.get("width", 0.0)), float(bn.get("height", 0.0)),
            ),
            bbox_pixels=BBoxPixels(
                int(bp["x1"]), int(bp["y1"]), int(bp["x2"]), int(bp["y2"]),
                int(bp["width"]), int(bp["height"]),
            ) if bp else None,
            fetched_at=now,
        ))
    return out
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_vision_parsers -v
```

预期：`Ran 5 tests in ... OK`

- [ ] **Step 5: Commit**

```bash
git add main/arm/vision.py main/arm/tests/test_vision_parsers.py
git commit -m "feat(arm): vision.py 数据类型 + 解析器（BBoxNorm / Detection / _parse）

- BBoxNorm.is_centered_at(tol) 默认 0.05
- _parse_cache: /v1/realtime/vision/task → 无 bbox_pixels
- _parse_sync: /v1/vision/task → 含 bbox_pixels
- Detection.__repr__ 用于日志

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 4: TargetSelector + SelectionStrategy + apply_strategy

**Files:**
- Modify: `main/arm/vision.py`（追加 SelectionStrategy + TargetSelector）
- Create: `main/arm/tests/test_vision_selector.py`

**Interfaces:**
- Consumes: `Label`, `LABEL_GROUPS`（Task 2 产出）, `Detection`（Task 3 产出）
- Produces:
  - `class SelectionStrategy(str, Enum)`: HIGHEST_SCORE / CLOSEST_TO_CENTER / LARGEST / LEFTMOST / RIGHTMOST / TOPMOST / BOTTOMMOST / LOCK_FIRST_SEEN
  - `class TargetSelector`: `(label, track_id, strategy, group)` + `.matches(det)` + `.apply_strategy(candidates) -> Optional[Detection]` + 工厂 `for_label / for_group`

- [ ] **Step 1: 写失败单测**

```python
# main/arm/tests/test_vision_selector.py
"""TargetSelector 单测"""
import unittest
from main.arm.vision import (
    BBoxNorm, Detection, TargetSelector, SelectionStrategy,
)
from main.arm.labels import Label, LABEL_GROUPS


def _det(label, cx, cy, score=0.9, w=0.1, h=0.1, tid=None):
    return Detection(
        label=label, score=score, track_id=tid, class_id=None,
        bbox_norm=BBoxNorm(x_center=cx, y_center=cy, width=w, height=h),
        bbox_pixels=None, fetched_at=0.0,
    )


class TestTargetSelectorFactory(unittest.TestCase):
    def test_for_label_with_enum(self):
        sel = TargetSelector.for_label(Label.H_DOU_JIAO)
        self.assertEqual(sel.label, "h_dou_jiao")

    def test_for_label_with_string(self):
        sel = TargetSelector.for_label("cylinder_1")
        self.assertEqual(sel.label, "cylinder_1")

    def test_for_group(self):
        sel = TargetSelector.for_group("vegetable")
        self.assertEqual(sel.group, "vegetable")
        self.assertIsNone(sel.label)

    def test_for_group_unknown_raises(self):
        with self.assertRaises(ValueError):
            TargetSelector.for_group("not_a_group")


class TestMatches(unittest.TestCase):
    def test_matches_by_label(self):
        sel = TargetSelector.for_label("h_dou_jiao")
        self.assertTrue(sel.matches(_det("h_dou_jiao", 0, 0)))
        self.assertFalse(sel.matches(_det("h_fan_qie", 0, 0)))

    def test_matches_by_group(self):
        sel = TargetSelector.for_group("vegetable")
        self.assertTrue(sel.matches(_det("h_dou_jiao", 0, 0)))
        self.assertTrue(sel.matches(_det("h_fan_qie", 0, 0)))
        self.assertFalse(sel.matches(_det("animal", 0, 0)))

    def test_matches_no_filter(self):
        sel = TargetSelector()  # 全 None
        self.assertTrue(sel.matches(_det("any_label", 0, 0)))


class TestApplyStrategy(unittest.TestCase):
    def test_highest_score(self):
        sel = TargetSelector(strategy="highest_score")
        candidates = [_det("x", 0, 0, score=0.5), _det("x", 0, 0, score=0.9), _det("x", 0, 0, score=0.7)]
        pick = sel.apply_strategy(candidates)
        self.assertEqual(pick.score, 0.9)

    def test_closest_to_center(self):
        sel = TargetSelector(strategy="closest_to_center")
        candidates = [
            _det("x", 0.3, 0.0),
            _det("x", 0.05, 0.05),
            _det("x", -0.2, 0.0),
        ]
        pick = sel.apply_strategy(candidates)
        # 第二个 (0.05, 0.05) 距离原点最近
        self.assertAlmostEqual(pick.bbox_norm.x_center, 0.05)

    def test_largest(self):
        sel = TargetSelector(strategy="largest")
        candidates = [
            _det("x", 0, 0, w=0.1, h=0.1),
            _det("x", 0, 0, w=0.5, h=0.5),
            _det("x", 0, 0, w=0.2, h=0.2),
        ]
        pick = sel.apply_strategy(candidates)
        self.assertAlmostEqual(pick.bbox_norm.width, 0.5)

    def test_leftmost(self):
        sel = TargetSelector(strategy="leftmost")
        candidates = [
            _det("x", 0.5, 0),
            _det("x", -0.8, 0),
            _det("x", 0.1, 0),
        ]
        pick = sel.apply_strategy(candidates)
        self.assertAlmostEqual(pick.bbox_norm.x_center, -0.8)

    def test_empty_returns_none(self):
        sel = TargetSelector(strategy="highest_score")
        self.assertIsNone(sel.apply_strategy([]))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_vision_selector -v
```

预期：`ImportError: cannot import name 'TargetSelector' from 'main.arm.vision'`

- [ ] **Step 3: 在 vision.py 追加 SelectionStrategy + TargetSelector**

```python
# 在 vision.py 末尾追加（保留 Task 3 的所有内容）
from enum import Enum


class SelectionStrategy(str, Enum):
    HIGHEST_SCORE      = "highest_score"
    CLOSEST_TO_CENTER  = "closest_to_center"
    LARGEST            = "largest"
    LEFTMOST           = "leftmost"
    RIGHTMOST          = "rightmost"
    TOPMOST            = "topmost"
    BOTTOMMOST         = "bottommost"
    LOCK_FIRST_SEEN    = "lock_first_seen"


@dataclass(frozen=True)
class TargetSelector:
    label: Optional[str] = None
    track_id: Optional[int] = None
    strategy: str = SelectionStrategy.HIGHEST_SCORE.value
    group: Optional[str] = None

    @classmethod
    def for_label(cls, label, *, strategy: str = SelectionStrategy.HIGHEST_SCORE.value) -> "TargetSelector":
        return cls(
            label=str(label.value if isinstance(label, Label) else label),
            strategy=strategy,
        )

    @classmethod
    def for_group(cls, group: str, *, strategy: str = SelectionStrategy.HIGHEST_SCORE.value) -> "TargetSelector":
        if group not in LABEL_GROUPS:
            raise ValueError(f"未知 group: {group!r}（{list(LABEL_GROUPS)}）")
        return cls(label=None, strategy=strategy, group=group)

    def matches(self, det: Detection) -> bool:
        if self.group is not None:
            return det.label in [l.value for l in LABEL_GROUPS[self.group]]
        if self.label is not None:
            return det.label == self.label
        return True

    def apply_strategy(self, candidates: List[Detection]) -> Optional[Detection]:
        if not candidates:
            return None
        s = self.strategy
        if s == SelectionStrategy.HIGHEST_SCORE.value:
            return max(candidates, key=lambda d: d.score)
        if s == SelectionStrategy.CLOSEST_TO_CENTER.value:
            return min(candidates, key=lambda d: abs(d.bbox_norm.x_center) + abs(d.bbox_norm.y_center))
        if s == SelectionStrategy.LARGEST.value:
            return max(candidates, key=lambda d: d.bbox_norm.width * d.bbox_norm.height)
        if s == SelectionStrategy.LEFTMOST.value:
            return min(candidates, key=lambda d: d.bbox_norm.x_center)
        if s == SelectionStrategy.RIGHTMOST.value:
            return max(candidates, key=lambda d: d.bbox_norm.x_center)
        if s == SelectionStrategy.TOPMOST.value:
            return min(candidates, key=lambda d: d.bbox_norm.y_center)
        if s == SelectionStrategy.BOTTOMMOST.value:
            return max(candidates, key=lambda d: d.bbox_norm.y_center)
        # LOCK_FIRST_SEEN 由 find_target 循环内处理（首帧锁定 track_id）
        return candidates[0]
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_vision_selector -v
```

预期：`Ran 11 tests in ... OK`

- [ ] **Step 5: Commit**

```bash
git add main/arm/vision.py main/arm/tests/test_vision_selector.py
git commit -m "feat(arm): vision.py TargetSelector + SelectionStrategy

- 8 种选择策略（HIGHEST_SCORE / CLOSEST_TO_CENTER / LARGEST / 4 个极值 / LOCK_FIRST_SEEN）
- 工厂 for_label(Label | str) + for_group(name)
- matches() 做 label/group 过滤；apply_strategy() 做最终挑一

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 5: ArmVisionClient（get_state / get_state_filtered / snap / find_target / find_targets_sequence / pick_one）

**Files:**
- Modify: `main/arm/vision.py`（追加 ArmVisionClient + ServoTrace + ServoResult + find_target 主路径）
- Create: `main/arm/tests/test_vision_find_target.py`

**Interfaces:**
- Consumes: `Detection`, `TargetSelector`（前序 Task 产出）；构造时注入 `RuntimeApiClient`
- Produces:
  - `class ServoTrace` `(t_s, iteration, dx_norm, dy_norm, x_mm, y_mm, score, selected_track_id)`
  - `class ServoResult` `(converged, selector, x_mm, y_mm, confidence, iterations, elapsed_s, final_detection, trace)`
  - `class ArmVisionClient`:
    - `labels()` 静态方法
    - `group(name)` 静态方法
    - `get_state() -> List[Detection]`
    - `get_state_filtered(selector) -> List[Detection]`
    - `snap(...) -> List[Detection]`
    - `find_target(selector, *, x_mm, y_mm, mm_per_norm=30.0, settle_tol_norm=0.05, min_step_mm=1.0, max_iter=500, timeout=10.0, on_missing_track="abort", move_fn=None) -> ServoResult`
    - `find_targets_sequence(selectors, *, x_mm, y_mm, **kwargs) -> List[ServoResult]`
    - `pick_one(selectors, *, x_mm, y_mm, **kwargs) -> Optional[ServoResult]`

- [ ] **Step 1: 写失败单测（用 fake detector 注入）**

```python
# main/arm/tests/test_vision_find_target.py
"""ArmVisionClient.find_target 单测 —— 用 mock http 模拟检测序列"""
import unittest
from typing import List
from main.api_client import RuntimeApiClient
from main.arm.vision import (
    ArmVisionClient, TargetSelector, Detection, BBoxNorm,
)


class FakeHttp:
    """模拟 RuntimeApiClient.get_vision_task_cache 行为"""
    def __init__(self, frames: List[List[Detection]]):
        self.frames = list(frames)
        self.call_count = 0

    def get_vision_task_cache(self):
        if self.call_count >= len(self.frames):
            # 走完后稳定返回最后一帧（避免 IndexError）
            self.call_count += 1
            return {"task_state": {"detections": [
                self._det_to_dict(d) for d in self.frames[-1]
            ]}}
        frame = self.frames[self.call_count]
        self.call_count += 1
        return {"task_state": {"detections": [self._det_to_dict(d) for d in frame]}}

    @staticmethod
    def _det_to_dict(d: Detection) -> dict:
        return {
            "label": d.label,
            "score": d.score,
            "track_id": d.track_id,
            "cls_id": d.class_id,
            "bbox_norm": {
                "x_center": d.bbox_norm.x_center,
                "y_center": d.bbox_norm.y_center,
                "width": d.bbox_norm.width,
                "height": d.bbox_norm.height,
            },
        }


def _det(cx, cy, label="h_dou_jiao", score=0.9, tid=0):
    return Detection(
        label=label, score=score, track_id=tid, class_id=8,
        bbox_norm=BBoxNorm(cx, cy, 0.1, 0.1),
        bbox_pixels=None, fetched_at=0.0,
    )


class TestFindTarget(unittest.TestCase):
    def test_converges_when_centered(self):
        # 第 0 帧：目标在右 (dx=0.3) → 应 move -9mm
        # 第 1 帧：目标居中 → 收敛
        frames = [
            [_det(cx=0.3, cy=0.0)],
            [_det(cx=0.01, cy=0.01)],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)  # type: ignore[arg-type]
        move_log: list = []
        def move(x, y): move_log.append((x, y)); return {}
        sel = TargetSelector.for_label("h_dou_jiao")
        result = vision.find_target(
            sel, x_mm=0.0, y_mm=-100.0,
            mm_per_norm=30.0, settle_tol_norm=0.05, min_step_mm=1.0,
            timeout=5.0, move_fn=move,
        )
        self.assertTrue(result.converged)
        self.assertGreaterEqual(result.iterations, 2)
        self.assertEqual(len(move_log), 1)  # 第一帧动了 1 步
        self.assertAlmostEqual(move_log[0][0], -9.0, places=1)  # -0.3 * 30 = -9mm

    def test_timeout_returns_unconverged(self):
        # 目标永远不收敛
        frames = [[_det(cx=0.5, cy=0.5)] for _ in range(100)]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)  # type: ignore[arg-type]
        sel = TargetSelector.for_label("h_dou_jiao")
        result = vision.find_target(
            sel, x_mm=0.0, y_mm=-100.0,
            mm_per_norm=30.0, timeout=0.05, max_iter=100,
            move_fn=lambda x, y: {},
        )
        self.assertFalse(result.converged)
        self.assertGreater(result.iterations, 0)

    def test_min_step_dead_band(self):
        # 偏差 < min_step_mm → 不动
        frames = [
            [_det(cx=0.01, cy=0.01)],  # 偏差 0.3mm < 1mm dead-band
            [_det(cx=0.01, cy=0.01)],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)  # type: ignore[arg-type]
        move_log: list = []
        result = vision.find_target(
            TargetSelector.for_label("h_dou_jiao"),
            x_mm=0.0, y_mm=-100.0, min_step_mm=1.0,
            move_fn=lambda x, y: move_log.append((x, y)) or {},
        )
        self.assertTrue(result.converged)
        self.assertEqual(len(move_log), 0)  # dead-band 阻断了 move

    def test_label_filter_excludes_other_labels(self):
        frames = [
            [_det(cx=0.0, cy=0.0, label="animal")],  # 不是 h_dou_jiao
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)  # type: ignore[arg-type]
        with self.assertRaises(RuntimeError) as ctx:
            vision.find_target(
                TargetSelector.for_label("h_dou_jiao"),
                x_mm=0.0, y_mm=-100.0,
                on_missing_track="abort",
                move_fn=lambda x, y: {},
                max_iter=10, timeout=5.0,
            )
        self.assertIn("连续", str(ctx.exception))


class TestFindTargetsSequence(unittest.TestCase):
    def test_runs_each_selector(self):
        frames = [
            [_det(cx=0.0, cy=0.0, label="h_dou_jiao")],
            [_det(cx=0.0, cy=0.0, label="h_fan_qie")],
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)  # type: ignore[arg-type]
        results = vision.find_targets_sequence(
            [TargetSelector.for_label("h_dou_jiao"),
             TargetSelector.for_label("h_fan_qie")],
            x_mm=0.0, y_mm=-100.0, timeout=2.0,
            move_fn=lambda x, y: {},
        )
        self.assertEqual(len(results), 2)
        self.assertTrue(all(r.converged for r in results))


class TestPickOne(unittest.TestCase):
    def test_picks_first_matching(self):
        frames = [
            [_det(cx=0.0, cy=0.0, label="h_fan_qie")],  # 第 1 个 selector 命中
        ]
        fake = FakeHttp(frames)
        vision = ArmVisionClient(fake)  # type: ignore[arg-type]
        result = vision.pick_one(
            [TargetSelector.for_label("h_dou_jiao"),  # 无
             TargetSelector.for_label("h_fan_qie")],   # 有
            x_mm=0.0, y_mm=-100.0, timeout=2.0,
            move_fn=lambda x, y: {},
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.selector.label, "h_fan_qie")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_vision_find_target -v
```

预期：`ImportError: cannot import name 'ArmVisionClient' from 'main.arm.vision'`

- [ ] **Step 3: 在 vision.py 追加 ArmVisionClient + ServoTrace + ServoResult**

```python
# 追加到 vision.py 末尾
import logging
from typing import Callable, Optional

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ServoTrace:
    t_s: float
    iteration: int
    dx_norm: float
    dy_norm: float
    x_mm: float
    y_mm: float
    score: float
    selected_track_id: Optional[int]


@dataclass(frozen=True)
class ServoResult:
    converged: bool
    selector: TargetSelector
    x_mm: float
    y_mm: float
    confidence: float
    iterations: int
    elapsed_s: float
    final_detection: Optional[Detection]
    trace: Tuple[ServoTrace, ...]


class ArmVisionClient:
    def __init__(self, http: RuntimeApiClient, *, default_timeout_s: float = 10.0):
        self.http = http
        self.default_timeout_s = default_timeout_s

    @staticmethod
    def labels() -> Tuple[LabelInfo, ...]:
        return LABELS

    @staticmethod
    def group(name: str) -> Tuple[Label, ...]:
        return LABEL_GROUPS[name]

    def get_state(self) -> List[Detection]:
        return _parse_cache(self.http.get_vision_task_cache())

    def get_state_filtered(self, selector: TargetSelector) -> List[Detection]:
        return [d for d in self.get_state() if selector.matches(d)]

    def snap(self, *, sort_pos=(0.0, 0.0), limit_x: float = 1.0,
             limit_y: float = 1.0, timeout: float = 20.0) -> List[Detection]:
        return _parse_sync(self.http.request_vision_task(
            sort_pos=sort_pos, limit_x=limit_x, limit_y=limit_y, timeout=timeout))

    def find_target(self, selector: TargetSelector, *,
                    x_mm: float, y_mm: float,
                    mm_per_norm: float = 30.0,
                    settle_tol_norm: float = 0.05,
                    min_step_mm: float = 1.0,
                    max_iter: int = 500,
                    timeout: float = 10.0,
                    on_missing_track: str = "abort",
                    move_fn: Optional[Callable[[float, float], dict]] = None) -> ServoResult:
        t0 = time.time()
        trace: List[ServoTrace] = []
        locked_track_id: Optional[int] = None
        consecutive_misses = 0
        last_x_mm, last_y_mm = x_mm, y_mm
        last_detection: Optional[Detection] = None
        current_selector = selector

        for i in range(max_iter):
            if time.time() - t0 > timeout:
                break

            candidates = self.get_state_filtered(current_selector)

            if current_selector.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if locked_track_id is None:
                    pick = current_selector.apply_strategy(candidates)
                    if pick is None:
                        consecutive_misses += 1
                        if consecutive_misses >= 5 and on_missing_track == "abort":
                            raise RuntimeError(f"find_target: 首帧未检测到 {current_selector}")
                        continue
                    locked_track_id = pick.track_id
                    current_selector = dataclasses.replace(current_selector, track_id=locked_track_id)
                candidates = [d for d in candidates if d.track_id == locked_track_id]
            elif current_selector.track_id is not None:
                candidates = [d for d in candidates if d.track_id == current_selector.track_id]

            pick = current_selector.apply_strategy(candidates) if candidates else None
            if pick is None:
                consecutive_misses += 1
                if on_missing_track == "abort" and consecutive_misses >= 5:
                    raise RuntimeError(
                        f"find_target: 连续 {consecutive_misses} 帧未检测到 {current_selector}"
                    )
                continue
            consecutive_misses = 0
            last_detection = pick

            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center
            if pick.bbox_norm.is_centered_at(settle_tol_norm):
                trace.append(ServoTrace(
                    t_s=time.time() - t0, iteration=i,
                    dx_norm=dx_norm, dy_norm=dy_norm,
                    x_mm=last_x_mm, y_mm=last_y_mm,
                    score=pick.score, selected_track_id=pick.track_id))
                return ServoResult(
                    converged=True, selector=current_selector,
                    x_mm=last_x_mm, y_mm=last_y_mm,
                    confidence=pick.score, iterations=i + 1,
                    elapsed_s=time.time() - t0,
                    final_detection=pick, trace=tuple(trace))

            dx_mm = -dx_norm * mm_per_norm
            dy_mm = -dy_norm * mm_per_norm
            if abs(dx_mm) < min_step_mm:
                dx_mm = 0.0
            if abs(dy_mm) < min_step_mm:
                dy_mm = 0.0

            new_x_mm = last_x_mm + dx_mm
            new_y_mm = last_y_mm + dy_mm
            trace.append(ServoTrace(
                t_s=time.time() - t0, iteration=i,
                dx_norm=dx_norm, dy_norm=dy_norm,
                x_mm=new_x_mm, y_mm=new_y_mm,
                score=pick.score, selected_track_id=pick.track_id))

            if move_fn is not None:
                move_fn(new_x_mm, new_y_mm)
            else:
                self.http.execute_arm_action(
                    "goto_position",
                    x=new_x_mm / 1000.0, y=new_y_mm / 1000.0,
                    timeout=5.0, sync=True,
                )
            last_x_mm, last_y_mm = new_x_mm, new_y_mm

        return ServoResult(
            converged=False, selector=current_selector,
            x_mm=last_x_mm, y_mm=last_y_mm,
            confidence=last_detection.score if last_detection else 0.0,
            iterations=max_iter, elapsed_s=time.time() - t0,
            final_detection=last_detection, trace=tuple(trace))

    def find_targets_sequence(self, selectors: List[TargetSelector], *,
                              x_mm: float, y_mm: float, **kwargs) -> List[ServoResult]:
        return [self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs) for sel in selectors]

    def pick_one(self, selectors: List[TargetSelector], *,
                 x_mm: float, y_mm: float, **kwargs) -> Optional[ServoResult]:
        for sel in selectors:
            try:
                result = self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs)
                if result.converged:
                    return result
            except RuntimeError:
                continue
        return None
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_vision_find_target -v
```

预期：`Ran 6 tests in ... OK`

- [ ] **Step 5: Commit**

```bash
git add main/arm/vision.py main/arm/tests/test_vision_find_target.py
git commit -m "feat(arm): vision.py ArmVisionClient 视觉伺服主路径

- find_target: 单目标伺服循环（缓存读 + 收敛/超时/丢失策略）
- find_targets_sequence: 多目标顺序伺服
- pick_one: 优先级短路
- ServoTrace / ServoResult 调试可观测（trace tuple）

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 6: ArmClient 增 composite_run / composite_run_reset / vision 属性 / _make_vision_with_move

**Files:**
- Modify: `main/arm/api.py:884-867` 区域（`composite_pick` 后面 + `get_state` 前面）
- Create: `main/arm/tests/test_arm_client_composite_run.py`

**Interfaces:**
- Consumes: `composite_run` action（runtime 已注册，commit `a48839d`）；`ArmVisionClient`（Task 5 产出）
- Produces:
  - `ArmClient.composite_run(*, arm=None, x_mm=None, y_mm=None, hand=None, speed=80, timeout=30.0) -> dict`
  - `ArmClient.composite_run_reset(*, arm_angle=90.0, hand_angle=-90.0, x_direction="right", reset_x_velocity_mms=20.0, timeout=60.0) -> dict`
  - `ArmClient.vision` 属性（懒构造 `ArmVisionClient`）
  - `ArmClient._make_vision_with_move() -> ArmVisionClient`（move_fn 注入 `_check_safe`）

- [ ] **Step 1: 写失败单测**

```python
# main/arm/tests/test_arm_client_composite_run.py
"""ArmClient.composite_run / composite_run_reset / vision 单测 —— 验参数透传"""
import unittest
from unittest.mock import MagicMock
from main.api_client import RuntimeApiClient
from main.arm.api import ArmClient
from main.arm.vision import ArmVisionClient


class TestCompositeRun(unittest.TestCase):
    def setUp(self):
        http = RuntimeApiClient.__new__(RuntimeApiClient)
        http.api_prefix = "/v1"
        http.base_url = "http://test:5050"
        http.timeout = 30.0
        self.captured = {}
        def fake_arm(name, *args, timeout=20.0, sync=True, **kwargs):
            self.captured["name"] = name
            self.captured["kwargs"] = kwargs
            self.captured["timeout"] = timeout
            return {"ok": True}
        http.execute_arm_action = fake_arm
        self.client = ArmClient(http=http)

    def test_composite_run_passes_all_four(self):
        self.client.composite_run(
            arm=30.0, x_mm=100.0, y_mm=-80.0, hand=-37.0, speed=80
        )
        self.assertEqual(self.captured["name"], "composite_run")
        kw = self.captured["kwargs"]
        self.assertEqual(kw["arm"], 30.0)
        self.assertEqual(kw["x"], 0.1)        # 100mm → 0.1m
        self.assertEqual(kw["y"], -0.08)     # -80mm → -0.08m
        self.assertEqual(kw["hand"], -37.0)
        self.assertEqual(kw["speed"], 80)

    def test_composite_run_passes_none_for_skipped(self):
        self.client.composite_run(arm=None, x_mm=None, y_mm=-100.0, hand=-90.0)
        kw = self.captured["kwargs"]
        self.assertIsNone(kw["arm"])
        self.assertIsNone(kw["x"])
        self.assertEqual(kw["y"], -0.1)
        self.assertEqual(kw["hand"], -90.0)

    def test_composite_run_reset(self):
        self.client.composite_run_reset(
            arm_angle=90.0, hand_angle=-90.0,
            x_direction="right", reset_x_velocity_mms=20.0, timeout=60.0
        )
        self.assertEqual(self.captured["name"], "composite_run_reset")
        kw = self.captured["kwargs"]
        self.assertEqual(kw["arm_angle"], 90.0)
        self.assertEqual(kw["hand_angle"], -90.0)
        self.assertEqual(kw["x_direction"], "right")
        self.assertEqual(kw["reset_x_velocity"], 0.02)  # 20mm/s → 0.02m/s


class TestVisionProperty(unittest.TestCase):
    def test_vision_property_lazy(self):
        http = MagicMock()
        client = ArmClient(http=http)
        v1 = client.vision
        v2 = client.vision
        self.assertIsInstance(v1, ArmVisionClient)
        self.assertIs(v1, v2)  # 懒属性：第二次拿同一对象


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_arm_client_composite_run -v
```

预期：`AttributeError: 'ArmClient' object has no attribute 'composite_run'`

- [ ] **Step 3: 在 api.py `__init__` 后加 `_vision` 字段；在 `composite_pick` 段后面加 3 个方法**

```python
# 在 ArmClient.__init__ 末尾追加：
        self._vision: Optional["ArmVisionClient"] = None

# 在 composite_go_home 后面（约第 763 行）追加：
    def composite_run(self, *, arm: Optional[float] = None, x_mm: Optional[float] = None,
                      y_mm: Optional[float] = None, hand: Optional[float] = None,
                      speed: int = 80, timeout: float = 30.0) -> dict:
        """薄封装 arm.composite_run(arm, x, y, hand)，任一 None 跳过。

        业务前置：所有非 None 参数必须先过 _check_y_protected / _check_safe。
        """
        if y_mm is not None:
            self._check_y_protected("composite_run")
            self._check_safe(y_mm=y_mm)
        return self._call_arm(
            "composite_run", timeout=timeout,
            arm=arm,
            x=_mm_to_m(x_mm) if x_mm is not None else None,
            y=_mm_to_m(y_mm) if y_mm is not None else None,
            hand=hand, speed=speed,
        )

    def composite_run_reset(self, *, arm_angle: float = 90.0, hand_angle: float = -90.0,
                            x_direction: str = "right", reset_x_velocity_mms: float = 20.0,
                            timeout: float = 60.0) -> dict:
        """薄封装 arm.composite_run_reset() —— x 撞墙 + arm + hand 并行 + y 触底收尾"""
        return self._call_arm(
            "composite_run_reset", timeout=timeout,
            arm_angle=arm_angle, hand_angle=hand_angle,
            x_direction=x_direction,
            reset_x_velocity=reset_x_velocity_mms / 1000.0,
        )

    @property
    def vision(self) -> "ArmVisionClient":
        """懒构造：首次访问时建 ArmVisionClient"""
        if self._vision is None:
            self._vision = ArmVisionClient(self.http)
        return self._vision

    def _make_vision_with_move(self) -> "ArmVisionClient":
        """返回一个 move_fn 已经被 _check_safe 包裹的 vision client（业务层用）。"""
        client = ArmVisionClient(self.http)
        original = client.find_target

        def safe_find(selector, *, x_mm, y_mm, **kwargs):
            move_fn = kwargs.pop("move_fn", None)
            if move_fn is None:
                def _safe_move(nx: float, ny: float) -> dict:
                    self._check_y_protected("find_target")
                    self._check_safe(y_mm=ny)
                    return self.move_xy(nx, ny, timeout=10.0)
                move_fn = _safe_move
            return original(selector, x_mm=x_mm, y_mm=y_mm, move_fn=move_fn, **kwargs)

        client.find_target = safe_find  # type: ignore[method-assign]
        return client
```

注意：需要在文件顶部 import `from .vision import ArmVisionClient`（避免循环 import）。

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_arm_client_composite_run -v
```

预期：`Ran 4 tests in ... OK`

- [ ] **Step 5: Commit**

```bash
git add main/arm/api.py main/arm/tests/test_arm_client_composite_run.py
git commit -m "feat(arm): ArmClient.composite_run / composite_run_reset / vision 懒属性

- composite_run 薄封装（None 跳过 + _check_y_protected + _check_safe + mm→m）
- composite_run_reset 薄封装
- vision property 懒构造 ArmVisionClient（每次访问同一对象）
- _make_vision_with_move() 给 ArmRunner 用：find_target 自动注入 _check_safe 的 move_xy

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 7: ArmRunner.move_to_vision_target + pick_by_vision

**Files:**
- Modify: `main/arm/loops/runner.py`（在 `release` 后面追加 2 方法）
- Create: `main/arm/tests/test_runner_vision.py`

**Interfaces:**
- Consumes: `ArmClient.composite_run / _make_vision_with_move / composite_pick`（Task 6 + 现有）
- Produces:
  - `ArmRunner.move_to_vision_target(selector, *, x_mm, y_mm, arm_angle=0.0, hand=-90.0, mm_per_norm=30.0, settle_tol_norm=0.05, timeout=10.0) -> ServoResult`
  - `ArmRunner.pick_by_vision(selector, *, x_mm, y_mm, arm_angle=-90.0, settle_tol_norm=0.05, timeout=10.0) -> dict`

- [ ] **Step 1: 写失败单测**

```python
# main/arm/tests/test_runner_vision.py
"""ArmRunner.move_to_vision_target / pick_by_vision 单测"""
import unittest
from unittest.mock import MagicMock
from main.arm.loops.runner import ArmRunner
from main.arm.vision import TargetSelector, ServoResult


def _fake_servo_result(converged=True, x_mm=0.0, y_mm=-100.0, label="h_dou_jiao"):
    sel = TargetSelector.for_label(label)
    return ServoResult(
        converged=converged, selector=sel,
        x_mm=x_mm, y_mm=y_mm, confidence=0.9,
        iterations=3, elapsed_s=0.5,
        final_detection=None, trace=(),
    )


class TestMoveToVisionTarget(unittest.TestCase):
    def setUp(self):
        client = MagicMock()
        self.runner = ArmRunner(client)
        self.calls = client.composite_run.call_args_list
        self.finder = client._make_vision_with_move.return_value

    def test_calls_composite_run_then_vision_servo(self):
        self.finder.find_target.return_value = _fake_servo_result()

        result = self.runner.move_to_vision_target(
            TargetSelector.for_label("h_dou_jiao"),
            x_mm=100.0, y_mm=-150.0, arm_angle=-90.0,
        )

        # composite_run 调用 1 次
        self.assertEqual(len(self.calls), 1)
        kw = self.calls[0].kwargs
        self.assertEqual(kw["arm"], -90.0)
        self.assertEqual(kw["x_mm"], 100.0)
        self.assertEqual(kw["y_mm"], -150.0)
        self.assertEqual(kw["hand"], -90.0)
        # find_target 也调了 1 次
        self.assertEqual(self.finder.find_target.call_count, 1)
        self.assertTrue(result.converged)


class TestPickByVision(unittest.TestCase):
    def test_calls_3_actions_in_order(self):
        client = MagicMock()
        self.finder = client._make_vision_with_move.return_value
        self.finder.find_target.return_value = _fake_servo_result()
        client.composite_pick.return_value = {"ok": True, "steps": {}}
        runner = ArmRunner(client)

        result = runner.pick_by_vision(
            TargetSelector.for_label("h_dou_jiao"),
            x_mm=100.0, y_mm=-150.0, arm_angle=-90.0,
        )

        # 顺序：composite_run (move_to_vision_target) → find_target → composite_pick
        self.assertGreaterEqual(client.composite_run.call_count, 1)
        self.assertEqual(self.finder.find_target.call_count, 1)
        client.composite_pick.assert_called_once()
        self.assertTrue(result["ok"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_runner_vision -v
```

预期：`AttributeError: 'ArmRunner' object has no attribute 'move_to_vision_target'`

- [ ] **Step 3: 在 loops/runner.py 末尾追加 2 方法**

```python
# 在 release 后面追加
from ..vision import TargetSelector, ServoResult  # type: ignore

    def move_to_vision_target(self, selector: TargetSelector, *,
                              x_mm: float, y_mm: float,
                              arm_angle: float = 0.0, hand: float = -90.0,
                              mm_per_norm: float = 30.0,
                              settle_tol_norm: float = 0.05,
                              timeout: float = 10.0) -> ServoResult:
        """高层组合：composite_run 粗定位 → 视觉伺服精调。

        业务前置：必须在 y < -30mm 保护区外（composite_run 入口会校验）。
        """
        self.client.composite_run(
            arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
        )
        return self.client._make_vision_with_move().find_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            mm_per_norm=mm_per_norm, settle_tol_norm=settle_tol_norm,
            timeout=timeout,
        )

    def pick_by_vision(self, selector: TargetSelector, *,
                       x_mm: float, y_mm: float, arm_angle: float = -90.0,
                       settle_tol_norm: float = 0.05,
                       timeout: float = 10.0) -> dict:
        """最高层：粗定位 → 伺服 → composite_pick → grasp。"""
        self.move_to_vision_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            arm_angle=arm_angle, hand=-90.0,
            settle_tol_norm=settle_tol_norm, timeout=timeout,
        )
        return self.client.composite_pick(
            arm_angle=arm_angle, x_mm=x_mm, y_mm=y_mm,
            hand=0.0, speed=80, timeout=30.0,
        )
```

- [ ] **Step 4: 跑测试确认通过**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest main.arm.tests.test_runner_vision -v
```

预期：`Ran 2 tests in ... OK`

- [ ] **Step 5: Commit**

```bash
git add main/arm/loops/runner.py main/arm/tests/test_runner_vision.py
git commit -m "feat(arm): ArmRunner.move_to_vision_target / pick_by_vision

- move_to_vision_target: composite_run 粗定位 + vision find_target 精调
- pick_by_vision: 上面 + composite_pick + grasp

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 8: __init__.py exports + ARM_API.md 文档

**Files:**
- Modify: `main/arm/__init__.py`
- Modify: `main/arm/ARM_API.md`（追加 §10 vision 章节）

**Interfaces:**
- Consumes: 全部前序 Task 产出
- Produces: 公开 import surface + 文档

- [ ] **Step 1: 改写 `main/arm/__init__.py`**

```python
"""main/arm 子包：机械臂业务层。

外部 import 只允许指向 main.*，不接触 runtime / smartcar。
"""
from .api import ArmClient
from .state import (
    ArmState,
    ArmOrigin,
    SIDES,
    HANDS,
    STORAGE_SIDES,
    STORAGE_DEFAULT_LEFT_ANGLE,
    STORAGE_DEFAULT_RIGHT_ANGLE,
)
from .origin import OriginCalibrator, run_calibrator
from .trajectory import (
    TrajectoryGenerator,
    TrajectoryPlan,
    TrajectorySample,
)
from .loops.runner import ArmRunner
# 2026-07-31 视觉伺服封装（VISION_SERVO_DESIGN.md）：
from .labels import (
    Label, LabelInfo, LABELS, LABEL_GROUPS,
    get_label_info, is_in_group,
)
from .vision import (
    ArmVisionClient,
    Detection, BBoxNorm, BBoxPixels,
    TargetSelector, SelectionStrategy,
    ServoTrace, ServoResult,
)

__all__ = [
    "ArmClient", "ArmRunner", "ArmState", "ArmOrigin",
    "SIDES", "HANDS", "STORAGE_SIDES",
    "STORAGE_DEFAULT_LEFT_ANGLE", "STORAGE_DEFAULT_RIGHT_ANGLE",
    "OriginCalibrator", "run_calibrator",
    "TrajectoryGenerator", "TrajectoryPlan", "TrajectorySample",
    # vision
    "Label", "LabelInfo", "LABELS", "LABEL_GROUPS",
    "get_label_info", "is_in_group",
    "ArmVisionClient",
    "Detection", "BBoxNorm", "BBoxPixels",
    "TargetSelector", "SelectionStrategy",
    "ServoTrace", "ServoResult",
]
```

- [ ] **Step 2: 在 ARM_API.md 末尾追加 §10 vision 章节**

```markdown
## 10. 视觉伺服（main/arm/vision.py，2026-07-31）

> 末端摄像头 = side cam（cam2 / USB1）。视觉主路径走 task_feed 30Hz 缓存；同步 `/v1/vision/task` 用于带 filter 的快照。

### 10.1 数据类型

| 类 | 说明 |
| --- | --- |
| `Label(str, Enum)` | 20 项业务目标类别枚举（`H_DOU_JIAO` 等） |
| `LabelInfo` | `(id, name, desc)` 三元组 |
| `LABELS` | 20 项 LabelInfo 元组（按 id 1-20 排序） |
| `LABEL_GROUPS` | 自然分组字典（`vegetable` / `ball` / `cylinder` / `water` 等） |
| `BBoxNorm` / `BBoxPixels` | 归一化 / 像素 bbox |
| `Detection` | 单个检测结果（`label, score, track_id, bbox_norm, bbox_pixels, fetched_at`） |
| `SelectionStrategy(str, Enum)` | 8 种选择策略 |
| `TargetSelector` | 多目标选择器（label/track_id/strategy/group 四元组） |
| `ServoTrace` / `ServoResult` | 单步状态 / 整体伺服结果（带 trace tuple 调试用） |

### 10.2 业务方法

| 方法 | 说明 |
| --- | --- |
| `ArmClient.vision` | 懒属性，返回 `ArmVisionClient` |
| `ArmClient.composite_run(*, arm, x_mm, y_mm, hand, speed, timeout)` | 4 电机并行（任一 None 跳过） |
| `ArmClient.composite_run_reset(...)` | x 撞墙 + arm + hand 并行 + y 串行 |
| `ArmVisionClient.get_state() -> List[Detection]` | 读 task_feed 缓存所有检测 |
| `ArmVisionClient.get_state_filtered(selector) -> List[Detection]` | 读缓存 + selector 过滤 |
| `ArmVisionClient.snap(...) -> List[Detection]` | 同步一次推理（含 bbox_pixels） |
| `ArmVisionClient.find_target(selector, *, x_mm, y_mm, mm_per_norm=30.0, ...)` | 单目标视觉伺服 |
| `ArmVisionClient.find_targets_sequence(selectors, ...)` | 多目标顺序伺服 |
| `ArmVisionClient.pick_one(selectors, ...)` | 优先级短路 |
| `TargetSelector.for_label(Label_or_str, *, strategy)` | 工厂：单 label |
| `TargetSelector.for_group(group_name, *, strategy)` | 工厂：整组（vegetable/water/...） |

### 10.3 高层组合（ArmRunner）

```python
runner.move_to_vision_target(
    selector=TargetSelector.for_label(Label.H_DOU_JIAO, strategy="highest_score"),
    x_mm=100, y_mm=-150, arm_angle=-90, hand=-90, timeout=10,
)
# → ServoResult

runner.pick_by_vision(
    selector=TargetSelector.for_group("vegetable"),
    x_mm=100, y_mm=-150, arm_angle=-90, timeout=10,
)
# → composite_pick job dict
```

### 10.4 真机 smoke 模板

```bash
export RAK_CAR_SERVER_ORIGIN=http://192.168.5.230:5050
python3 -c "
from main.arm import ArmClient, ArmRunner, TargetSelector, Label
runner = ArmRunner(ArmClient.connect())
print(runner.client.vision.get_state())
"
```
```

- [ ] **Step 3: 跑全套单测确认无回归**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest discover -s main/arm/tests -v
```

预期：`Ran 32 tests in ... OK`（Task 2=9 + Task 3=5 + Task 4=11 + Task 5=6 + Task 6=4 + Task 7=2）

- [ ] **Step 4: import smoke**

```bash
cd /home/xrak/Desktop/rak-car
python3 -c "from main.arm import ArmClient, ArmRunner, Label, TargetSelector, ArmVisionClient; print('imports OK')"
```

预期：`imports OK`

- [ ] **Step 5: Commit**

```bash
git add main/arm/__init__.py main/arm/ARM_API.md
git commit -m "feat(arm): export vision 公开符号 + ARM_API.md §10 文档

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 9: 真机端到端 smoke（4 个测试用例）

**Files:** 无新增代码；只跑命令 + 记录结果。

- [ ] **Step 1: TP1 - 缓存读取 smoke**

```bash
export RAK_CAR_SERVER_ORIGIN=http://192.168.5.230:5050
cd /home/xrak/Desktop/rak-car
python3 -c "
from main.arm import ArmClient, Label
client = ArmClient.connect()
dets = client.vision.get_state()
print(f'TP1: get_state returned {len(dets)} detections')
for d in dets[:5]:
    print(f'  {d}')
"
```

预期：打印 `TP1: get_state returned N detections`，N ≥ 0；如果场景中有任何 20 类目标，N ≥ 1。

- [ ] **Step 2: TP2 - 同步 snap smoke**

```bash
python3 -c "
from main.arm import ArmClient, Label
client = ArmClient.connect()
dets = client.vision.snap(limit_x=1.0, limit_y=1.0, timeout=10)
print(f'TP2: snap returned {len(dets)} detections')
for d in dets[:5]:
    print(f'  {d}')
    if d.bbox_pixels:
        print(f'    bbox_pixels: {d.bbox_pixels}')
"
```

预期：`TP2: snap returned N detections`，且每条都有 `bbox_pixels`。

- [ ] **Step 3: TP3 - move_to_vision_target 单目标（不抓，只伺服）**

```bash
python3 -c "
from main.arm import ArmClient, ArmRunner, TargetSelector, Label
runner = ArmRunner(ArmClient.connect())
# 先 composite_run 到安全位（arm=0, x=0, y=-150, hand=UP）
print('TP3 step 1: composite_run 粗定位到 (0, -150, arm=0, hand=UP)')
runner.client.composite_run(arm=0.0, x_mm=0.0, y_mm=-150.0, hand=-90.0)
print('TP3 step 2: 视觉伺服找一个蔬菜（不抓，只看收敛）')
result = runner.move_to_vision_target(
    TargetSelector.for_group('vegetable'),
    x_mm=0.0, y_mm=-150.0, arm_angle=0.0, hand=-90.0,
    timeout=8.0, mm_per_norm=30.0,
)
print(f'TP3 result: converged={result.converged} iters={result.iterations} conf={result.confidence:.2f}')
print(f'  trace len={len(result.trace)}')
for t in result.trace[-3:]:
    print(f'  iter={t.iteration} dx_norm={t.dx_norm:+.2f} dy_norm={t.dy_norm:+.2f} x={t.x_mm:.1f} y={t.y_mm:.1f}')
"
```

预期：
- step 1: 机械臂移动到 (0, -150, arm=0, hand=UP)（~2-3s）
- step 2: 视觉伺服跑 ≤8s；如果现场有蔬菜，converged=True 且 iterations ≥ 1；否则 iterations > 0 但 converged=False（不报错）

- [ ] **Step 4: TP4 - composite_run 4 路真并行 smoke**

```bash
python3 -c "
from main.arm import ArmClient
client = ArmClient.connect()
import time
print('TP4: composite_run 4 路真并行 (arm=-90, x=100mm, y=-150mm, hand=0)')
t0 = time.time()
result = client.composite_run(arm=-90.0, x_mm=100.0, y_mm=-150.0, hand=0.0, timeout=15.0)
print(f'TP4 result: {result}, elapsed={time.time()-t0:.2f}s')
"
```

预期：`composite_run` 返回 `{"ok": True, "steps": {"arm": True, "x": True, "y": True, "hand": True}}`，elapsed ≤ 3s（真并行而非 4×串行 ~10s）。

- [ ] **Step 5: 记录 + commit（仅当有发现时）**

如果有 bug / 异常发现，**先 stop** — 不要在这阶段改 vision.py，而是建新 task。记录到 `debug-2026-07-31-arm-vision-smoke.md`，并列 `.dbg/` artifacts（按 CLAUDE.md "Debug instrumentation" 约定）。

- [ ] **Step 6: 最终验证**

```bash
cd /home/xrak/Desktop/rak-car
python3 -m unittest discover -s main/arm/tests -v
```

预期：`Ran 32 tests in ... OK`

```bash
git log --oneline | head -15
```

预期看到本 plan 的 7 个 commits（Tasks 1-7 各一个 + Task 8 export/docs）。

---

## 自审（spec coverage）

| Spec § | 任务 |
|---|---|
| §3 Layer 1 `RuntimeApiClient` | Task 1 ✅ |
| §4 Layer 1.5 `labels.py` | Task 2 ✅ |
| §5.1 数据类型 | Task 3 ✅ |
| §5.2 `ArmVisionClient` + `TargetSelector` | Tasks 4 + 5 ✅ |
| §6 Layer 2.5 `ArmClient.composite_run / composite_run_reset / vision` | Task 6 ✅ |
| §7 Layer 3 `ArmRunner.move_to_vision_target / pick_by_vision` | Task 7 ✅ |
| §9 文档 + export | Task 8 ✅ |
| §10 真机验证（TP1-4） | Task 9 ✅ |

## Placeholder scan

- ❌ 无 "TBD" / "TODO"
- ❌ 无 "implement later"
- ❌ 无 "similar to Task N"（每步都展开代码）
- ✅ 所有代码块是完整可执行片段
- ✅ 测试用 `unittest`（项目无 pytest，避免引入）

## Type consistency

- `Detection` 在 Task 3 定义；Task 4 / 5 / 7 / 8 都引用 — ✅ 一致
- `TargetSelector` 在 Task 4 定义；Task 5 / 7 引用 — ✅
- `ArmVisionClient` 在 Task 5 定义；Task 6 / 7 / 8 引用 — ✅
- `ServoResult` 在 Task 5 定义；Task 7 返回类型 — ✅
- `composite_run` action 名（runtime 已注册）+ `arm` 业务方法名 — ✅ 对齐

---

**总任务数：9（实施 8 + smoke 1）**
**总代码量：~+1060 行（含单测 ~370 行）**
**预计提交：8 个 commits**
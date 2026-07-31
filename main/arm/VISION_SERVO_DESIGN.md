# 机械臂视觉伺服封装设计

> **状态**：draft, awaiting writing-plans
> **作者**：xrak (via Claude)
> **日期**：2026-07-31
> **关联提交**：`a48839d feat(arm): expose composite_run 并行驱动 + reset_position 并行化`（4 电机并行驱动已就位，本设计消费它）

## 1. 目标

`main/arm/` 是机械臂的 HTTP 业务封装，目前**完全是运动 + 状态层，零视觉代码**（已确认 grep 0 命中）。本设计在保持 CLAUDE.md 红线（"只调 `main.arm.*`，不要回退到 `client.call('arm', ...)`"）的前提下，叠加**视觉伺服**能力：

- 末端摄像头（= side cam / cam2 / USB1）检测 20 类业务目标
- `RuntimeApiClient` 暴露 vision 调用方法
- `main/arm/vision.py` 提供视觉伺服原语
- `main/arm/loops/runner.py` 提供高层组合（"目标到位 + 抓"）

业务层调用方最终能写：

```python
from main.arm import ArmClient, ArmRunner, Label, TargetSelector

runner = ArmRunner.connect()
result = runner.move_to_vision_target(
    selector=TargetSelector.for_label(Label.H_DOU_JIAO, strategy="highest_score"),
    x_mm=0, y_mm=-100, arm_angle=-90, hand=-90,
)
runner.pick_by_vision(Label.H_DOU_JIAO, x_mm=0, y_mm=-100, arm_angle=-90)
```

## 2. 架构（4 层）

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 4 · Business tasks (callers, e.g. car_task_function)  │
│   用 runner.pick_by_vision(label, x_mm, y_mm, arm_angle)    │
└──────────────┬──────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────┐
│ Layer 3 · ArmRunner  (main/arm/loops/runner.py, +2 方法)    │
│   move_to_vision_target()    — 粗定位 + 伺服                │
│   pick_by_vision()           — 粗定位 + 伺服 + composite_pick│
└──────────────┬──────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────┐
│ Layer 2 · ArmVisionClient + ArmClient  (main/arm/vision.py NEW)│
│   ArmVisionClient.find_target()        — 视觉伺服主路径     │
│   ArmVisionClient.find_targets_sequence() / pick_one()       │
│   ArmClient.composite_run() 薄封装  — 4 电机并行到位        │
└──────────────┬──────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────┐
│ Layer 1 · RuntimeApiClient  (main/api_client.py, +2 方法)    │
│   request_vision_task(...)  POST /v1/vision/task              │
│   get_vision_task_cache()   GET  /v1/realtime/vision/task     │
└──────────────┬──────────────────────────────────────────────┘
               │
          runtime HTTP :5050
               │
┌──────────────▼──────────────────────────────────────────────┐
│ Existing: ARM_ACTIONS (含 composite_run/composite_run_reset) │
│ Existing: infer_cfg [lane=5001, task=5002, ocr=5004]        │
│ Existing: task_feed daemon 30Hz                             │
└─────────────────────────────────────────────────────────────┘
```

**约束**：不动 runtime 一行代码 — 4 电机并行靠已有的 `composite_run`；视觉靠已有的 `/v1/vision/task` + `/v1/realtime/vision/task`。

## 3. Layer 1 · RuntimeApiClient 增量

文件：`main/api_client.py`，追加 2 个公开方法：

```python
class RuntimeApiClient:
    # ... 已有方法不变 ...

    def request_vision_task(
        self, *,
        sort_pos: Tuple[float, float] = (0.0, 0.0),
        limit_x: float = 1.0,
        limit_y: float = 1.0,
        timeout: float = 20.0,
    ) -> Dict[str, Any]:
        """POST /v1/vision/task — 同步单次推理（含 bbox_pixels）。
        
        返回 dict，结构对齐 runtime 服务定义：
            {"ok": bool, "model": "task", "camera": "cam2",
             "detections": [{"label", "score", "track_id", "bbox_norm", "bbox_pixels"}, ...],
             "count": int, "frame_shape": [h, w, c]}
        """
        return self._request(
            "POST",
            f"{self.api_prefix}/vision/task",
            payload={
                "sort_pos": list(sort_pos),
                "limit_x": float(limit_x),
                "limit_y": float(limit_y),
                "timeout": float(timeout),
            },
            timeout=timeout + 5.0,
        )

    def get_vision_task_cache(self) -> Dict[str, Any]:
        """GET /v1/realtime/vision/task — 读 task_feed 30Hz 缓存。
        
        返回 dict 结构（无 bbox_pixels）：
            {"ok": bool, "task_state": {"active": bool, "detections": [{"label", "score", "track_id", "bbox_norm"}], ...}}
        """
        return self._request("GET", f"{self.api_prefix}/realtime/vision/task")
```

实现要求：
- 走 `self._request(...)` 通用方法（与已有 `get_arm_state` / `get_task_state` 同模式）
- 不做结果解析（仅透传 JSON），由 Layer 2 在 `ArmVisionClient` 内解析
- 失败语义：HTTP error → 抛 `RuntimeError`（沿用 `_request` 既有行为）

## 4. Layer 1.5 · main/arm/labels.py（**新**）

文件：`main/arm/labels.py`，~90 行。20 项业务目标 catalog。

```python
"""业务目标类别 catalog —— 对齐 task backend 模型输出 (20 项)。

设计：
  - `Label` 是 str 子类 Enum，可直接当 str 传给 runtime
  - `LABELS` 是 (id, name, desc) 元组列表，按 id 升序（用户给定格式）
  - `LABEL_GROUPS` 是自然分组（animal / ball / cylinder / vegetable / water）
"""
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
    """label 名 → LabelInfo；不在表里抛 ValueError"""
    for info in LABELS:
        if info.name == name:
            return info
    raise ValueError(f"未知 label: {name!r}（共 20 项，参考 LABELS）")


def is_in_group(name: str, group: str) -> bool:
    return Label(name) in LABEL_GROUPS.get(group, ())
```

## 5. Layer 2 · main/arm/vision.py（**新**）

文件：`main/arm/vision.py`，~280 行。包含 `Detection` / `ServoTrace` / `ServoResult` / `TargetSelector` / `ArmVisionClient`。

### 5.1 数据类型

```python
@dataclass(frozen=True)
class BBoxNorm:
    x_center: float    # 归一化中心 [-1, +1]，0 = 图中心
    y_center: float
    width: float
    height: float

    @property
    def is_centered(self, tol: float = 0.05) -> bool:
        return abs(self.x_center) <= tol and abs(self.y_center) <= tol


@dataclass(frozen=True)
class BBoxPixels:
    x1: int; y1: int; x2: int; y2: int
    width: int; height: int


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    track_id: Optional[int]
    class_id: Optional[int]
    bbox_norm: BBoxNorm
    bbox_pixels: Optional[BBoxPixels]    # 缓存读时为 None
    fetched_at: float

    def __repr__(self) -> str:
        return f"Detection({self.label}#{self.track_id} score={self.score:.2f} cx={self.bbox_norm.x_center:+.2f})"


@dataclass(frozen=True)
class ServoTrace:
    t_s: float
    iteration: int
    dx_norm: float; dy_norm: float
    x_mm: float; y_mm: float
    score: float
    selected_track_id: Optional[int]


@dataclass(frozen=True)
class ServoResult:
    converged: bool
    selector: 'TargetSelector'
    x_mm: float; y_mm: float
    confidence: float
    iterations: int
    elapsed_s: float
    final_detection: Optional[Detection]
    trace: Tuple[ServoTrace, ...]   # 不可变


class SelectionStrategy(str, Enum):
    HIGHEST_SCORE      = "highest_score"
    CLOSEST_TO_CENTER  = "closest_to_center"
    LARGEST            = "largest"
    LEFTMOST           = "leftmost"
    RIGHTMOST          = "rightmost"
    TOPMOST            = "topmost"
    BOTTOMMOST         = "bottommost"
    LOCK_FIRST_SEEN    = "lock_first_seen"   # 首帧 track_id 锁定


@dataclass(frozen=True)
class TargetSelector:
    label: Optional[str] = None           # str 或 None（=任意 label）
    track_id: Optional[int] = None        # 锁定 track_id（用于跨帧稳定跟踪）
    strategy: str = SelectionStrategy.HIGHEST_SCORE.value
    group: Optional[str] = None           # 通过 LABEL_GROUPS 展开（None = 单 label）

    # ----- 工厂 -----
    @classmethod
    def for_label(cls, label, *, strategy: str = "highest_score") -> "TargetSelector":
        return cls(label=str(label.value if isinstance(label, Label) else label),
                   strategy=strategy)

    @classmethod
    def for_group(cls, group: str, *, strategy: str = "highest_score") -> "TargetSelector":
        if group not in LABEL_GROUPS:
            raise ValueError(f"未知 group: {group!r}（{list(LABEL_GROUPS)}）")
        return cls(label=None, strategy=strategy, group=group)

    def matches(self, det: Detection) -> bool:
        """det 是否被本 selector 接受（label/group 过滤）"""
        if self.group is not None:
            return det.label in [l.value for l in LABEL_GROUPS[self.group]]
        if self.label is not None:
            return det.label == self.label
        return True

    def apply_strategy(self, candidates: List[Detection]) -> Optional[Detection]:
        """对一组候选按 strategy 挑一个；空返回 None"""
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
        # LOCK_FIRST_SEEN 由 find_target 在循环里单独处理
        return candidates[0]
```

### 5.2 ArmVisionClient

```python
class ArmVisionClient:
    def __init__(self, http: RuntimeApiClient, *, default_timeout_s: float = 10.0):
        self.http = http
        self.default_timeout_s = default_timeout_s

    # ----- 元数据 -----
    @staticmethod
    def labels() -> Tuple[LabelInfo, ...]:
        return LABELS

    @staticmethod
    def group(name: str) -> Tuple[Label, ...]:
        return LABEL_GROUPS[name]

    # ----- 检测读 -----
    def get_state(self) -> List[Detection]:
        """读 task_feed 30Hz 缓存所有检测"""
        return _parse_cache(self.http.get_vision_task_cache())

    def get_state_filtered(self, selector: TargetSelector) -> List[Detection]:
        """读缓存 + selector 过滤"""
        return [d for d in self.get_state() if selector.matches(d)]

    def snap(self, *, sort_pos=(0.0, 0.0), limit_x=1.0, limit_y=1.0,
             timeout: float = 20.0) -> List[Detection]:
        """一次同步推理（含 bbox_pixels）"""
        return _parse_sync(self.http.request_vision_task(
            sort_pos=sort_pos, limit_x=limit_x, limit_y=limit_y, timeout=timeout))

    # ----- 多目标 -----
    def find_targets_sequence(self, selectors: List[TargetSelector], *,
                              x_mm: float, y_mm: float, **kwargs) -> List[ServoResult]:
        """按顺序对每个 selector 调 find_target；返回 list of ServoResult"""
        return [self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs) for sel in selectors]

    def pick_one(self, selectors: List[TargetSelector], *,
                 x_mm: float, y_mm: float, **kwargs) -> Optional[ServoResult]:
        """按优先级找第一个有命中的 selector 并伺服；返回首个成功的 ServoResult 或 None"""
        for sel in selectors:
            result = self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs)
            if result.converged:
                return result
        return None

    # ----- 主路径：视觉伺服 -----
    def find_target(self, selector: TargetSelector, *,
                    x_mm: float, y_mm: float,
                    mm_per_norm: float = 30.0,
                    settle_tol_norm: float = 0.05,
                    min_step_mm: float = 1.0,
                    max_iter: int = 500,
                    timeout: float = 10.0,
                    on_missing_track: str = "abort",   # "abort" | "wait"
                    move_fn: Optional[Callable[[float, float], dict]] = None,
                    ) -> ServoResult:
        """视觉伺服主路径：调缓存检测 → 微调 (x_mm, y_mm) → 收敛或超时。

        Args:
            selector: 选择器（label/group/strategy）
            x_mm, y_mm: 起始位姿（mm）
            mm_per_norm: bbox 归一化坐标 → mm 的转换系数（现场可调）
            settle_tol_norm: 收敛阈值（|dx_norm|<tol AND |dy_norm|<tol）
            min_step_mm: dead-band，单步 < 此值不发起 move（避免抖动）
            max_iter: 最大迭代次数（兜底）
            timeout: 总超时（秒）
            on_missing_track: 目标丢失行为（abort=5 帧无检测就 raise；wait=继续等）
            move_fn: 自定义 move 函数；默认 = client.move_xy(x_mm, y_mm)
                     （注入便于 main/arm/ 业务层替换为带软限位校验的版本）

        Returns:
            ServoResult(converged, x_mm, y_mm, confidence, iterations, elapsed_s,
                        final_detection, trace)

        Raises:
            ValueError: 当 y_mm 越界（被 ArmClient._check_safe 拦下）
        """
        t0 = time.time()
        trace: List[ServoTrace] = []
        locked_track_id: Optional[int] = None
        consecutive_misses = 0
        last_x_mm, last_y_mm = x_mm, y_mm
        last_detection: Optional[Detection] = None

        for i in range(max_iter):
            if time.time() - t0 > timeout:
                break
            candidates = self.get_state_filtered(selector)

            # lock_first_seen 锁定
            if selector.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if locked_track_id is None:
                    pick = selector.apply_strategy(candidates)
                    if pick is None:
                        consecutive_misses += 1
                        if consecutive_misses >= 5 and on_missing_track == "abort":
                            raise RuntimeError(f"find_target: 首帧未检测到 {selector}")
                        continue
                    locked_track_id = pick.track_id
                # 后续帧只跟 locked_track_id
                if selector.track_id is None:
                    selector = dataclasses.replace(selector, track_id=locked_track_id)
                candidates = [d for d in candidates if d.track_id == locked_track_id]
            elif selector.track_id is not None:
                candidates = [d for d in candidates if d.track_id == selector.track_id]

            pick = selector.apply_strategy(candidates) if candidates else None
            if pick is None:
                consecutive_misses += 1
                if on_missing_track == "abort" and consecutive_misses >= 5:
                    raise RuntimeError(f"find_target: 连续 {consecutive_misses} 帧未检测到 {selector}")
                continue
            consecutive_misses = 0
            last_detection = pick

            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center
            if abs(dx_norm) <= settle_tol_norm and abs(dy_norm) <= settle_tol_norm:
                # 收敛
                trace.append(ServoTrace(
                    t_s=time.time() - t0, iteration=i,
                    dx_norm=dx_norm, dy_norm=dy_norm,
                    x_mm=last_x_mm, y_mm=last_y_mm,
                    score=pick.score, selected_track_id=pick.track_id))
                return ServoResult(
                    converged=True, selector=selector,
                    x_mm=last_x_mm, y_mm=last_y_mm,
                    confidence=pick.score, iterations=i + 1,
                    elapsed_s=time.time() - t0,
                    final_detection=pick, trace=tuple(trace))

            # 方向取反：目标在右(dx>0)→ 车向左(-dx)
            dx_mm = -dx_norm * mm_per_norm
            dy_mm = -dy_norm * mm_per_norm
            if abs(dx_mm) < min_step_mm: dx_mm = 0.0
            if abs(dy_mm) < min_step_mm: dy_mm = 0.0

            new_x_mm = last_x_mm + dx_mm
            new_y_mm = last_y_mm + dy_mm
            trace.append(ServoTrace(
                t_s=time.time() - t0, iteration=i,
                dx_norm=dx_norm, dy_norm=dy_norm,
                x_mm=new_x_mm, y_mm=new_y_mm,
                score=pick.score, selected_track_id=pick.track_id))

            # move 注入
            if move_fn is not None:
                move_fn(new_x_mm, new_y_mm)
            else:
                # 默认路径：直接 _check_safe + move_xy 由 main/arm/ 业务层
                # 实际调用方在 ArmRunner 里注入带 _check_safe 的版本
                self.http.execute_arm_action("goto_position", x=new_x_mm/1000.0, y=new_y_mm/1000.0, timeout=5.0, sync=True)
            last_x_mm, last_y_mm = new_x_mm, new_y_mm

        # timeout / 超 max_iter
        return ServoResult(
            converged=False, selector=selector,
            x_mm=last_x_mm, y_mm=last_y_mm,
            confidence=last_detection.score if last_detection else 0.0,
            iterations=max_iter, elapsed_s=time.time() - t0,
            final_detection=last_detection, trace=tuple(trace))
```

### 5.3 解析辅助

```python
def _parse_cache(raw: Dict[str, Any]) -> List[Detection]:
    """GET /v1/realtime/vision/task → List[Detection]"""
    state = raw.get("task_state") or {}
    dets = state.get("detections") or []
    now = time.time()
    return [
        Detection(
            label=str(d["label"]),
            score=float(d["score"]),
            track_id=d.get("det_id") or d.get("track_id"),
            class_id=d.get("cls_id"),
            bbox_norm=BBoxNorm(
                float(d["bbox_norm"]["x_center"]),
                float(d["bbox_norm"]["y_center"]),
                float(d["bbox_norm"].get("width", 0.0)),
                float(d["bbox_norm"].get("height", 0.0)),
            ),
            bbox_pixels=None,
            fetched_at=float(state.get("updated_at") or now),
        )
        for d in dets
    ]


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

## 6. Layer 2.5 · ArmClient 增量（main/arm/api.py）

```python
class ArmClient:
    # ... 已有方法不变 ...

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
            arm=arm, x=_mm_to_m(x_mm) if x_mm is not None else None,
            y=_mm_to_m(y_mm) if y_mm is not None else None,
            hand=hand, speed=speed,
        )

    def composite_run_reset(self, *, arm_angle: float = 90.0, hand_angle: float = -90.0,
                            x_direction: str = "right", reset_x_velocity_mms: float = 20.0,
                            timeout: float = 60.0) -> dict:
        """薄封装 arm.composite_run_reset()"""
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
        """业务层用：返回一个 move_fn 已经被 _check_safe 包裹的 vision client"""
        client = ArmVisionClient(self.http)
        original = client.find_target

        def safe_find(selector, *, x_mm, y_mm, **kwargs):
            move_fn = kwargs.pop("move_fn", None)
            if move_fn is None:
                # 默认 move_fn：调自己的 move_xy（带 _check_safe + S 曲线 dry-run）
                def _safe_move(nx: float, ny: float) -> dict:
                    self._check_y_protected("find_target")
                    self._check_safe(y_mm=ny)
                    return self.move_xy(nx, ny, timeout=10.0)
                move_fn = _safe_move
            return original(selector, x_mm=x_mm, y_mm=y_mm, move_fn=move_fn, **kwargs)
        client.find_target = safe_find
        return client
```

## 7. Layer 3 · ArmRunner 增量（main/arm/loops/runner.py）

```python
class ArmRunner:
    # ... 已有方法不变 ...

    def move_to_vision_target(self, selector: TargetSelector, *,
                              x_mm: float, y_mm: float,
                              arm_angle: float = 0.0, hand: float = -90.0,
                              mm_per_norm: float = 30.0,
                              settle_tol_norm: float = 0.05,
                              timeout: float = 10.0) -> ServoResult:
        """高层组合：composite_run 粗定位 → 视觉伺服精调。

        业务前置：必须在 y < -30mm 保护区外（否则 composite_run._check_y_protected raise）。
        """
        # 1. 粗定位：arm + xy + hand=-90（抬手防撞）并发到位
        self.client.composite_run(arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0)
        # 2. 视觉伺服（带 _check_safe + S 曲线 dry-run 的 move_fn）
        return self.client._make_vision_with_move().find_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            mm_per_norm=mm_per_norm, settle_tol_norm=settle_tol_norm,
            timeout=timeout,
        )

    def pick_by_vision(self, selector: TargetSelector, *,
                       x_mm: float, y_mm: float, arm_angle: float = -90.0,
                       settle_tol_norm: float = 0.05, timeout: float = 10.0) -> dict:
        """最高层：粗定位 → 伺服 → composite_pick → grasp。

        业务前置：必须在 y < -30mm 保护区外。
        """
        self.move_to_vision_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            arm_angle=arm_angle, hand=-90.0,
            settle_tol_norm=settle_tol_norm, timeout=timeout,
        )
        # composite_pick：arm 并行 → hand=DOWN 串行 → grasp
        return self.client.composite_pick(
            arm_angle=arm_angle, x_mm=x_mm, y_mm=y_mm,
            hand=0.0, speed=80, timeout=30.0,
        )
```

## 8. 关键设计决策

| 决策 | 选择 | 理由 |
|---|---|---|
| 主路径用同步 vs 缓存 | **缓存**（30Hz task_feed）| 视觉伺服要 15-20Hz，同步 ~200ms 太慢 |
| 一次快照用同步 | `/v1/vision/task` | 拿 bbox_pixels + filter |
| 4 电机并行 | **`composite_run`**（已有）| 不重新发明；现成 API + 真机验证 |
| `cylinder_set` 处理 | **方案 1 — 透明** | 默认在 find_target 入口把 set 三等分为 1/2/3（保留可调用 `decompose_cylinder_set()` 显式分解） |
| frame↔mm 标定 | **相对位移**（`mm_per_norm`）| 绝对标定需现场采图，相对位移 + 单旋钮现场调 |
| 收敛判定 | `|dx_norm|<tol AND |dy_norm|<tol` | 默认 0.05（即 ~5% 框宽） |
| dead-band | `min_step_mm=1.0` | 避免抖动 |
| 目标丢失 | 5 帧连续未命中 abort | 默认 fail-fast；`on_missing_track="wait"` 改阻塞等待 |
| 业务层 move 注入 | `move_fn` 回调 | ArmRunner 注入带 _check_safe 的版本；纯 ArmVisionClient 直接走 HTTP |

## 9. 文件清单（实施时改）

| 文件 | 状态 | 行数 |
|---|---|---|
| `main/api_client.py` | 编辑（+2 方法） | +50 |
| `main/arm/labels.py` | **新** | +90 |
| `main/arm/vision.py` | **新** | +280 |
| `main/arm/api.py` | 编辑（+3 方法 + 1 属性） | +60 |
| `main/arm/loops/runner.py` | 编辑（+2 方法） | +60 |
| `main/arm/__init__.py` | 编辑（export） | +10 |
| `main/arm/ARM_API.md` | 编辑（文档） | +220 |
| **总计** | | **+770 行** |

## 10. 实施阶段

1. **阶段 0** — `git pull` 验证 composite_run 已就位 ✅（已完成）
2. **阶段 1** — Layer 1：RuntimeApiClient 加 2 方法；本地读 API 文档 + curl 验证返回结构
3. **阶段 2** — Layer 1.5：写 `labels.py`，单元测试覆盖 20 项映射 + group 查询
4. **阶段 3** — Layer 2：写 `vision.py` + 解析器；本地静态自检
5. **阶段 4** — Layer 2.5：`ArmClient.composite_run / composite_run_reset` 薄封装；curl `/v1/execute` 验证可达
6. **阶段 5** — Layer 3：`ArmRunner.move_to_vision_target / pick_by_vision`；写 demo `main/arm/examples/05_visual_servo_demo.py`
7. **阶段 6** — 真机验证：
   - TP1: 静止 side cam 检测一个目标（label=h_dou_jiao 或 cylinder_1）— `snap()` 返回非空
   - TP2: `ArmRunner.pick_by_vision(Label.H_DOU_JIAO, x_mm=0, y_mm=-100, arm_angle=-90)` — 端到端跑通
   - TP3: 多目标场景 `find_targets_sequence([Label.H_FAN_QIE, Label.H_QIN_CAI], ...)`
   - TP4: `cylinder_set` 透明分解 → 3 个 cylinder_* 都被识别
8. **阶段 7** — 文档 + commit；ARM_API.md 补 §3 / §4

## 11. 风险与回退

| 风险 | 缓解 |
|---|---|
| 目标在帧间闪烁 / 丢失 | 5 帧未命中 abort（默认）；可改 `on_missing_track="wait"` |
| 电机未到位就下发下一步 → 振荡 | `min_step_mm=1.0` dead-band；单步只下一路 move |
| `mm_per_norm` 估错 | log 每步 `dx_norm → dx_mm`；用户看 log 现场调 |
| y 进入保护区 → 撞车 | `_check_y_protected` 在每步 pre-check；保护区 raise `ValueError` |
| track_id 不稳（模型抖动）| `lock_first_seen` 退化到 `highest_score`（不报错） |
| composite_run 上 hand=DOWN 撞车 | `move_to_vision_target` 内部 hand 默认 -90；`pick_by_vision` 才用 0 |

## 12. Out of Scope（不做）

- ❌ 绝对 frame↔mm 标定（homography）— 用相对位移 + `mm_per_norm` 参数
- ❌ WebSocket 实时订阅（`subscribe_task_detection`）— HTTP GET 缓存 30Hz 够用
- ❌ 自定义目标检测模型 — 用现有 task backend (5002)
- ❌ 多机械臂并行 — 单车单臂
- ❌ 深度估计 — bbox_norm 单目近似即可

## 13. 自审（placeholder / ambiguity / contradiction）

- ✅ 无 TBD / TODO 占位
- ✅ 数据类型字段全
- ✅ 失败语义明确（raise ValueError / RuntimeError 区分）
- ✅ 边界条件覆盖（consecutive_misses, min_step_mm, settle_tol_norm）
- ✅ 复用已有 `composite_run` 而非重新实现
- ✅ 不动 runtime 一行
- ⚠️ `cylinder_set` 透明分解的具体 bbox 切分策略（在实施阶段细化；MVP 用三等分）
- ⚠️ 视觉伺服 `move_fn` 默认实现走 `arm.goto_position` 而非 `move_xy`（性能优先于 dry-run 日志），业务层 ArmRunner 注入带 `move_xy` 的版本
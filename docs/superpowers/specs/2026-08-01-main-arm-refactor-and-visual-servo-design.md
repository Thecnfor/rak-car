# main/arm 重整 + 视觉伺服算法优化

> **状态**：draft, 用户已批准方向
> **作者**：Thecnfor（via Claude）
> **日期**：2026-08-01
> **关联**：[`ARM_API.md`](../../../main/arm/ARM_API.md) · [`VISION_SERVO_DESIGN.md`](../../../main/arm/VISION_SERVO_DESIGN.md) · [`2026-07-31-runtime-refactor-design.md`](./2026-07-31-runtime-refactor-design.md)

## 0. 目标

本 spec 解决两件事：

1. **架构重整**：把 `main/arm/api.py`（953 行）和 `main/arm/vision.py`（702 行）按职责拆为 mixin 聚合，目标单文件 < 400 行。
2. **视觉伺服算法优化**：4 自由度（y / x / 大臂 / 末端爪）在 3 维空间内，对目标做"框居中 + 尺度匹配"追踪。物理世界配合现场调参。

## 1. 背景与约束

### 1.1 现有规模

| 文件 | 行 | 角色 | 评价 |
|---|---|---|---|
| `main/arm/api.py` | 953 | ArmClient 业务动作全集 | 显胖，但逻辑自洽 |
| `main/arm/vision.py` | 702 | ArmVisionClient 视觉伺服 | 3 个 find_* 循环，重复骨架多 |
| `main/arm/loops/runner.py` | 306 | ArmRunner 编排 | 合理，不拆 |
| `main/arm/state.py` / `origin.py` / `labels.py` / `trajectory.py` | 629 | 状态/原点/标签/轨迹 | 合理，不拆 |
| `main/arm/tasks/` (4 文件) | 99 | thin wrapper | 合理，不拆 |
| `main/arm/examples/` | 89 | 真机 smoke | 用户已说"不要动" |
| `main/arm/tests/` (10 文件) | 960 | mock 单测 | 公开 API 不动 → 零修改 |
| `main/arm/__init__.py` | 46 | export 10 个符号 | 公开符号零变动 |
| **合计** | **3784** | | |

### 1.2 硬约束

- **冻结区**：`runtime/` / `smartcar/` / `config_car.yml` / `ecosystem.config.js` 不动
- **可改区**：`main/`（项目硬约束，本 spec 在主航道）
- **公共 API 100% 兼容**：`from main.arm import ...` 全部符号不变；`ArmClient.method()` / `ArmRunner.method()` / `ArmVisionClient.method()` 名字、签名、默认参数、返回值零变动
- **业务层零改动**：`tasks/` / `examples/` / 任何调用 `main.arm.*` 的代码不动
- **tests/ 零修改**：原有 10 个 mock 单测继续过；新增 1-2 个薄烟测验新结构

### 1.3 当前视觉伺服的不足

现有 `find_target` / `find_target_realtime` / `find_target_track` 用最简单的 PD：dx_mm = -dx_norm * mm_per_norm（线性比例，无积分、无死区细化、无深度补偿）。问题：

| 问题 | 表现 | 根因 |
|---|---|---|
| 收敛震荡 | 居中时来回反复，encoder 抖动 | 死区（dead-band）只有 min_step_mm=1.0 一道，无 adaptive；PID 无 D 项 |
| 远距冲过头 | 远目标大步走过去，近端很难拉回 | 单固定 mm_per_norm 系数；远目标应大比例，近目标应小比例 |
| 深度不知道 | 抓取时不知道目标离摄像头多远 | 只看 bbox 中心，不看 bbox 大小（尺度信息未利用） |
| 4 自由度只用 2 个 | y/x 在动，大臂/手爪闲置 | bbox 偏移 → 端到端视觉映射到 4 自由度策略没设计 |
| 大目标/小目标 | 同 selector 没区别 | 同样只看中心 |
| 收敛后还会有 settle_jitter | 抓取时轻微偏移 | min_step_mm=1.0 死区太宽 |

## 2. 重整方案（架构）

### 2.1 api.py 拆分为 9 文件

`main/arm/api.py`（953 行）→ 8 个 mixin + 1 个聚合类：

| 新文件 | 行（估） | 包含 | MRO 顺序 |
|---|---|---|---|
| `api/safety.py` | 140 | `SafetyMixin`: `_check_safe` / `_check_y_protected` / `_validate_arm_angle_client` / `_validate_hand_angle_client` / `_check_step_loss` + 类常量（`_Y_PROTECTED_THRESHOLD_MM` / `_ARM_ANGLE_MIN/MAX` / `_HAND_ANGLE_MIN/MAX` / `_ARM_SAFE_BAND_MIN/MAX`） | 最先（被其他 mixin 调） |
| `api/motion.py` | 150 | `MotionMixin`: `set_pose` / `move_xy` / `move_x` / `move_y` | 2 |
| `api/setters.py` | 110 | `SettersMixin`: `set_arm_angle` / `set_hand_angle` | 2 |
| `api/composite.py` | 150 | `CompositeMixin`: `composite_pick` / `composite_release` / `composite_go_home` / `composite_run` / `composite_run_reset` | 3 |
| `api/reset_ops.py` | 110 | `ResetOpsMixin`: `reset_y` / `reset_x` / `reset_all` / `reset_origin` | 3 |
| `api/state_io.py` | 150 | `StateIOMixin`: `get_state` / `get_pose_mm` / `get_x_mm` / `get_y_mm` / `_read_raw_state` / `emergency_stop` / `ping` | 4 |
| `api/storage.py` | 90 | `StorageMixin`: `set_storage` / `get_storage` / `set_storage_angle` + `_normalize_storage_side` | 3 |
| `api/vis_servo.py` | 70 | `VisServoMixin`: `_make_vision_with_move` + `vision` property | 5 |
| `api/__init__.py` | 80 | `ArmClient` 聚合类（保留 `connect()` / `_load_origin_or_default` / `_read_origin_yaml` / `save_origin` / `_call_arm` / `_call_car` + `_origin_path`），MRO = `(MecanumDriver, SafetyMixin, MotionMixin, SettersMixin, CompositeMixin, ResetOpsMixin, StorageMixin, StateIOMixin, VisServoMixin)` | — |

`★ Insight`：api.py 不再"超 400 行"约束后，自然就 < 200 行；origin yaml 读写 / `_call_arm` / `_call_car` 这些"低层胶水"放在聚合类里——它们不是单个 mixin 的责任，是 ArmClient 作为 client 的本职。

### 2.2 vision.py 拆分为 6 文件

`main/arm/vision.py`（702 行）→ 5 个模块 + 1 个聚合类：

| 新文件 | 行（估） | 包含 |
|---|---|---|
| `vision/types.py` | 110 | `BBoxNorm` / `BBoxPixels` / `Detection` / `ServoTrace` / `ServoResult` dataclass |
| `vision/parsers.py` | 80 | `_parse_cache` / `_parse_sync`（私有名不导出，但只在本子包里用） |
| `vision/selector.py` | 120 | `SelectionStrategy` enum + `TargetSelector` dataclass + `apply_strategy` / `matches` |
| `vision/servo.py` | 200 | `find_target` HTTP 轮询主循环（不变） |
| `vision/realtime.py` | 220 | `find_target_realtime` WS 推送 + `find_target_track` 持续追踪 |
| `vision/__init__.py` | 80 | `ArmVisionClient` 聚合类（保留 `__init__` / `labels()` / `group()` / `get_state()` / `get_state_filtered()` / `snap()`），MRO = `(ServoLoop, RealtimeLoop)` |

`★ Insight`：vision 三个 find_* 循环骨架重复（"读 cache → apply selector → dead-band → 下发"），拆完再 §3 重构它们的算法时才有空间。

### 2.3 import 边界

每个 mixin 只 import 自己直接依赖：

```
api/safety.py     : 无内部依赖
api/motion.py     : api/safety.py
api/setters.py    : api/safety.py
api/composite.py  : api/safety.py
api/reset_ops.py  : 无内部依赖（只调 _call_arm）
api/state_io.py   : 无内部依赖（只调 _call_arm / _call_car）
api/storage.py    : 无内部依赖
api/vis_servo.py  : vision/__init__.py（懒 import 防循环）
api/__init__.py   : 8 个 mixin + 聚合

vision/types.py    : 无
vision/parsers.py  : vision/types.py
vision/selector.py : vision/types.py
vision/servo.py    : vision/types.py + vision/parsers.py + vision/selector.py
vision/realtime.py : 同上 + RuntimeWsClient（懒 import）
vision/__init__.py : 上述全部 + ArmVisionClient 聚合
```

无循环 import。

### 2.4 MRO 验证

`ArmClient.__mro__`（包含 MecanumDriver 是历史保留，main/arm 业务不直接继承硬件）：

```python
class ArmClient(SafetyMixin, MotionMixin, SettersMixin, CompositeMixin,
                 ResetOpsMixin, StorageMixin, StateIOMixin, VisServoMixin):
    pass
```

`super().method()` 调用链检查（每个 mixin 不调 super()，仅用 `self._xxx` / `self.arm` / `self.http` / `self.origin`）。**安全**——不像 runtime `my_car` 那样有 `close()` 必须留聚合类。

### 2.5 公共 API 零变动

| 符号 | 当前位置 | 拆后位置 | 是否变动 |
|---|---|---|---|
| `ArmClient` | `main/arm/api.py` | `main/arm/api/__init__.py` | import 路径不变（`main.arm` 是包，Python 自动找 `__init__.py`） |
| `ArmRunner` | `main/arm/loops/runner.py` | 不动 | 否 |
| `ArmState` / `ArmOrigin` / 枚举 | `main/arm/state.py` | 不动 | 否 |
| `OriginCalibrator` / `run_calibrator` | `main/arm/origin.py` | 不动 | 否 |
| `TrajectoryGenerator` / `TrajectoryPlan` / `TrajectorySample` | `main/arm/trajectory.py` | 不动 | 否 |
| `Label` / `LabelInfo` / `LABELS` / `LABEL_GROUPS` | `main/arm/labels.py` | 不动 | 否 |
| `ArmVisionClient` / `Detection` / `BBoxNorm` / `BBoxPixels` / `TargetSelector` / `SelectionStrategy` / `ServoTrace` / `ServoResult` | `main/arm/vision.py` | `main/arm/vision/__init__.py` 等 | import 路径不变 |

## 3. 视觉伺服算法优化

### 3.1 现有算法回顾

```python
# vision.py 现有 find_target
dx_mm = -dx_norm * mm_per_norm        # 比例项，P 控制
dy_mm = -dy_norm * mm_per_norm
if abs(dx_mm) < min_step_mm: dx_mm = 0  # 死区
if abs(dy_mm) < min_step_mm: dy_mm = 0
```

问题：

- 无积分 → 比例系数偏小就拉不回，偏大就震荡
- 无深度感知 → 远近用同 mm_per_norm
- 无 4 自由度策略 → y/x/大臂/手爪没分工
- 无 adaptive gain → 远目标大步走，近目标精细调

### 3.2 新算法：4-DOF 视觉伺服（PID + 尺度 + 4 自由度策略）

引入两个核心概念：

#### 3.2.1 深度估计：从 bbox 高度反推距离

```
depth_m = (target_real_height_m * focal_length_px) / bbox_height_px
```

- `target_real_height_m`：目标的物理高度（如青椒 ~0.15m），由 `Label` 元数据提供
- `focal_length_px`：摄像头焦距像素值，由 `arm_origin.yaml` 一次性标定
- `bbox_height_px`：检测器输出的 bbox 像素高

深度估计的好处：

- 同一 selector 用同一 `target_real_height_m`，深度稳定
- 抓取距离校验（depth < grasp_depth_m 触发 grasp）

#### 3.2.2 adaptive gain：根据距离动态调比例

```
mm_per_norm_eff = mm_per_norm_base * (depth_m / ref_depth_m)
```

- `ref_depth_m`：参考深度（默认 0.30m，对应 bbox_height_px_ref）
- 远目标 → gain 大 → 大步快走；近目标 → gain 小 → 小步精调

#### 3.2.3 PID 控制替代纯 P

```
error = (dx_norm, dy_norm)
P = error
I += error * dt
D = (error - last_error) / dt
output_norm = Kp * P + Ki * I + Kd * D
output_mm = -output_norm * mm_per_norm_eff
```

- 默认 Kp=1.0 / Ki=0.05 / Kd=0.2（Ki 留 0.05 让稳态误差收敛，Kd 抑制震荡）
- 收敛后 Ki 自动冻结（避免积分饱和）

#### 3.2.4 4 自由度策略：把 bbox 偏移映射到 (y, x, arm, hand)

目标在视野中的位置 → 4 自由度策略：

| bbox 位置 / 尺度 | 触发动作 |
|---|---|
| `\|dx_norm\| > x_tol` | y/x 调中心（PID 输出） |
| `\|dy_norm\| > y_tol` | y 调中心（PID 输出） |
| `\|dx_norm\| > 0.3` 大偏移 | 大臂转（用 `composite_run(arm=...)` 一把到位） |
| bbox_height_px < min_px | 手爪 DOWN（`set_hand_angle(0)`） |
| bbox_height_px >= ideal_px 范围 | 抓取（`grasp(True)`） |

具体阈值在 §3.4 给出"默认参数表"，物理世界配合时按现场调。

#### 3.2.5 收敛判据 + settle 二次校验

- 一次收敛：`|dx_norm| < tol AND |dy_norm| < tol`（当前实现）
- 新增：连续 N 帧（默认 3）满足一次收敛才算"稳定收敛"，返回 `ServoResult(converged=True)`，否则继续追
- 目的：避免单帧抖动误判

#### 3.2.6 4 自由度并发：利用 composite_run 一次发多个轴

主路径仍然 y/x PID 闭环（车端 PID 兜底）。当需要"大动作"时（大臂转、手爪 DOWN），发一次 `composite_run` 把多轴一起动，避免串行时延。

### 3.3 公共 API 变化

| 函数 | 变化 |
|---|---|
| `ArmVisionClient.find_target` | **签名加 5 个可选参数**（默认不传走原行为）：`depth_m` / `kp` / `ki` / `kd` / `target_real_height_m` / `focal_length_px` |
| `ArmVisionClient.find_target_realtime` | 同上 |
| `ArmVisionClient.find_target_track` | 同上 |
| `ArmRunner.move_to_vision_target` | **同**（调用 find_target 透传） |
| `ArmRunner.move_to_vision_target_realtime` | 同上 |
| `ArmRunner.pick_by_vision` | **新增 4 自由度策略**：自动在合适时机用 composite_run 转大臂、手爪，触发 grasp |
| `ArmRunner.pick_by_vision_realtime` | 同上 |
| `ArmRunner.track_vision_target` | 同上 |
| **新增**：`ArmVisionClient.compute_depth(bbox, target_real_height_m, focal_length_px)` | 显式深度估计入口 |
| **新增**：`ArmVisionClient.servo_4dof(selector, *, x_mm, y_mm, arm_angle=0, hand=-90, ...)` | 4 自由度高层：粗定位 → bbox 居中 → 4 自由度策略触发 |

公共 API 兼容策略：**所有新参数都有默认值，调用方零改动**。新方法用新名字。

### 3.4 默认参数表（物理世界配合入口）

| 参数 | 默认值 | 物理含义 |
|---|---|---|
| `mm_per_norm_base` | 30.0 mm | 单位归一化偏移 → mm（基准 0.30m 深度下） |
| `ref_depth_m` | 0.30 m | adaptive gain 参考深度 |
| `focal_length_px` | 600.0 px | USB 摄像头典型值，待现场标定 |
| `target_real_height_m` | None（按 Label 自动） | 业务目标的真实物理高度；`Label` 元数据先填 8 项主要类别（cylinder/ball/4 种蔬菜/animal/water） |
| `kp` / `ki` / `kd` | 1.0 / 0.05 / 0.2 | PID 三项增益 |
| `settle_tol_norm` | 0.05 | 单次收敛阈值 |
| `settle_stable_frames` | 3 | 稳定收敛需连续帧数 |
| `min_step_mm` | 1.0 | 死区 |
| `grasp_bbox_height_px_min` | 80 | bbox 高度达到此值触发 grasp |
| `arm_dx_threshold_norm` | 0.3 | 大偏移触发大臂转 |

现场调节入口：把这些参数从 `ArmClient` 的实例属性读，business 脚本里直接 `client.vision.kp = 1.5` 临时改。

### 3.5 Label 元数据扩展

`main/arm/labels.py` 加 `real_height_m` 字段：

```python
@dataclass(frozen=True)
class LabelInfo:
    id: int
    name: str
    desc: str
    real_height_m: float = 0.0  # 业务目标物理高度（米），用于深度估计
```

8 项主要类别填值（待现场校准）：

| label | real_height_m | 备注 |
|---|---|---|
| `cylinder_1/2/3` | 0.10 | 圆柱体 |
| `ball_blue` / `ball_yellow` | 0.06 | 球 |
| `h_dou_jiao` (豆角) | 0.20 | 长豆角 |
| `h_fan_qie` (番茄) | 0.07 | 圆番茄 |
| `h_qing_jiao` (青椒) | 0.10 | |
| `h_tu_dou` (土豆) | 0.08 | |
| `animal` | 0.30 | 动物模型（最大） |
| `water` | 0.15 | 水容器 |

其它 label 默认 0.0（不参与深度估计）。

## 4. 数据流

### 4.1 重整后的导入链

```
main/arm/__init__.py
├── api/__init__.py → ArmClient (聚合)
│   ├── api/safety.py → SafetyMixin
│   ├── api/motion.py → MotionMixin
│   ├── api/setters.py → SettersMixin
│   ├── api/composite.py → CompositeMixin
│   ├── api/reset_ops.py → ResetOpsMixin
│   ├── api/storage.py → StorageMixin
│   ├── api/state_io.py → StateIOMixin
│   └── api/vis_servo.py → VisServoMixin
├── vision/__init__.py → ArmVisionClient (聚合)
│   ├── vision/types.py → DTO
│   ├── vision/parsers.py → 解析
│   ├── vision/selector.py → 选择器
│   ├── vision/servo.py → HTTP 轮询
│   └── vision/realtime.py → WS 推送
├── state.py / origin.py / labels.py / trajectory.py
├── loops/runner.py → ArmRunner
└── tasks/ (4 files) → 薄包装
```

### 4.2 算法升级后的运行时调用

```python
runner = ArmRunner.connect()
result = runner.pick_by_vision(
    selector=TargetSelector.for_label(Label.H_DOU_JIAO),
    x_mm=100, y_mm=-150, arm_angle=-90,
    # 新参数（默认不传也跑得通）：
    # kp=1.0, ki=0.05, kd=0.2,
    # target_real_height_m=None,  # 自动从 Label 读
    # focal_length_px=600.0,
    # settle_stable_frames=3,
)
# → composite_run(arm=-90, x=100, y=-150) → servo_4dof 循环 → composite_pick → grasp
```

## 5. 错误处理

- **PID 饱和**：当 PID 输出超过 max_output_norm（默认 1.0）时截断
- **深度估失败**（bbox_height_px = 0）：用 ref_depth_m 兜底，不抛错
- **4 自由度动作失败**：composite_run 返回 ok=False 时，回退到单轴重试
- **稳定收敛超时**：到 max_iter 仍未稳定 → 返回 `converged=False, settle_stable=False`
- **目标丢失（5 帧）**：保持现有 `on_missing_track="abort"` 语义，抛 RuntimeError
- **新算法 vs 旧算法**：旧 `find_target`（纯 P）保留为 `find_target_legacy` 入口，便于回归测试

## 6. 测试

### 6.1 重整测试

新增 2 个薄烟测：

| 文件 | 验 |
|---|---|
| `tests/test_imports.py` | `from main.arm import ArmClient, ArmRunner, ArmVisionClient, TargetSelector, Label, ...` 全部成功 |
| `tests/test_aggregate_mro.py` | `ArmClient.__mro__` 顺序正确；`ArmVisionClient.__mro__` 顺序正确；每个方法都能从 `dir()` 找到 |

### 6.2 算法优化测试

| 文件 | 验 |
|---|---|
| `tests/test_servo_pid.py` | mock bbox 输入，验证 PID 输出公式正确（给定 kp/ki/kd 输入，验证 dx_mm） |
| `tests/test_servo_depth.py` | `compute_depth` 在已知 focal/height 下深度正确；bbox=0 时返回 ref_depth_m |
| `tests/test_servo_4dof.py` | mock Detection 序列，验证 4 自由度触发时机（大偏移→arm 转，bbox_height 达标→grasp） |
| `tests/test_servo_settle.py` | 单帧抖动不应触发稳定收敛；连续 3 帧满足阈值才 converged=True |
| **保留**：`test_vision_find_target.py` / `test_vision_realtime.py` 等 10 个 | 不变（公开 API 100% 兼容） |

## 7. 提交粒度

按"独立可回滚"分 4 commit：

| 序号 | commit | 内容 |
|---|---|---|
| 1 | `refactor(arm/api): 953 行 → 8 mixin + 1 聚合类` | 拆 api.py |
| 2 | `refactor(arm/vision): 702 行 → 5 模块 + 1 聚合类` | 拆 vision.py |
| 3 | `perf(arm/vision): 4-DOF 视觉伺服（PID + 深度 + 4 自由度策略）` | 算法升级 |
| 4 | `test(arm): 薄烟测（imports + MRO） + 算法单测（PID/depth/4dof/settle）` | 测试 |

每个 commit 独立可 `git revert`。

## 8. 风险与缓解

| 风险 | 缓解 |
|---|---|
| 拆文件漏 import | 拆完跑 `python -c "from main.arm import *"` + 跑 tests/ 全部 10 个单测 |
| MRO 顺序错（safety 排在 motion 后） | 写完调 `ArmClient.__mro__` 打印验证；写单测 |
| PID 系数现场调不好 | 默认值保守（kp=1.0 ki=0.05 kd=0.2），所有系数可运行时覆写 |
| 深度估计不准确（焦距没标） | `focal_length_px` 默认 600 待现场测；估计值与真值偏差通过 `mm_per_norm_base` 兜底 |
| 4 自由度策略触发误判 | 阈值保守（大偏移 0.3 才转大臂），物理世界配合时再调 |
| 公共 API 签名破坏 | 所有新参数都默认；不删任何方法；旧 `find_target` 行为用 `find_target_legacy` 保留 |
| 真机验证耗时长 | §3 算法默认走保守系数，先在 examples/05 smoke 上跑通 4 步；4 自由度策略本轮不抓取（只居中 + 触发 grasp 走 `pick_by_vision` 已有路径） |

## 9. 验收标准

- [ ] `main/arm/api/` 拆为 8 mixin + 1 聚合类（每文件 < 200 行）
- [ ] `main/arm/vision/` 拆为 5 模块 + 1 聚合类（每文件 < 250 行）
- [ ] `python -c "from main.arm import *"` 通过
- [ ] `python -m pytest main/arm/tests/` 全部 10 个原单测通过
- [ ] 新增 test_imports.py + test_aggregate_mro.py + test_servo_pid.py + test_servo_depth.py + test_servo_4dof.py + test_servo_settle.py 通过
- [ ] examples/05_visual_servo_smoke.py 真机跑通 TP1/TP2/TP3/TP4 全部
- [ ] 公共 import 路径零变化（`from main.arm import ArmClient` 仍可用）
- [ ] 任务函数 `go_home/pick_left/pick_right/release` 行为零变化
- [ ] `find_target` 旧行为保留（默认参数走 PID+深度+4DOF 链；新参数全默认；旧调用方零改动）

## 10. 不在本 spec 范围

- runtime 重构（已 4 phase 完成）
- 摄像头焦距自动标定（先用默认 600.0 像素；现场手测一次后写回 `arm_origin.yaml`）
- 多目标同时抓取（一次只抓一个）
- 8 类以外 label 的 `real_height_m` 字段（默认 0.0）

## 11. 物理世界配合（关键里程碑）

- 现场跑通 `examples/05_visual_servo_smoke.py` 的 TP1/TP2/TP3/TP4 → 校准焦距 → 写回 `arm_origin.yaml`
- 跑通 `runner.pick_by_vision(label, x_mm, y_mm, arm_angle)` → 验证 4 自由度策略触发 → 抓取成功
- 把现场实际调好的 PID 系数（kp/ki/kd）写回 spec §3.4 默认值表

---

## 附 A：与上次 runtime 重构的对照

| 维度 | runtime 重构 | 本次 main/arm 重整 |
|---|---|---|
| 触发 | 9430 行屎山 | 953 + 702 单文件胖 |
| 拆解方式 | mixin 聚合（4+6+1）| mixin 聚合（8+5+1）|
| 公共 API 兼容 | 100% | 100% |
| 测试保留 | 全部 | 全部 |
| 额外 | 重构 + clean | 重构 + 算法优化 |
| 提交 | 4 phase 拆开 | 4 commit（拆 + 拆 + 优化 + 测试） |
| 不动区 | hardware/ | examples/ |

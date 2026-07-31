# 实时视觉伺服 — 设计

> **状态**：draft, 实施中
> **作者**：xrak (via Claude)
> **日期**：2026-07-31
> **关联**：[VISION_SERVO_DESIGN.md](./VISION_SERVO_DESIGN.md) (HTTP cache 版本) · [VISION_SERVO_PLAN.md](./VISION_SERVO_PLAN.md)

## 1. 目标

把 `find_target` 从 **HTTP GET 30Hz 轮询** 升级到 **WS push** 模式：
- 服务端主动推送 `task_state`（边走边看）
- client 收到推送 → 立刻算校正 → 立刻发 move action
- **底层 fix**：`task_push_hz` 默认 10Hz 提升到 30Hz（匹配 `task_feed` 实际频率）

## 2. 为什么需要实时

### HTTP cache 路径的限制

- 每次 HTTP GET 5-10ms 延迟
- 主动轮询 — 没数据也在 ping
- 多客户端时**重复消费** server 资源

### WS push 路径的收益

- 服务端主动推送，延迟 ~5ms
- 多客户端复用同一 task_feed（不增加服务端 GPU 负担）
- 30Hz 满频率（cache 也是 30Hz 但需要 client 主动 GET）
- 框架干净（callback-based，不用 while-loop 轮询）

## 3. 架构（增量）

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 4 · ArmRunner.move_to_vision_target_realtime()        │
│   用 WS 路径替代 HTTP cache（行为同 HTTP 版）              │
└──────────────┬──────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────┐
│ Layer 2.5 · ArmVisionClient.find_target_realtime()          │
│   - 用 RuntimeWsClient.subscribe_task_detection 订阅推送    │
│   - callback: 收到 task_state → 解析 Detection → 算校正   │
│   - 复用 Detection / TargetSelector / 收敛判断 / trace    │
│   - move_fn 注入（默认走 _make_vision_with_move）           │
└──────────────┬──────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────┐
│ Layer 1 · RuntimeWsClient.subscribe_task_detection (已有)    │
│   main/ws_client.py:276 — hz=10.0 → 我们用 hz=30.0         │
└──────────────┬──────────────────────────────────────────────┘
               │
       runtime WS /v1/ws
               │
┌──────────────▼──────────────────────────────────────────────┐
│ Runtime (底层 fix)                                          │
│   runtime/api/routes.py:1422                                │
│   task_push_hz: 10.0  →  30.0  (匹配 task_feed)             │
└─────────────────────────────────────────────────────────────┘
```

## 4. API 表面（Layer 2.5 + Layer 4）

```python
class ArmVisionClient:
    # 已有 HTTP 版本
    def find_target(self, selector, *, x_mm, y_mm, ...) -> ServoResult: ...

    # 新增 WS 版本（行为一致，路径不同）
    def find_target_realtime(self, selector, *, x_mm, y_mm,
                             hz: float = 30.0,
                             mm_per_norm: float = 30.0,
                             settle_tol_norm: float = 0.05,
                             min_step_mm: float = 1.0,
                             max_iter: int = 500,
                             timeout: float = 10.0,
                             on_missing_track: str = "abort",
                             move_fn: Optional[Callable[[float, float], dict]] = None,
                             ws: Optional[RuntimeWsClient] = None,
                             ) -> ServoResult: ...

class ArmRunner:
    # 已有 HTTP 版本
    def move_to_vision_target(self, selector, *, x_mm, y_mm, ...) -> ServoResult: ...
    def pick_by_vision(self, selector, *, x_mm, y_mm, ...) -> dict: ...

    # 新增 WS 版本（高层包装）
    def move_to_vision_target_realtime(self, selector, *, x_mm, y_mm,
                                        arm_angle: float = 0.0, hand: float = -90.0,
                                        hz: float = 30.0,
                                        mm_per_norm: float = 30.0,
                                        settle_tol_norm: float = 0.05,
                                        timeout: float = 10.0) -> ServoResult: ...

    def pick_by_vision_realtime(self, selector, *, x_mm, y_mm,
                                 arm_angle: float = -90.0,
                                 settle_tol_norm: float = 0.05,
                                 timeout: float = 10.0) -> dict: ...
```

**向后兼容**：HTTP 路径 (`find_target` / `move_to_vision_target` / `pick_by_vision`) 全部保留，不改。

## 5. 关键设计点

| 决策 | 选择 | 理由 |
|---|---|---|
| WS 推送 hz | **30.0** | 匹配 task_feed 上限 |
| WS 客户端复用 | 每次 find_target_realtime 自建 | 简单；subscribe 自动 cleanup |
| miss 策略 | 同 HTTP 版（5 帧未命中 abort） | 一致行为 |
| move 注入 | 沿用 `move_fn` 模式 | HTTP 版兼容 |
| 回退路径 | HTTP find_target 保留 | WS 不可用时降级 |

## 6. 底层 fix（runtime）

### 改动：`runtime/api/routes.py:1422`

```python
# 改前
task_push_hz = 10.0  # task_feed 默认刷新频率

# 改后
task_push_hz = 30.0  # 匹配 task_feed 默认 30Hz（my_car.py:1461）
```

**理由**：
- `task_feed` 默认 30Hz（`my_car.py:1461 MyCar.start_task_feed(self, hz: float = 30.0)`）
- WS 推送限速 10Hz → 服务端 3 倍数据被吞
- 修这个一行就够，零行为风险

### 不动：

- `lane_push_hz = 50.0` ✅ 已匹配 `lane_feed` 50Hz
- `arm_push_hz = 20.0` ✅ 已匹配 `arm_feed` 20Hz
- `ir_push_hz = 50.0` ✅ 已匹配 `ir_feed` 50Hz
- `odom_push_hz = 50.0` ✅ 已匹配 `odom_feed` 50Hz

## 7. 实测预期

| 路径 | 单帧延迟 | 闭环 Hz |
|---|---|---|
| HTTP find_target（保留） | ~30ms | ~30Hz |
| WS find_target_realtime（旧 10Hz 推送） | ~100ms | ~10Hz |
| WS find_target_realtime（新 30Hz 推送） | ~25-60ms | ~15-20Hz |

**30Hz 推送 + arm_queue 单 worker** 物理上限 ~20Hz（受 action queue + 电机 PID 闭环限制）。

## 8. 文件改动

| 文件 | 状态 | 行数 |
|---|---|---|
| `main/arm/vision.py` | 加 `find_target_realtime` | +80 |
| `main/arm/loops/runner.py` | 加 2 个高层方法 | +50 |
| `main/arm/tests/test_vision_realtime.py` | 新增单测 | +90 |
| `main/arm/ARM_API.md` | 文档 | +40 |
| **runtime/api/routes.py** | **底层 fix: 10 → 30** | **+1** |
| **总计** | | **~260 行** |

## 9. 风险与缓解

| 风险 | 缓解 |
|---|---|
| WS 断线 | subscribe_task_detection 自带心跳（client 端 ws_client 实现） |
| arm_queue 单 worker 阻塞 | move timeout=5s + caller 限速 |
| 服务端 task_push_hz 改了但 runtime 没 reload | pm2 restart rak-car-api（CLAUDE.md 日常流程） |
| 多客户端都订阅导致服务端繁忙 | task_feed 已经在跑，多个订阅只增加 WS 推送数（廉价） |

## 10. 实施阶段

1. **底层 fix**：runtime/api/routes.py:1422 task_push_hz 10→30
2. **Layer 2.5**：`find_target_realtime` + 单测
3. **Layer 4**：`move_to_vision_target_realtime` / `pick_by_vision_realtime` + 单测
4. **真机 smoke**：
   - TP1: WS 订阅成功（验证 push 频率确实提升）
   - TP2: 实时伺服（跟 HTTP 版对比延迟）
5. **文档 + commit**

## 11. 自审

- ✅ 不破坏向后兼容（HTTP 路径保留）
- ✅ 底层 fix 一行改动 + 零行为风险
- ✅ 单测覆盖（callback + 收敛 + miss + timeout）
- ✅ move_fn 注入模式一致（与 HTTP 版对齐）
- ✅ 失败语义一致（5 帧未命中 raise）
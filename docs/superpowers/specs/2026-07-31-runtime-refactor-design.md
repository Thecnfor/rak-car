# Runtime Refactor Design (2026-07-31)

> Status: **approved (待 commit + 下轮开干)**
> Author: brainstorming session with user
> Target: `rak-car/runtime/` — `api/` 和 `services/` 子目录

## 1. 背景

`runtime/` 经过几次快速叠加（最近 20 次提交里有 11 次触及 runtime/），已经从最初几百行的 FastAPI 入口长成 9430 行的"屎山"，单文件峰值 3059 行（`services/my_car.py`）。这是 race-day 飞控底盘，**任何重构都要保证线上不停摆**。

## 2. 用户决策（已锁定）

| # | 决策点 | 选择 |
|---|--------|------|
| 1 | 项目硬约束冲突：原 "runtime/ 冻结" | **本次特批，重构整个 runtime/** |
| 2 | feeds 守护线程（lane/arm/task/ir/odom）拆法 | **FeedsMixin 继承** |
| 3 | runtime_service.py 拆法 | **按职责拆为 4-5 个模块** |
| 4 | API 兼容性 | **内部清理 + 公开 API 100% 兼容** |

约束保留：
- `hardware/` 完全不动（MC602 controller-download-stuck 议题 OPEN）
- `core/` 不动
- `services/camera_stream_service.py` 和 `services/inference_service.py` 不在本轮范围

## 3. 不变量（必须保留）

| 不变量 | 说明 |
|--------|------|
| 公开 endpoint 100% 兼容 | URL 路径、HTTP 方法、参数名、JSON 响应**字节级相同** |
| MyCar 类签名 | `class MyCar(MecanumDriver)` 外部可见，**继承链 `isinstance` 仍工作** |
| 三层锁语义 | `_ref_lock` / `_realtime_gate` / `car_lock` 的获取顺序与超时不变 |
| ZMQ 端口 + 环境变量 | `infer_cfg` 端口、`RAK_CAR_*` 环境变量行为不变 |
| WebSocket op 协议 | WS 客户端 (`main/ws_client.py`) 调用契约不变 |
| jobs 协议 | `/v1/jobs/{id}` 状态机与 cancel 语义不变 |

## 4. 目标架构

```
runtime/
├── api/                              ← 拆分后
│   ├── app.py                        (60, 不变)
│   ├── router_registry.py            (新, ~30)  # 顶层 include_router 注册
│   └── routers/
│       ├── _helpers.py               (~280)  # WS op + execute payload + 格式化
│       ├── stream.py                 (~130)  # /stream/* /video_feed/* /captures/*
│       ├── vision.py                 (~300)  # /v1/vision/*
│       ├── realtime.py               (~210)  # /v1/realtime/*
│       ├── jobs.py                   (~280)  # /v1/jobs* /execute /init /stop_mode /close /estop /reset_stop /emergency_stop
│       ├── system.py                 (~100)  # /v1/health /runtime /actions /config /infer_*
│       ├── keypress.py               (~15)
│       └── ws.py                     (~620)  # /v1/ws 单独留——ws push loop 多
├── core/                             ← 不动
├── hardware/                         ← 不动（OPEN 议题）
└── services/                         ← 拆分后
    ├── camera_stream_service.py      (1099, 不动)
    ├── inference_service.py          (315, 不动)
    ├── car_runtime_service.py        (~280, Façade)  # 替换原 runtime_service.py
    ├── looper.py                     (~520)  # 4 个后台循环统一接口
    ├── controller_watcher.py         (~210)  # 控制器健康
    ├── queue.py                      (~260)  # jobs 队列 + payload helper
    ├── lifecycle.py                  (~360)  # _create_car_locked / ensure_initialized / close / shutdown
    ├── feed_degrade.py               (~220)  # 降级逻辑
    └── my_car/                       (3059 → mixin 拆分)
        ├── __init__.py               (~40)   # class MyCar(MecanumDriver, *Mixins)
        ├── state_mixin.py            (~280)  # 紧急停止 + beep + IR + arm_state 查询
        ├── sensors_mixin.py          (~400)  # sensor_init + storage + pwm + light + show_text + bt_pad + battery
        ├── motion_mixin.py           (~700)  # move_base + move_time + move_distance + lane_base + lane_det_location
        ├── detection_mixin.py        (~450)  # 5 个 *analysis + paddle_infer_init + ernie_bot_init
        ├── hardware_io_mixin.py      (~280)  # 12 个 realtime 接口
        ├── pid.py                    (~200)  # 4 个 PidCal 类 + util
        └── feeds.py                  (~650)  # FeedsMixin + 5 个 feed 循环主体
```

总行数 9430 → 9430，**最大单文件 700 行（motion_mixin），最大降幅 70%**。

## 5. 拆分阶段（每阶段独立可回滚）

### Phase 1: `api/routers/` 子包
- 拆 `routes.py` (1735) → 8 个 router 文件 + `_helpers.py` + `router_registry.py`
- 验证：所有现有 endpoint 路径、参数、响应字节级一致
- 风险：WS op 注册表 dict 化时易漏 case → 用 `grep` 比对原有 op 列表

### Phase 2: `services/my_car/` mixin 拆分
- 拆 `my_car.py` (3059) → 6 个 mixin + pid.py + feeds.py
- `MyCar` 改为多继承：`MyCar(MecanumDriver, StateMixin, SensorsMixin, MotionMixin, DetectionMixin, HardwareIOMixin, FeedsMixin)`
- MRO 顺序敏感 → 必须保留 super().__init__() 调用链
- 验证：`isinstance(my_car, MyCar)` 仍工作，所有公开方法签名不变

### Phase 3: `services/` 按职责拆分
- 拆 `runtime_service.py` (1633) → `car_runtime_service.py` (Façade) + 5 个模块
- 锁实现迁到 `car_runtime_service.py`，后台循环迁到 `looper.py`
- 验证：三层锁语义不变；后台循环重启/降级行为不变

### Phase 4: 内部清理
- 死代码（重复 if-else、未使用 import、未引用 helper）
- 拼写错误（`_lane_feed_inner_loop` 拼写、`sellect_program` → `select_program`、`filter_chinese_letter` 等）
- 抽出纯函数到独立模块（util / 计算器 / formatter）
- WS op 注册表 dict 化

每个 phase 单独 commit + 验证。

## 6. 风险与回退

| 风险 | 缓解 |
|------|------|
| MRO 顺序错误导致 super() 链坏 | 每个 mixin 单独写测试 + 启动时打印 `MyCar.__mro__` |
| API 兼容性回归（路径/响应） | 启动后 `curl` 全 endpoint 跑一遍；维护 `compat_endpoint_list.md` |
| WS op 协议破坏 | `main/ws_client.py` 加回归测试（独立 ping/pong/ops） |
| 后台循环时序变化 | `_ref_lock`/`_realtime_gate` 单元保留（用 stopwatch 测延迟微秒级） |
| 拆分时漏 import | 每个 phase 跑 `python -c "import runtime.server"` 做 import smoke test |

回退策略：每个 phase 是独立 commit；任意 phase 失败 → `git revert <phase-commit>` 不影响其他 phase。

## 7. 验收标准

- [ ] `python -m runtime.server` 启动成功，所有 endpoint 可达
- [ ] `isinstance(my_car, MyCar)` 工作
- [ ] `main/quick_start.py` 全连通
- [ ] WS 客户端（`main/ws_client.py`）全部 op 通过
- [ ] 飞控三件套（lane_feed / arm_feed / ir_feed）启动/停止/restart 行为不变
- [ ] `services/my_car/` 任一单文件 ≤ 750 行；`runtime_service.py` ≤ 350 行；`routes.py` 不存在（已拆）

## 8. 不在本轮范围（明确）

- `runtime/hardware/*` 任何修改（OPEN 议题）
- `core/*` 任何修改
- `services/camera_stream_service.py` (1099) 拆分——单独议题
- 新功能（任何 endpoint 增删改）
- 性能优化（仅清理，不主动优化）
- 单元测试基础设施（本轮结束后才考虑）

## 9. 下一步

→ 调用 `writing-plans` skill，把每个 phase 拆成可执行步骤（每步 ≤ 30 分钟，单一意图）。
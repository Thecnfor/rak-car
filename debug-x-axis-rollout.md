# debug-x-axis-rollout.md (v2)

> **目的**：压缩上下文用。下次会话只需读这一份即可恢复完整进度。
> **覆盖 session**：2026-07-17 上午（merge origin/main / origin.py 修复 / 方向映射） + 2026-07-17 下午（x_get_position vs realtime 真相 / safety watchdog / 同步带打滑诊断）。
> **当前日期**：2026-07-17
> **当前分支**：`am`（head `2a0c7d9`，merge origin/main 后 + 多组 WIP 改动**未提交**）
> **运行时**：`http://192.168.6.231:5050`（Jetson Nano + MC602，**已确认在线**）

---

## §1 主要调用的文档 / 文件

### 1.1 项目级文档（CLAUDE.md 指路，先读这一份）
- `CLAUDE.md` — 项目总览（分支说明、三入口、配置 surface、debug 约定、runtime 并发模型）。
- `main/arm/README.md` / `main/arm/ARM_API.md` / `main/arm/QUICKSTART.md` — 机械臂业务层文档。**ARM_API.md §0/§7/§9 严重过时**（merge 后没跟），详见 §2.3 / §4.4。
- `main/API_REFERENCE.md` / `main/API.md` / `main/CAPABILITY_LIST.md` / `main/BUSINESS_API_GUIDE.md` — 业务 API 参考。
- `runtime/README.md` — runtime 服务架构、并发任务模型、双 worker 队列、/v1/execute 异步语义。
- `runtime/VISION_API.md` / `runtime/STREAM_API.md` — 视觉 + 流。

### 1.2 本会话反复读的源码（按重要性）
| 文件 | 关键内容 | 本会话改过？ |
|---|---|---|
| `main/arm/origin.py` | OriginCalibrator 调 `reset_position` 触发 y 触底定原点 | ✅ 改了 2 次（见 §3.1） |
| `main/arm/api.py` | ArmClient 业务层（move_xy / move_x / move_y / reset_x / reset_all / set_storage / set_arm_angle / set_hand_angle / grasp / 状态读） | ✅ 加 safety watchdog（见 §3.3） |
| `main/arm/state.py` | ArmOrigin / ArmState dataclass | ❌ |
| `main/arm/__init__.py` | 导出 | ✅ 加 `ArmSafetyError` |
| `main/arm/test/aaa_origin.py` | **新建**：全物理复位 + 写 origin.yaml | ✅ |
| `main/arm/test/test_x_speed_safety.py` | 之前某次 session 写过的文件，**现在不在 working tree**（被覆盖/丢失） | ❌ |
| `main/arm/test/x_simple.py` | **新建**：x 轴开环冒烟（事件驱动 + safety watchdog） | ✅ 重写多次 |
| `smartcar/whalesbot/vehicle/arm/arm_base.py` | 车端 ArmController SDK（**底层，业务层不能改**） | ❌ 读了多次 |
| `smartcar/whalesbot/vehicle/arm/arm_cfg.yaml` | x/y/hand/pose 配置（**底层**） | ❌ |
| `runtime/core/actions.py` | arm/car action 注册表 | ❌ |
| `main/api_client.py` | RuntimeApiClient，execute_*_action 默认 `sync=False` | ❌ |

### 1.3 跑过的命令模板（curl 直调 runtime）
```bash
# 启速度
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"x_speed","kwargs":{"velocity":0.03},"sync":true}'

# 停速度
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"x_speed","kwargs":{"velocity":0},"sync":true}'

# PID 闭环定位
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"move_x_position","kwargs":{"target":-0.2,"out_time":8.0},"sync":true}'

# reset_x 撞墙（api.py wrapper 不透传 probe_time；这些参数只能直打 runtime）
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"reset_x","kwargs":{"direction":"left","reset_velocity":0.05,"probe_time":0,"min_pre_trigger_disp_m":0.04},"sync":true}'

# 读 x/y 原始坐标（**别用，下面 §2.4 解释**）
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"x_get_position","sync":true}'

# ★ 真值：从 20Hz arm_feed 守护线程读
curl http://192.168.6.231:5050/v1/realtime/arm/state
# → {"ok":true,"arm_state":{"x_mm":-447.62,"y_mm":-78.50,"ref_encoder":0.0787, ...}}

# 健康检查
curl http://192.168.6.231:5050/v1/health
```

---

## §2 目前的情况

### 2.1 同步状态
- `am` 分支 head `2a0c7d9`（merge origin/main，4 个冲突文件全 `checkout --theirs` 取 origin/main）
- merge 前的本地存档 commit `8ecf18d`
- **未提交的 WIP**（working tree 改）：
  - `main/arm/origin.py`（import 块 + sync=True + None 守卫）
  - `main/arm/api.py`（safety watchdog：+threading / +ArmSafetyError / +x_speed_with_safety / +stop_x_speed_safety / +is_x_safety_active / +_read_x_mm_realtime / +_cancel_x_watchdog；emergency_stop 钩子）
  - `main/arm/__init__.py`（暴露 ArmSafetyError）
  - `main/arm/test/aaa_origin.py`（新建）
  - `main/arm/test/x_simple.py`（新建 + 重写多次）
- 还删了 `main/arm/test/test_y_negative.py`（D 状态）
- `test_x_speed_safety.py` 早些时候写过，**现在不在 working tree**（被覆盖或丢失）

### 2.2 真机方向映射（2026-07-17 实测，本机专属）
| 物理位置 | 实时 x_mm | 软件 velocity | 用户原话叫法 |
|---|---|---|---|
| **近端**（图里靠近摄影师那端，M3-M6 板 + 相机） | **低值/负**（墙在 ≈ -447） | `-`（负方向）| "右" / "最右" / "物理最左" |
| **远端**（x 电机那头，rail 远端） | **高值**（待测） | `+`（正方向）| "左" / "另一个方向" / "最左" |

- **`direction="left"`** = 负方向 = 跑向近端墙（实测稳定在 -447.62mm）
- **`direction="right"`** = 正方向 = 跑向远端墙（远端墙位置未知待测）

### 2.3 origin/main arm 模型（2026-07-16 后）
- `reset_position` 只做 y 触底定原点，x 不再 calibrate
- x 轴软限位已取消（`_check_safe` 只校验 y）
- 大臂硬限 [-150°, 0°]；手爪硬限 [-90°, 0°]
- y 保护区 [0, -30]mm；set_storage 要求 y < -100mm
- 存储仓仅 LEFT(-42°)/RIGHT(165°)
- sync 语义：`/v1/execute` 默认 async，execute_arm_action 默认 sync=False；**业务层 `_call_arm` 默认 sync=True**
- `reset_x` / `reset_all` wrapper 还在 api.py（语义已废弃，仅 escape hatch）

### 2.4 **最关键发现：x_get_position 是坏的，realtime 才是真值**
| 读数方式 | 走哪条路 | 状态 |
|---|---|---|
| `x_get_position`（`/v1/execute`）| 车端 `motor_x.get_dis()` → 走 calibrate 框架 | **❌ 坏掉**：calibrate 后 `x_pose_start` 没正确更新，读数飘（实测 1.6mm、24mm 这种小数） |
| `/v1/realtime/arm/state` | 20Hz `arm_feed` 守护线程，直接读 motor 编码器 | **✅ 真值**：实测稳定 -447.62mm，3 次连读抖动 ±0.1mm |

**业务影响**：
- `api.py` 的 `get_state()` 里的 x_mm/y_mm 走 `x_get_position`/`y_get_position` 路径 → **不可信**
- realtime endpoint 不在 `_call_arm`/`_call_car` 路径，绕开 car_lock → 任何时候都能读
- safety watchdog / x_simple.py 都改用 realtime 读数
- 业务代码未来读 x/y 位置**统一改走 realtime**

### 2.5 同步带打滑（最关键硬件问题）🔴 **2026-07-17 下午确诊**
**症状**：
- 滑车开 x_speed 命令后能走 24-46mm，然后**卡住**
- 编码器照报数（报 200mm/s，比命令 30mm/s 快 6x）
- watchdog 2s 后兜底自动停
- 起点不同时打滑点 Δx 几乎一样 → 跟绝对位置无关，**是带传动的"包络极限"**
- 滑车**没撞物理墙**，是带子弧度变了之后摩擦力矩不够

**🔴 关键确认（2026-07-17 x_shuju 5 次跑）**：
| Run | target | 起点 | Δx | active | 类别 |
|---|---|---|---|---|---|
| R1 | -∞ | +46.9 | -24.1 | 0.30s | belt |
| R2 | -∞ | +22.8 | -25.8 | 0.41s | belt |
| R3 | -200 | +1.9 | -43.4 | 0.40s | belt |
| R4 | -200 | +0.6 | -46.4 | 0.47s | belt |
| R5 | -1 | +2.5 | -44.5 | 0.40s | belt |
| R6 | -100 | -173.4 | -46.6 | 3.87s | belt |

**Δx 稳定在 47mm 左右，**与 target / velocity / 起点位置 / 命令速度无关**。
这是 belt slip 的铁证——业务层/脚本/参数 全部无关。

**用户决定（2026-07-17 17:xx）**：belt slip 治根必须**现场查带子**，代码层面不再修。

**根因（待现场确认）**：
- 同步带涨紧度不够（最可能）
- 电机轴小皮带轮紧固螺钉松（编码器报转但轮不转）
- 皮带齿磨损
- motor_280 扭矩不够（带子长 + 滑车+相机+舵机总重量）

### 2.6 其他真机 bug（仍待修）
| Bug | 位置 | 现象 |
|---|---|---|
| 🔴 **aaa_origin.py reset_x 时好时坏** | `main/arm/api.py:598` reset_all wrapper | wrapper **不透传 `probe_time`**，导致走底层默认 `probe_time=0.3`；x 复位是否成功取决于"反向探针 0.3s 内能否 ≥1mm" |
| reset_x 撞墙未归零 | `arm_base.py:619` 附近 calibrate 路径 | 撞墙成功 result=true 但 x_get_position 不归零 |
| reset_x 假撞墙 | `arm_base.py:500` 附近 probe 逻辑 | probe_time=0.3 + 20mm/s 太保守，1mm/0.05s dwell 太短被反向探针吃掉 |
| x_get_position 读数飘 | `arm_base.py` calibrate 框架 | x_pose_start 没正确更新（已确认） |
| ARM_API.md §0/§7/§9 过时 | `main/arm/ARM_API.md` | merge origin/main 后没跟，写的是旧模型 |

**🔴 关键诊断（2026-07-17）**：
- `reset_all` wrapper 默认 `reset_x_velocity_mms=20`，但 `aaa_origin.py` 传 30mm/s
- **wrapper 完全没透传 `probe_time`** —— 走底层默认 `probe_time=0.3` 反向探针
- 反向探针成功条件：**0.3s 内电机 ≥1mm**
- **belt slip 状态下电机确实走 1mm 但**实时位置可能被 belt slip 抵消**——所以探针有时通有时不通**
- 探针通过 → 后续撞墙放宽 5cm gate → 通常能撞墙成功
- 探针失败（stall）→ 进 hard-stop → reset_all 返回 failed

**时好时坏的真因**：belt slip 让"反向探针是否能在 0.3s 内 ≥1mm"变成概率事件 —— 带子瞬时咬合就能通过；带子打滑就失败。

**业务层 workaround**：让 aaa_origin.py 调底层 reset_x（绕过 reset_all），透传 `probe_time=0` 关闭探针，依赖老路径走 5cm gate（实测 0.03-0.05 m/s 能撞出）。

---

## §3 我的改动（按文件，全部未提交）

### 3.1 `main/arm/origin.py`（M）
- import 块加 try/except fallback，让 `python main/arm/origin.py [left|right]` 直跑
- 两处 `execute_arm_action` 加 `sync=True`（修 `float(None)` TypeError）
- `y_get_position` result 加 None 守卫
- 加 `__main__` 入口

### 3.2 `main/arm/api.py`（M，**最复杂**）
- `+ import threading`
- `+ class ArmSafetyError(RuntimeError)` 新异常
- `+ __init__` 初始化 `_x_watchdog_thread` / `_x_watchdog_stop` / `_x_watchdog_active`
- `+ x_speed_with_safety(velocity, max_stale_s=2.0, position_tol_mm=0.5, poll_interval_s=0.2, timeout=10.0, on_stale=None)`：开环 x_speed + 后台 watchdog
- `+ stop_x_speed_safety(velocity=0.0, timeout=5.0)`：正常停 + 取消 watchdog
- `+ is_x_safety_active()`：轮询 watchdog 状态
- `+ _cancel_x_watchdog()`：内部取消 helper
- `+ _read_x_mm_realtime()`：从 `/v1/realtime/arm/state` 读，绕开 x_get_position
- 修改 `emergency_stop`：先 `_cancel_x_watchdog()` 再发停命令

### 3.3 `main/arm/__init__.py`（M）
- 暴露 `ArmSafetyError` 到 `from .api import ...` 和 `__all__`

### 3.4 `main/arm/test/aaa_origin.py`（**新建**）
- 全物理复位脚本：arm.reset_all + 读 y 写 arm_origin.yaml
- CLI 参数化：`--x-direction` / `--x-velocity`（默认 30mm/s=0.03m/s）/ `--arm-angle` / `--hand-angle` / `--timeout` / `--skip-yaml` / `--dry-run`
- 走 realtime 读数（绕开 x_get_position）

### 3.5 `main/arm/test/x_simple.py`（**新建 + 重写**）
- v1：sleep + x_speed + stop，ratio 受 HTTP 延迟污染（实测 0.37）
- v2：事件驱动 + safety watchdog，等撞墙自动停，**永远撞墙**
- v3（当前）：加 `--max-displacement`（默认 200mm）和默认反向 `-0.03`，watchdog 兜底
- 输出用 realtime 读数
- debug docstring 反向黑体警告带打滑点

### 3.6 没有改底层
- `smartcar/whalesbot/**` — 0 改动（按用户硬约束）
- `runtime/**` — 0 改动
- `car_wrap_2026.py` / `car_start_2026.py` / `car_task_function.py` — 0 改动
- `config_car.yml` / `ecosystem.config.js` — 0 改动

---

## §4 面临的问题

### 4.1 [硬件 · 治根] 同步带打滑
**软件层面完全无法根治**。必须现场查带子涨紧 / 轮紧固 / 齿磨损。
不治这个，所有 x 轴业务层代码都建在沙子上。

### 4.2 [底层 · 治本但当前不能动] x_get_position 校准 bug
- `arm_base.py` calibrate 路径 `x_pose_start = dis; x_pose_now = 0` 后没正确传递
- 业务层 workaround：所有读 x 的地方改走 realtime endpoint
- 已经做了 watchdog / x_simple 改用 realtime
- **遗留污染**：`api.py` 的 `get_state().x_mm` 仍走 x_get_position，业务用 state 仍会被坑

### 4.3 [底层 · 当前不能动] reset_x 假撞墙
- probe_time=0.3 + reset_velocity=0.02 默认参数太保守
- 关探针（probe_time=0）+ 0.05m/s 实际能用
- 业务层：reset_all 仍依赖 reset_x，所以 reset_all 也不稳

### 4.4 [文档] ARM_API.md 严重过时
- §0 还写"reset_x 撞墙定原点"、"x 软限位"
- §7 还写 x 软限位
- §9 还写 reset_x 撞右墙
- merge origin/main 后没跟

### 4.5 [测试] 三个 test 文件状态
- `test_x_speed_safety.py` — 写过，**现在不在**（被覆盖/丢失）。要不要重写？
- `test_x_simple.py` — 显示已删除（git status `D`），新的 `x_simple.py`（无 test_ 前缀）做轻量冒烟
- 老 `test_x_simple.py`（综合 5 case）还在 working tree 标记 D，要不要 restore

### 4.6 [架构 · 待定] ArmClient.reset_x / reset_all wrapper
- origin/main 已删 reset_x 撞墙定原点，但 wrapper 还在
- 业务层目前没真用，但留着是"半个 zombie"
- 选项 A：删（强一致 origin/main）
- 选项 B：保留 + deprecation warning
- 取决于 §4.1 带子修了之后是否还需要 reset_x

---

## §5 准备在解决（按优先级）

### 5.1 [立刻 · 用户能马上做] 现场查同步带
优先级最高。其他都是围绕带子能转的前提。

### 5.2 [业务层可做 · 5 分钟] 修 aaa_origin.py 默认 max-displacement
现在 200mm 远大于实际有效行程 25mm，watchdog 永远先到。改成 25mm 让测试自然停：
```python
DEFAULT_MAX_DISPLACEMENT_MM = 25.0  # 实测软打滑点
```

### 5.5 [业务层可做 · 30 分钟] 改 `api.py` get_state() 走 realtime
`x_mm` / `y_mm` 字段从 x_get_position/y_get_position 改为读 realtime endpoint。需要：
- 加 fallback（realtime 读不到时再退回 x_get_position）
- 加缓存（避免每帧都打 HTTP）
- 单元测试覆盖

### 5.4 [业务层可做 · 30 分钟] ARM_API.md 重写或加 deprecation 横幅
最小改：顶部加 "⚠️ 本文档 §0/§7/§9 已过时，merge origin/main 后没跟；真机模型见 debug-x-axis-rollout.md"
最完整：按 origin/main 新模型重写这 3 节。

### 5.5 [业务层可做 · 1 小时] `x_simple.py` 加 belt slip 检测
用 polling 监测：如果首步 Δx 远大于理论值（200mm/s vs 30mm/s），就是打滑信号。提前报警：
```python
if abs(dx_now - last_x_seen) > VELOCITY_UPPER_BOUND * poll_interval:
    print("[BELT SLIP DETECTED]")
```

### 5.6 [业务层可做 · 1 小时] 重写 test_x_speed_safety.py
之前写过但丢失。要重写覆盖 4 个 case：启 / 停 / latest wins / emergency_stop 取消。

### 5.7 [业务层可做 · 待用户授权] 给 reset_x wrapper 加 deprecation
按用户决定删 or 留。

### 5.8 [需底层 · 等用户授权] 修 arm_base.py calibrate bug
- reset_x 撞墙未归零
- x_get_position 校准
- 修完后 `get_state().x_mm` 才可信，业务层 workaround 可以撤

### 5.9 [需底层 · 等授权] 修 reset_x 假撞墙
- 默认参数改合理（probe_time=0.1, reset_velocity=0.04）
- 加 wall-confirmed 信号

### 5.10 [业务层可做 · 2 小时] "kick" 模式 if 带子治不好
带打滑后短停 100-200ms 让带子重新咬合。伪代码：
```python
while remaining > 0:
    arm.x_speed_with_safety(v=±0.05)
    if abs(dx_now) - last_dx > 18:  # 一段 18mm 后主动停
        arm.stop_x_speed_safety()
        time.sleep(0.15)  # 让带重咬合
        last_dx = dx_now
        remaining -= 20
```

---

## §6 注意事项 / 约束

### 6.1 用户硬约束
**当前只能改 `main/**` 业务层**。`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py`、`config_car.yml`、`ecosystem.config.js` 都不动。`incoming/submission/` gitignore 可改。

### 6.2 读 x/y 坐标：永远走 realtime，**别用 x_get_position**
```python
import requests
r = requests.get(f"{http.api_base}/v1/realtime/arm/state", timeout=2.0)
arm = r.json()["arm_state"]
x_mm = arm["x_mm"]
y_mm = arm["y_mm"]
```
`api.py` 的 `_read_x_mm_realtime()` 已封装。

### 6.3 sync=True 是关键
- `/v1/execute` 默认异步 → `status=queued, result=None`
- 业务层 `_call_arm` 默认 `sync=True`（长动作要等）
- 出现 `float() argument must be...not 'NoneType'` 99% 是这个

### 6.4 坐标系
- 业务层单位 mm；车端 m；转换在 `_mm_to_m` / `_m_to_mm`
- y 触底=0，向下取正、向上取负
- 业务 x 软限位已取消（仅 y 软限）
- 大臂硬限 [-150°, 0°]；手爪硬限 [-90°, 0°]

### 6.5 当前分支 `am` 还没 push
- `git push -u origin am` 前不要 `git push`
- commit 前 `git status --short && git log --oneline -5` 确认

### 6.6 safety watchdog 用法
```python
arm = ArmClient.connect()
arm.x_speed_with_safety(velocity=0.05)  # 后台 watchdog
# ... 业务 ...
arm.stop_x_speed_safety()  # 正常停
# 或 arm.emergency_stop() 也会取消 watchdog
```

### 6.7 aaa_origin.py 速度
**当前默认 30 mm/s（0.03 m/s）**——之前改过 50 → 40 → 30，是用户偏好。

### 6.8 x_simple.py 速度
**当前默认 -0.03 m/s（反向，跑近端）**——用户要求"试另外一个方向"。

---

## §7 快速恢复清单（下次会话第一件事）

1. 读这一份 → 读 `MEMORY.md`（auto-memory） → 读 `CLAUDE.md` §"Runtime concurrency model"
2. `git status --short && git log --oneline -5`：确认 `am` 分支、WIP 在
3. `curl http://192.168.6.231:5050/v1/health`：确认 runtime 在线
4. `curl http://192.168.6.231:5050/v1/realtime/arm/state`：看当前 x_mm / y_mm
5. 看 §5 待办决定是否动手

---

## §8 关联 artefact
- `.dbg/x-axis-rollout.env` — 本次会话 runtime URL / commit hash / 日期 / 关键参数
- `MEMORY.md` — 跨会话的简明指针（多条目）
- 关键 memory：
  - `x-axis-rollout-session.md` — 这次会话总览
  - `execute-sync-default.md` — sync=True 强约束
  - `arm-business-layer-only.md` — 业务层边界
  - `x-speed-safety-watchdog.md` — safety watchdog 用法
  - `arm-api-reference.md` — ARM_API.md 速查（已加过时警告）

# debug-target4-rebuild-2026-07-29.md **[REVERTED — 2026-07-29 同日回滚]**

**会话日期**：2026-07-29
**用户硬约束**：只能改 `main/**` 业务层；`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py` **暂不能动**
**分支**：`am` (HEAD `ac1413b`)
**主要工作**：target4.py 删除 + 重写 v8 (2D 扫描) + 改 constants.py 阈值为 [0.05, 0.50]
**当前状态** ⚠️ **REVERTED**:
- `target4.py` → 已恢复 `.target4_v7-final.bak` (v7 识别即停版, 715 行, 二段式状态机 SEARCH/found)
- `constants.py` → TARGET_AREA_MIN/MAX 已回滚到 [0.20, 0.30] (历史 2026-07-28 baseline)
- `constants.py` → BALL_VERIFIED_* 保留 UNION (本次会话成果, 见 [[ball-best-grasp-2026-07-29]])
- `api.py` → BALL_VERIFIED_* fallback 保留 UNION

---

## 1. 主要调用文档

### 文档 (本会话开始时确认)
- `CLAUDE.md` — 项目主入口文档
- `main/arm/README.md` + `main/arm/ARM_API.md` + `main/arm/QUICKSTART.md` — 机械臂业务层 API
- `runtime/README.md` — runtime 服务（并发任务模型）
- `runtime/VISION_API.md` — 视觉相关端点
- `debug-task4-rebuild-2026-07-28.md` — 2026-07-28 全部 task4 改动 + BALL_VERIFIED 校准 + 状态机 + 待办
- `main/arm/each_task/task4/target1.py` + `target2.py` + `target3.py` — 4 个任务脚本
- `main/arm/each_task/task4/constants.py` — 业务常量 (BALL_VERIFIED_* + TARGET_* + BIN_*)

### 源码 (按调用频率)
- `main/arm/api.py` — `ArmClient` (1144 行)
- `main/arm/each_task/task4/constants.py` — 业务常量
- `main/arm/each_task/task4/target1.py` — 摆位姿 (y=-133, arm=90°, hand=0°, x=-260)
- `main/arm/each_task/task4/target2.py` — 球检测 (score/area/aspect 过滤 + BALL_VERIFIED_* 验证)
- `main/arm/each_task/task4/target4.py` — **本会话重写**
- `runtime/services/runtime_service.py` — runtime + auto-init + job queue
- `main/arm/each_task/task4/__init__.py` — 不引用 target4 (重写时确认)

---

## 2. 本会话核心改动

### 2.1 删除旧 target4.py (用户原话 "全删了")

旧文件状态: v3 baseline (SEARCH/ADJUST/LOCKED 状态机, 929 行), 包含 11 个 ADJUST_* 常量
+ BALL_VERIFIED_* 验证 + 0.15s 重检测兜底。

**用户决定**: 完全删除,不保留。

**留底备份**:
- `main/arm/each_task/task4/.target4_v6.bak` — 51662 bytes (含 forward_rest 实验期代码)
- `main/arm/each_task/task4/.target4_v7-final.bak` — 36519 bytes (v7 识别即停, 删除前最终版本)

恢复命令: `cp main/arm/each_task/task4/.target4_v7-final.bak main/arm/each_task/task4/target4.py`

### 2.2 新重写 target4.py v8 (用户原话 "慢慢移动底盘 + x 从 -240 慢慢扫到 -280")

策略 (扁平循环, 无状态机):
```
while not timeout:
    [1] 慢慢动底盘 3cm 前进 (用户原话 "先慢慢移动底盘")
    [2] x sweep 从 -240 慢扫到 -280 (10mm 步, 实时检测)
    [3] 命中 → 立即 break
    [4] x 扫到 -280 没中 → 回到 [1]
```

**默认参数**:
| 项 | 值 | 备注 |
|---|---|---|
| `x_start_mm` | -240 | 对齐 prep_x |
| `x_end_mm` | -280 | 从 start 走到 end (x 更负) |
| `x_step_mm` | 10 | "慢慢"细步 |
| `chassis_step_m` | 0.03 (3cm) | "慢慢"小步 |
| `step_interval_s` | 0.3 | 实时检测短间隔 |
| `prep_y / x / arm / hand` | -133 / -240 / +90° / 0° | 跟 target1 一致 |
| `max_chassis_steps` | 30 | 防跑远 (60s × 0.3s × 3cm ≈ 90cm) |
| `max_duration_s` | 60 | 总超时 |

**关键设计**:
- 无状态机 (无 ADJUST / LOCKED / confirm_frames)
- 颜色匹配即 stop, 不做 BALL_VERIFIED_* 验证
- 无 PD 微调, 无方向符号
- 退出前必停底盘 (`try/finally _stop_chassis`)
- belt-slip 兜底: 每步 move_x 后读 realtime x, 偏差 >30mm 打 warning
- chassis_client=None 退化: 仅 x 扫描, 不动底盘

### 2.3 修复 state machine bug (用户报 "x 后续不是从 -240 到 -280")

**bug**: 外层 `if do_chassis and not dry_run and chassis_client is not None` 把"动底盘"和"状态机推进"绑一个分支。
- 后果 1: `--dry-run` 时 `n_chassis_steps` 永远 0, **死循环到 max_duration_s**
- 后果 2: `chassis_client=None` 时同 1

**修法**: 分开物理动作和状态推进:
```python
should_advance = do_chassis and n_chassis_steps < max_chassis_steps

# 物理底盘 (dry-run / 无 client 跳过)
if should_advance and not dry_run and chassis_client is not None:
    try: chassis_client.http.execute_car_action(...)
    except ...: print(...)

# 状态机永远推进 (max_chassis_steps 兜底)
if should_advance:
    n_chassis_steps += 1
    time.sleep(...)
elif n_chassis_steps >= max_chassis_steps:
    break
```

**dry-run 验证** (改后):
```
step #1/3 [dry-run] 跳过
r001-r005 x=-240..-280 (chassis_steps=1)
step #2/3 [dry-run] 跳过
r006-r010 x=-240..-280 (chassis_steps=2)
step #3/3 [dry-run] 跳过
r011-r015 x=-240..-280 (chassis_steps=3)
chassis 步数 3 达到上限 3, 退出
耗时 1.8s (旧 ~30s)
```

### 2.4 改 constants.py: TARGET_AREA_MIN/MAX 放宽

| 字段 | 旧 | 新 | 触发原因 |
|---|---|---|---|
| `TARGET_AREA_MIN` | 0.20 | **0.05** | 现场 target2 远距球 area=0.151 被 MIN 挡 |
| `TARGET_AREA_MAX` | 0.30 | **0.50** | 现场 target4 prep 近距球 area=0.306 被 MAX 挡 |

**现场实测数据**:
- 早先: 1 ball (黄), score=0.949, area=0.306 → 被 MAX 0.30 挡
- 改 MAX 后: 3 balls (1黄+2蓝), 黄球 area=0.151 → 被 MIN 0.20 挡
- 改 C 方案后 (MIN=0.05, MAX=0.50): 1 ball (黄), cx=0.192, cy=-0.748, area=0.262, score=0.928 ✅

**docstring 已更新** (constants.py:88-93), 标注 2026-07-29 现场校准来源 + 历史 baseline 0.246-0.265。

---

## 3. 当前状态

| 文件 | 状态 |
|---|---|
| `main/arm/each_task/task4/target4.py` | **新重写 v8** (扁平 2D 扫描, 444 行, 自包含) |
| `main/arm/each_task/task4/constants.py` | TARGET_AREA 放宽到 [0.05, 0.50] |
| `main/arm/each_task/task4/.target4_v6.bak` | 51662 bytes 备份 |
| `main/arm/each_task/task4/.target4_v7-final.bak` | 36519 bytes 备份 |
| `main/arm/each_task/task4/__init__.py` | 不引用 target4 (确认过) |
| 其他 task4 模块 (target1/target2/target3/grasp/pick_up_blue) | 未动 |

**现场跑通验证**:
- `target2.py --color yellow` → 1 ball (cx=0.192, cy=-0.748, area=0.262, score=0.928) ✅
- `target4.py --dry-run` → 3 chassis 步 + 15 x 步 + 因 max_chassis_steps 退出 ✅
- `target4.py` 真跑(EXECUTE) → 准备位姿 + 3 步 chassis + 12 步 x + 60s 超时退出 (没找到球)

---

## 4. 面临的问题

### 🔴 P0: target4 真跑 60s 超时仍找不到球

- 现场跑 `target4.py` (EXECUTE) 61.5s 才超时退出 (chassis_steps=3, x_steps=12)
- 上次 prep pose 时 target2 能看到 1 黄球 (area=0.262, cy=-0.748)
- **现在 target4 prep 完成后 / 跑到 x=-240 时, 可能球不在视野里**
- 原因待定位: 摄像头位置 / 球位置 / 臂位姿是否覆盖到球

### 🟡 P1: 现场 storage_open 失败 (1次性)

```
[FAIL] set_storage_angle(75°)  err=None
[after] ctrl_state=PROGRAM_READY  usb_present=True
```
- 失败信息 `None`, 看不出具体原因
- 后续 storage_close / get_blue 全部 OK,系统没死
- 待查 `api.py:720-729` (memory 里 P1 待办: 缺 `job =` 赋值, **但用户硬约束暂不能改 api.py**)

### 🟡 P2: target1 y 值冲突

- `constants.py / target1.py / target4.py` 当前 y=-133
- memory [[arm-target-y-position-trajectory]] 警告 "target1 改 -125 后必须重测 BALL_VERIFIED_*"
- memory 描述 target1 y=-125 "实测最稳", 但代码用 -133
- **两套数据时间戳不一致, 以当前代码为准**

### 🟢 P3: target4.py x 物理墙

- 默认 `--x-end -280` 低于物理墙 -119.5mm
- target1.py 撞墙 stall 兜底 (test_x_to_150.py 模式)
- 业务层接受这个行为, 只打 warning

---

## 5. 准备在解决 (下一步)

### 步骤 1: 现场验证 target4 真跑能否找到球

```bash
# 在 v8 改完后, 真跑一遍
python -m main.arm.each_task.task4.target4 --color yellow
```

**期望**:
- 如果球在 chassis 推进的前 9cm + x [-240, -280] 范围 → 命中 break
- 如果球不在 → 60s 超时 (上一轮就是这情况)

### 步骤 2: 如果 60s 超时, 定位球位置

```bash
# 跑 target2 看球在画面哪个位置
python -m main.arm.each_task.task4.target2 --color yellow --debug --show-raw --score-min 0
```

**根据 cx_norm 调整 x 范围**:
- cx 在 [0.0, 0.2] → 当前范围 [-240, -280] OK
- cx 在 [-0.3, -0.1] → 改 `--x-start -180 --x-end -220`
- cx 在 [0.3, 0.5] → 改 `--x-start -260 --x-end -300`

### 步骤 3: 现场 chassis 范围校准

- ball.cy = -0.748 (上次 raw data) → 球离相机比较远
- 如果 ball.cy 一直 < -0.5 表示球够远, 可以加大 chassis_step_m (0.05) 让步数更密
- 如果 ball.cy > -0.3 表示球近, 小步 0.02 让搜索更精细

### 步骤 4: 找球后接 target1 抓取

```bash
# target4 找到球 → 退出 (return ball dict)
# 下游调用 target1.move_to(x=ball.x, ...) → grasp
```

---

## 6. 注意事项 (下次会话必读)

### 用户硬约束
- ⚠️ **只能改 `main/**` 业务层**；`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py` 暂不能动
- 任何改 API / 业务逻辑都行；改硬件 SDK / runtime 服务 / 顶层入口都禁止

### 本会话已删/改关键信息
- **target4.py** 已从 v3 (SEARCH/ADJUST/LOCKED 状态机) 重写为 v8 (扁平 2D 扫描)
- **constants.py** TARGET_AREA_MIN/MAX 已从 [0.20, 0.30] 放宽到 [0.05, 0.50]
- 这两点不冲突, 但下次如果再诊断 "0 球", **先看 TARGET_AREA 当前值**

### 几个被踩过的坑 (避免重蹈)
1. **`target2.py` 默认 score/area/aspect 过滤** — 加 `--score-min 0 --show-raw --debug` 看 raw detection
2. **target4 dry-run 死循环** — 修过了: 状态机推进和物理动作分开
3. **x 物理墙 -119.5mm** — target1.py 撞墙 stall 兜底, 现场接受
4. **BALL_VERIFIED_* 是 target1 位姿下校准** — target4 不用这个 (只用 matched break)
5. **target4 默认每次 x sweep 都从 -240 重新开始** (用户原话 "再重复 x 轴")

### 重要调试入口
- 现场 trace: `pm2 logs rak-car-api --lines 200 | grep -iE "task_feed|cap_side|frame missing"`
- task_feed 状态: `curl http://10.253.70.20:5050/v1/realtime/vision/task`
- 整体健康: `curl http://10.253.70.20:5050/v1/health`
- 视觉预览: `http://10.253.70.20:5050/stream/`
- raw detection: `python -m main.arm.each_task.task4.target2 --debug --show-raw --score-min 0`

### 当前内存 (`.claude/projects/.../memory/MEMORY.md`) 含
- `arm-grasp-call-arm-base` — grasp 用 http.execute_arm_action 直调
- `ball-verified-2026-07-28-recalibration` — BALL_VERIFIED_* 校准 (target1 位姿)
- `target4-adjust-state-machine` — **过期**, 上一个会话的 SEARCH/ADJUST/LOCKED 已拆
- `arm-target-y-position-trajectory` — y 轨迹 (target1=-133 当前, memory 提 -125 待核)
- `jetson-current-ip` — Jetson IP = `10.253.70.20`

---

## 7. 备注 / 风险点

1. **TARGET_AREA 放宽副作用**: 远景噪点 / 光线 / 阴影会被识别成球。后续如果出现误检, 优先用 `--aspect-tol` / `--score-min` 临时收紧, 而非改 constants。
2. **target4 prep pose 跟 target1 一样**: y=-133, arm=90°, hand=0°, x=-240。如果现场看到 ball 但 cx/cy 偏离 BALL_VERIFIED_*, 调整臂位姿不是调 target4。
3. **chassis_step_m=0.03 用户可能嫌慢**: 60s 跑 3cm × 30 = 90cm, 比赛节奏允许。但 0.05 也能跑, 视现场判断。
4. **target4 prep 后 realtime x 不是精确 -240**: 现场跑出来 -239.85mm, 0.15mm 偏差是 belt-slip 引起的, 不影响后续 (都是相对偏移)。

---

## 8. 续接指南 (新会话第一件事)

1. 读本文件
2. 跑现场三步验证:
   ```bash
   # a. 球在画面里吗?
   python -m main.arm.each_task.task4.target2 --color yellow
   
   # b. target4 跑得通吗?
   python -m main.arm.each_task.task4.target4 --color yellow
   
   # c. 如果超时, 看 raw + 调 x 范围
   python -m main.arm.each_task.task4.target2 --debug --show-raw --score-min 0
   python -m main.arm.each_task.task4.target4 --color yellow --x-start -200 --x-end -260
   ```
3. 根据 (a) 调整 x 范围 / chassis_step_m
4. 找球后下一步: 写 target1 接 ball → target3 抓取

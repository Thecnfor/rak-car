# 2026-07-30 上下文压缩文档（task4 校准 + IP 切换 + target4 重写待续）

会话日期: 2026-07-30
分支: `am` (default branch: main)
硬件: Jetson 192.168.5.230 PC 192.168.5.231 (192.168.5.x 段)

本会话主线:
1. 改 Jetson IP 192.168.6.231 → 192.168.5.230 (三处同步)
2. PPT 任务4 规则确认 (Slide 9)
3. 球检测校准三次迭代 (BALL_VERIFIED_* + TARGET_AREA_*)
4. 设计 target4.py 三阶段重写方案 (未落盘)

**下次会话第一件事读这个文档**。

---

## 1. 调用的文档 / 文件

### 1.1 任务规范
- `C:\Users\29368\Desktop\智能车\百度智慧交通创意组21届-任务解析与技术流程(1).pptx` (任务4 在 Slide 9)
  - 2 色 4cm 球, 随机位置, 赛前公布
  - 完全脱离任务模型 + 不与场地接触 → 10分/球
  - 技术方案: 目标检测 + 精准定位 + 存储结构

### 1.2 仓库指南
- `C:\Users\29368\Desktop\智能车\rak-car\CLAUDE.md` (项目结构 + 入口)
- `C:\Users\29368\Desktop\智能车\rak-car\main\arm\ARM_API.md` (机械臂 API 速查)
- `C:\Users\29368\Desktop\智能车\rak-car\runtime\README.md` (runtime 服务)

### 1.3 业务层代码 (本次主要工作区)
- `main\arm\api.py` (ArmClient, 业务层包 snapshot)
- `main\arm\each_task\common.py` (belt-slip 安全 move_x + wall + overshoot)
- `main\arm\each_task\task4\__init__.py` (task4 包说明)
- `main\arm\each_task\task4\constants.py` (BALL_VERIFIED_* + TARGET_AREA_* 主源)
- `main\arm\each_task\task4\target1.py` (target1 位姿: y=-133, arm=+90°, hand=0°, x=-260)
- `main\arm\each_task\task4\target2.py` (side 球检测, fetch_balls 入口)
- `main\arm\each_task\task4\target3.py` (抓取: 吸气 → 下降 -58 → 抬回 -133 → 放气)
- `main\arm\each_task\task4\target4.py` (待重写, 旧版 v7 715 行 "识别到即停")
- `main\arm\each_task\task4\grasp.py` (真空泵冒烟)
- `main\arm\each_task\task4\pick_up_blue.py` (高层编排: target1 → target3 → test_blue)

### 1.4 Runtime 配置 (本次覆盖)
- `main\settings.py:9` (DEFAULT_SERVER_ORIGIN, 业务层连)
- `runtime\core\settings.py:12` (PUBLIC_HOST, runtime 对外广播)
- `ecosystem.config.js:19` (RAK_CAR_PUBLIC_HOST PM2 env)

### 1.5 内存 (memory) - 我创建的/更新的
- `memory\ball-best-grasp-2026-07-30.md` (新建, run 9 = cx=0.026/cy=-0.748/aspect=1.057)
- `memory\jetson-current-ip.md` (更新到 192.168.5.230, 三处同步)
- `memory\MEMORY.md` (加新 memory 索引 + 更新 IP 描述)

### 1.6 旧 memory (历史基线, 重要参考)
- `memory\ball-verified-2026-07-28-recalibration.md` (历史 baseline source)
- `memory\ball-best-grasp-2026-07-29.md` (run 8 = cx=0.120/cy=-0.719/aspect=0.923)
- `memory\x-axis-rollout-session.md` (x 轴基础)
- `memory\target4-adjust-state-machine.md` (旧版 target4 v2 SEARCH/ADJUST/LOCKED)
- `memory\arm-target-y-position-trajectory.md` (target1 y 轨迹)
- `memory\arm-business-layer-only.md` (业务层硬约束)

### 1.7 旧 debug 文档 (历史续接)
- `debug-task4-rebuild-2026-07-28.md` (task4 7-28 重建)
- `debug-task5-rebuild-2026-07-22.md` (task5 重建)
- `debug-x-axis-rollout-v3.md` (x 轴 v3)
- `debug-belt-slip-checklist.md` (x 带打滑)

---

## 2. 目前情况（会话末状态）

### 2.1 现场实际 (PC 端跑 target2.py)

- runtime 端 `task_feed` 检测 **间歇性 1↔2 球** (不是稳定 2 球)
- 球几何跟历史 baseline 偏差大:
  - 历史 (7-28 ~ 7-29): aspect 0.7~1.06, area 0.246~0.265
  - 现场 (7-30): **aspect 0.35~0.5** (球被压扁 1.5×), **area 0.158~0.346** (一半到 1.5×)
- 原因待查: 可能是底盘位置/相机角度变了 (跟 7-29 7-30 校准时位置不一样)

### 2.2 仓库修改 (am 分支 working tree)

- `M runtime\core\settings.py` (PUBLIC_HOST 192.168.6.231 → 192.168.5.230)
- `M ecosystem.config.js` (RAK_CAR_PUBLIC_HOST 同上)
- `M main\arm\api.py` (BALL_VERIFIED_H_MIN fallback 0.55 → 0.48, ASPECT_MAX 0.95 → 1.10)
- `M main\arm\each_task\task4\constants.py` (BALL_VERIFIED_* + TARGET_AREA_* 全部放宽)
- `M main\settings.py` (本次会话内已经是 192.168.5.230, 不是本次改)
- `M main\arm\each_task\task4\__init__.py` (本次会话内已有改动, 跟本会话无关)
- 9 个新 memory 文件 / 索引 (跟本会话相关: ball-best-grasp-2026-07-30, jetson-current-ip 更新)

### 2.3 target4.py 状态

- 旧版 (v7, 715 行) "识别到即停" 状态机
- 本会话设计了 v8 三阶段重写方案 (Phase A x 扫描 + Phase B 首抓 + Phase C 7 次底盘前移 + 抓取/跳过), **但没写文件**
- 用户上一轮指令被打断, 没确认是否继续

---

## 3. 本会话改动 (commit 序列)

### 3.1 IP 切换 (三处同步)

```python
# runtime\core\settings.py:12
PUBLIC_HOST = "192.168.5.230"  # 192.168.6.231 →

# ecosystem.config.js:19
RAK_CAR_PUBLIC_HOST: "192.168.5.230",  # 192.168.6.231 →

# main\settings.py:9 (会话前已是 192.168.5.230, 本会话未改)
DEFAULT_SERVER_ORIGIN = "http://192.168.5.230"
```

**背景**: PC 切到 192.168.5.231, Jetson 切到 192.168.5.230。README/CLAUDE.md/runtime/*.md 文档 example 仍写 192.168.6.231 旧地址, **文档未同步**。

### 3.2 BALL_VERIFIED_* 9 次校准 (target1 位姿下)

```python
# constants.py + api.py fallback 同步
BALL_VERIFIED_H_MIN: 0.55 → 0.48      # 球 h=0.505 (aspect 1.057) 撑爆下限
BALL_VERIFIED_ASPECT_MAX: 0.95 → 1.10 # 球 aspect=1.057 (横宽>纵高) 撑爆上限
```

**历史**: 7-28 run 7 (aspect 0.70 扁) → 7-29 run 8 (cx=0.120, aspect 0.923 纵高>横宽) → 7-30 run 9 (cx=0.026, aspect 1.057 横宽>纵高)。UNION + buffer 扩到 9 次全过。

**临界警告**:
- H_MIN=0.48 已接近噪声框边界 (h≤0.5 几何上不像球)
- ASPECT_MAX=1.10 跨 1.0 (几何上不太圆)
- 现场如有误检, 优先用 `--aspect-tol` / `--score-min` 临时收紧, 别动 constants

### 3.3 TARGET_AREA_* 两次校准 (side 球检测, target2.py 入口)

```python
# constants.py
TARGET_AREA_MAX: 0.30 → 0.50   # 现场右球 area=0.457 完全可见, 不放宽会被拒
TARGET_AREA_MIN: 0.20 → 0.15   # 现场 1 球间歇状态 area=0.158~0.183, 不放宽会被拒
```

**触发顺序**:
1. 用户跑 2 球截图 → 我估右边 area=0.457 → 改 MAX 0.30→0.50
2. 用户再跑还是 0 球 → 我直接 curl runtime 看到其实 1 球 area=0.183 → 改 MIN 0.20→0.15
3. 验证 fetch_balls 返 2 球 (blue area=0.346 + yellow area=0.192)

**等待用户确认**: 是否同步放宽 `BALL_VERIFIED_AREA_MIN_VERIFY: 0.20 → 0.15` (target1 位姿下 `--verify-target1-pose` 用, 现在不开, 但迟早要开)

### 3.4 Memory 文件

- 全新建 `memory\ball-best-grasp-2026-07-30.md` (run 9 详细对照表 + Why/How to apply)
- 覆盖 `memory\jetson-current-ip.md` (192.168.5.230 三处同步, 历史 IP 链)
- 更新 `memory\MEMORY.md` (加 ball-best-grasp-2026-07-30 入口, 更新 jetson IP 描述)

---

## 4. 面临的问题 (Open Problems)

### 4.1 球几何漂移

- 球 aspect 0.35~0.5 vs 历史 0.7~1.06 (球被压扁 1.5×)
- 球 area 0.158~0.346 vs 历史 0.246~0.265 (一半到 1.5×)
- 原因未查清: 可能是
  - 底盘/相机位置跟 7-30 校准时不一样
  - 摄像头角度变了
  - 球场上球位置变了
- 多次校准 constants 会引入误检风险 (已接近噪声框边界)

### 4.2 runtime 检测间歇性

- task_feed 1↔2 球间歇, 不是稳定 2 球
- 1 球状态时是小球 (area 0.183), 2 球状态时一球大一球小
- 跟过往 target1.py 校准预期不匹配 (target1 校准是稳定 1 球, cy ≈ -0.7)

### 4.3 target4.py 重写未落地

- 用户上一轮设计完 v8 三阶段方案 (Phase A x 扫描 + Phase B 首抓 + Phase C 7 次底盘前移 + 抓取/跳过)
- 详: 用户说 "先只移动x轴从-240到-280，找到最佳抓取位置，然后到达最佳抓取位置后，先抓取，x轴后续一直保持不动，底盘向前移动80mm，然后再识别有没有球，有的话就抓取，没有的话就不用，重复7个这样的底盘向前移动和识别球抓取的动作"
- 方案草稿在会话上下文里 (没写文件), 包括:
  - Phase A `_phase_a_scan` (x 扫描 [-240, -280] 找 BALL_VERIFIED_* 范围内最佳)
  - Phase B `_phase_b_first_grab` (复用 target3.step_target3)
  - Phase C `_phase_c_loop` (7 次底盘前移 80mm + 识别 + 抓取/跳过)
  - 用 `common.move_x_with_split` 替代旧 `_is_ball_like` 路径
- 用户中断后转去做 BALL_VERIFIED_* 校准, 没回来确认

### 4.4 task4 存储仓逻辑

- task4 业务层目前没有 "抓到球后存仓" 的逻辑
- task5 才做分拣入库 (task4 收球 → task5 入仓)
- 用户新方案 (target4) 是 "x 不动 + 底盘前移串行抓", 抓到球后球放哪? 球舱? 地面? 存储仓?
- 没有结论, **用户没明确**

### 4.5 文档 IP 未同步

- README.md, CLAUDE.md, runtime/README.md, runtime/STREAM_API.md, runtime/VISION_API.md, main/README.md, main/arm/ARM_API.md, main/CAPABILITY_LIST.md, main/BUSINESS_API_GUIDE.md, main/QUICKSTART.md
- 全部 example 还写 192.168.6.231 旧地址
- 用户今天没要求改, **下次再说**

### 4.6 target2.py 缺 CLI 临时覆盖

- 现在只有 `--score-min / --verify-target1-pose`, 没有 `--area-min / --area-max / --aspect-tol`
- fetch_balls 内部已经接这些 kwargs, 缺 CLI 透传
- 现场下次再遇到类似问题, 改 constants 太重

### 4.7 球检测 score 临界

- 现场 1 球间歇 score=0.883, 2 球一球 0.916 一球 0.903
- TARGET_SCORE_MIN=0.85, 0.883 擦线过
- 0.85 已接近噪声框边界, 下次校准可能也要放宽

---

## 5. 准备在解决 (Planned Next Steps)

### 5.1 优先

1. **target4.py v8 重写** (上一轮设计完, 用户中断)
   - Phase A x 扫描 [-240, -280] 找 BALL_VERIFIED_* 最佳位置
   - Phase B 首抓 (在最佳位置)
   - Phase C 7 次重复: 底盘前移 80mm + 识别 + 抓取/跳过 (x 全程不动)
   - 复用 `common.move_x_with_split` + `target2.fetch_balls` + `target3.step_target3`
   - 用 `chassis_client.http.execute_car_action("move_for", [0.080, 0, 0])` 走底盘
2. **同步放宽 BALL_VERIFIED_AREA_MIN_VERIFY: 0.20 → 0.15** (待用户确认)
   - 当前 0.183 球会被拒, 跟 TARGET_AREA_MIN 同步
   - 用户上一轮问 "要不要顺手也放宽", 没确认
3. **target2.py 加 `--area-min / --area-max / --aspect-tol` CLI 覆盖**
   - 现场下次类似问题不用每次改 constants

### 5.2 中期

4. 文档 IP 同步 (README/CLAUDE.md/runtime/*.md 等 example 改 192.168.5.230)
5. 球几何漂移根因调查 (探头 / 相机 / 相机角度 / 球位置)
6. task4 → task5 存储仓逻辑衔接 (边采边存 vs 一次采完再分)

### 5.3 长期

7. 重写后跑 target4.py 现场全流程 (Phase A → B → C 1 球到 8 球)
8. 比赛策略: 8 球 vs 实际场地球数 (PPT 说随机), 抓不全的兜底

---

## 6. 注意事项 (硬约束)

### 6.1 业务层限定 (最重要)

**只能改 `main/**` 业务层:**
- `main\arm\api.py` (业务层 ArmClient)
- `main\arm\each_task\**` (task4/target1-4 等)
- `main\settings.py` (client 默认入口)
- `main\chassis\**` (chassis 业务层)
- `main\test\**` (冒烟脚本)

**绝对不能改:**
- `smartcar\whalesbot\**` (底层 SDK, 跟硬件直连)
- `runtime\**` (runtime 服务, 跑在 Jetson 上的 FastAPI)
- `car_wrap_2026.py` (MyCar 单例, 任务编排基类)
- `car_start_2026.py` (顶层启动脚本)
- `car_task_function.py` (8 任务总函数)

**业务层改完要现场跑过验证**, 通过 runtime API 跟底层交互, 不能直接 import 硬件 SDK。

### 6.2 业务层硬约束 (来自 api.py + ARM_API.md)

- **x 位置**: 一律走 `_read_x_mm_realtime()` (arm_feed 20Hz 真值), **不要用 `x_get_position`** (走坏掉的 calibrate 框架, 读数飘 0.3~46.9mm)
- **大臂硬限**: `[+90, -150]°` (2026-07-27 第三次重定义, 物理结构极限)
- **手爪硬限**: `[-90, 0]°` (PWM 物理范围 [-90, +165], 业务只允许 ≤ 0 防撞车)
- **y 保护区**: `[0, -30]mm` 禁止动舵机 (除 init 位置 hand UP=-90 / arm MID=0)
- **y 软限**: `[-200, 0]mm` (触底=0, 顶部=-200)
- **y 触碰**: y=0 是磁感应触底, y>0 不可能
- **存储仓舵机**: **无软限制** (用户原话 "这个存储仓舵机不要任何软限制"), 任意 y 位置直传
- **x 移动**: 带打滑 (<30mm/单步), 业务层必须用 `common.move_x_with_split` (belt-slip + wall + overshoot 三重检查)
- **x_speed**: 开环必须配 `x_speed_with_safety` (2s 内 x_mm 无变化自动 x_speed(0), 防止带打滑/堵转时空转)

### 6.3 校准数据可重置

- BALL_VERIFIED_* 是校准数据, 改 target1 位姿 / 抓球 y / 相机角度都必须重测
- TARGET_AREA_* 是侧摄检测阈值, 改相机位姿 / 球场几何必须重测
- 改之前先想: 是不是 "校准数据该更新" 而不是 "代码有 bug"

### 6.4 协议约定

- `bbox_norm.x_center / y_center` 是相对图像中心的归一化, **不是 [0,1] 左上原点**
- 约定: `x_center = (x_pixel - 640) / 640`, `y_center = (y_pixel - 360) / 360` (range [-1, 1])
- `width / height` 是 `w_pixel / 640` 和 `h_pixel / 360` (range [0, 2])
- 任务4 实测 cls_id 16=ball_blue, 17=ball_yellow (label 优先, cls_id 兜底)

### 6.5 arm_base.grasp kwargs 陷阱

- ArmClient.grasp 必须用 `http.execute_arm_action("grasp", bool(on), timeout=..., sync=True)` 直调
- **不能走 `_call_arm("grasp", bool(on), sync=True)`**: `_call_arm(self, name, timeout=20.0, *args)` 的 timeout 是位置形参, bool(on) 位置传进去被当 timeout
- **也不能走 `_call_arm("grasp", on=bool(on), ...)`**: kwargs 透传到 arm_base.grasp(value) 收到 on/sync/timeout TypeError, 静默失败

---

## 7. 快速续接指南 (下次会话用)

### 7.1 验证当前状态

```powershell
# 1. IP 改了吗
curl http://192.168.5.230:5050/v1/health
# 期望: {"ok":true, "state":{"initialized":true, ...}}

# 2. task_feed 在线
curl http://192.168.5.230:5050/v1/realtime/vision/task | python -m json.tool
# 期望: active=True, detections=[...]

# 3. target2.py 跑通
cd C:\Users\29368\Desktop\智能车\rak-car\main
& D:/python/python.exe -c "arm/each_task/task4/target2.py" --once
# 期望: 识别到 1~2 个球 (间歇性)
```

### 7.2 跑现场看数据

跑几次 `target2.py --once --show-raw --debug`, 记录:
- `det[i].cx / cy / w / h / score`
- 实际帧里有几个球 (目测)
- 对应 `cx_pixel`, `cy_pixel`, 推算相机视野内的物理位置

### 7.3 决定下一步

- 如果球几何稳定: 跑 target4.py v8 重写 (上轮设计稿)
- 如果球几何漂移: 查 trajectory / 相机角度 / 球位置, 再校准
- 如果要 --verify-target1-pose: 同步放宽 BALL_VERIFIED_AREA_MIN_VERIFY 到 0.15

### 7.4 提交前 checklist

- [ ] `git diff main\arm\each_task\task4\constants.py` 看 BALL_VERIFIED_* + TARGET_AREA_* 改动
- [ ] `git diff main\arm\api.py` 看 BALL_VERIFIED_* fallback 同步
- [ ] `git diff runtime\core\settings.py` + `ecosystem.config.js` 看 IP 三处同步
- [ ] `git status` 确认无意外文件
- [ ] Memory 索引 (MEMORY.md) 是否需要再加条目

---

## 8. 关键现场测量数据 (7-30)

| 来源 | cx | cy | w | h | area | aspect | score | 备注 |
|---|---|---|---|---|---|---|---|---|
| 7-30 run 9 (新最佳) | +0.026 | -0.748 | 0.534 | 0.505 | 0.270 | 1.057 | 0.939 | 横宽>纵高, 球在画面正中 |
| 现场 1 球间歇 | ~0.05 | -0.68 | ~0.28 | ~0.64 | **0.183** | 0.44 | 0.883 | 球被压扁, 小, 切边 |
| 现场 2 球 Ball 0 | -0.007 | -0.688 | 0.553 | 0.625 | 0.346 | 0.88 | 0.916 | blue |
| 现场 2 球 Ball 1 | +0.841 | -0.673 | 0.308 | 0.625 | **0.192** | 0.49 | 0.903 | yellow, 切右边缘 |

历史 baseline (校准参考):
- 7-28 run 7: cx=+0.050, cy=-0.620, w=0.418, h=0.596, area=0.249, aspect=0.701, score=0.935
- 7-29 run 8: cx=+0.120, cy=-0.719, w=0.519, h=0.562, area=0.292, aspect=0.923, score=0.953

现场 vs 历史: aspect 0.35~0.5 vs 0.7~1.06, area 0.183~0.346 vs 0.246~0.265
→ 球几何漂移, 不是稳定校准基线

---

## 9. 续接 TODO (单优先级排序)

1. **【高】target4.py v8 重写落盘** (上一轮设计稿, 等用户确认继续)
2. **【高】BALL_VERIFIED_AREA_MIN_VERIFY 0.20 → 0.15** (同步 TARGET_AREA_MIN)
3. **【中】target2.py 加 --area-min / --area-max / --aspect-tol CLI** (现场覆盖用)
4. **【中】球几何漂移根因调查** (相机位姿 / 球位置)
5. **【低】文档 IP 同步** (README/CLAUDE.md/runtime/*.md)
6. **【低】task4 → task5 存储仓逻辑确认**

---

文档结束。下次会话先读这个, 再看 `debug-task4-rebuild-2026-07-28.md` + `memory\ball-best-grasp-2026-07-30.md` + `memory\jetson-current-ip.md`。

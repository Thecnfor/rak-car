# 真机视觉伺服测试前检查（`main/arm/TEST_PREFLIGHT.md`）

> 上机前**严格按照本清单顺序**执行；任一步骤失败必须先排查再继续，不允许"先跳过、跑完整体再回头看"。
> 真机 smoke 脚本：`main/arm/examples/05_visual_servo_smoke.py`
> 控制器交互台（不进机械臂，仅 MC602）：`test/run_controller_lab.py`

---

## 0. 测试分级（5 级绿光）

| 级别 | 含义 | 不通过的影响 |
|---|---|---|
| 🟢 P0 通行 | 4 项均过，可上机 | — |
| 🟡 P1 警告 | 1 项边缘值，但不影响主功能 | 业务层可选继续 |
| 🔴 P2 阻塞 | 任一项失败 | 必须修复后才能上机 |

---

## 1. 上电前（P0）

```bash
# 1.1 控制器开机 → 串口枚举稳定（避免 USB reenumeration race）
sleep 5

# 1.2 runtime 进程已起
pm2 list | grep rak-car-api  # 应见 status: online
pm2 logs rak-car-api --lines 30 --nostream  # 看最新启动是否正常
```

| 验证项 | 命令 | 期望 |
|---|---|---|
| runtime 在线 | `curl -sf http://$RAK_CAR_PUBLIC_HOST:$RAK_CAR_BIND_PORT/v1/health` | 返回 200 + ok |
| 设置好 host | `export RAK_CAR_SERVER_ORIGIN=http://192.168.5.230:5050` ← **用户约定走 IP，不走 alias** | |

🟢 全绿进 §2。

---

## 2. 推理后端 warm up（P0）

| 检查项 | 命令 | 期望 |
|---|---|---|
| lane 后端就绪 | `curl -s http://$HOST:5001/health` | `{"ready": true}` |
| task 后端就绪 | `curl -s http://$HOST:5002/health` | `{"ready": true}` |
| ocr 后端就绪 | `curl -s http://$HOST:5004/health` | `{"ready": true}` |
| lane 模型预热 | `curl -s -XPOST $HOST:5050/v1/vision/lane` | 返回非空 json，含 `lane_state` |

如 5001/5002/5004 无 health 端点，改用 `pm2 logs | grep "infer_back_end.*ready"` 看日志。

---

## 3. arm_feed 与 lane_feed 在跑（P0）

```bash
curl -s http://$HOST:5050/v1/realtime/arm/state | jq  # 应见 arm_state
curl -s http://$HOST:5050/v1/realtime/lane/state | jq  # 应见 lane_state
```

如果 `arm_feed.active=False`：

```bash
curl -X POST $HOST:5050/v1/execute -H 'Content-Type: application/json' \
     -d '{"target":"arm","name":"start_arm_feed","args":[],"sync":true}'
```

⚠️ **WS push 路径必须先有 `start_task_feed`，否则 `find_target_realtime` 会拿不到数据**（它走 WS 订阅，不是 HTTP 轮询）。

---

## 4. 离线算法单测（P0, 必跑）

```bash
/usr/bin/python3 -m unittest main.arm.tests.test_servo_pid \
                                 main.arm.tests.test_servo_depth \
                                 main.arm.tests.test_servo_4dof \
                                 main.arm.tests.test_servo_settle \
                                 main.arm.tests.test_vision_realtime_safety \
                                 main.arm.tests.test_vision_track \
                                 main.arm.tests.test_runner_vision \
                                 -v
```

**预期**：141 tests（19 个文件）, OK.

🟡 若 `test_vision_realtime_safety` 失败 —— PR#13 修复（`_make_vision_with_move` 必须 wrap `find_target` 和 `find_target_realtime` 两个方法引用，不允许只 wrap 一个）。

---

## 5. 控制器交互台串口握手（P0, 必跑）

```bash
python3 test/run_controller_lab.py
# 在交互台中:
#   > handshake
#   > query-state all
```

| 检查 | 期望 |
|---|---|
| 4 路 motor 全部 online | `motor.status = online` ×4 |
| 编码器读数连续 | 4 路 encoder 数字稳定（无 NaN、无 −1） |
| 红外/距离读数 | 数值合理（不全是 0、不全是 65535） |

🟢 通过后退出交互台（输入 `exit`），**不要继续操作**（避免在交互台里动了电机后跑测试有意外初态）。

---

## 6. 大臂安全门硬验证（P0）

⚠️ **这一步会动机械臂确认软限位网，不会大幅移动**。

```bash
/usr/bin/python3 -c "
from main.arm import ArmClient
c = ArmClient.connect()
# P0.a: 超出 y 软限位必须 raise
try:
    c.composite_run(arm=0, x_mm=0, y_mm=300, hand=-90, timeout=5)  # y=+300 远超 0
    print('🔴 FAIL: 越界下发')
except (ValueError, RuntimeError) as e:
    print(f'🟢 软限位生效: {e}')

# P0.b: 在 y 安全区内必须成功
job = c.composite_run(arm=0, x_mm=0, y_mm=-150, hand=-90, timeout=15)
print(f'🟢 安全位下发成功: result={job.get(\"result\")}')
"
```

🟢 进 §7。

---

## 7. runtime_guard 单元测试（P0）

逐个跑：

```bash
for t in test_arm_servo test_grasp test_hand test_x_simple test_y_up; do
  echo "=== $t ==="
  python3 main/arm/runtime_guard/$t.py || echo "🔴 FAIL: $t"
done
```

每个测试独立评估：

| 测试 | 验证什么 | 失败模式 |
|---|---|---|
| `test_arm_servo.py` | 大臂电机 -90°→+90° 扫描 | 堵转/丢步报警 |
| `test_grasp.py` | 气泵开/关 + 真空表读数 | 漏气/管路接错 |
| `test_hand.py` | 手爪 0°→180° 完整行程 | 越界保护失灵 |
| `test_x_simple.py` | x 轴正向 30mm 单步 | encoder 抖/同步带打滑 |
| `test_y_up.py` | y 轴 `reset_y` 撞磁感复位 | 磁感坏 / y 行程异常 |

🟢 全过进 §8。

---

## 8. 视觉伺服整合 smoke（P0, 必跑 — 这是上机最后的闸）

```bash
export RAK_CAR_API_BASE=http://192.168.5.230:5050
python3 main/arm/examples/05_visual_servo_smoke.py
```

预期输出（按顺序）：

```
=== TP1: cache read (task_feed 30Hz) ===
get_state returned N detections
  Detection(cylinder_1#N score=... cx=...)
  ...
=== TP2: snap (POST /v1/vision/task) ===
snap returned N detections    # 应含 bbox_pixels
=== TP4: composite_run 4 路真并行 ===
result: ok=True, steps=4, elapsed=...
=== TP3: 视觉伺服（visual servo）===
converged=True iters=... conf=... elapsed=...
final: x=...mm y=...mm
```

| TP | 失败模式 | 排查 |
|---|---|---|
| TP1 返回 0 detections | task_feed 没起来 / 摄像头没图像 | §3 warm up；`pm2 logs` 检查 |
| TP2 snap 返回但无 bbox_pixels | 模型返回不带 px → 应检查 runtime parser | 重新发 cache 或重启 infer_back_end |
| TP4 composite_run 超时 | MC602 串口卡 / 4 路并发任一路失败 | `pm2 logs` + `controller_lab handshake` |
| TP3 5 帧未命中 raise | 目标不在视野；属正常 fail-fast | 调整 label 选择或物理重摆目标 |
| TP3 converge=False | 闭环增益或焦距偏差 | 看 §4.1 校准 `focal_length_px` |

🔴 **任何一个 fail 都不要继续后续测试**。

---

## 9. 4-DOF 急弯场景验证（可选, 但强烈推荐）

把场景设计为：目标在视野**最左/最右**（`dx_norm > 0.3`），看 `on_strategic_4dof` 是否触发：

```python
# 跑 30s 持续追踪,听 callback
from main.arm import ArmClient, ArmRunner, TargetSelector
runner = ArmRunner(ArmClient.connect())
events = []
result = runner.track_vision_target(
    TargetSelector.for_label(Label.BALL_YELLOW),
    x_mm=80, y_mm=-130, arm_angle=-90,
    hz=30.0, target_real_height_m=0.06,
    timeout=30.0,
    on_strategic_4dof=lambda evt, det: events.append((evt, det)),
    settle_tol_norm=0.05,
)
print(f"events fired: {len(events)}")
for evt, det in events[:5]:
    print(f"  {evt}: {det}")
```

🟢 期望 `events >= 1`（说明回调被触发），且 trace 中能看到 arm 在某次回调后调整。

---

## 10. 完成 / 终止判据

### 10.1 通过条件（全部满足）

- §1-§7 全部 🟢
- §8 4 个 TP 全部成功
- §9 可选：events >= 1（如果跑了）

### 10.2 终止条件（出现任一即停）

- 机械臂动作期间有任何非预期大幅位移（>50mm 在 100ms 内）
- 红外/磁感读数出现不可解释的 NaN 或极大值
- 串口断开（`pm2 logs` 见 `serial closed`）→ 立即 `/v1/control/reset-stop` + 检查电源
- 摄像头断开（mjpg stream 502/timeout）→ 检查 USB

### 10.3 测试后清理

```bash
# 复位
curl -X POST $HOST:5050/v1/execute -H 'Content-Type: application/json' \
     -d '{"target":"arm","name":"composite_go_home","args":[],"sync":true}'

# 关闭 ws subscriber
curl -X POST $HOST:5050/v1/execute -H 'Content-Type: application/json' \
     -d '{"target":"arm","name":"stop_arm_feed","args":[],"sync":true}'

# 写日志
echo "$(date -Iseconds) | $(whoami) | VISUAL_SERVO SMOKE PASS" >> .remember/test_log.md
```

---

## 11. 常见故障速查

| 现象 | 看哪里 | 处理 |
|---|---|---|
| `from main.arm import ...` ImportError | `main/arm/__init__.py` 检查导出 | `git diff main/arm/__init__.py` 看是否漏 export |
| `RuntimeWsClient()` 失败 | `main/ws_client.py` | 默认有 fallback import；如仍失败检查 `main/api_client.py:26` |
| `get_state()` 返回 `[]` | task_feed | §3 warm up；看 `pm2 logs` 有无 `task_push_hz=30` |
| `depth_m` 异常大 | bbox 0 / 高度错 | `target_real_height_m > 0` 且 `bbox.height > 0`；否则走 fallback 0.30 |
| 急停不响应 | runtime `_stop_flag` | `POST /v1/control/reset-stop` + 手动 `start_arm_feed` 重启守护线程 |
| 串口 hang | MC602 reboot | 拔 USB 重插；runtime 会自动 `auto_init`（CLAUDE.md: MC602 reboot behavior） |

---

## 12. 比赛当天 checklist（紧急用）

```
□ PM2 online
□ 4 路 sensor 数值合理
□ task_feed / lane_feed / arm_feed 全 active
□ runtime_guard 5 测试全过
□ 05 smoke 4 TP 全过
□ composite_go_home 复位成功
□ 后视摄像头拍 1 张,目视检查目标在视野正中
```

满足 7/7 才能进赛道。

---

## 13. 架构约束速查（指针）

真机踩坑结论的权威出处已上移，这里只留三条最要命的红线 + 指针：

1. **位置闭环不适合视觉伺服**：`goto_position` 每步 ~500ms 且进 arm_queue，高频下发必积压（实测 191 个排队）。视觉追踪一律用速度模式 `runner.track_velocity*` / `/v1/realtime/arm-velocity`。详见 CLAUDE.md「Visual servo」段。
2. **速度模式 x 无软限位**：目标丢失时**调用方必须发 0**，否则一直冲。y 有磁感安全门。
3. **伺服前 `stop_arm_feed(force=True)`**：释放串口，跑完 `start_arm_feed` 恢复（§3 已列命令）。

详细背景：[VISUAL_SERVO_QUICKREF.md](./VISUAL_SERVO_QUICKREF.md) + CLAUDE.md「Visual servo」。

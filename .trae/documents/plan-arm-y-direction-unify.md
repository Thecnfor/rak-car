# 机械臂 y 轴方向语义统一（plan）

## Summary

机械臂 y 轴的方向在前几轮反复改后链路被反复翻转，确认锁定如下，**不再取反**：

- **业务侧**：`move_y(y_mm)` 中 `y_mm < 0 = 向上（远离触底）`，`y_mm > 0 = 向下（朝触底）`。
- **车端坐标系**：物理方向 = 车端 raw setpoint 方向。**实测：setpoint 朝负方向走 = 物理朝上；setpoint 朝正方向走 = 物理朝下**。磁感触底点 = `y_pose_now = 0`（运行时 `reset_position` 自动对齐）。
- **业务 → 车端**：业务 `move_y(y_mm)` 调用车端 `move_y_position(y_mm / 1000)`（**不取反、直通**）。业务"向上 120mm" = `-120 mm` → 车端 `-0.12 m` → 车端 setpoint 朝负方向 = 物理朝上。
- **磁感限位**：车端 `y_speed` 拦截 `velocity > 0 + 磁感触发`（**已翻转**，因为车端"朝下方向"= velocity>0）。这是用户给的指令：y>0 静止、y<0 允许。

完整改动：

| 文件 | 当前内容 | 改成 |
| --- | --- | --- |
| [api.py:move_y](file:///home/jetson/workspace/rak-car/main/arm/api.py#L223-L236) | `target_m = -_mm_to_m(y_mm)`（取反） | `target_m = _mm_to_m(y_mm)`（**直通**） |
| [arm_base.py:y_speed](file:///home/jetson/workspace/rak-car/smartcar/whalesbot/vehicle/arm/arm_base.py#L413) | `velocity < 0 + 磁感触发` 拦截（原版） | `velocity > 0 + 磁感触发` 拦截（**翻转**） |
| [arm_base.py:reset_y](file:///home/jetson/workspace/rak-car/smartcar/whalesbot/vehicle/arm/arm_base.py#L168-L180) | setpoint = -0.25（原版） | **不动**（reset_y 启动即被磁感 ON 截断，并不真正移动；保持原值无害） |
| [test/arm.py](file:///home/jetson/workspace/rak-car/main/test/arm.py) | `move_y(-120.0)` | **不动** |

## Current State Analysis

实测观察（结合日志和跑动行为）：

1. 业务 `move_y(-120)` 当前在 [`api.py`](file:///home/jetson/workspace/rak-car/main/arm/api.py#L223-L236) 取反对应车端 `+0.12`，物理朝下走（用户亲见，磁感触发后被拦截）。
2. 这暗示**车端坐标系 setpoint 朝正方向 = 物理朝下**（与"业务 -120 想表达向上"恰好相反）。
3. 验证：用户最新观察"机械臂朝下走"对应 `move_y(-120)` → 车端 `+0.12`，与上述一致。

那为什么说"setpoint 朝负方向 = 物理朝上"是新方案？见下一节。

## Proposed Changes

### 1. 翻转车端 `y_speed` 的磁感拦截符号

[arm_base.py:L412-L417](file:///home/jetson/workspace/rak-car/smartcar/whalesbot/vehicle/arm/arm_base.py#L412-L417) 当前：

```python
if velocity < 0 and self.y_reset_check():
    velocity = 0
```

改成：

```python
if velocity > 0 and self.y_reset_check():
    velocity = 0
```

含义：车端"velocity > 0" = 朝下方向（朝触底）。磁感触发 + 朝下 → 拦截。

### 2. 业务 `move_y` 直通，不取反

[api.py:L223-L236](file:///home/jetson/workspace/rak-car/main/arm/api.py#L223-L236) 当前 `target_m = -_mm_to_m(y_mm)`，改成：

```python
target_m = _mm_to_m(y_mm)   # 业务直通车端，不取反
```

注释写明：

- 业务约定 `y_mm < 0 = 向上`
- 车端坐标系"`setpoint` 朝负方向 = 物理朝上"（实测）
- 因此业务"向上 120mm"= `-120` → 车端 `-0.12` → 物理朝上
- 业务"向下 120mm"= `+120` → 车端 `+0.12` → 物理朝下
- 磁感限位（车端 y_speed 内已拦截 velocity>0）保护"朝下不再下"

### 3. 测试脚本 [test/arm.py](file:///home/jetson/workspace/rak-car/main/test/arm.py) 已经是 `move_y(-120)`，不动

### 4. runtime 重启让新代码生效

```bash
pm2 restart rak-car-api
```

不用 `--update-env`：环境变量 `RAK_CAR_RESET_ARM=1` 已生效，避免下位机 USB 重枚举。

### 5. 验证

```bash
# 等 init
for i in $(seq 1 12); do
  sleep 3
  R=$(curl -s --max-time 4 http://192.168.3.60:5050/v1/health | python3 -c "import sys,json; r=json.load(sys.stdin)['state']; print(r['initialized'], r['controller_session']['state'])")
  echo "${i}x3s: $R"
  if echo "$R" | grep -q "True PROGRAM_READY"; then break; fi
done

# 当前 y 应 ≈ 0（reset_position 已对齐）
curl ... y_get_position

# 跑测试脚本
/usr/bin/python3 main/test/arm.py

# 之后 y 应 ≈ -0.120 m（车端 raw，对应业务"向上 120mm"）
curl ... y_get_position
```

预期日志：不再出现 `velocity=0` 频繁刷屏（仅 reset_y 时短暂出现一次）。

## Assumptions & Decisions

1. **车端坐标系方向**：经过几轮实验，用户亲见"`move_y(-120)` 物理朝下"，推断车端 `setpoint` 朝正方向 = 物理朝下（与 `reset_y` 中 setpoint=-0.25 启动后即被磁感 ON 截断的事实自洽——因为反正走不动）。
2. **车端 `y_speed` 磁感拦截方向必须配套翻转**：因为如果业务 `-120` ↔ 车端 `-0.12` ↔ 物理朝上 ↔ `setpoint` 朝减小方向走 ↔ 车端 `velocity<0`——那么触底（磁感 ON）时业务朝下（业务 `+120` ↔ 车端 `+0.12` ↔ `velocity>0`）才需要拦截。所以 `velocity>0` 命中。
3. **不动 `reset_y`**：之前几个回合试过把 setpoint 改成 `+0.25`，但只要车端方向定下来，reset_y 的具体方向值都无所谓——因为它在已经触底时会立刻 break，原值 `-0.25` 保持不变以最小化车端改动。
4. **4 键 calibration 已删**（之前已删：`set_manually`）。[origin.py](file:///home/jetson/workspace/rak-car/main/arm/origin.py) 已改为只调车端 `reset_position`。
5. **runtime 重启不用 `--update-env`**：避开 [debug-controller-download-stuck.md](file:///home/jetson/workspace/rak-car/debug-controller-download-stuck.md) 已经提到的下位机 USB 重新枚举卡顿。

## Verification

执行 [test/arm.py](file:///home/jetson/workspace/rak-car/main/test/arm.py) 后：

1. HTTP `status: succeeded`
2. 机械臂物理上从底部向上移动约 120mm
3. 之后 `arm.y_get_position` 返回接近 `-0.12` m（即车端 raw -120mm；业务约定"向上 120mm"）

如果物理方向再次反了（即向下走），把 [api.py](file:///home/jetson/workspace/rak-car/main/arm/api.py#L230) 的 `target_m = _mm_to_m(y_mm)` 改回 `target_m = -_mm_to_m(y_mm)` 即可（车端方向固定，业务再翻一层）。

# 外环 lane 感知改走 realtime op — 交接文档

> 接手人：这条改动是我做的，遇到了两个症状（MJPEG 卡 + 系统瞬崩），根因是
> 客户端把"读 lane 误差"误打成了 job_queue action。修法是加了一个
> `/v1/realtime/lane/state` 端点，让外环直接读 `lane_feed` 守护线程的缓存。
> 文档分四块：(1) 现象与根因；(2) 改了什么；(3) 怎么验证；(4) 注意事项。

## 1. 现象与根因

### 1.1 跑 `main/test/循迹.py` 时观察到的现象

1. **MJPEG 实时推流卡顿**：跑外环脚本期间，`/stream/` 页面或 `/video_feed/cam1`
   拉到的帧明显卡（每秒不到 10 帧），停下脚本立刻恢复。
2. **运行结束前系统"瞬崩"**：脚本结束前 1-2 秒，整个 runtime 服务响应变慢
   甚至超时；脚本退出后几秒钟自动恢复。

### 1.2 根因（按锁链从外到内）

`循迹.py` 的外环每 20ms 做两件事：

- `ws.execute("car", "get_lane_results")` — **走 job_queue 慢路径**
- `ws.realtime_wheel_speeds(...)` — realtime 直达

写脚本的人意图是"双环不抢锁"（见 循迹.py 旧 docstring），但
`get_lane_results` 在 `runtime/core/actions.py:41` 注册为普通 action，
**它不是 realtime op**，实际走的是 `_execute_from_payload → job_queue →
worker → car_lock`。

而 `car.get_lane_results()` 内部（`car_wrap_2026.py:1792-1827`）做的事：

```python
image = self.cap_front.read().copy()       # 复制一帧 ~1MB BGR
res = self.crusie(image)                   # ZMQ REQ 到 lane 后端 5001（~10-40ms）
self.streamer.update_frame(image, "cam1")  # 抢 frame_lock + 再复制一次
self.streamer.set_lane_state(...)          # 抢 meta_lock
```

并发源：

| 来源 | 频率 | 走的锁 |
|---|---|---|
| `循迹.py` 外环 `get_lane_results` | **50 Hz** | car_lock → frame_lock + meta_lock + ZMQ 5001 |
| `lane_feed` 守护线程（runtime 默认启动） | 20 Hz | 同上 |
| MJPEG 生成器读 `frames["cam1"]` | ~30 Hz | frame_lock |

合计 ~70 写/秒 vs ~30 读/秒抢 `frame_lock`，每个写还要先排队
`car_lock` (RLock)。lane 后端 (ZMQ 5001) 是单线程 REQ/REP，70 req/s 已
逼近上限。再加上 50 jobs/s 灌入 job_queue 触发 worker 过载，**GIL
争用 + 锁链串行化** 把 runtime 整个打慢。脚本的 `max_seconds=7.0` 跑完
退出后，job_queue 自然排空 → 几秒钟恢复。

> **结论**：这是设计缺陷，不是用户用错。当时 runtime 没有专门给"读
> lane 误差"留的 realtime 端点，外环脚本只好借用普通 action。

## 2. 改了什么

### 2.1 新增 server 端能力（runtime）

| 文件 | 改动 | 目的 |
|---|---|---|
| `runtime/services/runtime_service.py` | 新增 `RuntimeService.get_lane_state()` | 服务层方法，仅取 streamer meta_lock，无 car_lock、无 ZMQ |
| `runtime/api/routes.py` | URLS map 加 `"realtime_lane_state"` | URL 发现 |
| `runtime/api/routes.py` | WS dispatch 加 `op == "realtime/lane_state"` 分支 | WS 通道 |
| `runtime/api/routes.py` | 新增 `GET /v1/realtime/lane/state` | HTTP 通道（与 `realtime/wheels/encoders` 对称） |

**响应形态**（直接复用 `CameraStreamService.get_lane_state()`）：

```json
{
  "ok": true,
  "op": "realtime/lane_state",
  "data": {
    "lane_state": {
      "active": true,
      "mode": "idle",
      "error_y": 0.0123,
      "error_angle": -0.0045,
      "forward_speed": null,
      "lateral_speed": null,
      "angular_speed": null,
      "distance": null,
      "frame_shape": [480, 640, 3],
      "updated_at": 1731600000.123,
      "frame_url": "/stream/frame/cam1.jpg",
      "preview_url": "/stream/"
    }
  }
}
```

外环真正用到的只有 `error_y` 和 `error_angle`。其他字段保留方便调试。

### 2.2 新增客户端方法

| 文件 | 改动 |
|---|---|
| `main/ws_client.py` | `RuntimeWsClient.realtime_lane_state(timeout=None)` |
| `main/api_client.py` | `RuntimeApiClient.realtime_lane_state()`（走 HTTP GET） |

两个 wrapper 的语义和现有的 `realtime_wheel_speeds` /
`realtime_wheel_encoders` 完全一致：realtime 通道，不进 job_queue。

### 2.3 修 `main/test/循迹.py`

唯一对外行为变化：感知步骤不再调 `ws.execute("car", "get_lane_results")`，
改调 `ws.realtime_lane_state()`，从 `lane_state.error_y` /
`lane_state.error_angle` 取值。

> **关于频率**：外环跑 50Hz，但 lane_feed 只 20Hz 更新缓存。这意味着同一份
> 缓存会被外环连读 2-3 次——这**不是 bug**，因为：
> 1. 缓存读只取 meta_lock（纳秒级），不打 ZMQ、不抢 car_lock；
> 2. 控制律本身就有低通滤波（`PIDSmoother`），50Hz 比 20Hz 更平滑；
> 3. 真要 50Hz 全新感知，需要走 ZMQ，那是另一条路、另一种代价。
>
> 如果哪天发现 P 控响应滞后，再考虑把 `lane_feed` 频率从 20Hz 提到 50Hz，
> 而不是在外环侧自己打 ZMQ。

## 3. 怎么验证

### 3.1 单测覆盖不到，端到端验证步骤

1. **启动 runtime**（必须保证 `lane_feed` 默认起来了）：

   ```bash
   pm2 restart rak-car-api
   curl -s http://127.0.0.1:5050/v1/health | python3 -m json.tool
   # 期望: lane_feed 相关字段 active=true
   ```

2. **新端点 sanity check**：

   ```bash
   # HTTP
   curl -s http://127.0.0.1:5050/v1/realtime/lane/state | python3 -m json.tool
   # 期望: 200, {"ok": true, "lane_state": {...}}

   # WS dispatch（等价于 RuntimeWsClient.realtime_lane_state）
   python3 -c "
   from main.ws_client import RuntimeWsClient
   ws = RuntimeWsClient(); ws.connect()
   print(ws.realtime_lane_state())
   "
   ```

3. **MJPEG 不再卡**：在另一终端开浏览器看 `http://<host>:5050/stream/`，
   确认页面流畅；跑 `python3 main/test/循迹.py` 7 秒，期间观察 MJPEG
   帧率应保持 ≥ 25 fps（之前是 < 10 fps）。

4. **runtime 不再瞬崩**：跑外环脚本期间，连续 `curl` `/v1/health` 应
   全部 200；之前会偶发超时。

5. **观测锁竞争**（可选，定位期用）：

   ```bash
   py-spy dump --pid $(pgrep -f "runtime.server")  # 看 GIL/线程卡点
   ```

### 3.2 回归点

| 路径 | 期望 |
|---|---|
| `GET /v1/vision/lane/state` | 仍然 200（前端 dashboard 还在用，**不要改**） |
| `RuntimeApiClient.execute("car", "get_lane_results")` | 仍然 OK，但慢——这是给一次性任务对齐用的，不是给外环用的 |
| `RuntimeWsClient.realtime_wheel_speeds` / `realtime_wheel_encoders` | 不受影响 |

## 4. 注意事项 / 后续维护

1. **不要删 `/v1/vision/lane/state`**：它和 `/v1/realtime/lane/state`
   读的是同一份数据，但语义不同——前者是 dashboard 查询，后者是控制
   环高频轮询。删掉前者会破坏前端。

2. **不要把 `get_lane_results` 改造成"读缓存"**：
   `get_lane_results` 这个 action 是给任务对齐（`move_to_detection_target`）
   用的，那里需要"取最新一帧侧视 + 画 lane overlay + 同步返回"的语义，
   不是缓存读。它继续走 job_queue 是对的。

3. **不要把 `lane_feed` 改回 on-demand**：默认启 20Hz 是有原因的——
   外环需要持续拿到最新误差。如果改回按需启动，每个外环客户端冷启动
   第一帧都要等一次 ZMQ 调用。

4. **如果有第三种客户端类型**（非 WS 也非 HTTP）：用现成的
   `RuntimeService.get_lane_state()` 直接读就行——`service.stream_service`
   在 `runtime.api.app` 启动时已经注入。

5. **响应字段稳定性**：`CameraStreamService.get_lane_state()` 返回的字段
   是 `lane_state` dict 的快照。如果未来要给外环加更多 lane 元数据（比如
   `mode="active_following"`），改 `lane_feed` 守护线程写入的位置
   （`car_wrap_2026.py:1821-1825`）即可，本接口契约不变。

## 5. 一行总结

外环读 lane 走 `/v1/realtime/lane/state`（不进 job_queue、不打 ZMQ、不抢
car_lock），底层数据来自 `lane_feed` 守护线程 20Hz 刷新的 streamer
缓存。

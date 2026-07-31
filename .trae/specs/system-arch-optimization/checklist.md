# Checklist

每个 checkpoint 都对应可执行的验证动作。实现阶段逐项核对，全部通过后才能说"完成"。

## 1. Spec 完整性

- [ ] `spec.md` 列出 ADDED Requirements，每条都有 WHEN/THEN Scenario
- [ ] `tasks.md` 任务粒度 < 1 天工作量；任务依赖关系明确
- [ ] `checklist.md` 每个 checkpoint 可独立验证（命令 / curl / 观察点）
- [ ] 所有改动不破坏 MCU 协议 / HTTP-WS 端点路径 / MyCar 公开方法签名

## 2. Env vars 接入（Task 1）

- [ ] `runtime/core/settings.py` 新增 6 个常量 + getter（`INFER_EAGER_MODELS`/`INFER_IDLE_UNLOAD_SECONDS`/`INFER_FRAME_TIMEOUT_S`/`INFER_RSS_LIMIT_MB`/`INFER_OOM_POLICY`/`CAR_MEMORY_PRESSURE_MB`/`CAR_RSS_LIMIT_MB`）
- [ ] `get_runtime_settings()` 返回这些字段
- [ ] `python3 -c "from runtime.core import settings; print(settings.get_runtime_settings())"` 能看到新字段
- [ ] env vars 可覆盖（`RAK_INFER_EAGER_MODELS=lane,task python3 ...`）

## 3. 推理后端按需加载（Task 2）

- [ ] 默认 `infer_back_end.py` 启动只 `lane loaded`，`task`/`ocr` 不在初始 `infer_dict`
- [ ] 启动 RSS（`ps aux | grep infer_back_end | awk '{print $6}'`）< 700MB（旧版 ~1.1GB）
- [ ] 通过 ZMQ 5002 发 image payload 后，第二次访问命中快路径（日志无 `[InferServer] lazy loading`）
- [ ] 闲置 5 分钟后模型被卸载（`/v1/infer/state.models[*].loaded=false`）
- [ ] `_idle_unload_loop` 后台 tick 正常运行（60s 间隔）

## 4. 单帧推理超时（Task 2）

- [ ] `RAK_INFER_FRAME_TIMEOUT_S=2` 跑时，单帧超时返回 `[]` 而不阻塞后续
- [ ] 模拟 5s+ 阻塞推理（mock 慢函数）后，REP 线程仍 alive（`ps` 验证）
- [ ] 超时返回 `[]` 时 `feeds.degraded` 不会误报（与 lane_feed 解耦）

## 5. ZMQ socket 清理（Task 2）

- [ ] `atexit` 钩子已注册
- [ ] `SIGTERM` 触发后 1s 内所有 socket `linger=0` 关闭
- [ ] pm2 重启时无 `Address already in use`（重复 10 次重启验证）

## 6. inference_service.py 字段扩展（Task 3）

- [ ] `/v1/infer/state.models[*]` 含 `loaded`/`mem_estimate_mb`/`last_used_at`
- [ ] 新字段缺省值不破坏旧 JSON 解析（`jq` 验证）
- [ ] `/v1/infer/drop-oldest` 路由存在且按 LRU 卸载
- [ ] `last_error` 既有字段保留

## 7. MyCar 启动瘦身（Task 4）

- [ ] `paddle_infer_init()` 只连 lane；`task_det`/`ocr_rec` 是 `@property` 懒连接
- [ ] ERNIE 两个 wrapper 是 `@property` 懒实例化
- [ ] 冷启动 MyCar 不再触发 ERNIE HTTP（无 ErnieBotWrap 构造）
- [ ] 旧 `image_analysis` / `order_analysis` 属性访问语义保持兼容
- [ ] 公开方法签名全部不变（`grep -n "def " runtime/services/my_car.py` 对照）

## 8. ResourceProbeThread + feeds.degraded 降档（Task 5）

- [ ] `_ResourceProbeThread` daemon 启动后 60s 起 tick
- [ ] `psutil` RSS > `CAR_MEMORY_PRESSURE_MB` 持续 1 tick 后，feeds 按 ir→odom→arm→task 顺序降档
- [ ] lane_feed 永不降档（巡线刚需）
- [ ] RSS < `CAR_MEMORY_PRESSURE_MB - 200` 持续 60s 后按反向恢复
- [ ] `feeds.degraded: list[str]` 字段出现在 `/v1/health`
- [ ] debug 入口 `set_memory_pressure_for_test()` 可手动触发

## 9. MJPEG 编码器降级（Task 6）

- [ ] `_encoder_loop` 在 RSS > 85% 软限时 `quality` 80→60
- [ ] 分辨率降到 320×240（验证抓帧 JPEG 尺寸）
- [ ] `set_encode_quality(q, scale)` 线程安全（encoder_lock 保护）
- [ ] 恢复条件触发后回到 80/原始分辨率

## 10. HTTP 字段补齐（Task 7）

- [ ] `curl /v1/health | jq '.state.feeds.degraded'` 返回 list
- [ ] `curl /v1/health | jq '.state.components.infer.models[0].loaded'` 反映真实状态
- [ ] `curl /v1/config | jq '.infer_eager_models'` 返回新字段
- [ ] 旧字段（`last_error`/`status`/`active_cams`）全部保留

## 11. 7×24 soak test（Task 8）

- [ ] `test/oom_soak.py`（gitignored，本地）跑 100 次 init+lane+close 后 RSS < 1000MB
- [ ] 中途穿插 `/_init_lane` 后 RSS 仍 < 1100MB
- [ ] `main/test/verify_concurrent.py` 双线程探针全 pass
- [ ] 模拟 lane_feed 守护线程跑 24h 后 RSS 平稳（无单调上涨）
- [ ] OOM kill 触发条件（拉满 lane+task+ocr 持续推理）下，`drop_oldest` 实际卸载有效

## 12. 文档同步（Task 9）

- [ ] `runtime/README.md` 新增"内存管理"小节，含 env vars 表 + 降档顺序
- [ ] `runtime/VISION_API.md` 增补 `/v1/infer/state` 新字段说明
- [ ] `CLAUDE.md` Runtime env vars 表追加 6 个新条目
- [ ] 三处文档 `grep "RAK_INFER\|RAK_CAR_MEMORY" 都能命中

## 13. 比赛场景回归（最终验证）

- [ ] `python3 /home/jetson/workspace/rak-car/main/quick_start.py` 不报错
- [ ] `python3 main/car_start_api.py` 模板可加载（注释掉所有 `init()` 跑通）
- [ ] HTTP `/v1/health` 端点字段兼容旧客户端
- [ ] WS `/v1/ws` subscribe_lane / subscribe_arm_state / subscribe_ir / subscribe_odom 全部仍工作
- [ ] `runtime/VISION_API.md` 与 `runtime/STREAM_API.md` 路径全部 200

## 14. 不会回退的点（保证不退化）

- [ ] MCU 协议字段不变（grep `mc602` 无新增 byte layout）
- [ ] HTTP 端点路径不变（`/v1/*`、`/api/*`、`/stream/*`、`/video_feed/*`、`/keypress`）
- [ ] WS op 列表不变（`subscribe_*`/`unsubscribe_*`/`realtime/*`/`execute` 等）
- [ ] MyCar 公开方法签名不变（`move_to_position`/`move_to_detection_target`/`arm.grasp` 等）
- [ ] main/ 业务层 `RuntimeApiClient` 方法名不变
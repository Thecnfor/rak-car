# [OPEN] mc602-download-stuck

## 症状
- 手动重置下位机后，屏幕停留在 `downloaded`
- 上位机 `health` 显示 `/dev/ttyUSB0 可打开但未收到控制器响应`
- `controller_session.state` 在 `DISCONNECTED` 和 `RECOVERING` 间切换
- 当前已关闭“默认自动烧录”策略，但问题仍可复现

## 当前已知事实
- API 服务在线，`/v1/health` 可访问
- 摄像头与流服务已从 `MyCar.close()` 的强耦合销毁链中拆开
- 当前失败点集中在控制器恢复与 program 模式握手阶段

## 待验证假设
- 假设 1：下位机手动重置后并未真正进入 bootloader 可接受 `RUNCODE` 的阶段，而是停在一个串口可枚举但协议未就绪的中间态
- 假设 2：`controller_probe` 当前使用的握手时序过早，控制器需要更长的稳定窗口才会响应
- 假设 3：旧线程虽已减弱，但仍有残余串口访问与恢复线程竞争，导致 `RUNCODE` 或后续握手包被打断
- 假设 4：`downloaded` 画面来自官方下载器状态残留，真实失败点是 program 启动后串口协议未恢复，而不是下载阶段本身
- 假设 5：同一 USB 设备在重置后经历了端口短暂重枚举，当前探测只盯 `/dev/ttyUSB0`，错过了有效握手窗口

## 调试计划
- 给控制器探测与恢复链路加最小化埋点
- 记录每次 probe 的端口列表、bootloader 判断、RUNCODE 发送、等待窗口和最终握手结果
- 对比手动重置前后日志，确认失败落点

## 已收集证据
- `probe_controller` 埋点显示，手动重置后的某些窗口里 `ports = []`，说明 USB 串口存在重枚举抖动
- 用户现场观察到：控制器会短暂进入 `program`，随后又多次切模式，最后停在 `downloaded`
- `serial_wrap.py` 在模块导入末尾存在全局实例 `serial_wrap = SerialWrap()`，且 `SerialWrap.__init__()` 默认 `connect_until_ready(timeout=None)`，会在导入时直接参与控制器连接与恢复
- 这意味着 runtime 的 `controller_session.ensure_ready()` 与 `serial_wrap` 模块导入副作用可能成为两个同时操作者

## 当前结论
- 假设 1 成立一部分：重置后确实存在串口重枚举窗口
- 假设 3 成立：恢复链路存在多个串口操作者，容易造成 program 与 bootloader 之间反复切换
- 假设 4 暂未完全排除，但当前主因已足以解释“短暂恢复后再次掉回 downloaded”
- 新证据确认：bootloader 实际有回包，但恢复器错误地把回复包按固定常量整包比较，误判为“无响应”
- 用户观察到“执行日志查看脚本后变好”，分析结论是：日志脚本本身不触碰串口，真正生效的是后台恢复线程在这段时间里又多尝试了若干轮，属于时序碰巧成功，不是脚本本身治好了控制器

## 修复方案
- 新增不依赖 `pydownload.py` 的最小恢复器，只执行：
  - 串口稳定等待
  - program 协议 ping
  - bootloader ping
  - 单次 `RUNCODE`
  - program 握手确认
- runtime 导入 `serial_wrap` 前设置环境变量，禁用其“导入即自动连接”副作用
- `controller_probe` 改为只调用最小恢复器，不再混入官方下载逻辑
- bootloader ping / RUNCODE ack 改为按官方 `checksum` 规则验包，只校验关键字段，不再整包匹配固定字节串
- 恢复器进一步增强为“短窗口多次拉起”：
  - bootloader 确认后允许多次 `RUNCODE`
  - 每轮后都短暂等待 program 握手
  - 增加冷却时间，避免过度敲击串口
  - 对外暴露更明确的失败阶段文案

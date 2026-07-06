# Reference Package — 百度智能车 2026 完整参考实现

> 本目录记录了 main 分支上保留的参考包，**不参与 git 跟踪**（307MB 太大），但在工作目录可用作查阅。

## 内容

| 文件/目录 | 大小 | 用途 |
|----------|------|------|
| `baidu_smartcar_2026.zip` | 307 MB | 完整 zip 原始存档（保留以便追溯） |
| `reference/baidu-2026/` | ~327 MB | zip 解压后的内容（可浏览） |
| `fishros` | 994 B | ROS 一键安装脚本（FishROS 镜像源） |

## 来源

`baidu_smartcar_2026.zip` 是从 `develop/ros2-sidecar` 分支带过来的（之前在 stash 里），由用户在 2026-07-06 移动到 main。原始来源是 **百度智能车 2026 智慧农业赛道** 的参赛代码。

## 用途

参考包**不是要被集成到 main 的代码**。它的作用是：

1. **任务模式参考** — `car_task_function.py` 含 8 个任务：
   - `auto_seeding` (播种)
   - `auto_pest_scout` (除害侦察)
   - `auto_watering` (灌溉)
   - `auto_shoot_pest` (射击除害)
   - `auto_harvest` (采收)
   - `auto_sort` (分类储存)
   - `auto_read_order` (OCR 读单)
   - `auto_deliver` (订单配送)

2. **WhalesBot SDK 参考** — `smartcar/whalesbot/` 含 WhalesBot 官方 SDK 的封装层（机械臂、底盘、传感器），可学习他们如何组织硬件抽象。

3. **比赛架构参考** — `car_wrap_2026.py` (63KB) 是他们的 MyCar 实现。比我们 main 的 `car_wrap.py` (55KB) 大但结构可能更清晰。**对比学习两个实现**能看出不同的设计权衡。

4. **配置模式参考** — `config_car.yml` 含完整比赛参数（速度、距离、PID），可对照我们的 `config_car.yml` 看参数选择。

## 怎么用

```bash
# 1. 浏览任务流程（无需执行）
less reference/baidu-2026/car_task_function.py

# 2. 对比硬件抽象
diff -u car_wrap.py reference/baidu-2026/car_wrap_2026.py | less

# 3. 查看 WhalesBot SDK 如何用 PaddlePaddle
ls reference/baidu-2026/smartcar/paddlebaidu/

# 4. 复制具体模式到 main（如果觉得有用）
cp reference/baidu-2026/smartcar/whalesbot/tools/x2coco.py tools/
```

## 重要警告

- **不要 import 参考包**：他们用 Jetson Nano + 不同的 PaddlePaddle 推理流。混用会污染 main 的依赖
- **不要 sync 反向**：参考包是「别人怎么做」，不是「我们要改成这样」。借鉴要经过团队评审
- **不要删除 .gitignore 规则**：307MB 会让 git 慢 100 倍，github 直接 reject

## 8.10 比赛后

参考包里有 8 个任务的完整流程，可以作为我们 9 月新赛季「任务库」的基础。届时可以：
- 借鉴任务编排模式
- 提取 WhalesBot SDK 通用部分
- 比较 PaddlePaddle vs ZMQ 推理性能

## 文件统计

```
8 个任务文件: car_task_function.py (20K)
1 个 MyCar:    car_wrap_2026.py (64K)
1 个启动:     car_start_2026.py (1.3K)
1 个数据收集: collect_data.py
1 个 README + 1 个 CLAUDE.md (10K)
1 个 smartcar/whalesbot SDK: 多文件
总: 649 个文件, 327 MB
```

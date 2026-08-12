# main/misc —— 杂项 mini 任务

放各种**单文件、可独立运行**的小任务/小实验。

跟 `main/arm/`、`main/chassis/` 这两个子包的区别：

| 子包 | 定位 | 风格 |
| --- | --- | --- |
| `main/arm/` | 机械臂业务子包 | 完整 client/runner/state 体系，软限位保护 |
| `main/chassis/` | 底盘控制子包 | controllers + loops + tasks 三层 |
| `main/misc/` | 杂项 mini 任务 | 一个脚本一件事，能直接 `python3 xxx.py` 跑 |

## 约定

- 每个脚本**只用 `RuntimeApiClient`**（来自 `main/api_client.py`）
- 不引入新的 client/runner/state 类，需要时在脚本里直接写
- 脚本顶部必须能 `python3 -m main.misc.<filename>` 直接跑
- 每个脚本开头用 docstring 说明：做什么、依赖什么硬件、跑前要做什么准备
- 出错就 raise，不要吞——让上层看到

## 当前收录

| 文件 | 类型 | 备注 |
| --- | --- | --- |
| [single_shot.py](./single_shot.py) | mini 任务 | 单发点射，连响 3 次确认触发 |
| [burst_shot.py](./burst_shot.py) | mini 任务 | 连发，间隔可调 |
| [drive_and_shoot.py](./drive_and_shoot.py) | mini 任务 | 边走边打：边巡线边周期性射击 |
| [diag_shoot.py](./diag_shoot.py) | 诊断 | 连发测每发电压跌落，判定子弹弱是供电还是气压/机构 |
| [test_order_read.py](./test_order_read.py) | 冒烟 | OCR 读订单（task6 在调它的 `run()`） |
| [test_veggie_detect.py](./test_veggie_detect.py) | 冒烟 | 蔬菜检测（task6 在调它的 `run()`） |
| [llm_config.yml](./llm_config.yml) | 配置 | ERNIE token / 端点（task3 LLM 判别与 test_pest_llm_shoot 用） |

## 射击硬件时序（红线，勿改）

枪走 **`PoutD(4)`** 数字口（MC602 第 4 路继电器）。runtime 暴露两个 car action（见 [API_INDEX.md §6](../API_INDEX.md)）：

| action | 行为 |
| --- | --- |
| `shooting` | 单发脉冲：拉低 50ms → 拉高 `SHOOT_RELAY_HOLD_S`（默认 0.32s，`RAK_CAR_SHOOT_RELAY_HOLD_S` 环境变量可调，clamp [0.05,1.0]）→ `finally` 拉低 → 200ms，总 ~0.6s。`try/finally` 保证异常也拉低，防继电器常吸烧枪。**默认值继承 2026-08-12 现场 0.25→0.28→0.32 逐档上调结论**；过长有过烧风险，改前先想清楚。射完会读电池电压并随 job result 返回（`{hold_s, v_idle, v_during, v_after, v_drop}`），诊断用 |
| `set_shoot_state` | 直接写电平，**无收尾逻辑**，只适合调试时常开/常闭，不要拿来连发 |

三条红线：

1. **不要 `set_shoot_state(True)` 持续触发**——没有收尾，硬件常吸。
2. **`shooting` 调用间隔 ≥500ms**——内部 sleep 被打断会乱序；连发每发之间还要留装填/补气时间（task 层实测 sleep 5s）。
3. **端口占用**：当前只有 `PoutD(4)` 接枪；以后加继电器别撞端口号。

## 子弹疲软无力怎么查（2026-08-12）

软件命令链已核实无误（`PoutD(4).set(1)` 帧 `10 02 04 01`、脉冲 ≥0.32s 可靠）。若射出的子弹**能出但弱**：

1. 先跑 `python3 -m main.misc.diag_shoot` —— 每发打印 `v_drop = v_idle - v_during`：
   - **v_drop > ~1.5V** → 供电不足：查电池带载跌落、电源线径/接头氧化、继电器触点老化、阀线圈电流。这是"传输的电流"问题，修硬件不修代码。
   - **v_drop 很小但子弹仍弱** → 供电正常，查**气罐压力**（每发耗气、后几发更弱 = 没补气/漏气）和机构（弹丸配合/管壁摩擦）。
2. 对比**第 1 发 vs 最后一发**：越射越弱且 v_drop 变大 → 供电；v_drop 不变但越射越弱 → 气压。
3. 排除供电/气压/机构后，若还想加力，试 `RAK_CAR_SHOOT_RELAY_HOLD_S=0.40 python3 -m ...` 逐档上调——但注意：电磁阀全开 <200ms，时长不是主要杠杆，别指望它救弱弹。

## 加新 mini 任务的姿势

1. 在 `main/misc/` 下新建 `xxx_yyy.py`
2. 顶部写 docstring：目的 / 硬件依赖 / 跑前准备 / 参数
3. 复用 [single_shot.py](./single_shot.py) 的 `connect()` 模式
4. 在本 README 表格里登记一行
5. 如果是**通用模式**（能被多个任务复用），考虑提到 `main/` 下，而不是留在这里

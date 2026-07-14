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
| [shooting_logic.md](./shooting_logic.md) | 笔记 | 枪射击逻辑调研结论（硬件层 + MyCar API + 任务层用法） |
| [single_shot.py](./single_shot.py) | mini 任务 | 单发点射，连响 3 次确认触发 |
| [burst_shot.py](./burst_shot.py) | mini 任务 | 连发，间隔可调 |
| [drive_and_shoot.py](./drive_and_shoot.py) | mini 任务 | 边走边打：边巡线边周期性射击 |

## 加新 mini 任务的姿势

1. 在 `main/misc/` 下新建 `xxx_yyy.py`
2. 顶部写 docstring：目的 / 硬件依赖 / 跑前准备 / 参数
3. 复用 [single_shot.py](./single_shot.py) 的 `connect()` 模式
4. 在本 README 表格里登记一行
5. 如果是**通用模式**（能被多个任务复用），考虑提到 `main/` 下，而不是留在这里

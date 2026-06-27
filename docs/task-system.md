# 任务系统与竞赛脚本

## MyCar — 中央调度器

`car_wrap.py` 中的 `MyCar` 继承自 `CarBase`，是整个系统的枢纽。

### 初始化顺序

```python
class MyCar(CarBase):
    def __init__(self):
        super().__init__()              # 1. 底盘运动学
        self.task = MyTask()            # 2. 任务原语
        self.screen = ScreenShow()      # 3. 显示屏
        self.cfg = self.get_cfg()       # 4. 加载配置

        # 5. 传感器
        self.key4btn = Key4Btn(cfg['io']['key'])
        self.led = LedLight(cfg['io']['light'])
        self.inf_left = Infrared(cfg['io']['left_sensor'])
        self.inf_right = Infrared(cfg['io']['right_sensor'])

        # 6. PID 控制器
        self.lane_pid = PidCal2(cfg['lane_pid'])
        self.det_pid = PidCal2(cfg['det_pid'])

        # 7. 摄像头
        self.cap_front = Camera(cfg['camera']['front'])
        self.cap_side = Camera(cfg['camera']['side'])

        # 8. 推理客户端
        self.lane = ClintInterface("lane")
        self.front_det = ClintInterface("front")
        self.task_det = ClintInterface("task")
        self.ocr_rec = ClintInterface("ocr")

        # 9. 按键监控线程
        Thread(target=self.key_monitor).start()
```

### MyCar 方法分类

#### 运动控制

| 方法 | 功能 |
|------|------|
| `move_base(sp, end_function, stop)` | 核心运动循环 |
| `move_advance(sp, value_h, value_l, times, sides, dis_out)` | 红外传感器触发移动 |
| `move_time(sp, dur_time)` | 按时间移动 |
| `move_distance(sp, dis)` | 按距离移动 |

#### 车道线跟随

| 方法 | 功能 |
|------|------|
| `lane_base(speed, end_function)` | 车道线 PID 跟随（核心） |
| `lane_time(speed, dur_time)` | 车道线跟随-按时间 |
| `lane_dis(speed, dis_end)` | 车道线跟随-按距离 |
| `lane_dis_offset(speed, dis_hold)` | 车道线跟随-增量距离 |
| `lane_sensor(speed, value_h, value_l, sides, stop)` | 车道线跟随-传感器触发 |
| `lane_det_base(speed, end_function)` | 检测目标跟随 |
| `lane_det_dis2pt(speed, dis_end)` | 检测跟随-接近目标 |

#### 目标定位

| 方法 | 功能 | 区别 |
|------|------|------|
| `lane_det_location_v4(speed, pt_tar, dis_out, side)` | 单目标定位 | 按距离排序 |
| `lane_det_location_vert(speed, pt_tar, dis_out, side)` | 单目标定位(垂直) | 按 x 排序 |
| `lane_det_location_v8_multi(speed, targets, dis_out, side)` | 多目标定位 | 逐个定位 |
| `lane_det_location_plant(speed, targets, dis_out, side)` | 植物定位 | 按置信度过滤 |

#### OCR

| 方法 | 功能 |
|------|------|
| `get_ocr(time_out)` | 单文字 OCR |
| `get_ocr_list(time_out)` | 多文字 OCR（本地推理） |
| `get_ocr_list_plus(time_out)` | 多文字 OCR（百度云 API） |

#### 其他

| 方法 | 功能 |
|------|------|
| `get_card_side()` | 读取左转/右转指令卡 |
| `manage(functions, count)` | 按键菜单系统 |

### 菜单系统 manage()

```python
my_car.manage([task1, task2, task3, ...], count)
```

- 按键4 = 上翻
- 按键2 = 下翻
- 按键3 = 执行
- 长按按键1 = 退出

---

## 竞赛脚本结构

### 入口文件

| 文件 | 模式 | 状态 |
|------|------|------|
| `main/qqq.py` | systemd 启动入口 | ✅ 生产用 |
| `main/main.py` | 竞赛脚本 | ✅ 最完整 |
| `car_start.py` | 菜单驱动 | ⚠️ 旧版 |
| `important_car.py` | 类式编排 | ⚠️ 旧版 |
| `main/finalall.py` | 功能最全 | ⚠️ 备选 |

### 竞赛任务清单

| 任务 | 函数名 | 描述 |
|------|--------|------|
| 汉诺塔 | `hanoi()` | 检测方向卡 → 3个位置抓放圆柱体 |
| BMI | `bmi()` | 导航到BMI站 → OCR → AI计算 → 显示 |
| 营地 | `camp()` | 红外+里程计导航 → 旋转 → 行驶 |
| 弹射 | `eject()` / `send_fun()` | 巡航到目标 → 发射 |
| 食材识别 | `get_food()` | OCR识别 → AI判断 → 取两个食材 |
| 答题 | `answer()` | 导航 → OCR读题 → AI回答 → 按按钮 |
| 放食材 | `put_food()` | AI匹配菜名 → 放到正确位置 |
| 帮人 | `old_people()` | 导航 → 挥臂 |
| 魔术 | `magic()` | 抓取 → 换面 → 放置 |
| 植物护理 | `plants()` | 检测圆柱类型 → 浇水/蜂鸣/亮灯 |
| 送药 | `medicine()` | 180°转弯 → 送药 |
| 天气 | `weather_action()` | 获取天气 → 显示 |

### 竞赛方案编排

`main/qqq.py` 定义了多个方案：

```python
push_A() = weather + camp + eject + food + answer1 + eject2 + put_food + old_people
push_B() = plant + eject + food + answer2 + put_food + medicine + old_people
push_C() = hanoi + bmi + camp + magic + eject + food + answer3 + eject2 + put_food + old_people
push_D() = hanoi + bmi + camp + magic + eject + food + answer4 + eject2 + put_food + old_people
```

`answer1~4` 的区别仅在于距离参数（0.15, 0.235, 0.32, 0.43）。

### index_form — 检测类别映射

```python
index_form = {
    0: 'cauliflower', 1: 'chili', 2: 'tofu', 3: 'tomato',
    4: 'meat', 5: 'egg', 6: 'mushroom', 7: 'turn_right',
    8: 'turn_left', 9: 'text_det', 10: 'cylinder1',
    11: 'cylinder2', 12: 'cylinder3', ...
}
```

**⚠️ 此字典在 15+ 个文件中重复定义，且 cls_id 在不同模型版本中可能不同！**

---

## 任务执行流程示例

### 食材识别任务

```
1. lane_sensor() → 车道线跟随到货架位置
2. task.get_ingredients(side, arm_set=True) → 机械臂到观测位
3. get_ocr_list_plus() → OCR 识别食材文字描述
4. answer.ask1(ocr_text) → AI 判断食材名称
5. task.pick_ingredients(num, row) → 机械臂抓取
6. 重复步骤 2-5 取第二个食材
7. 返回 [food1, food2]
```

### 答题任务

```
1. lane_sensor() → 车道线跟随到答题面板
2. task.get_answer(arm_set=True) → 机械臂到观测位
3. get_ocr_list_plus() → OCR 读取题目
4. answer.ask3(ocr_text) → AI 计算答案
5. task.get_answer() → 机械臂按下对应按钮
```

---

## 草稿/实验文件（可删除）

根目录和 main/ 下有大量迭代副本，**只保留 `main/qqq.py` 和 `main/main.py`**：

| 文件 | 状态 | 说明 |
|------|:---:|------|
| `1.py` ~ `11.py` | 🗑️ | 数字命名草稿 |
| `1 hanoi.py`, `4 eject.py` 等 | 🗑️ | 文件名含空格 |
| `bmi.py`, `food.py`, `food2.py` | 🗑️ | 单任务实验 |
| `trytest.py`, `test2.py`, `lanetext.py` | 🗑️ | 测试脚本 |
| `main/scripy.py` ~ `scripy5.py` | 🗑️ | 迭代快照（有语法错误） |
| `main/aaa.py`, `bbb.py`, `111.py` | 🗑️ | 草稿 |
| `main/finalall.py` | 🗑️ | 与 qqq.py 重复 |
| `old people.py` | 🗑️ | 文件名含空格 |

# 文件结构与文件清单

## 目录树

```
vehicle_wbt/                             # 项目根目录
├── docs/                                # 📖 项目文档（本目录）
├── config_car.yml                       # 主配置文件
├── car_wrap.py                          # MyCar 中央调度器（1438行）
├── task_func.py                         # MyTask 任务原语
├── car_start.py                         # 竞赛入口（旧版）
├── car_test.py                          # 测试脚本
├── important_car.py                     # 竞赛入口（旧版）
│
├── vehicle/                             # 🚗 车辆硬件层
│   ├── base/
│   │   ├── serial_wrap.py               # 串口通信、控制器探测
│   │   ├── mc601_ctl2.py                # MC601 协议实现
│   │   ├── mc602_ctl2.py                # MC602 协议实现
│   │   ├── controller_wrap.py           # 统一硬件抽象层
│   │   ├── pydownload.py                # MC602 固件下载
│   │   └── mc602_cfg.yaml               # MC602 校准数据
│   ├── driver/
│   │   ├── vehicle_base.py              # 底盘运动学核心
│   │   ├── world_base.py                # 世界坐标系
│   │   └── cfg_vehicle.yaml             # 底盘配置
│   ├── arm/
│   │   ├── arm_base.py                  # 机械臂控制
│   │   └── arm_cfg.yaml                 # 机械臂配置
│   └── test/
│       ├── controller_test.py           # 控制器测试
│       ├── vehicle_base.py              # ⚠️ 副本（不要使用）
│       ├── pos_set.py                   # 位置设置测试
│       └── cfg_vehicle.yaml             # ⚠️ 另一台机器人的配置
│
├── camera/                              # 📷 摄像头
│   └── base/
│       └── camera.py                    # 摄像头采集（线程化）
│
├── infer_cs/                            # 🧠 推理服务
│   └── base/
│       ├── infer_front.py               # ZMQ 客户端 ClintInterface
│       ├── infer_back_end.py            # ZMQ 服务端 InferServer
│       └── infer.yaml                   # 推理服务配置
│
├── paddle_jetson/                       # 🏷️ PaddlePaddle 模型
│   └── base/
│       ├── infer_wrap.py                # 模型封装类
│       ├── lane_model/                  # 车道线模型
│       ├── task_wbt2025/                # 任务检测模型
│       ├── front_model2/                # 前方检测模型
│       ├── ch_PP-OCRv3_det_infer/       # OCR 检测模型
│       ├── ch_PP-OCRv3_rec_infer/       # OCR 识别模型
│       ├── mot_ppyoloe_s_36e_pipeline/  # 多目标跟踪模型
│       ├── PPLCNet_x1_0_person_attribute_945_infer/ # 人体属性模型
│       └── deploy/                      # PaddleDetection 部署代码（498文件）
│
├── ernie_bot/                           # 🤖 大模型集成
│   └── base/
│       ├── ernie_bot_wrap.py            # 百度文心封装 + Prompt 类
│       ├── ernie_bot_tmp.py             # 旧版文心封装
│       ├── gpt_bot_wrap.py              # OpenAI 兼容封装（阿里/DeepSeek）
│       ├── answer.py                    # DeepSeek 竞赛接口
│       ├── answer_wenxin.py             # 文心竞赛接口
│       ├── weather_api.py               # 高德天气 API
│       └── ernie_test.py                # 测试脚本
│
├── tools/                               # 🔧 工具函数
│   └── base/
│       └── tools_class.py               # PID, CountRecord, get_yaml, limit_val
│
├── log_info/                            # 📝 日志
│   └── base/
│       ├── log_wrap.py                  # 日志封装
│       └── logs/                        # 日志文件（按日期）
│
├── collect_wrap/                        # 📊 数据采集
│   └── base/
│       ├── collect_data.py              # 数据采集
│       ├── joystick.py                  # 手柄控制
│       ├── quick_collect.py             # 快速采集
│       ├── remote_control.py            # 遥控
│       └── image_set2/                  # 采集的图片
│
├── main/                                # 🏁 竞赛入口
│   ├── qqq.py                           # ✅ systemd 启动入口（生产用）
│   ├── main.py                          # ✅ 竞赛脚本（最完整）
│   ├── finalall.py                      # 备选竞赛脚本
│   ├── boot_py.sh                       # systemd 安装脚本
│   ├── hri/                             # HRI 机器人表情
│   │   ├── main.py                      # PySide2/QML 入口
│   │   ├── RobotFace.qml                # 机器人脸 QML
│   │   ├── Eye.qml                      # 眼睛动画 QML
│   │   └── hri-autostart.service        # systemd 单元
│   └── [草稿文件，见下方可删除列表]
│
├── model/                               # 📦 其他模型
│   └── RT-DETR-R18/inference/           # RT-DETR 检测模型
│
├── __init__.py                          # 空文件
├── CLAUDE.md                            # Claude Code 指引
├── .gitignore                           # Git 忽略规则
│
│   ⚠️ 以下文件可以删除：
├── 1.py ~ 11.py                         # 🗑️ 数字命名草稿
├── 1 hanoi.py, 3camp.py, 4 eject.py    # 🗑️ 文件名含空格
├── 5 hanoi2.py, 6hanno.py               # 🗑️ 草稿
├── bmi.py, food.py, food2.py            # 🗑️ 单任务实验
├── det.py, grab.py, location.py         # 🗑️ 实验
├── lanetext.py, magic.py, ocrlist.py    # 🗑️ 实验
├── old people.py, primary_question.py   # 🗑️ 文件名含空格
├── putfood.py, task_detect.py           # 🗑️ 实验
├── test2.py, trytest.py, all.py         # 🗑️ 测试
├── car_start.ipynb                      # 🗑️ Jupyter 笔记本
├── Lane Detection_screenshot_*.png      # 🗑️ 截图
├── Overload                             # 🗑️ 空文件
├── nohup.out                            # 🗑️ 运行日志
│
└── main/ [草稿文件]
    ├── 111.py, 333.py, 5.py, 9.py      # 🗑️
    ├── aaa.py, bbb.py                   # 🗑️
    ├── scripy.py ~ scripy5.py           # 🗑️ 迭代快照
    ├── test.py, grab.py, angle.py       # 🗑️ 测试
    ├── .qqq.py.swp                      # 🗑️ Vim 交换文件
    └── nohup.out                        # 🗑️ 运行日志
```

## 统计

| 类别 | 文件数 | 行数（估计） |
|------|:---:|---:|
| 底层驱动 (vehicle/) | 9 | ~2,500 |
| 推理系统 (infer_cs/ + paddle_jetson/) | 3 | ~800 |
| 摄像头 (camera/) | 1 | ~120 |
| 大模型 (ernie_bot/) | 7 | ~1,200 |
| 工具 (tools/ + log_info/) | 2 | ~350 |
| 应用层 (car_wrap + task_func) | 2 | ~2,100 |
| 竞赛脚本 (main/) | 有效 5 个 | ~3,000 |
| 数据采集 (collect_wrap/) | 5 | ~600 |
| 草稿/垃圾 | ~35 个 | ~3,000 |
| **总计** | **~83** | **~13,700** |

其中有效代码约 8,000 行，重复/垃圾代码约 5,000 行。

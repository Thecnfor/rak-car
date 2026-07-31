"""task5 —— 分拣入库 (PPT Slide 10)。

PPT Slide 10:
  - 任务区内有高低两个存储仓; 高储存仓有颜色标签 (单色), 储存仓双色。
  - 智能车需将采集的果实模型, **依据种类放入正确的存储仓内**。
  - 赛前指定对应颜色的储存仓。
  - 得分: 果实位于对应颜色的储存仓内 → +20 分/果实。
  - 技术方案: 目标检测, 精准定位, 高位存放。

⚠️ task5 与 task4 的边界 (务必分清):
  - task4 = **车载自带存储仓** (采收 → 把果实存到车上的 PWM 舵机仓)
  - task5 = **场地存储仓** (从车载仓取出果实, 按颜色放入场地对应色仓)
  - 方向与 task4 相反: task4 是 "任务模型 → 车载仓"; task5 是 "车载仓 → 场地仓"。
  - 果实在 task4 里已按色分好装进车载仓 (蓝 x=0 / 黄 x=-65), 所以 task5
    知道哪个车载仓装哪个色, 取出时直接按色定位。

子模块 (执行顺序 a → b1→b2→b3 循环 → c):

  ┌──────────────────────────────────────────────────────────────────────┐
  │ a_approach        底盘进分拣区 + 摆臂到可取/可放姿态                  │
  │   ↓                                                                     │
  │ b1_detect_bins    识别场地仓颜色标签 (高储存仓单色/双色仓) + 定映射   │
  │   ↓                                                                     │
  │ b2_extract_fruit  从车载仓按色取果实 (move_x → 开仓 → 下探 → grasp)   │
  │   ↓                                                                     │
  │ b3_place_fruit    移到对应色场地仓 + 高位存放 + grasp(False) 放果实   │
  │   ↓                                                                     │
  │ c_finish          reset_y + 回 init + 清 task5_bin_latest.json         │
  └──────────────────────────────────────────────────────────────────────┘

跑法:
    python -m main.arm.each_task.task5.run_full                # 全流程
    python -m main.arm.each_task.task5.run_full --dry          # 只跑 a+b1+c, 不动果实
    python -m main.arm.each_task.task5.run_full --max-rounds 3 # 最多 3 个果实

约束要点 (详见 main/arm/ARM_API.md):
  - 业务层禁改 smartcar / runtime / car_wrap_2026
  - 一律走 _read_x_mm_realtime() 读 x (x_get_position 坏, §11)
  - 开仓 75° 必在 y ∈ [-205, -145] mm (§6.3 Round 15)
  - belt-slip: 单次 x 行程 24-46mm, 跨 bin 必分段 (§7.2.1)
  - 大量物理坐标 (车载仓取果实 y / 场地仓放置 x/y / 高位存放) **待现场校准**,
    见 constants.py 里标 '待现场校准' 的常量。
"""

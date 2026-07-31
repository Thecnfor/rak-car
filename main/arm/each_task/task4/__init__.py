"""task4 —— 作物采收。

PPT Slide 9:
  - 收割区内 2 色 4cm 球（蓝/黄）
  - 球放在任务模型上
  - 智能车需完成采集: 果实完全脱离任务模型且不与场地接触 → +10 分/球
  - 技术方案: 目标检测 + 精准定位 + 存储结构

边界:
  - 任务5（Slide 10，分拣入库 +20/球）才按颜色分仓;
    任务4 业务层只确保"颜色对得上 bin", 不参与分拣评分。
  - 用户决策：边采边存（采一个 → 直接放对应 bin，不在臂上累积）。

子模块（执行顺序 a → b1→b2→b3 循环 → c):

  ┌────────────────────────────────────────────────────────────────────┐
  │ a_approach        底盘进采收区 + 摆可采姿态 (y 出保护区 / hand DOWN)│
  │   ↓                                                                   │
  │ b1_detect_fruit   侧摄 task_feed 扫球 + 过滤 + 选 next target       │
  │   ↓                                                                   │
  │ b2_pick_fruit     move_x → move_y → grasp → move_y 抬回 (-190)      │
  │   ↓                                                                   │
  │ b3_store_fruit    按 color 入仓 (蓝 x=0 / 黄 x=-65, belt-slip 分段) │
  │   ↓                                                                   │
  │ c_finish          reset_y + 回 init + 清 task4_target_latest.json   │
  └────────────────────────────────────────────────────────────────────┘

跑法:
    python -m main.arm.each_task.task4.run_full                # 全流程
    python -m main.arm.each_task.task4.run_full --dry 1        # 只跑 1 个循环
    python -m main.arm.each_task.task4.run_full --max-rounds 3 # 最多 3 轮

辅助/历史文件（保留勿删）:
  - target_test.py   侧摄 task_feed 冒烟（b1 复用 fetch_balls/save_latest）⚠️ 已删,
                      fetch_balls/save_latest 逻辑合并进 target2.py
  - grasp.py         真空泵冒烟
  - test_blue.py     入仓位姿子工具 (x=0 / y=-150); 开仓见 open_storage.py
  - test_yellow.py   入仓位姿子工具 (x=-65 / y=-150) ⚠️ belt-slip; 开仓见 open_storage.py
  - open_storage.py  单职责开仓模块 (2026-07-31 简化: 不再读 y 不动 y,
                     调用方自己把 y 摆到 [-205, -145] 区间, 本脚本只下舵机)
  - target1.py       用户指定目标位姿 (y=-195→arm=+90°→hand=0°→x=-260), 含
                      belt-slip 分段兜底 ⚠️ x 超物理墙
  - target2.py       侧摄识别球类 (蓝/黄) + 返回归一化坐标; 替代旧 target_test.py
                      的 fetch_balls/save_latest; 支持 --once / --loop / --color / --save
  - x_to_zero.py     x 撞墙回 0 (跟 task5/get_blue.py 同款, 走底层 reset_x + probe_time=0.3)
  - pick_up_blue.py  抓蓝球序列 (9 步: 记+吸+抓+抬+移 bin(0)+放+释+定 y+回抓取位)
                     v6 加回抓取位 (默认 -260 = target1 抓取位)
  - pick_up_yellow.py 抓黄球序列 (跟 pick_up_blue 同 9 步, 唯一区别: 步骤 5 移 -65)
  - dipan.py         底盘单步直行 (默认 80mm, sync 阻塞到完成; 走 car.move_for)
  - target4.py       target1 起手 + 循环 7 次 (底盘前移 80mm → 识别 → 抓球),
                     2026-07-31 重写替代 v7 状态机 (用户原话 "识别到就停")

约束要点（详见 main/arm/ARM_API.md）:
  - 业务层禁改 smartcar / runtime / car_wrap_2026
  - 一律走 _read_x_mm_realtime() 读 x (x_get_position 坏, §11)
  - 开仓 75° 必在 y ∈ [-205, -145] mm (§6.3 Round 15)
  - belt-slip: 单次 x 行程 24-46mm, 跨 bin 必分段 (§7.2.1)
  - 球场几何: 左手边=放球侧, x 负方向=向左, 大臂 +90°=复位位 (2026-07-27 硬限上界),
                    手爪 -90°=init/前
"""

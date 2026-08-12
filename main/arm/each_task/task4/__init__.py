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

子模块（执行顺序 a → (b1→b2→b3→b4)×N, 预算式收尾):

  ┌────────────────────────────────────────────────────────────────────┐
  │ a_start          触发后三步并发: 四轴联动 P 姿态(已到位跳过) ∥ lane 前进 0.1m ∥ 开仓 75°│
  │   ↓                                                                   │
  │ b1_creep_search   慢速前移 (realtime 速度 0.045m/s) + 10Hz fetch_balls, 见球即停│
  │   ↓                                                                   │
  │ b2_track_chassis  track_chassis select_mode="leftmost", 最左球拉到画面中心│
  │   ↓                                                                   │
  │ b3_pick_fruit     move_to_vision_target(hand=0°, 防手爪挡侧摄) 吸嘴中心对准 → 吸│
  │   ↓                                                                   │
  │ b4_store_fruit    composite_run (抬 y=-190 ∥ 移 bin 蓝0/黄-65) → y=-155 → 放气│
  │   ↓                                                                   │
  │ 收尾              creep 预算 0.8m / picks 8 / 总时 180s 任一命中 → summary│
  └────────────────────────────────────────────────────────────────────┘

跑法:
    python -m main.arm.each_task.task4.target4                 # 真跑 (默认预算: creep 0.8m / picks 8 / 180s)
    python -m main.arm.each_task.task4.target4 --dry-run       # 只打印不动硬件
    python -m main.arm.each_task.task4.target4 --max-picks 3   # 最多抓 3 个
    python -m main.arm.each_task.task4.target4 --no-prep       # 跳过 target1 起手 (假设已在位姿)
    # orchestrator 全流程走 main/task/task4_harvest.py → step_target4 (run.py --task 4)

辅助/历史文件（保留勿删）:
  - target_test.py   侧摄 task_feed 冒烟（b1 复用 fetch_balls/save_latest）⚠️ 已删,
                      fetch_balls/save_latest 逻辑合并进 target2.py
  - grasp.py         真空泵冒烟
  - test_blue.py     入仓位姿子工具 (x=0 / y=-150); 开仓见 open_storage.py
  - test_yellow.py   入仓位姿子工具 (x=-65 / y=-150) ⚠️ belt-slip; 开仓见 open_storage.py
  - open_storage.py  单职责开仓模块 (2026-07-31 简化: 不再读 y 不动 y,
                     调用方自己把 y 摆到 [-205, -145] 区间, 本脚本只下舵机)
  - target1.py       起手势 (y=-133; composite_run arm=+90°/hand=0° 并行 → x=-260
                      hard_reach) ⚠️ x 超物理墙 (-119.5mm), 有 reset_x 撞墙兜底
  - target2.py       侧摄识别球类 (蓝/黄) + 返回归一化坐标; 支持 --once / --loop /
                      --color / --save / --debug; 2026-08-01 删 BALL_VERIFIED_*
                      位姿验证 (位姿偏移下误伤过滤)
  - x_to_zero.py     x 撞墙回 0 (跟 task5/get_blue.py 同款, 走底层 reset_x + probe_time=0.3)
  - pick_up_blue.py  抓蓝球序列 (独立调试工具; 主流程已走 target4 内置
                     pick_by_vision + composite_run); 2026-08: 步骤 4+5/8+9
                     改 composite_run 并行
  - pick_up_yellow.py 抓黄球序列 (同 pick_up_blue, 唯一区别: 步骤 5 移 -65)
  - dipan.py         底盘工具: step_chassis_forward 单步直行 (旧离散步进工具) +
                     _stop_chassis_quietly 停车兜底 (target4 finally 复用)
  - target4.py       主流程 (2026-08-03 底盘视觉伺服版): target1 起手 +
                     (creep 慢速搜索 → track_chassis 最左球定位 → pick_by_vision
                     吸嘴中心抓 → composite_run 放 bin) ×N, 预算式收尾;
                     orchestrator 入口 main/task/task4_harvest.py

约束要点（详见 main/arm/ARM_API.md）:
  - 业务层禁改 smartcar / runtime
  - 一律走 _read_x_mm_realtime() 读 x (x_get_position 坏, §11)
  - 开仓 75° 必在 y ∈ [-205, -145] mm (§6.3 Round 15)
  - belt-slip: 单次 x 行程 24-46mm, 跨 bin 必分段 (§7.2.1)
  - 球场几何: 左手边=放球侧, x 负方向=向左, 大臂 +90°=复位位 (2026-07-27 硬限上界),
                    手爪 -90°=init/前
"""

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

子模块（执行顺序 a → (b1→b2→b3)×N, 预算式收尾):

  ┌────────────────────────────────────────────────────────────────────┐
  │ a_approach        P 姿态起手: composite_run(arm=+90°/x=-295/y=-160/hand=+10°)│
  │   ↓                                                                   │
  │ b1_creep_search   慢速前移 (realtime 速度 0.12m/s) + 20Hz fetch_balls, 见球即停│
  │   ↓                                                                   │
  │ b2_arm_servo_pick 机械臂智能抓取 (track_velocity_pick): 大臂控 cx + x 十字控 cy,│
  │                   y 锁 0, 高位伺服 → 最后盲降 pick_y → 吸 (2026-08-10 替换底盘对齐)│
  │   ↓                                                                   │
  │ b3_store_fruit    composite_run (中转 x → bin x) → 降 y/hand → 放气    │
  │   ↓                                                                   │
  │ 收尾              IR 丢失+0.3m 主终止; creep 0.58m / picks / 总时兜底  │
  └────────────────────────────────────────────────────────────────────┘

跑法:
    python -m main.arm.each_task.task4.target4                 # 真跑 (默认预算, IR 丢失+0.3m 主终止)
    python -m main.arm.each_task.task4.target4 --dry-run       # 只打印不动硬件
    # orchestrator 全流程走 main/task/task4_harvest.py → step_target4 (run.py --task 4)

模块内文件:
  - target4.py       主流程 (2026-08-10 机械臂智能抓取版): P 姿态起手 +
                     (creep 慢速搜索 → 判色 → track_velocity_pick 大臂+x 轴对齐
                     吸嘴中心抓 → composite_run 放 bin) ×N, 预算式收尾;
                     orchestrator 入口 main/task/task4_harvest.py
  - target1.py       起手势调试工具
  - target2.py       侧摄识别球类 (蓝/黄) + 返回归一化坐标 (--once/--loop/--color/--save)
  - dipan.py         底盘工具: step_chassis_forward 单步直行 + _stop_chassis_quietly
                     停车兜底 (target4 finally 复用)

约束要点（详见 main/arm/ARM_API.md）:
  - 业务层禁改 smartcar / runtime
  - 一律走 _read_x_mm_realtime() 读 x (x_get_position 坏, §11)
  - 开仓 75° 必在 y ∈ [-205, -145] mm (§6.3 Round 15)
  - belt-slip: 单次 x 行程 24-46mm, 跨 bin 必分段 (§7.2.1)
  - 球场几何: 左手边=放球侧, x 负方向=向左, 大臂 +90°=复位位 (硬限 ±150°),
    手爪 init 实测舵机识别 -75° (非 -90°, 出厂标定差 15°)
  - ⚠️ 真空吸力未实测: 需现场 grasp(True)+y 微抬验证球跟上 (GRASP_HOLD 调优)
"""

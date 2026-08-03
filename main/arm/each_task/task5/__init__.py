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

实际结构 (2026-08-03 重写; 旧文档描述的 a/b1/b2/b3/c 子模块 + run_full +
constants.py + task5_bin_latest.json **从未存在**, 已删除误导):

  入口: main/task/task5_sort.py → the_final.main() 四阶段流水线:

  ┌────────────────────────────────────────────────────────────────────────┐
  │ [1/4] target.run()          摆检测位姿 + HSV 识别高仓色标 → color A   │
  │ [2/4] last_{A}_to_high      同色球进高仓 (检测球数 → 循环 test_run)   │
  │ [3/4] 底盘到 LOW 仓取位     --align-area 传了 → 视觉闭环对仓          │
  │                             (main.chassis.make_align_runner 按面积     │
  │                              前后微调); 默认 → dipan 开环后撤 165mm   │
  │ [4/4] last_{反色}_to_low    反色球进低仓                              │
  └────────────────────────────────────────────────────────────────────────┘

  last_*_to_{high,low} (4 个薄 wrapper, 共享 _last_loop 骨架):
    prep_pose (摆检测位姿) → target_{blue,yellow}.detect_balls (数球)
    → 循环 test{1..4}.run()

  test1-4 (薄 wrapper, 共享 pick_and_place 执行器):
    get_{blue,yellow} (取球位姿) → 取球 → {high,low}_tower (放仓位姿)
    → grasp(False) → reset_x 撞墙归零

  取球两模式 (2026-08-03 新增视觉闭环, pick_and_place.py 统一实现):
    - 默认开环: grasp(True) + sleep(hold_s) 盲吸 (旧行为)
    - --vision: runner.track_velocity_pick("ball_blue"/"ball_yellow") 视觉伺服
      对准球 → y 到 grasp_y_mm → 吸气; 失败自动回退开环 (vision_fallback)
      ⚠️ sign_arm/sign_x 是 task1 姿态标定值, task5 位姿首跑需现场确认方向

文件清单:
  the_final.py            四阶段总入口 (task5_sort.py 调它)
  pick_and_place.py       统一 pick→place 执行器 (开环 + 视觉闭环双模式)
  _last_loop.py           last_* 共享骨架 (prep → 检测 → 循环)
  target.py               高仓色标识别 (HSV 阈值, 自包含)
  target_blue.py / target_yellow.py   摆检测位姿 + 只检测蓝/黄球
  last_blue_to_high.py 等 4 个        检测 → 循环 test_run 的接线 wrapper
  test1-4_from_*_to_*.py              pick→place 的接线 wrapper
  get_blue.py / get_yellow.py         取球位姿 (开环摆位)
  high_tower.py / low_tower.py        放仓位姿 (高/低仓)
  dipan.py                            底盘开环直线位移 (move_for)

跑法:
    python main/arm/each_task/task5/the_final.py               # 端到端 (全开环默认)
    python main/arm/each_task/task5/the_final.py --vision      # 取球视觉闭环
    python main/arm/each_task/task5/the_final.py --align-area 0.04 --align-label water
    python main/arm/each_task/task5/the_final.py --color blue  # 手动指定高仓色
    python -m main.arm.each_task.task5.last_blue_to_high       # 单阶段
    python main/arm/each_task/task5/test1_from_yellow_to_high.py --vision  # 单轮

约束要点 (详见 main/arm/ARM_API.md):
  - 业务层禁改 smartcar / runtime / car_wrap_2026
  - 一律走 _read_x_mm_realtime() 读 x (x_get_position 坏, §11)
  - 开仓 75° 必在 y ∈ [-205, -145] mm (§6.3 Round 15)
  - belt-slip: 单次 x 行程 24-46mm, 跨 bin 必分段 (§7.2.1, common.move_x_with_split)
  - 大量物理坐标 (取球位姿 y / 放仓 x/y) **待现场校准**, 见各文件常量注释。
"""

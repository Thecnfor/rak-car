"""main/task 共享常量.

历史: 这些 dict 在 task1_seeding 重构前被复制了 4 份 (两个函数里各写一份,
       重复定义). 2026-08 重构提取到此处, 让 task1_seeding._transport_to_slot
       和主循环的 step 5 共享同一份.

来源: task_config.yml auto_seeding.spacing_along_row_m = 0.15.
"""
from __future__ import annotations

# 源位置 (S1/S2/S3) 在底盘坐标系下的绝对位置 (m), 以 S1 为原点
SOURCE_POSITIONS_M = {1: 0.0, 2: 0.15, 3: 0.30}

# 目标种植槽 (T1/T2/T3) 在底盘坐标系下的绝对位置 (m), 与源位置纵向对齐:
#   S1↔T1 同列, S2↔T2 同列, S3↔T3 同列 → 运输时底盘无需纵向移动,
#   大臂转 +90° + X 伸出即可跨放 (release/task1 物理验证值).
SLOT_POSITIONS_M = {1: 0.0, 2: 0.15, 3: 0.30}

__all__ = ["SOURCE_POSITIONS_M", "SLOT_POSITIONS_M"]
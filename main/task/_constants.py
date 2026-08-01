"""main/task 共享常量.

历史: 这些 dict 在 task1_seeding 重构前被复制了 4 份 (两个函数里各写一份,
       重复定义). 2026-08 重构提取到此处, 让 task1_seeding._transport_to_slot
       和主循环的 step 5 共享同一份.

来源: task_config.yml auto_seeding.spacing_along_row_m = 0.15.
"""
from __future__ import annotations

# 源位置 (S1/S2/S3) 在底盘坐标系下的绝对位置 (m), 以 S1 为原点
SOURCE_POSITIONS_M = {1: 0.0, 2: 0.15, 3: 0.30}

# 目标种植槽 (T1/T2/T3) 在底盘坐标系下的绝对位置 (m), 以 T2 为原点
SLOT_POSITIONS_M = {1: -0.15, 2: 0.0, 3: 0.15}

__all__ = ["SOURCE_POSITIONS_M", "SLOT_POSITIONS_M"]
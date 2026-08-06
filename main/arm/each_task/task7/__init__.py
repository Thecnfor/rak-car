"""task7 —— 产品配送
将 task6 拿到的货物投递到对应住户(配送点)。
与 task6 强绑定。

⚠️ **本 __init__.py 显式导出 task7 包内模块** (2026-08-06 修复 the_final.py ImportError):
  - 背景: the_final.py 是 task7 编排器, module 级别 (line 138-146) 就引用
    ``pingcang_mod.DEFAULT_ANGLE_DEG`` 等常量, 必须**导入时**所有子模块就绪。
  - 之前 __init__.py 是空的, 导致
    ``from main.arm.each_task.task7 import get_position1`` 报 ImportError。
  - 修法: 显式 ``from . import xxx``, 让包导入能找到子模块。
  - 注意: task7 自包含脚本 (target.py / dipan.py / aaashouzhua.py)
    **不依赖本包**, 它们走 sys.path 注入 + 直接模块导入,
    跟 task5 教训一致 (见 [[task5-rebuild-2026-07-22]])。
  - 本 __init__.py **只服务于编排器** (the_final.py): 让编排器可以
    ``from main.arm.each_task.task7 import the_final_position1 as xxx_mod`` 这种包导入语法。

⚠️ **2026-08-06 v2 改名** (用户已删除 position{1..6}.py / get_position{1,2}.py 8 个旧文件):
  - 旧 8 个文件: ``position1..6.py`` (投递脚本) + ``get_position1..2.py`` (抓取脚本)
  - 新 8 个文件: ``the_final_position1..6.py`` (投递) + ``the_final_get_position1..2.py`` (抓取)
  - 新文件命名风格统一为 ``the_final_*``, 跟 ``the_final.py`` 编排器对齐。
  - 旧位置脚本的"3 阶段 (底盘+臂+底盘)"逻辑 → 新编排器 ``the_final_position{1,3,4,6}.py`` (委托子任务)
  - 旧位置脚本的"纯臂"逻辑 → 新纯臂脚本 ``the_final_position{2,5}.py`` (单独跑/被委托)
  - 旧 get_position{1,2} 抓取脚本 → 新 ``the_final_get_position{1,2}.py`` (1:1 镜像)

⚠️ **导出范围** (按 the_final.py 实际 import 的 10 个模块, 不多导):
  - duiying, pingcang (2 个辅助)
  - the_final_get_position1, the_final_get_position2 (2 个抓取脚本, 替代旧 get_position{1,2})
  - the_final_position1, the_final_position2, the_final_position3, the_final_position4,
    the_final_position5, the_final_position6 (6 个投递/编排器, 替代旧 position{1..6})
  - 没导出的 (target / dipan / aaashouzhua) 不影响编排器, 保持 task7 自包含。
"""
from . import (  # noqa: F401  (only re-exported, not "used" in __init__.py itself)
    duiying,
    pingcang,
    the_final_get_position1,
    the_final_get_position2,
    the_final_position1,
    the_final_position2,
    the_final_position3,
    the_final_position4,
    the_final_position5,
    the_final_position6,
)
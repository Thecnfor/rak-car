"""main/arm/each_task —— 把每个比赛任务拆成"一步一步"的脚本。

每个子目录(task1, task2, ...)对应一个比赛任务,里面是该任务的:
  step_a_*.py / step_b1_*.py ...   单步动作脚本(可独立运行)
  run_one.py                       单个种子的完整循环
  run_full.py                      整个任务的完整流程

调用约定:
  每个 step 文件:
    - 暴露一个 step_xxx(client, runner, **kwargs) 函数
    - 也有 if __name__ == "__main__": main() 独立可跑
"""

# TaskStatus.msg — 行为层任务状态机心跳。
# Spec: docs/superpowers/specs/2026-08-09-ros2-layering-interfaces-design.md §5.4
#
# 行为层节点按任务心跳发到 /rak/state/task/<task_id>/status (TRANSIENT_LOCAL),
# 让 RViz/操作员/上层看到"现在在干什么、到哪了、成功没有"。

std_msgs/Header header
string task_id          # 任务实例名,如 "demo"
string state            # IDLE | RUNNING | SUCCEEDED | FAILED | ABORTED
string current_step     # 当前步骤名,如 "navigate" / "grip"
float32 progress        # 0.0 - 1.0
string message          # 人可读的说明 / 错误信息

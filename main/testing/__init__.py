"""main/testing —— 无真机验证工具包（fake runtime + 任务动作 trace）。

业务 client 复用生产 `create_runtime_client(transport="fake")`，fake runtime
把所有物理动作路由到 FakeRobotSim 运动学仿真（关节姿态真实、可观测），
任务动作包（target/action/args/kwargs/queue/job_id/phase）由 ActionRecorder 记录。

- `TaskHarness`：把 `TASK_RUNNERS[1..8]` 跑在 fake 上，带 wall-clock 期限
  + 急停兜底，返回每任务的 ok/status/reason + 动作 trace。
- 参考入口：`main/task/tests/test_task_end_to_end.py`、`main/testing/dry_run.py`。
"""

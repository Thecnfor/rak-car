"""TaskHarness：把 TASK_RUNNERS[1..8] 端到端跑在 fake runtime 上并采集动作 trace。

设计要点（与生产同构）：
- `os.environ["RAK_CAR_TRANSPORT"] = "fake"` 让所有 `create_runtime_client()` /
  `ArmClient.connect()` / `ChassisClient.connect()` 拿到同一个 fake service
  （fake 单例），harness 注入的 fixture 在任务自建的 client 上同样可见。
- 每任务独立 `reset_fake_runtime()`（隔离 odom/arm/detections/recorder）。
- 任务跑在 daemon 线程 + wall-clock deadline；超时 `emergency_stop()` 兜底
  （`_invoke` 见 stop_flag 即抛，任务的下一个 client 调用会中止），不会真挂。
- 动作 trace = recorder 按 target/action 统计 + 末帧关节姿态快照。

对外不可用真机/网络：OCR / ERNIE / 相机帧（task6 order_read_run、task7
target OCR、task3 ERNIE 判定）默认走"预期 unsupported"——由各任务测试注入
确定性 stub，断言 stub 被调用而非静默跳过。
"""
from __future__ import annotations

import os
import threading
import time
from collections import Counter
from typing import Any, Callable, Dict, List, Optional

from main.local_api_client import create_runtime_client
from main.task import TASK_RUNNERS
from runtime.services.fake_runtime import get_fake_runtime, reset_fake_runtime

# 每任务默认墙钟期限 (s)。task4 creep 单次墙钟兜底 30s，给足余量。
DEFAULT_DEADLINES_S = {
    1: 45.0, 2: 30.0, 3: 20.0, 4: 45.0, 5: 30.0, 6: 45.0, 7: 30.0, 8: 15.0,
}


class HarnessResult:
    """一次任务运行的结果 + 动作 trace。"""

    def __init__(self, *, task_id: int, done: bool, result: Any, error: Optional[str],
                 elapsed_s: float, actions: Dict[str, Counter],
                 final_arm: Dict[str, Any], final_wheels: List[float],
                 notes: Optional[List[str]] = None):
        self.task_id = task_id
        self.done = done          # False = 期限到急停兜底
        self.result = result      # 任务返回 dict 或 None
        self.error = error        # 异常 repr（任务抛错）
        self.elapsed_s = elapsed_s
        self.actions = actions    # {"car": Counter(action), "arm": Counter(action), "realtime": ...}
        self.final_arm = final_arm
        self.final_wheels = final_wheels
        self.notes = notes or []

    @property
    def ok(self) -> bool:
        if self.result is None:
            return False
        if isinstance(self.result, dict):
            return bool(self.result.get("ok"))
        return bool(self.result)

    def summary(self) -> str:
        status = "done" if self.done else "TIMEOUT"
        ok = "ok" if self.ok else "fail"
        return (f"task{self.task_id} {status} {ok} {self.elapsed_s:.1f}s "
                f"car={dict(self.actions.get('car', Counter()))} "
                f"arm={dict(self.actions.get('arm', Counter()))}")


class TaskHarness:
    """fake runtime 上的任务端到端 runner。"""

    def __init__(self, *, deadline_s: Optional[float] = None,
                 base_env_fake: bool = True):
        if base_env_fake:
            os.environ["RAK_CAR_TRANSPORT"] = "fake"
        self.deadline_s = deadline_s
        self.client = None
        self._restore = []  # (callable) tearDown 还原

    # ---------------- 生命周期 ----------------

    def setUp(self) -> None:
        reset_fake_runtime()
        self.client = create_runtime_client(transport="fake")

    def tearDown(self) -> None:
        for fn in self._restore:
            try:
                fn()
            except Exception:
                pass
        self._restore = []
        reset_fake_runtime()
        self.client = None

    # ---------------- fixture ----------------

    @property
    def service(self):
        return get_fake_runtime()

    def set_detections(self, detections: List[dict]) -> None:
        self.service.set_task_detections(detections)

    def set_ir(self, left=None, right=None) -> None:
        self.service.set_ir_distances(left=left, right=right)

    def patch(self, target: str, replacement: Callable) -> None:
        """替换模块属性，tearDown 自动还原（stdlib unittest.mock.patch）。"""
        import unittest.mock as mock
        p = mock.patch(target, replacement)
        p.start()
        self._restore.append(p.stop)

    # ---------------- 跑任务 ----------------

    def run(self, task_id: int, *, fixtures: Optional[Callable] = None,
            kwargs: Optional[dict] = None,
            deadline_s: Optional[float] = None,
            reset: bool = True) -> HarnessResult:
        """跑 `TASK_RUNNERS[task_id]`，期限兜底，返回结果 + 动作 trace。

        reset: True（默认）时每次 run 前重建 fake 单例 + client —— 多任务共用一个
        harness（如 dry_run）时各任务隔离；fixtures 在 reset 之后应用，不会被清掉。
        fixtures: 可选回调，拿到 service 引用后注入检测/IR/OCR。
        kwargs:   透传给任务 runner 的额外参数（如 task4 的 dry_run）。
        """
        if reset:
            reset_fake_runtime()
            self.client = create_runtime_client(transport="fake")
        svc = self.service
        if fixtures is not None:
            fixtures(svc)
        box: Dict[str, Any] = {}

        def worker():
            try:
                box["r"] = TASK_RUNNERS[task_id](self.client, **(kwargs or {}))
                box["done"] = True
            except Exception as exc:
                box["e"] = repr(exc)
                box["done"] = True

        t = threading.Thread(target=worker, daemon=True,
                             name=f"harness-task{task_id}")
        limit = self.deadline_s or deadline_s or DEFAULT_DEADLINES_S.get(task_id, 45.0)
        t0 = time.monotonic()
        t.start()
        t.join(limit)
        elapsed = time.monotonic() - t0
        done = bool(box.get("done"))
        if not done:
            svc.emergency_stop()
            t.join(3.0)
            done = bool(box.get("done"))

        rec = svc.recorder
        actions = {"car": Counter(e.action for e in rec.matching(target="car")),
                   "arm": Counter(e.action for e in rec.matching(target="arm")),
                   "realtime": Counter(e.action for e in rec.matching(target="realtime"))}
        final_arm = dict(svc.get_arm_state() or {})
        final_wheels = list(svc.state.get("wheels", []))
        return HarnessResult(
            task_id=task_id, done=done, result=box.get("r"),
            error=box.get("e"), elapsed_s=elapsed, actions=actions,
            final_arm=final_arm, final_wheels=final_wheels,
        )

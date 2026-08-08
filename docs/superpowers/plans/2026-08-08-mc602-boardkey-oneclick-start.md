# MC602 BoardKey One-Click Start 實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** MC602 板上鍵按下 → 立即開始完整 8 任務，按下到車子動作 ≤200ms（等待階段完成全部初始化）。

**Architecture:** runtime 只加讀鍵鉤子（`read_key` CAR_ACTION + `MyCar.read_key()`，純新增）。orchestrator 把 `_run_mission` 拆成 `_init_mission()`（等待階段建好全套機制，lane runner **不啟動不挪車**）+ `_walk_waypoints()`（跑任務）。`run.py --wait-key` 先 init → 20Hz 輪詢讀鍵（邊沿+去抖）→ 按下瞬間啟動 runner 線程 + 進 waypoint 迴圈。

**Tech Stack:** Python 3.8 (Jetson) / stdlib unittest / FastAPI runtime / PM2。

## Global Constraints

- 業務層（`main/`）不 import runtime/，只經 `RuntimeApiClient` HTTP。讀鍵必須走 `execute("car","read_key")`。
- 現有 runtime 邏輯**零改動**，只加讀鍵鉤子（純新增）。
- `run.py` 不帶 `--wait-key` 時行為完全不變。
- 註解用中文，貼齊既有風格。
- 測試用 stdlib unittest，離線（mock HTTP），命令：`/usr/bin/python3 -m unittest discover -s main/start/tests -p 'test_*.py'`。
- BoardKey raw→bool 的極性映射**只收斂在** `SensorsMixin._board_key_pressed()` 一處（真機標定點）。

---
### Task 1: Runtime 讀鍵鉤子（sensors_mixin + actions）

**Files:**
- Modify: `runtime/services/my_car/sensors_mixin.py`（`sensor_init` 加一行 + 兩個方法）
- Modify: `runtime/core/actions.py`（`CAR_ACTIONS` 加一行）
- Test: `main/start/tests/test_wait_key.py`（新建，含 runtime 鉤子測試）

**Interfaces:**
- Produces: `MyCar.read_key() -> bool`；`CAR_ACTIONS["read_key"]` → `car.read_key()`；`SensorsMixin._board_key_pressed(raw) -> bool`（module 級純函數，供測試直接 import）。

- [ ] **Step 1: 寫失敗測試** — 建 `main/start/tests/test_wait_key.py`：

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""MC602 板上鍵一鍵啟動 — 讀鍵鉤子 + 邊沿偵測 離線單測（無硬體）。"""
import os
import unittest

# sensors_mixin 會 import smartcar（serial_wrap 預設自動連串口），測試前關掉
os.environ.setdefault("RAK_CAR_SERIAL_AUTO_CONNECT", "0")


class TestBoardKeyPressed(unittest.TestCase):
    """raw(2 bytes) → bool 映射。預設任一 byte 非零 = 按下（真機標定點）。"""

    @classmethod
    def setUpClass(cls):
        from runtime.services.my_car.sensors_mixin import _board_key_pressed
        cls._f = _board_key_pressed

    def test_released(self):
        self.assertFalse(self._f((0, 0)))

    def test_any_pressed(self):
        self.assertTrue(self._f((1, 0)))
        self.assertTrue(self._f((0, 1)))
        self.assertTrue(self._f((255, 255)))

    def test_scalar_input(self):
        self.assertFalse(self._f(0))
        self.assertTrue(self._f(1))


class TestReadKeyActionRegistered(unittest.TestCase):
    def test_car_action_registered(self):
        from runtime.core.actions import CAR_ACTIONS
        self.assertIn("read_key", CAR_ACTIONS)
        car = type("FakeCar", (), {"read_key": lambda self: True})()
        self.assertTrue(CAR_ACTIONS["read_key"](car))
```

- [ ] **Step 2: 跑測試確認失敗**（`ImportError: cannot import name '_board_key_pressed'`）：

```bash
/usr/bin/python3 -m unittest main.start.tests.test_wait_key -v
```

- [ ] **Step 3: 實作** — `runtime/services/my_car/sensors_mixin.py`：

```python
def _board_key_pressed(raw):
    """BoardKey raw → bool「是否按下」。真機標定點：極性/哪個 byte 只改這裡。

    raw 來自 BoardKey_2.no_act()（"bbb" 格式去掉首 byte 後的 2 ints tuple）。
    預設：任一 byte 非零 = 按下（實際極性待真機確認，確認後改這一行即可）。
    """
    if raw is None:
        return False
    vals = raw if isinstance(raw, (tuple, list)) else [raw]
    return any(int(v) != 0 for v in vals)
```

`sensor_init` 末尾加 `self.key = BoardKey()`（`from smartcar.whalesbot.vehicle import ...` 那行 import 加 `BoardKey`），並加方法：

```python
    def read_key(self) -> bool:
        """讀 MC602 板上鍵，回傳是否按下（異常視為未按下，讓等待迴圈重試）。"""
        try:
            return _board_key_pressed(self.key.read())
        except Exception:
            return False
```

`runtime/core/actions.py` 的 `CAR_ACTIONS` 加一行：
```python
    "read_key": lambda car, *args, **kwargs: car.read_key(),
```

- [ ] **Step 4: 跑測試確認通過**：
```bash
/usr/bin/python3 -m unittest main.start.tests.test_wait_key -v
```

- [ ] **Step 5: 提交**
```bash
git add runtime/services/my_car/sensors_mixin.py runtime/core/actions.py main/start/tests/test_wait_key.py
git commit -m "feat(runtime): read_key CAR_ACTION + MyCar.read_key() (BoardKey 讀鍵鉤子)
Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---
### Task 2: Orchestrator 拆分 + wait_key_then_run

**Files:**
- Modify: `main/start/orchestrator.py`（`_run_mission` 拆 `_init_mission` + `_walk_waypoints`；+`PressDetector`、`_wait_board_key`、`wait_key_then_run`）
- Test: `main/start/tests/test_wait_key.py`（追加）

**Interfaces:**
- Consumes: `execute("car","read_key",sync=True)` → job dict，`job["status"]=="succeeded"`、`job["result"]` bool。
- Produces: `Orchestrator.wait_key_then_run()`（無參數）；`_init_mission(start_lane: bool=True) -> Dict[str,Any]`；`_walk_waypoints(state: Dict, waypoints: List[Waypoint]) -> List[str]`；`PressDetector.feed(pressed: bool) -> bool`。

- [ ] **Step 1: 寫失敗測試**（追加到 `main/start/tests/test_wait_key.py`）— 純邏輯 `PressDetector` 邊沿/去抖：

```python
from main.start.orchestrator import PressDetector


class TestPressDetector(unittest.TestCase):
    """邊沿偵測 + 去抖：釋放→按下連續 confirm_samples 次才觸發；開機按住不誤觸發。"""

    def test_no_fire_when_never_pressed(self):
        d = PressDetector(confirm_samples=2)
        for _ in range(10):
            self.assertFalse(d.feed(False))

    def test_fire_on_press_edge(self):
        d = PressDetector(confirm_samples=2)
        d.feed(False)          # 穩定釋放
        d.feed(False)
        self.assertFalse(d.feed(True))    # 第 1 個按下樣本
        self.assertTrue(d.feed(True))     # 連續第 2 個 → 觸發

    def test_debounce_short_glitch(self):
        d = PressDetector(confirm_samples=2)
        d.feed(False)
        d.feed(False)
        d.feed(True)           # 一次雜訊
        d.feed(False)          # 又釋放 → 不觸發
        self.assertFalse(d.feed(True))    # 重新按下，streak 重計
        self.assertTrue(d.feed(True))

    def test_held_at_boot_does_not_fire(self):
        d = PressDetector(confirm_samples=2)
        # 開機時按鍵已被壓住：首採樣 True，之後持續 True → 永不觸發
        self.assertFalse(d.feed(True))
        for _ in range(5):
            self.assertFalse(d.feed(True))
        # 釋放後再按才觸發
        d.feed(False)
        d.feed(False)
        d.feed(True)
        self.assertTrue(d.feed(True))
```

- [ ] **Step 2: 跑測試確認失敗**（`ImportError: cannot import name 'PressDetector'`）：
```bash
/usr/bin/python3 -m unittest main.start.tests.test_wait_key -v
```

- [ ] **Step 3: 實作 `PressDetector`**（放 `main/start/orchestrator.py`，`DEFAULT_WAYPOINTS` 之後、`class Orchestrator` 之前）：

```python
class PressDetector:
    """按鍵邊沿偵測 + 去抖（純邏輯，無 IO，可離線單測）。

    feed(pressed) 每採樣呼叫一次；回傳 True 表示「確認了一次按下事件」。
    - 邊沿：僅「釋放→按下」後開始累計，開機時按鍵被壓住不會誤觸發（首採樣不算）。
    - 去抖：連續 confirm_samples 個按下樣本才判定觸發。
    - 觸發後等待釋放才重新武裝（armed）。
    """

    def __init__(self, confirm_samples: int = 2) -> None:
        self.confirm = max(1, int(confirm_samples))
        self.prev: Optional[bool] = None
        self.streak = 0

    def feed(self, pressed: bool) -> bool:
        if self.prev is None:            # 開機首採樣：只記錄，不觸發
            self.prev = bool(pressed)
            return False
        rising = (not self.prev) and bool(pressed)
        self.streak = self.streak + 1 if pressed else 0
        if rising:
            self.streak = 1
        self.prev = bool(pressed)
        return rising and self.streak >= self.confirm
```

- [ ] **Step 4: 實作拆分 `_init_mission` + `_walk_waypoints`**（純搬移，行為不變）：

把現 `_run_mission`（`orchestrator.py:187`）的 193–287 行搬進 `_init_mission(start_lane=True)`，回傳 state dict；288–446 行搬進 `_walk_waypoints(state, waypoints)`。關鍵差別：

```python
    def _init_mission(self, start_lane: bool = True) -> Dict[str, Any]:
        """建好整套任務機制並回傳 state。start_lane=False 時 lane runner 線程不啟動
        （等待階段車子不許動），由 wait_key_then_run 在按下瞬間啟動。"""
        # ...（現有 193-286 行原樣搬入，含 wait_until_ready / reset_position /
        #     start_lane_feed / ir_feed 等待 / runner 構建 / odom / TUI / display 線程）...
        runner_thread = threading.Thread(
            target=runner.run, kwargs={"max_seconds": math.inf},
            daemon=True, name="lane",
        )
        if start_lane:
            runner_thread.start()
        return {
            "client": client, "api": api, "runner": runner,
            "runner_thread": runner_thread,
            "dis_buf": dis_buf, "dis_epoch": dis_epoch,
            "tui_buf": tui_buf, "tui_running": tui_running,
            "display_ui": display_ui, "display_running": display_running,
            "post_task1": post_task1, "post_task6": post_task6,
        }

    def _run_mission(self, waypoints: List[Waypoint]) -> List[str]:
        """全流程：初始化 → 依序巡線 + 任務。由 run() / run_single_task() 共用。"""
        state = self._init_mission(start_lane=True)
        return self._walk_waypoints(state, waypoints)
```

`_walk_waypoints(state, waypoints)`：現 288–446 行原樣搬入，把 `runner`/`api`/`runner_thread`/`dis_buf`/`dis_epoch`/`tui_buf`/`tui_running`/`display_ui`/`display_running`/`post_task1`/`post_task6` 換成 `state["..."]` 取值；`finally` 清理照舊（`runner_thread.join` 對未啟動線程安全返回）。`_ball_counts` 繼續用 `self._ball_counts`（跨次 mission 保留）。

- [ ] **Step 5: 實作 `_wait_board_key` + `wait_key_then_run`**（`class Orchestrator` 內，`_run_mission` 之後）：

```python
    @staticmethod
    def _read_key_pressed(client) -> Optional[bool]:
        """讀一次下位機按鍵。回傳 True/False；job 失敗（控制器掉線）回傳 None。"""
        try:
            job = client.execute("car", "read_key", sync=True, timeout=0.5)
        except Exception:
            return None
        if not isinstance(job, dict) or job.get("status") != "succeeded":
            return None
        return bool(job.get("result"))

    def _wait_board_key(self, client, tui_buf) -> None:
        """等待 MC602 板上鍵按下（20Hz 輪詢 + 邊沿/去抖）。等待期間屏幕顯示 READY。"""
        det = PressDetector(confirm_samples=2)
        error_streak = 0
        tui_buf[0] = {"wp": "PRESS BOARD KEY", "dis": 0.0,
                      "ir_left": None, "ir_right": None, "state": "READY"}
        while True:
            pressed = self._read_key_pressed(client)
            if pressed is None:
                error_streak += 1
                tui_buf[0] = {"wp": "CTRL ERR", "dis": 0.0,
                              "ir_left": None, "ir_right": None, "state": "ERR"}
                time.sleep(min(1.0, 0.1 * error_streak))   # 退避重試
                continue
            error_streak = 0
            tui_buf[0] = {"wp": "PRESS BOARD KEY", "dis": 0.0,
                          "ir_left": None, "ir_right": None, "state": "READY"}
            if det.feed(pressed):
                logger.info("board key pressed → mission start")
                return
            time.sleep(0.05)   # 20Hz

    def wait_key_then_run(self) -> None:
        """--wait-key 模式：預先初始化（不挪車）→ 等板上鍵 → 立即開跑完整任務。

        比賽計時從按下開始：按下瞬間只做「啟動 lane runner 線程 + 進 waypoint 迴圈」，
        beep 非阻塞不擋第一步挪車。任務完成後回到 READY 可再按重跑。
        """
        state = self._init_mission(start_lane=False)
        while True:
            self._wait_board_key(state["client"], state["tui_buf"])
            # 按下 → 立刻開始：啟動 runner 線程（首幀輪速 ~20-40ms 內下發）
            state["runner_thread"].start()
            try:
                from main.api_client import RuntimeApiClient
                threading.Thread(target=self._beep_async,
                                 args=(state["client"],), daemon=True).start()
            except Exception:
                pass
            try:
                self._walk_waypoints(state, self.waypoints)
            except KeyboardInterrupt:
                logger.info("interrupted, back to READY")
            except Exception as exc:
                logger.exception("mission failed: %s", exc)
            # 完成/失敗 → 重建機制（_walk_waypoints 已清理 runner/線程），回 READY
            state = self._init_mission(start_lane=False)
```

`_beep_async`（同 class，`_run_mission` 附近新增，不擋主流程）：
```python
    @staticmethod
    def _beep_async(client, times: int = 1) -> None:
        """非阻塞蜂鳴：按下確認 beep×1 / 完成 beep×3，跑在背景線程。"""
        try:
            for _ in range(times):
                client.execute("car", "beep", sync=True, timeout=2.0)
                time.sleep(0.3)
        except Exception:
            pass
```

- [ ] **Step 6: 跑全部離線測試確認無回歸 + 新增通過**：
```bash
/usr/bin/python3 -m unittest discover -s main/start/tests -p 'test_*.py' -v
/usr/bin/python3 -m unittest discover -s main/task/tests -p 'test_*.py'
/usr/bin/python3 -m unittest discover -s main/arm/tests -p 'test_*.py'
```

- [ ] **Step 7: 提交**
```bash
git add main/start/orchestrator.py main/start/tests/test_wait_key.py
git commit -m "feat(oneclick): orchestrator 拆分 _init_mission/_walk_waypoints + wait_key_then_run
按下即開始：等待階段全套初始化，按瞬間啟動 runner 線程；PressDetector 邊沿/去抖
Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---
### Task 3: run.py `--wait-key` flag

**Files:**
- Modify: `run.py`

**Interfaces:**
- Consumes: `Orchestrator.wait_key_then_run()`。

- [ ] **Step 1: 實作**（`run.py` 加 flag + 分支）：

```python
    p.add_argument(
        "--wait-key", action="store_true",
        help="按 MC602 板上鍵開始全流程：先完成全部初始化（不挪車），按下瞬間立即開跑",
    )
    args = p.parse_args()
    ...
    orch = Orchestrator(lane_hz=args.lane_hz, ir_interval_s=args.ir_interval_s)
    if args.wait_key:
        orch.wait_key_then_run()
    elif args.task is not None:
        orch.run_single_task(args.task)
    else:
        orch.run()
```

- [ ] **Step 2: 冒煙驗證**（`--help` 顯示新 flag；`--task 1` 仍走單任務）：
```bash
/usr/bin/python3 run.py --help | grep -A1 wait-key
```

- [ ] **Step 3: 提交**
```bash
git add run.py
git commit -m "feat(run): --wait-key flag → wait_key_then_run
Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---
### Task 4: PM2 上電自啟

**Files:**
- Modify: `ecosystem.config.js`

**Interfaces:**
- Consumes: `run.py --wait-key`。產生 `rak-car-oneclick` app。

- [ ] **Step 1: 實作**（`ecosystem.config.js` 加第二個 app，同款 cwd/interpreter/env；`main/settings.py` 走 `RAK_CAR_SERVER_ORIGIN`，同機用 `127.0.0.1` 免寫死 LAN IP）：

```js
    {
      name: "rak-car-oneclick",
      cwd: "/home/jetson/workspace/rak-car",
      script: "run.py",
      args: ["--wait-key"],
      interpreter: "/usr/bin/python3",
      exec_mode: "fork",
      instances: 1,
      autorestart: true,
      watch: false,
      max_restarts: 10,
      restart_delay: 3000,
      kill_timeout: 5000,
      env: {
        PYTHONUNBUFFERED: "1",
        /* run.py --wait-key 是業務層，經 HTTP 連 runtime；同機用 localhost */
        RAK_CAR_SERVER_ORIGIN: "http://127.0.0.1",
        RAK_CAR_API_PORT: "5050",
      },
      max_memory_restart: "1200M",
    },
```

- [ ] **Step 2: 語法驗證**
```bash
node -e "const c=require('./ecosystem.config.js'); console.log(c.apps.map(a=>a.name).join(','))"
```

- [ ] **Step 3: 提交**
```bash
git add ecosystem.config.js
git commit -m "feat(pm2): rak-car-oneclick 上電自啟 (run.py --wait-key)
Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---
### Task 5: 真機標定與驗收

- [ ] **Step 1: 標定 BoardKey 極性**：用現有 `board_key_test` 或臨時腳本讀 `BoardKey_2().no_act()` raw 值，確認「按下」時的 bytes；若與 `_board_key_pressed` 預設（任一非零）不符，只改 `sensors_mixin._board_key_pressed` 一行。
- [ ] **Step 2: 真機 checklist**：
  1. 上電 → PM2 兩 app 起來 → 屏幕 `ST:READY WP:PRESS BOARD KEY`。
  2. 按板上鍵 → beep + 車立即動作（按下→移動 <0.5s）。
  3. 完整 8 任務 → 蜂鳴×3 → 回 READY。
  4. 開機按住按鍵 → 不誤觸發。
  5. 拔掉控制器 USB → 屏幕 `CTRL ERR`，插回後恢復 READY（不崩）。

# MC602 板上鍵一鍵啟動（BoardKey One-Click Start）設計

> 日期：2026-08-08
> 場景：比賽現場不帶筆電，Jetson 上電 → PM2 自動拉起 runtime + `run.py --wait-key` → 按 MC602 板上鍵 → 立即開始完整 8 任務。
> 硬性約束：**按下按鍵即開始計時，從按下到車子開始動作必須最快**——所有能提前做的初始化都必須在等待階段完成。

---

## 1. 背景與現狀

- 指南 `mc602_一鍵啟動指南.md` 描述的舊單體架構（`MyCar.manage()` 選單、`car_auto_run.py`）已隨 `c23c871` 刪除；`manage()` 在倉庫中不存在。
- 新架構：Jetson 上跑 runtime（PM2 常駐，獨佔 `MyCar()` 單例與串口），`run.py` → `main.start.orchestrator.Orchestrator` 是任務入口（50Hz 巡線外環 + 20Hz 里程計 + waypoint 迴圈）。
- **現有 runtime 沒有讀 MC602 板上鍵的任何路徑**（無 `read_key` action、無 realtime 按鍵端點、`MyCar` 無 `key` 屬性）。現有 `/keypress` 只是網頁鍵盤轉發。
- 業務層（`main/`）禁止直接碰硬體（CLAUDE.md），串口由 runtime 獨佔 → 讀鍵**必須**經 runtime，最少要加一個讀鍵鉤子（純新增，不改現有邏輯）。
- 燒錄：不進自動流程。runtime 連接時若控制器不在 program 模式會自動 `download_bin`（`serial_wrap.py:744`），賽前需手動重燒時用 `runtime/hardware/controller_download.py::download_mc602_program(isrun=True)`。

## 2. 目標 / 非目標

**目標**
1. MC602 板上鍵（BoardKey）按下 → 自動開始完整 8 任務。
2. 從按下到車子開始動作：**延遲最小化**（按鍵偵測 ≤100ms，按下後到第一步挪車 µs 級；全套初始化已在等待階段完成）。
3. Jetson 上電自啟：PM2 自動拉起 `run.py --wait-key`。
4. 現有 runtime 邏輯零改動，只加讀鍵鉤子（純新增）。

**非目標**
- 不做選單 / 多按鍵 / 單任務選擇（比賽就是「一鍵全流程」）。
- 不做顯式燒錄步驟（靠 runtime 自動路徑 + 手動指令）。
- 不改任何任務業務邏輯（task1~7 原樣）。
- 不用 `key_feed` 守護線程 / realtime 按鍵端點（低頻按鍵，輪詢足夠）。

## 3. 元件設計

### 3.1 Runtime — 讀鍵鉤子（純新增，2 處）

**`runtime/services/my_car/sensors_mixin.py`**
- `sensor_init(cfg)` 加一行：`self.key = BoardKey()`（`smartcar/whalesbot/vehicle/base/controller_wrap.py:138` 現成 wrapper，`vehicle/__init__.py` 已導出）。
- 加方法 `read_key() -> bool`：
  - `raw = self.key.read()`（回傳 2 bytes tuple）。
  - 標準化 → bool「是否按下」。**byte→bool 的映射收斂到私有函數 `_board_key_pressed(raw)`**，真機標定（極性 / 哪個 byte）只改這一處。
  - 異常 → `False`（讀不到當成未按下，讓等待迴圈重試而非崩）。

**`runtime/core/actions.py`**
- `CAR_ACTIONS` 加一行：`"read_key": lambda car, *args, **kwargs: car.read_key()`。
- 走現有 `car_queue` 執行（等待階段車隊空閒，無搶佔；`execute` 已支援 `sync=True`）。

> 現有 runtime 邏輯（init / feeds / jobs / 併發模型 / auto-init）**完全不動**。

### 3.2 `main/start/orchestrator.py` — 拆分 `_init_mission` + `_walk_waypoints`

現有 `_run_mission`（`orchestrator.py:187`）是「初始化 + waypoint 迴圈」的單一函數。為讓 `--wait-key` 在等待階段完成初始化、按下瞬間直跑，拆成兩段（行為不變，純搬移）：

- **`_init_mission()` → 回傳 `state`**：現在 `_run_mission` 的 193–287 行
  - `wait_until_ready(runtime)`、`reset_position`（里程計清零，等待期間車不動所以按下時仍≈0）
  - `start_lane_feed(hz)`、等 `ir_feed` active（最長 5s——**放在等待階段**，不佔比賽時間）
  - 建 `DoubleLoopRunner`（PAUSE 起步）、里程計線程、TUI 線程、`Mc602Display` 線程
  - lane 模型由 `lane_feed` 常駐 50Hz 保持熱載
  - 回傳包含 runner / api / threads / buffers / display_ui 的 state 物件
- **`_walk_waypoints(state)`**：現在 288–446 行（`completed` 迴圈 + try/finally 清理）。`_run_mission` 變成 `state = _init_mission(); return _walk_waypoints(state)`（向後相容）。

### 3.3 `run.py --wait-key` → `Orchestrator.wait_key_then_run()`

```
Jetson 上電 → PM2: run.py --wait-key
  ├─ Phase 0   state = _init_mission()          # 全套初始化，runner 保持 PAUSE 不挪車
  ├─ Phase 1   Mc602Display: "RAK-CAR READY"
  │            └─ PRESS BOARD KEY"
  ├─ Phase 2   輪詢 read_key（20Hz，interval 0.05s，sync=True）
  │            ├─ 邊沿偵測：prev=False 且 pressed=True 才觸發（防開機按鍵被壓住誤觸發）
  │            ├─ 去抖：連續 2 個樣本為按下才確認（≈100ms，物理按鍵量級，體感無感）
  │            ├─ 控制器掉線 / runtime 異常 → 屏幕 "CTRL ERR" + 退避重試，不崩
  │            └─ 觸發偵測延遲 ≤100ms（2 樣本 × 50ms 去抖）」
  ├─ Phase 3   按下確認 → 立刻：
  │            ├─ _resume_lane(runner)          # unpause，µs 級
  │            ├─ 非阻塞 beep（背景線程，不擋第一步挪車）
  │            └─ _walk_waypoints(state)        # 進入 waypoint 迴圈，車開始動作
  └─ Phase 4   完成 → 蜂鳴×3 → 回 Phase 1（可再按重跑）
```

**快速啟動保證**：
- 初始化（含 5s IR feed 等待、模型加載）全部在按下前完成。
- 按下 → 只剩 unpause + 進迴圈（µs）＋ beep（非阻塞）。
- 按鍵偵測 20Hz + 2 樣本去抖 → 最壞 100ms 偵測延遲（物理按鍵按下量級 ≫ 100ms，體感無感）。

**邊沿偵測細節**（`_wait_board_key`）：
- 維護 `prev_pressed`；`pressed and not prev_pressed` 為一次按下。
- 去抖：同一按下狀態連續 2 個樣本（2×50ms=100ms）才判定觸發；釋放後回到偵測態。
- 開機時若按鍵被壓住（初值 `pressed=True`），`prev` 從 `True` 起算 → 不會誤觸發。

### 3.4 `ecosystem.config.js` — PM2 上電自啟

- 加第二個 app `rak-car-oneclick`：`cwd` 同現有，`script` 指向 `run.py`，`args: ["--wait-key"]`，`interpreter` 用 `/usr/bin/python3`，`autorestart: true`，`max_memory_restart` 同現有。
- 啟動順序靠 `wait_key_then_run` 的 `wait_until_ready` 重試兜底（runtime 先起來）。

## 4. 真機標定項目（不確定性收斂）

`BoardKey_2.no_act()` 回傳 2 bytes（`dev_id=0x0d, format="bbb"` 去掉首 byte）。哪個 byte / 什麼值代表「按下」需真機確認：
- 用 `main/test/` 的離線冒煙腳本（或 `board_key_test` 方式）先讀 raw 值。
- 把極性/映射寫死在 `_board_key_pressed(raw)` 一處，**不散落在流程**。

## 5. 測試

**離線單測**（stdlib unittest，mock HTTP/WS，無硬體）
- `read_key` action 註冊存在（`runtime/core/actions.py` 有 `read_key`）。
- `_board_key_pressed` 映射（極性假設可測）。
- `_wait_board_key` 邊沿/去抖：未按下不觸發；釋放→按下觸發；開機按住不誤觸發；掉線重試不崩。
- `_run_mission` 拆 `_init_mission`+`_walk_waypoints` 後行為不變（沿用現有 `test_post_task1_maneuver.py` 等）。

**真機 checklist**
1. 上電 → 屏幕 "RAK-CAR READY / PRESS BOARD KEY"。
2. 按板上鍵 → beep + 車立即開始動作（停止計時：按下→移動 <0.5s）。
3. 完整 8 任務跑完 → 蜂鳴×3 → 回 READY。
4. 再按一次 → 重跑。
5. 開機按住按鍵 → 不誤觸發。

## 6. 文件清單

| 文件 | 改動 |
|---|---|
| `runtime/services/my_car/sensors_mixin.py` | +`self.key = BoardKey()`、+`read_key()`、+`_board_key_pressed()` |
| `runtime/core/actions.py` | +`CAR_ACTIONS["read_key"]` |
| `main/start/orchestrator.py` | `_run_mission` 拆 `_init_mission`+`_walk_waypoints`；+`wait_key_then_run()`、+`_wait_board_key()` |
| `run.py` | +`--wait-key` flag → `orch.wait_key_then_run()` |
| `ecosystem.config.js` | +`rak-car-oneclick` app |
| `main/start/tests/`（新）或 `main/task/tests/` | 離線單測 |

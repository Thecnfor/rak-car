# rak-car MC602 ROS2 Protocol + Song Playback — Design Spec

**Date:** 2026-07-08
**Branch:** `robot-stable`
**Status:** Approved (pending user review of written spec)
**Author:** Claude (brainstorming session)

---

## 1. Problem statement

The `mc602_peripheral_node.py` in `vehicle_wbt_smartcar_bridge` (commit
`a00fee0` and earlier `067cc1a`) reimplements the MC602 USB frame protocol
from scratch using `ctypes` direct calls to libc (termios / fcntl / os.read).
This layer:

- duplicates ~150 lines of protocol code already proven to work in the
  official Baidu SmartCar 2026 SDK (`smartcar/whalesbot/vehicle/base/{serial_wrap,mc602_ctl2}.py`),
- uses raw libc instead of `pyserial`, missing battle-tested error paths
  (USB unplug handling, baud rate negotiation, write retries),
- is fragile — one byte-order or `dev_id` typo silently produces wrong
  hardware commands.

The user has verified, via a standalone script importing the official SDK
classes, that the canonical protocol (`pyserial` + `_DevCmdInterface`
subclasses `Buzzer_2`, `ServoPwm`, `PoutD`) successfully drives the MC602
controller on this Jetson. The current rak-car ROS2 path bypasses that
proven protocol entirely.

Additionally, the `smartcar_bridge.launch.py` does **not** start the
`mc602_peripheral_node`, so even though commit `a00fee0` adds the Happy
Birthday melody code, there is no single `ros2 launch` invocation that
results in audible playback.

## 2. Goal

End-to-end: a single `ros2 launch` brings up the smartcar bridge and the
MC602 peripheral node. Triggering a single ROS2 topic message results in
~30 seconds of Happy Birthday played through the MC602 buzzer — using the
**proven pyserial + SDK-class protocol**, not the ctypes reimplementation.

## 3. Non-goals

- Not touching `vehicle_wbt_smartcar_sdk` (dev-box-side ROS2 client SDK).
- Not changing chassis / arm protocol paths (they're not ctypes-reimplemented).
- Not adding a song-selection service — one melody is sufficient.
- Not changing inference / ERNIE / vision pipelines.
- Not modifying baud rate beyond the SDK default 1,000,000.

## 4. Architecture

### 4.1 New package: `vehicle_wbt_smartcar_hw`

A new ament_python ROS2 package living at
`rak-car/ros2_ws/src/vehicle_wbt_smartcar_hw/` containing the hardware
protocol layer. Single Python module `mc602.py` mirroring the official SDK
class surface 1:1.

```
vehicle_wbt_smartcar_hw/
├── package.xml              # format 3, depends on pyserial (system)
├── setup.py                 # ament_python, install_requires=['pyserial']
├── resource/vehicle_wbt_smartcar_hw
└── vehicle_wbt_smartcar_hw/
    ├── __init__.py          # re-export MC602Serial, Buzzer_2, ServoPwm, PoutD
    └── mc602.py             # protocol implementation (~120 lines)
```

### 4.2 Class surface in `mc602.py`

Mirrors `baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/mc602_ctl2.py`
exactly:

| Class | SDK source | Frame format | Notes |
|-------|------------|--------------|-------|
| `MC602Serial` | (new, replaces `SerialWrap` for our use) | — | pyserial wrapper; open / close / write_frame |
| `Buzzer_2` | mc602_ctl2.py | `BBB` (freq/2, dur*20, 0) | `rings(freq_hz, dur_sec)` |
| `ServoPwm` | mc602_ctl2.py | `bbBB` (angle_lo, angle_hi, pad, pad) | `set_angle(angle_deg)`, encodes 0..180° → 0..9000 |
| `PoutD` | mc602_ctl2.py | `bbb` (state, 0, 0) | `set(0\|1)` |

All inherit `_DevCmdInterface` for `struct.pack` + frame assembly.

### 4.3 Frame format (unchanged from SDK)

```
[77 68] [len] [dev_id mode port_id args...] [0A]
```

- Header: `0x77 0x68`
- Length: `len(payload) + 4`
- Payload: `<dev_id:u8> <mode:u8> <port_id:u8> <args...>`
- Tail: `0x0A`
- Modes: `1=get, 2=set, 3=reset`
- Device IDs: `0x0a=beep, 0x05=servo_pwm, 0x10=dout`

### 4.4 Modified: `mc602_peripheral_node.py`

Becomes a thin ROS2 wrapper. Removes:
- `import ctypes, fcntl, termios, errno, struct`
- Hand-rolled `MC602Serial` (ctypes-based)
- `_build_mc602_frame` / `build_beep_frame` / `build_servo_frame` / `build_dout_frame` / `build_ping_frame`
- `MC602_BEEP / MC602_SERVO_PWM / MC602_DOUT / MC602_HEADER / MC602_TAIL` constants

Adds:
- `from vehicle_wbt_smartcar_hw.mc602 import MC602Serial, Buzzer_2, ServoPwm, PoutD`

`_on_beep` rewrite uses `self._buzzer.rings(freq, dur)` per note, where
`self._buzzer = Buzzer_2(self.serial)` is cached in `__init__` (created
once, reused across all 25 notes — no per-note object churn).
`HAPPY_BIRTHDAY_MELODY` constant + rclpy-timer orchestration remain
identical to current behavior — verified in commit `a00fee0`.

### 4.5 Modified: `smartcar_bridge.launch.py`

Adds a second `Node` entry launching `mc602_peripheral_node` alongside
`smartcar_bridge_node`. Adds two `DeclareLaunchArgument` lines for
`serial_port` (default `/dev/ttyUSB0`) and `baud` (default `1000000`)
matching `full_system.launch.py` convention.

### 4.6 Data flow

```
$ ros2 topic pub --once /vehicle_wbt/v1/cmd/peripheral/beep_event std_msgs/Empty
                              │
                              ▼
              mc602_peripheral_node._on_beep()
                              │
                              │ for note in HAPPY_BIRTHDAY_MELODY:
                              │   self._buzzer.rings(freq, dur)
                              │     └─→ _DevCmdInterface.send_cmd
                              │           └─→ MC602Serial.write_frame
                              ▼
                 pyserial → /dev/ttyUSB0 @ 1 Mbaud → MC602 → buzzer
```

(Trigger via the built-in `ros2 topic pub` CLI; no custom Python script
needed. The CLI command is documented in `CLAUDE.md` §CLI quick reference.)

## 5. File-level change summary

| File | Change | Lines (approx) |
|------|--------|----------------|
| `vehicle_wbt_smartcar_hw/package.xml` | NEW | ~25 |
| `vehicle_wbt_smartcar_hw/setup.py` | NEW | ~25 |
| `vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/__init__.py` | NEW | ~10 |
| `vehicle_wbt_smartcar_hw/vehicle_wbt_smartcar_hw/mc602.py` | NEW | ~120 |
| `vehicle_wbt_smartcar_bridge/vehicle_wbt_smartcar_bridge/mc602_peripheral_node.py` | REWRITE | -120 +30 = ~230 (was ~370) |
| `vehicle_wbt_smartcar_bridge/package.xml` | EDIT (add hw depend) | +5 |
| `vehicle_wbt_smartcar_bridge/setup.py` | EDIT (hw entry) | +5 |
| `vehicle_wbt_smartcar_bridge/launch/smartcar_bridge.launch.py` | EDIT (add Node + DeclareLaunchArgument) | +20 |
| `CLAUDE.md` | EDIT (CLI quick reference) | +5 |

## 6. Implementation plan (high-level)

1. Create `vehicle_wbt_smartcar_hw` package skeleton (package.xml,
   setup.py, resource/).
2. Implement `mc602.py` with `MC602Serial` + `_DevCmdInterface` + 3 device
   classes. Verify against the official SDK line-by-line.
3. Rewrite `mc602_peripheral_node.py` to import from `vehicle_wbt_smartcar_hw`.
   Preserve `HAPPY_BIRTHDAY_MELODY` and the rclpy timer orchestration logic
   from commit `a00fee0`.
4. Update `vehicle_wbt_smartcar_bridge/{package.xml,setup.py}` to depend on
   the new hw package.
5. Add `mc602_peripheral_node` Node + launch args to
   `smartcar_bridge.launch.py`.
6. `colcon build --packages-up-to vehicle_wbt_smartcar_bridge`.
7. Live test: source install/setup.bash → launch → trigger via CLI → listen.
8. Update `CLAUDE.md` CLI quick reference.

## 7. End-to-end verification

```bash
cd ~/workspace/rak-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_wbt_smartcar_bridge
source install/setup.bash

# Terminal A — launch
ros2 launch vehicle_wbt_smartcar_bridge smartcar_bridge.launch.py \
    serial_port:=/dev/ttyUSB0 baud:=1000000

# Terminal B — trigger
ros2 topic pub --once /vehicle_wbt/v1/cmd/peripheral/beep_event std_msgs/Empty
```

**Pass criteria:**
- [ ] `colcon build` returns 0 with no warnings about missing deps
- [ ] `ros2 node list` shows both `smartcar_bridge_node` and `mc602_peripheral_node`
- [ ] Terminal A logs `MC602 link open: /dev/ttyUSB0 @ 1000000 baud`
- [ ] Trigger produces log line `playing Happy Birthday (25 notes, ~30s)`
- [ ] Buzzer audibly plays for ~30 seconds

If `/dev/ttyUSB0` is unavailable on this Jetson, override with the actual
port (e.g. `/dev/ttyUSB1` per earlier diagnostic) via the launch arg.

## 8. Risks & mitigations

| Risk | Mitigation |
|------|------------|
| `/dev/ttyUSB0` permission denied | launch-arg override; udev rules already on Jetson |
| pyserial not installed | verify via `python3 -c "import serial"` before build; install if missing |
| `Buzzer_2.rings` returns the readback bytes that may stall if MC602 echoes | use SDK's pattern: ignore readback after write |
| USB unplug mid-melody | MC602Serial catches `SerialException`, logs warn, returns False; timer continues but logs failures |
| `colcon build` sees new package as missing dep | build with `--packages-up-to` so transitive deps resolve |

## 9. Reference

- Working protocol verification script:
  `/home/xrak/workspace/scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base/`
  - `serial_wrap.py` (line 208-220): `MC602.send_cmd`
  - `mc602_ctl2.py` (line 53-88): `_DevCmdInterface` base + device classes
- Bridge launch context: `rak-car/ros2_ws/src/vehicle_wbt_smartcar_bridge/launch/smartcar_bridge.launch.py`
- Last known working melody: commit `a00fee0` ("feat(mc602): beep plays Happy Birthday melody (25 notes, ~30s)")
- Project conventions: `rak-car/CLAUDE.md` (No mocks, ENABLE_ROS2 gate, no hardcoded paths)
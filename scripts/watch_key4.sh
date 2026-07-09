#!/usr/bin/env bash
# watch_key4.sh — 监听 Key4Btn 4 键按键板(P1),显示哪个键被按(1/2/3/4)
#
# 用法:
#   ./scripts/watch_key4.sh [duration_sec]   # default 30s
#
# SDK KeyMap (从 SDK ctl602_dev_list 校准):
#   355   -> key 3
#   1366  -> key 1
#   2137  -> key 2
#   2988  -> key 4
# no-press baseline: ~3975 (open circuit / pull-up)
#
# 你按 Key4Btn 上的键,会看到:
#   >>> KEY 2 PRESSED (ADC 2023) <<<
# 松开会看到:
#   >>> KEY 2 RELEASED <<<
#
# 退出 Ctrl+C。

set -eo pipefail

DURATION="${1:-30}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$REPO_DIR"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ros2_ws/install/setup.bash 2>/dev/null || true

python3 << EOF
import sys, time, importlib.util, os
BASE = '$REPO_DIR/../scratch/baidu_smartcar_2026/smartcar/whalesbot/vehicle/base'
import numpy as np
sys.modules['numpy'] = np
import types
logger_mod = types.ModuleType('logger_mod')
class _L:
    def info(self,*a,**k): pass
    def error(self,*a,**k): pass
    def critical(self,*a,**k): pass
logger_mod.logger = _L()
sys.modules['smartcar.whalesbot.tools'] = logger_mod
sys.modules['smartcar.whalesbot.tools.logger'] = logger_mod.logger
def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod
load_file('smartcar.whalesbot.vehicle.base.pydownload', os.path.join(BASE, 'pydownload.py'))
sw_mod = load_file('smartcar.whalesbot.vehicle.base.serial_wrap', os.path.join(BASE, 'serial_wrap.py'))
mc_mod = load_file('smartcar.whalesbot.vehicle.base.mc602_ctl2', os.path.join(BASE, 'mc602_ctl2.py'))

sw = sw_mod.SerialWrap()
key4 = mc_mod.Key4Btn_2(port_id=1)
KEY_MAP = {3: 355, 1: 1366, 2: 2137, 4: 2988}
THRESHOLD = 0.10  # 10% deviation

def closest_key(adc):
    if adc is None:
        return 0
    try:
        adc = int(adc)
    except (TypeError, ValueError):
        return 0
    if abs(adc) < 100:
        return 0
    best_k, best_d = 0, 1.0
    for k, mid in KEY_MAP.items():
        try:
            d = abs(mid - adc) / mid
            if d < THRESHOLD and d < best_d:
                best_k, best_d = k, d
        except ZeroDivisionError:
            pass
    return best_k

print(f'connected: {sw.port} @ {sw.baudrate}')
print(f'Key4Btn on P1, monitoring $DURATION sec')
print(f'KEY_MAP: {KEY_MAP}')
print()
print('>>> PRESS keys 1/2/3/4 on P1 Key4Btn board NOW <<<')
print()

prev_key = 0
t0 = time.time()
poll_count = 0
while time.time() - t0 < $DURATION:
    raw = key4.no_act()
    poll_count += 1
    cur = closest_key(raw)
    if cur != prev_key:
        t = time.time() - t0
        if cur > 0:
            print(f'  [t=+{t:5.1f}s] >>> KEY {cur} PRESSED (ADC {raw}) <<<', flush=True)
        else:
            print(f'  [t=+{t:5.1f}s] >>> KEY {prev_key} RELEASED <<<', flush=True)
        prev_key = cur
    time.sleep(0.05)

print()
print(f'=== done: {poll_count} polls in {$DURATION}s ===')
sw.close()
EOF

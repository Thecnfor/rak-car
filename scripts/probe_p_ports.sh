#!/usr/bin/env bash
# probe_p_ports.sh — 扫描 P1-P8 + on-board button,看哪个有按键响应
#
# 用法:
#   ./scripts/probe_p_ports.sh [duration_sec_per_port]   # default 5s
#
# 每个 P 端口扫描 5 秒,打印原始 ADC。你按 Key4Btn 板上的键,会看到:
#   - ADC 跳变(从 3974 跳到 355/1366/2137/2988)
#   - 终端打印 ">>> KEY PRESS DETECTED <<<"
# 5 秒后切到下一个 P 口。
# on-board button (BoardKey_2) 也测试 (没有 port)。
#
# 退出 Ctrl+C。

set -eo pipefail

DURATION="${1:-5}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$REPO_DIR"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ros2_ws/install/setup.bash 2>/dev/null || true

# 直接调 SDK script (绕开我们的 MC602 wrapper 的 state corruption 问题)
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
print(f'connected: {sw.port} @ {sw.baudrate}')
print(f'probe duration: $DURATION sec per port')
print()

# Test 1: on-board button (BoardKey_2, no port)
print('=== Test 1: on-board button (BoardKey_2, no port) ===')
print(f'>>> PRESS the on-board button on MC602 NOW (within $DURATION sec) <<<')
key = mc_mod.BoardKey_2()
baseline = key.no_act()
print(f'  baseline: {baseline}')
prev = baseline
t0 = time.time()
detected = 0
while time.time() - t0 < $DURATION:
    cur = key.no_act()
    if cur != prev:
        # Detect press: value (last elem) changes
        if isinstance(cur, (list, tuple)) and isinstance(prev, (list, tuple)) and len(cur) >= 2 and len(prev) >= 2:
            if cur[1] != prev[1] and cur[1] > 0:
                print(f'  >>> KEY PRESS DETECTED on on-board button: prev={prev} cur={cur} <<<')
                detected += 1
        prev = cur
    time.sleep(0.05)
print(f'  on-board button: {detected} press events detected')
print()

# Test 2-9: P1 through P8
for port in range(1, 9):
    print(f'=== Test {port+1}: P port {port} (AnalogInput_2) ===')
    print(f'>>> PRESS Key4Btn key (or other P{port} sensor) within $DURATION sec <<<')
    ai = mc_mod.AnalogInput_2(port_id=port)
    baseline = ai.no_act()
    print(f'  baseline: {baseline}')
    prev = baseline
    t0 = time.time()
    detected = 0
    while time.time() - t0 < $DURATION:
        cur = ai.no_act()
        if cur != prev:
            print(f'  >>> P{port} ADC CHANGED: {prev} -> {cur} <<<')
            detected += 1
            prev = cur
        time.sleep(0.05)
    print(f'  P{port}: {detected} changes detected')
    print()

sw.close()
print('=== probe done ===')
EOF

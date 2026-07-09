#!/usr/bin/env bash
# probe_all.sh — 一次性综合测所有已知设备
#
# 用法:
#   ./scripts/probe_all.sh                 # 默认每设备 3 秒
#   ./scripts/probe_all.sh 5              # 每设备 5 秒
#
# 跑完会打印:
#   - M1 麦轮 (forward 2s, back 2s, stop) — 实际看到轮子转
#   - Buzzer (3 短鸣)
#   - Battery voltage
#   - Key4Btn 4 键 (按 1/2/3/4 看 ADC 跳变)
#   - IR A1/A2 (5 read,wave 手看 ADC 变化)
#   - Bus 舵机 (4 角度,看动)
#   - PWM 舵机 (4 角度,看动)
#   - Encoder 4 路 (M1 有值,其他 0)
#   - on-board button (手动按)
#
# 全部用 SDK-direct (绕开我们 MC602 wrapper 的 state pollution 问题)。

set -eo pipefail

DURATION="${1:-3}"
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
print(f'=== Phase 1 综合硬件 probe (每设备 $DURATION 秒) ===')
print(f'connected: {sw.port} @ {sw.baudrate}')
print(f'按提示操作:看到 \"PRESS X / WAVE / WATCH\" 时做对应动作')
print()

# 1. Battery (1 read, 1s)
print('[1] Battery')
b = mc_mod.Battry_2()
v = b.read()
print(f'  voltage: {v} V' if v is not None else '  FAIL')
print()

# 2. Buzzer (3 short beeps)
print('[2] Buzzer (3 short beeps)')
buz = mc_mod.Buzzer_2()
for i in range(3):
    r = buz.rings(262, 0.15)
    print(f'  beep {i+1}: {r!r}')
    time.sleep(0.3)
print()

# 3. M1 motor (forward 2s + back 2s + stop)
print('[3] M1 motor (forward 2s, back 2s, stop)')
m1 = mc_mod.Motor_2(port_id=1)
m1.set_speed(50)
print('  forward 50% 2s — WATCH WHEEL SPIN')
time.sleep(2.0)
m1.set_speed(-50)
print('  back 50% 2s — WATCH WHEEL REVERSE')
time.sleep(2.0)
m1.set_speed(0)
print('  stopped')
print()

# 4. Encoders (3 reads, after motor moved)
print('[4] Encoders (M1-M4 after motor run)')
enc4 = mc_mod.EncoderMotors4_2()
e = enc4.get()
print(f'  enc: {e} (M1 should be > 0, others 0)')
print()

# 5. Key4Btn on P1 (manual press test)
print(f'[5] Key4Btn on P1 (PRESS keys 1/2/3/4 within $DURATION sec)')
k = mc_mod.Key4Btn_2(port_id=1)
KEY_MAP = {3: 355, 1: 1366, 2: 2137, 4: 2988}
t0 = time.time()
prev = -1
while time.time() - t0 < $DURATION:
    raw = k.no_act()
    if raw is not None and abs(raw - prev) > 50:
        # Find closest key
        closest = 0; best_d = 1.0
        for kid, mid in KEY_MAP.items():
            try:
                d = abs(mid - raw) / mid
                if d < 0.1 and d < best_d:
                    closest = kid; best_d = d
            except: pass
        print(f'  >>> KEY {closest} PRESSED (ADC {raw}) <<<')
        prev = raw
    time.sleep(0.05)
print()

# 6. on-board button (manual press)
print(f'[6] on-board button (PRESS the button on MC602 within $DURATION sec)')
ob = mc_mod.BoardKey_2()
t0 = time.time()
prev = None
while time.time() - t0 < $DURATION:
    v = ob.no_act()
    if v != prev:
        print(f'  >>> button state: {prev} -> {v} <<<')
        prev = v
    time.sleep(0.05)
print()

# 7. IR A1, A2 (5 reads, wave hand)
print(f'[7] IR A1, A2 (WAVE HAND in front within $DURATION sec)')
for p in [1, 2]:
    ir = mc_mod.Sensor_Analog2_2(port_id=p)
    print(f'  A{p}:')
    t0 = time.time()
    prev = -1
    while time.time() - t0 < $DURATION:
        v = ir.read()
        if v is not None and abs(v - prev) > 30:
            print(f'    ADC changed: {prev} -> {v} (closer to 0 = closer obstacle)')
            prev = v
        time.sleep(0.05)
print()

# 8. PWM servo on P1 (4 angles)
print(f'[8] PWM servo on P1 (WATCH FOR MOVEMENT)')
pwm = mc_mod.ServoPwm_2(port_id=1)
for a in [0, 90, 180, 90]:
    r = pwm.set_angle(angle=a, speed=100)
    print(f'  set_angle({a:3d}, 100) = {r!r} (WATCH!)')
    time.sleep(0.6)
print()

# 9. Smart bus servo on bus 1 (4 angles)
print(f'[9] Smart bus servo on bus 1 (WATCH FOR MOVEMENT)')
bus = mc_mod.ServoBus_2(port_id=1)
for a in [0, 90, 180, 90]:
    r = bus.set_angle(angle=a, speed=80)
    print(f'  set_angle({a:3d}, 80) = {r!r} (WATCH!)')
    time.sleep(0.6)
print()

# 10. Stepper on M1 (test PWM)
print('[10] Stepper on M1 (PWM frequency 1000 Hz)')
st = mc_mod.Stepper_2(port_id=1)
r = st.set_pwm(1000)
print(f'  set_pwm(1000) = {r!r}')
time.sleep(0.3)
r = st.set_pwm(0)
print(f'  set_pwm(0) = {r!r}')
print()

# 11. PoutD (digital out)
print('[11] PoutD on port 1 (toggle high/low)')
pout = mc_mod.PoutD_2(port_id=1)
for v in [1, 0, 1, 0]:
    r = pout.set(v)
    print(f'  set({v}) = {r!r}')
    time.sleep(0.3)
print()

sw.close()
print('=== probe done ===')
EOF

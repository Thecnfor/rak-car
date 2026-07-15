#!/usr/bin/python3
"""test_x_axis.py
x 轴单轴测试:依次走到指定 mm,读回实际位置,核对误差。
要求:
  1) runtime 已起来(pm2 status rak-car-api)
  2) 首次上电已跑过 python3 main/arm/examples/01_calibrate_origin.py left
运行:
  export RAK_CAR_SERVER_ORIGIN=http://192.168.3.60
  python3 main/arm/examples/test_x_axis.py
"""
import os
import sys

# 把项目根目录(rak-car/)加到 sys.path,这样才能 import main.*
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# 测试点序列(mm):在 soft_x_min=5 .. soft_x_max=300 软限位内任选
TARGETS_MM = [0.0, 100.0, 0.0, 200.0, 0.0]
TOL_MM = 5.0   # 允许误差 mm(车端 PID 闭环,典型稳态误差 < 2mm)


def main() -> int:
    client = ArmClient.connect()
    if not client.ping():
        raise SystemExit("runtime 不在线,先 pm2 restart rak-car-api")

    runner = ArmRunner(client)

    st0 = client.get_state()
    print(f"start: x={st0.x_mm:.1f}mm  y={st0.y_mm:.1f}mm")
    if not st0.y_origin_valid or not st0.x_origin_valid:
        print("⚠️  坐标系未标定,先跑 01_calibrate_origin.py")
    print()

    fails = 0
    for tx in TARGETS_MM:
        y_before = client.get_state().y_mm
        runner.move_x(x_mm=tx)                       # ★ 单轴移动
        st = client.get_state()
        dx_err = st.x_mm - tx
        dy_drift = st.y_mm - y_before
        ok = abs(dx_err) < TOL_MM and abs(dy_drift) < TOL_MM
        flag = "OK  " if ok else "FAIL"
        print(f"[{flag}] cmd x={tx:6.1f}mm  actual x={st.x_mm:6.1f}mm  "
              f"err={dx_err:+5.1f}mm  y_drift={dy_drift:+5.1f}mm")
        if not ok:
            fails += 1

    print()
    total = len(TARGETS_MM)
    print(f"{'PASS' if fails == 0 else 'FAIL'}: {total - fails}/{total} ok")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

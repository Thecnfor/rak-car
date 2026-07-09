# Phase 1 硬件冒烟测试报告 (2026-07-09)

## 静态冒烟(不需 /dev/ttyUSB*)

✅ **全部通过**:
- colcon build: 5 packages OK (vehicle_wbt_platform_cpp, vehicle_wbt_platform, vehicle_wbt_smartcar_hw, vehicle_wbt_smartcar_msgs, vehicle_wbt_smartcar_bridge) in 38.3s
- 19 个 hw names import OK (MC602 + serial_mc602 + 14 device classes + DevCmdInterface + DevListWrap + MecanumChassis + Odometry + ArmController)
- 13 个 msg types import OK (12 新 .srv + 1 新 RawState.msg, all coexist with legacy 19 .srv + 3 .msg)
- mc602_node 12 service handlers + 3 timer callbacks import OK
- smartcar_bridge_node (legacy) still imports OK — regression pass
- vehicle_wbt_smartcar_sdk.MyCar still imports OK — legacy SDK regression pass
- mc602_node script in `install/.../lib/<pkg>/`
- mc602.launch.py in `install/.../share/<pkg>/launch/`, `ros2 launch --show-args` lists 4 args
- smartcar_bridge.launch.py ALSO in install — additive coexistence confirmed
- 2 launch files can run in parallel (different node names, no conflict)

## 动态冒烟(需要 /dev/ttyUSB*)

❌ **DEFER**: 无 /dev/ttyUSB* 接入此 dev box(无 MC602 物理连接)。需要同事:
- Jetson 上插 CH340 USB 串口模块连接到 MC602
- 然后跑:
  ```bash
  # Jetson 端
  sudo systemctl daemon-reload
  sudo systemctl start vehicle-wbt-mc602
  source /opt/ros/humble/setup.bash
  source ~/workspace/rak-car/ros2_ws/install/setup.bash
  ros2 service list | grep /vehicle_wbt/v1/mc602
  # 应看到 12 个 service

  # 然后逐个测
  ros2 service call /vehicle_wbt/v1/mc602/buzzer vehicle_wbt_smartcar_msgs/srv/Buzzer "{freq_hz: 440, duration_ms: 200}"
  # → 听到 0.2 秒 440Hz 蜂鸣
  ros2 service call /vehicle_wbt/v1/mc602/read_battery vehicle_wbt_smartcar_msgs/srv/ReadBattery {}
  # → voltage_v: 11.x 伏
  ros2 service call /vehicle_wbt/v1/mc602/set_wheels vehicle_wbt_smartcar_msgs/srv/SetWheels "{v0: 30, v1: 30, v2: 30, v3: 30}"
  # → 4 轮慢转
  ```

## dev box 端 quick-verify(需要 Jetson systemd 已启)

```bash
cd ~/workspace/rak-car
./scripts/check_link.sh  # 5 步诊断
./scripts/quick_beep.sh  # 听到蜂鸣 → 信任建立
```

## 已知非阻塞

- vehicle_wbt_smartcar_bridge package 的 `__init__.py` 强依赖 `smartcar_bridge_node`,后者需要 `vehicle_wbt_platform_cpp` build。`colcon build --packages-up-to vehicle_wbt_smartcar_bridge` 解决。
- 老的 `vehicle_wbt_platform_cpp` build 期间有 stderr 输出(ros2 run 提示性 warning,非 error)。不影响功能。
- udev 规则 ID_PATH 模式 `*usb-3*` / `*usb-4*` 是 placeholder;真硬件上需 `udevadm info /dev/videoX` 验证后调整。

## 结论

**Phase 1 foundation ready**:同事可以在 dev box 上 `git clone` + colcon build msgs + `quick_beep.sh`,前提是 Jetson systemd unit 已 enable。**真硬件 e2e test 待同事接好 USB 串口后做**。

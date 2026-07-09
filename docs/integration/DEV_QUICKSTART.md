# 同事开发上手 5 分钟

你是上层同事(底盘/机械臂/枪/LLM),只需要:

## 0. 1 行验证:Jetson 在 LAN 上响应吗?

**这一步是 trust signal**——如果你听到 Jetson 蜂鸣器响,后续步骤都能 work。如果没响,跑下面的 `check_link.sh` 诊断。

```bash
cd ~/workspace/rak-car
./scripts/quick_beep.sh
# 听到 0.2 秒 440Hz 蜂鸣 → ✅ 信任建立,可以进入第 1 步
```

如果 quick_beep.sh 失败:

```bash
./scripts/check_link.sh
# 5 步诊断:ros2 CLI / ROS_DOMAIN_ID / CycloneDDS / LAN / DDS service discovery
# 找出哪一步挂了
```

## 1. 在你的开发机上拉 repo + colcon build

```bash
cd ~/workspace/rak-car/ros2_ws
git pull
colcon build --packages-up-to vehicle_wbt_smartcar_msgs  # 只需要 msgs,不需要 hw/bridge
source install/setup.bash
```

## 2. 设置环境

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 建议加到 ~/.bashrc 让每次新 shell 都生效
```

## 3. 创建你自己的 package

```bash
cd ~/workspace/rak-car/ros2_ws/src
ros2 pkg create --build-type ament_python vehicle_wbt_smartcar_<你的> \
    --dependencies vehicle_wbt_smartcar_msgs rclpy
```

## 4. 写你的节点,只调 `/vehicle_wbt/v1/mc602/*` service

API 文档:`docs/integration/LOWLEVEL_API.md`

## 5. 在 dev 机器上启动你的节点

```bash
cd ~/workspace/rak-car/ros2_ws
colcon build --packages-select vehicle_wbt_smartcar_<你的>
source install/setup.bash
ros2 run vehicle_wbt_smartcar_<你的> <节点名>
```

你的节点会通过 LAN DDS 自动发现 Jetson 端的 `/vehicle_wbt/v1/mc602/*`,**不需要 SSH Jetson**。

## 6. 调试技巧

```bash
# 看 Jetson 端发的数据流
ros2 topic echo /vehicle_wbt/v1/mc602/state/raw

# 看 Jetson 端有什么 service/topic
ros2 service list | grep mc602
ros2 topic list | grep mc602

# 看 Jetson 节点日志(从你的开发机)
ros2 node info /mc602_io
```

## 7. 常见问题

**Q: ros2 service list 看不到 /vehicle_wbt/v1/mc602/* 怎么办?**
A: 跑 `./scripts/check_link.sh`,看哪一步失败。

**Q: 我能不能 SSH Jetson 直接看?**
A: 紧急 debug 可以,但**不要部署代码到 Jetson**。Jetson 端 hw/bridge 是我独占维护,你的代码在 dev box 跑,通过 LAN DDS 自动连。

**Q: Jetson 上 `mc602_io` 挂了怎么重启?**
A: `ssh jetson@192.168.3.69 sudo systemctl restart vehicle-wbt-mc602`(需 sudo 权限,我来开)

**Q: 我的同事在 LAN 上同时开发,会冲突吗?**
A: 不会。`mc602_io` 内部 SDK 单例 + lock 保证串口帧不交错。但**应用层应节流**(50Hz 调一次足够)。
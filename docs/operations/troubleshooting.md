# Troubleshooting — RViz 连不上 Jetson 怎么办

> **第一件事**：跑 `bash scripts/diagnose.sh`，把输出贴团队群。

## 症状速查

| 症状 | 根因 | 一键命令 | 修法 |
|------|------|----------|------|
| `bash scripts/start_team_rviz.sh` 启 RViz 但 GUI 没图像 | TF / topic / QoS 问题 | `bash scripts/diagnose.sh` | 看下面 5 个常见原因 |
| `ping orin` 不通 | 网络 / Jetson 没电 | `ping -c 3 192.168.3.69` | 检查网线 / Jetson 电源 / 内网 |
| `ros2 topic list` 看不到 Jetson topic | DDS discovery 没通 | `cat ~/.ros/cyclonedds.xml` | 拷贝 config + 防火墙 UDP 7400-7500 |
| 看到 topic 但 RViz "no image" | 字段名错 / QoS 不匹配 | 看下面 § "RViz 'no image' 怎么办" | 改 `Topic:` + 改 `RELIABLE` |
| `ros2 node list` 空但 `topic list` 有 topics | daemon cache stale | `ros2 daemon stop && ros2 daemon start` | 重新跑 `diagnose.sh` |

## RViz "no image" 怎么办（最常见）

按顺序检查：

### 1. .rviz 字段名错
```bash
grep "Image Topic\|Topic:" team_cameras.rviz
# ❌ 错:  Image Topic: /vehicle_wbt/.../image_raw
# ✅ 对:  Topic: /vehicle_wbt/.../image_raw
```

`_RosTopicDisplay` 的 property 名字是 `Topic`（不是 `Image Topic`）。改完重启 RViz2。

### 2. 用了 image_raw（带宽太重）
640x480 bgr8 @ 30Hz = **27 MB/s per camera**。两个 cameras = 54 MB/s，RViz2 + Qt 会卡。

**改用 image_compressed**（轻 30×）：

```bash
sed -i 's|image_raw|image_compressed|g; s|Transport Hint: raw|Transport Hint: compressed|g' \
  ros2_ws/src/vehicle_wbt_platform_cpp/config/team_cameras.rviz
```

### 3. QoS 不匹配
Jetson `camera_node.cpp` 的 `image_qos()` 必须是 `RELIABLE`（不是 `BEST_EFFORT`）—— 否则 RViz2 默认 RELIABLE subscriber 收不到。

```cpp
// camera_node.cpp
inline rclcpp::QoS image_qos() { return rclcpp::QoS(1).reliable(); }  // ✅
// ❌ 不要写: .best_effort()
```

修完在 Jetson 端 `colcon build` + 重启 `camera_node`。

### 4. CycloneDDS config 没部署
```bash
test -f ~/.ros/cyclonedds.xml || \
  cp ros2_ws/src/vehicle_wbt_platform_cpp/config/cyclonedds.xml ~/.ros/
```

### 5. ROS_DOMAIN_ID 不一致
```bash
echo $ROS_DOMAIN_ID   # 应该是 42
# Jetson 端和 dev 端必须都是 42
ssh orin "source /opt/ros/humble/setup.bash && printenv ROS_DOMAIN_ID"
```

## 怎么 report bug

把以下贴团队群：

```bash
# 1. 完整 diagnose 输出
bash scripts/diagnose.sh > /tmp/diag.txt 2>&1

# 2. Jetson 端 launch log
ssh orin "tail -50 /tmp/team_view*.log 2>/dev/null"

# 3. 本机 dev 端 RViz2 log（如有）
tail -50 /tmp/rviz_*.log 2>/dev/null
```

## 相关文档

- [`scripts/diagnose.sh`](../../scripts/diagnose.sh) — 自动 15 项检查
- [lan-rviz-camera.md](../development/lan-rviz-camera.md) — 详细 DDS / RViz 排查

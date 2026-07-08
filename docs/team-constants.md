# 团队硬约定 (Team Constants)

> **不要改这些值。** 它们是项目级硬约定，全队统一。
> 改一个，所有 dev 机 + Jetson + 文档都要同步，**得不偿失**。

| 名称 | 值 | 改它的后果 |
|------|----|------|
| **Jetson IP** | **`192.168.3.69`** | 改 IP 需要全队改 `~/.ssh/config` + 防火墙规则 + 文档 |
| **Jetson 用户** | `xrak` | 改 username 需要全队改 `ssh-copy-id` |
| **团队子网** | `192.168.3.0/24` | 改子网要重做所有防火墙规则 |
| **dev 端可用 IP 段** | `192.168.3.50 ~ 192.168.3.200` | DHCP 分配避开 Jetson 静态 IP |
| **ROS_DOMAIN_ID** | **`42`** | 改这个值 dev 端 / Jetson 端 / `~/.bashrc` 全部要改 |
| **CycloneDDS config** | `ros2_ws/src/vehicle_wbt_platform_cpp/config/cyclonedds.xml` | 全队 `~/.ros/cyclonedds.xml` 来源 |

## Jetson 网络配置（硬编码）

Jetson Orin Nano 4GB 上：
- **静态 IP**：`192.168.3.69/24`（**团队约定**，**所有连接必须走 IP**，不允许用 hostname / mDNS）
- **Wi-Fi** vs **有线**：当前用 Wi-Fi (`wlP1p1s0`)，比赛建议**切换到有线**（更稳定的多播）

## dev 端网络配置

新成员 dev 机连上团队 Wi-Fi/路由器后：
1. **自动获取 IP**（DHCP），通常落在 `192.168.3.50 ~ 192.168.3.200`
2. **ping Jetson 验证**：
   ```bash
   ping -c 3 192.168.3.69
   ```
3. **SSH 全部用 IP**：`ssh xrak@192.168.3.69`

## 团队开发工作流（dev 机日常）

> 这不是建议 — 是团队的**默认**开发模式。LAN + 固定 IP 之上，大家这么干活。

### 1. 连上团队网络
- Wi-Fi 或有线接入团队路由器
- 验证：`ping -c 3 192.168.3.69`（3 个包通即可）
- SSH 用 IP：`ssh xrak@192.168.3.69`

### 2. 现场发现 Jetson 在发什么（不假设）
- **不要假设** Jetson 端的话题/类型/频率/QoS
- 一键看相机 + 列所有话题：
  ```bash
  bash scripts/start_team_rviz.sh
  ```
- 纯 CLI（headless / 脚本里）：
  ```bash
  ros2 topic list
  ros2 topic info <topic> --verbose   # 消息类型 + QoS
  ros2 topic hz <topic>               # 实际发布频率
  ros2 topic echo <topic> --once      # 看一次消息内容
  ```
- `config_sensors.yml` 是话题的"权威清单"，但**实时状态**（是否在发、实际频率、QoS 兼容性）必须现场查

### 3. 在自己 dev 机上开发 + 测试
- 代码改完 → dev 端 `colcon build`（**不要 Jetson build，ABI 不兼容**）
- 无硬件 smoke test → dev 端 `ros2 launch ... mock_system.launch.py`（5 节点假数据）
- 真机联调 → dev 端订阅 Jetson 话题（同 `ROS_DOMAIN_ID=42` DDS 自动发现），dev 端发指令、Jetson 节点执行
- 真机部署 → `git push` + ssh Jetson `colcon build` + ssh Jetson launch

### 4. 出问题先 `bash scripts/diagnose.sh`
- 15 项检查：dev 端 ROS2 + Jetson 在线 + DDS 互通 + 防火墙
- CI 用：`scripts/diagnose.sh --json`

## 修改这些值的工作量（不要做）

| 改动 | 工作量 |
|------|------|
| 改 Jetson IP | 改 Jetson 静态 IP + `~/.ssh/config` × 团队 N 台 + `team-constants.md` + `onboard.sh` + `diagnose.sh` + `start_team_rviz.sh` + onboarding 文档 |
| 改 ROS_DOMAIN_ID | 改 Jetson systemd + `~/.bashrc` × N 台 + `team-constants.md` + `onboard.sh` + 所有文档 |
| 改 CycloneDDS config | 改源文件 + 重新部署到 N 台 + 验证 DDS 工作 |

## 比赛窗口（2026-08-10 ~ 08-12）

- **T-4 周（2026-07-13）开始冻结 main 分支**
- 比赛期间 main 只能 critical bug fix
- 开发继续在 `develop/ros2-sidecar`

## 详见

- [`CLAUDE.md`](../CLAUDE.md) — 项目根说明
- [`docs/contributing/branch-strategy.md`](contributing/branch-strategy.md) — Git 流程
- [`scripts/diagnose.sh`](../../scripts/diagnose.sh) — 现场 15 项检查
- [`docs/onboarding/day-one.md`](onboarding/day-one.md) — Day-1 详细指南

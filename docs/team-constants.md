# 团队硬约定 (Team Constants)

> **不要改这些值。** 它们是项目级硬约定，全队统一。
> 改一个，所有 dev 机 + Jetson + 文档都要同步，**得不偿失**。

| 名称 | 值 | 改它的后果 |
|------|----|------|
| **Jetson IP** | **`192.168.3.69`** | 改 IP 需要全队改 `~/.ssh/config` + 防火墙规则 + 文档 |
| **Jetson hostname** | `orin` | mDNS 解析可能不稳，**脚本里用 IP**（见 `scripts/diagnose.sh`） |
| **Jetson 用户** | `xrak` | 改 username 需要全队改 `ssh-copy-id` |
| **团队子网** | `192.168.3.0/24` | 改子网要重做所有防火墙规则 |
| **dev 端可用 IP 段** | `192.168.3.50 ~ 192.168.3.200` | DHCP 分配避开 Jetson 静态 IP |
| **ROS_DOMAIN_ID** | **`42`** | 改这个值 dev 端 / Jetson 端 / `~/.bashrc` 全部要改 |
| **CycloneDDS config** | `ros2_ws/src/vehicle_wbt_platform_cpp/config/cyclonedds.xml` | 全队 `~/.ros/cyclonedds.xml` 来源 |

## Jetson 网络配置（硬编码）

Jetson Orin Nano 4GB 上：
- **静态 IP**：`192.168.3.69/24`
- **hostname**：`orin`
- **Wi-Fi** vs **有线**：当前用 Wi-Fi (`wlP1p1s0`)，比赛建议**切换到有线**（更稳定的多播）

## dev 端网络配置

新成员 dev 机连上团队 Wi-Fi/路由器后：
1. **自动获取 IP**（DHCP），通常落在 `192.168.3.50 ~ 192.168.3.200`
2. **ping Jetson 验证**：
   ```bash
   ping -c 3 192.168.3.69
   ```
3. **DNS 解析**（可选）：`sudo vi /etc/hosts` 加一行 `192.168.3.69 orin`（让 `ssh orin` 也能用）

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

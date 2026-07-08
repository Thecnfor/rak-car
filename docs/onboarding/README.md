# Onboarding — 团队成员入口

> 第一次加入 vehicle_wbt 项目？从这里开始。

## 30 秒 TL;DR

```bash
git clone git@github.com:Thecnfor/rak-car.git
cd rak-car
bash scripts/onboard.sh              # 一键装依赖 + build
bash scripts/diagnose.sh             # 全套健康检查
bash scripts/start_team_rviz.sh      # 一键通过 RViz 看 Jetson (192.168.3.69) cameras
```

跑完上面 4 行，你就跟老成员一样能看了。

## 项目 5 句话

1. **vehicle_wbt** 是 2026 百度智能车比赛的小车，NVIDIA Jetson Orin Nano + ROS2 Humble
2. **架构**：Jetson 跑 sidecar（采集/控制），dev 桌面负责 RViz 监控 + 算法开发
3. **网络**：Jetson **硬编码 192.168.3.69**，团队 `192.168.3.0/24` 内网（详见 [team-constants.md](../team-constants.md)）
4. **DDS**：项目用 `ROS_DOMAIN_ID=42`，多机自动发现
5. **比赛日期**：2026-08-10 ~ 08-12（开发窗口 7.13 进入冻结期）

## 文档地图

| 我想… | 看这个 |
|-------|-------|
| 第一次连 dev 机器 | [day-one.md](day-one.md) |
| 配 LAN / 静态 IP / 防火墙 | [network-setup.md](network-setup.md) |
| 看 Jetson cameras（实时） | [../development/lan-rviz-camera.md](../development/lan-rviz-camera.md) |
| 无真机开发（dev 桌面） | [../development/no-hw-dev.md](../development/no-hw-dev.md) |
| 比赛现场出问题 | [../operations/troubleshooting.md](../operations/troubleshooting.md) |
| 比赛当天流程 | [../operations/competition-day.md](../operations/competition-day.md) |
| Git 流程 / PR 规范 | [../contributing/branch-strategy.md](../contributing/branch-strategy.md) |
| SSH / rsync / Jetson 同步 | [../development/ssh-workflow.md](../development/ssh-workflow.md) |

## 工具速查

| 命令 | 干什么 | 何时用 |
|------|------|--------|
| `bash scripts/onboard.sh` | 第一次 setup | 换 dev 机器 / 新成员 |
| `bash scripts/diagnose.sh` | 15 项健康检查 | 出问题 / 赛前 |
| `bash scripts/start_team_rviz.sh` | 一键通过 RViz 看 Jetson cameras | 日常调试 |
| `bash scripts/dev.sh` | 无真机开发 | 改代码（无硬件） |
| `python3 scripts/calibrate_camera.py --help` | 相机标定 | 换镜头 |

完整说明见 [../../scripts/README.md](../../scripts/README.md)。

## 出问题了？

**先跑**：

```bash
bash scripts/diagnose.sh
```

把输出贴到团队群里——15 项检查会精确定位是哪一层坏了（dev 端 / SSH / Jetson / DDS / topic）。

常见症状表见 [../operations/troubleshooting.md](../operations/troubleshooting.md)。

## 下一步

- **看到 cameras 之后**：跑 `bash scripts/dev.sh --with-rviz` 看 3D robot model
- **想改代码**：从 [../../docs/development/dev-target-architecture.md](../../docs/development/dev-target-architecture.md) 开始
- **提交 PR**：看 [../../docs/contributing/branch-strategy.md](../../docs/contributing/branch-strategy.md)

---

**需要帮助？** 找任意老成员，或者在团队群里 @ Thecnfor。

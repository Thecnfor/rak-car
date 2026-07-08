# Claude Code 工具链使用（团队必读）

> 这项目**强烈推荐用 [Claude Code](https://claude.com/claude-code)** 协助开发。
> 复杂 ROS2 / DDS / 跨节点调试，AI 帮你节省 30-50% 时间。

## 第一次用 Claude Code

1. **安装**：`curl -fsSL https://claude.com/install.sh | bash`（参考 [官方文档](https://docs.claude.com/claude-code)）
2. **进项目目录**：`cd ~/Desktop/rak-car`
3. **启动**：`claude`（CLI 交互）或 `claude --print "你的问题"`（一次性）
4. **自动加载**：本文件（CLAUDE.md）每次启动**自动注入** Claude 的 context，所以保持它准确

## CLAUDE.md 自己的作用

- 每次 `claude` 启动**自动读**这个文件
- 团队约定（**硬约定**）写在这里，Claude 就会遵守：
  - `ROS_DOMAIN_ID=42`
  - Jetson IP `192.168.3.69`
  - 不破坏 `start_team_rviz.sh` 行为
  - 见 [docs/team-constants.md](team-constants.md)
- **改 CLAUDE.md** 改的是 Claude 的"项目记忆"，不是代码——用 `docs commit` 提交

## Memory 体系（每用户）

Claude Code 在 `~/.claude/projects/-home-xrak-Desktop-rak-car/memory/` 存**用户级**知识：

- `now.md` — 当前会话 buffer
- `today-*.md` — 今日 daily notes
- `recent.md` — 最近 7 天
- `core-memories.md` — 关键项目知识
- `archive.md` — 旧知识

格式：每个 memory 是一个文件，frontmatter 有 name / description / type / metadata。

## 常用 workflow 模板

### 1. 改 C++ 节点代码（带 TDD）

```text
帮我加一个新的 ROS2 node：my_sensor_node，读 /dev/ttyUSB0 串口数据
（格式：$RPM,1234\n），发布到 /vehicle_wbt/v1/sensors/my_sensor/rpm (std_msgs/Int32)。

要求：
- 用 rclcpp::Node 基类
- 串口 read 用 std::async + std::chrono
- 包含 unit test (gtest + gmock)
- 失败时 throw std::runtime_error，不静默吞错
- 更新 CLAUDE.md 第 N 行的节点列表
```

### 2. 调试 DDS 看不到 Jetson topic

```text
我在 dev 端 `ros2 topic list` 看不到 Jetson (192.168.3.69) 的话题。

先跑 `bash scripts/diagnose.sh`，贴输出，然后：
- 检查 ROS_DOMAIN_ID 一致
- 检查 ~/.ros/cyclonedds.xml 是否部署
- 检查防火墙 UDP 7400-7500
- 用 ros2 daemon stop && start 清缓存
```

### 3. 加新比赛任务（如 pest_scout）

```text
参考 docs/development/no-hw-dev.md 加一个新任务 pest_scout：
- BaseTask 抽象类
- 任务列表注册到 TaskRegistry
- 主题：vehicle_wbt/v1/sensors/...
- 含 dev.sh --with-mission 测试
```

### 4. 提 PR 前 review

```text
帮我 review 这个 diff：<git diff 输出>

按 CONTRIBUTING.md 流程检查：
- Conventional Commits 格式
- 跑 pytest + colcon test 通过
- 没改 Jetson 端不可改文件
- 加了对应测试
```

## 关键约定（写 CLAUDE.md / commit message 时）

- **永远用 `set -euo pipefail`**（shell 脚本）
- **改 Jetson 端前**：先 ssh xrak@192.168.3.69 备份，`pkill -f <node>` 再启
- **新成员**先跑 `bash scripts/onboard.sh`
- **出问题**：先 `bash scripts/diagnose.sh`
- **DDS 配置**：不动 `cyclonedds.xml` 除非同步更新全队

## 故障排除

| 症状 | 解决 |
|------|------|
| Claude 不知道 Jetson IP | 检查本文件"硬约定"章节 |
| Claude 建议改 main 分支 | 提醒：main 8/10 比赛冻结，只 critical bug fix |
| Claude 编译过本地 ROS 但 Jetson 上失败 | ssh Jetson 端 `colcon build` 而不是本地 |
| 上下文太长 (token 超限) | `/compact` 命令压缩；或 `clear` + 重读 CLAUDE.md |
| Claude 给出错的 bash 命令 | 检查 `set -e` / `cd` 路径；用 `/plan` 模式先 plan |

## 进阶

- **自定义 slash command**：在 `.claude/commands/` 加 `<name>.md`（团队可共享）
- **Hooks**：在 `.claude/settings.json` 配 pre/post 命令（防 `git push --force` 等）
- **MCP servers**：`.claude/mcp.json` 配外部工具（git/perf/browser）
- **Subagents**：复杂任务让 Claude 派 subagent 并行做（如"同时看 docs/ 和 tests/"）

## 相关资源

- [Claude Code 官方文档](https://docs.claude.com/claude-code)
- [Anthropic prompt engineering](https://docs.anthropic.com/claude/docs/prompt-engineering)
- 项目 docs/ — 详细技术文档
- `scripts/diagnose.sh` — 系统健康检查

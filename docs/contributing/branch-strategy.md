# 分支策略与协作工作流

本文档定义 vehicle_wbt 项目团队的 Git 分支管理规范与协作流程。所有人(4-6 人)必须遵守,目标是让 main 分支随时可以上场比赛,同时保留 ROS2 sidecar 等长周期实验的并行空间。

## 当前分支拓扑

```
                            ┌──────────────────────────────┐
                            │       main (LTS-like)        │
                            │  仅 bug fix · 仅 Thecnfor    │
                            │  可合并 · 比赛冻结线         │
                            └──────────────▲───────────────┘
                                           │ fast-forward only
                                           │ (hotfix/* 直 merge)
                                           │
                            ┌──────────────┴───────────────┐
                            │   develop/ros2-sidecar       │
                            │  当前测试线 · 可破坏         │
                            │  ROS2 sidecar + 重构 + 新功能 │
                            └──────────────▲───────────────┘
                                           │ PR via gh pr create
                            ┌──────────────┴───────────────┐
                            │   feat/*  fix/*  refactor/*  │
                            │   个人开发分支 (短命)        │
                            └──────────────────────────────┘

未来 (JetPack 6 刷机后):
                            ┌──────────────────────────────┐
                            │ develop/ros2-humble-post-    │
                            │ flash (从 develop/ros2-      │
                            │ sidecar 当前 commit 切出)     │
                            └──────────────────────────────┘
```

## main 分支约定

`main` 分支是部署到 Jetson 的 systemd 服务 (`py_boot.service` → `main/qqq.py`) 直接对应的代码线,扮演 LTS 角色。

**承诺**:
- main 上的代码必须能在真车上跑通一整套比赛任务(从启动到 arm_set → 推理 → 卸货)
- main 永远是可发布状态,任何 commit 都不能破坏既有功能

**规则**:
- **禁止新功能**:任何非 bug 修复的 PR 一律退回
- **禁止重构**:除 critical bug fix 外的 PR 一律退回
- **禁止直接 push**:所有变更通过 PR + 1 个 reviewer 批准才能合并
- **仅 Thecnfor 可以合并**:其他成员可以开 PR、讨论、approve,但 merge 按钮只能 Thecnfor 按下
- **合并方式**:只接受 fast-forward / squash merge,禁用 merge commit(保持 main 历史线性)

**典型场景**:某个比赛动作卡住、机械臂 offset 偏移、ZMQ 推理偶尔超时——这些都是 bug fix,允许进 main。

## develop/ros2-sidecar 约定

`develop/ros2-sidecar` 是当前活跃开发线,所有新功能、重构、ROS2 sidecar 实验都在这里进行。

**当前状态**(截至 2026-07-05):
- 从 main `e0680f5` 切出,初始内容 = main(尚未有任何 ROS2 sidecar 代码提交)
- 可破坏:CI 失败、临时 print、注释掉的代码段都可以存在
- 不上真车:不在 Jetson systemd 中跑这个分支

**规则**:
- 任何人可以 push,无需 Thecnfor 批准
- 鼓励频繁 squash 提交,不需要保持历史美感
- 合并 PR 需要至少 1 个 reviewer(自己审自己的 PR 不算)
- 主干保持基本可运行:`python main/qqq.py --dry-run` 能 import 通即可,真机跑通留给 main

**典型场景**:ROS2 sidecar 节点实现、MyCar God Object 拆分、新增 PaddlePaddle 模型适配——这些都进 develop/ros2-sidecar。

## 未来 develop/ros2-humble-post-flash

这是一个**占位章节**,等以下条件全部满足后才会启用:

1. Jetson 完成 JetPack 6 (L4T R36.x) 刷机
2. ROS2 Humble / Jazzy 在新系统上装好并通过 smoke test
3. 旧 `develop/ros2-sidecar` 的 sidecar 代码迁移到 Humble 兼容版本

**启用流程**:
```bash
# 在 develop/ros2-sidecar 当前 HEAD 切出新分支
git checkout develop/ros2-sidecar
git pull origin develop/ros2-sidecar
git checkout -b develop/ros2-humble-post-flash
git push origin develop/ros2-humble-post-flash
```

在那之前,所有 ROS2 工作仍然在 `develop/ros2-sidecar` 上推进。

## 个人开发分支

**命名约定**(强制):

| 前缀 | 用途 | 生命周期 |
|------|------|----------|
| `feat/<name>-<short-desc>` | 新功能 | 完成 PR 合并后删除 |
| `fix/<name>-<short-desc>` | bug 修复 | 同上 |
| `refactor/<name>-<short-desc>` | 重构(无功能变化) | 同上 |
| `hotfix/<name>-<desc>` | main 紧急修复 | 合 main 后立即删除 |
| `experiment/<name>-<topic>` | 探索性实验(允许失败) | 7 天不动则可被清理 |

`<name>` 取团队花名/英文名缩写,如 `thecnfor` / `alice` / `bob`。

**示例**:
```bash
git checkout develop/ros2-sidecar
git pull origin develop/ros2-sidecar
git checkout -b feat/thecnfor-tf-broadcaster
git push origin feat/thecnfor-tf-broadcaster
```

**规则**:
- 个人分支必须从 develop/ros2-sidecar 切出(主分支禁止直接开 feat/*)
- 命名必须含 `<name>` 前缀,以便追溯
- 超过 14 天未更新的个人分支,owner 需要在 PR 或 issue 中说明状态,否则会被强制清理

## PR 工作流

**从 feat/* → develop/ros2-sidecar → main 的路径**:

```
feat/<name>-<feature>
    │  gh pr create --base develop/ros2-sidecar
    │  1 reviewer approve
    ▼
develop/ros2-sidecar  (测试线,可包含多个 feat 合并)
    │  gh pr create --base main
    │  Thecnfor 单独 review + merge
    │  (仅 bug fix 与 critical patch 可走这条)
    ▼
main (LTS, 比赛线)
```

**PR 标题规范**(参考 Conventional Commits):
- `feat(camera): add ROS2 camera_node publishing /cam_0/image_raw`
- `fix(arm): correct stepper_1 offset calibration`
- `refactor(mycar): extract lane_pid into LaneController`

**PR 描述必须包含**:
1. 改动的动机(为什么需要)
2. 改动范围(动了哪些文件/模块)
3. 测试方式(怎么验证)
4. 是否影响比赛动作(yes/no)

## WHEN TO MERGE 决策表

| 改动类型 | 目标分支 | 谁能 merge | 是否需要 PR |
|---------|---------|-----------|------------|
| 比赛相关 bug fix | main | 仅 Thecnfor | 必须,1 reviewer |
| critical hotfix(比赛前 3 天发现) | main | 仅 Thecnfor | 可口头同步后补 PR |
| 新功能 / 重构 | develop/ros2-sidecar | 任一 reviewer 批准 + 作者 push | 必须 |
| ROS2 sidecar 实验 | develop/ros2-sidecar | 任一 reviewer | 必须 |
| 实验性 spike | experiment/* | 仅自己 | 不需要 PR(直接 push) |
| 文档更新 | 直接 PR 到 main | 任一 reviewer | 必须 |
| 配置调参(PID、阈值) | develop/ros2-sidecar | 任一 reviewer | 必须,真机验证后再合 main |

## 比赛前 4 周的冻结规则

比赛日期:**2026-08-10**。从 **2026-07-13**(T-4 周)开始进入冻结期。

**冻结期 main 分支规则**:
- main 仅接受 critical bug fix
- 任何改动必须附带真机回归测试结果(录屏或日志)
- 涉及 ARM / 步进 / 舵机的改动需要 2 个 reviewer approve
- 涉及 systemd 启动流程的改动需要 Thecnfor + 至少 1 个对启动链路熟悉的成员 approve
- PID 参数调整允许(比赛前常用),但必须记录到 `docs/config-changelog.md`

**冻结期 develop/ros2-sidecar 规则**:
- 仍然允许新功能与重构(给 ROS2 实验留空间)
- 但禁止大改:`MyCar` 拆分、`infer_back_end.py` 重写、相机/推理协议变更等需先在 issue 中讨论
- 建议在比赛结束前 1 周(T-7 天)暂停 develop 上的所有 push,只允许 cherry-pick critical fix 到 main

## 紧急修复流程

比赛现场或比赛前一天发现 critical bug,使用 hotfix 分支直 merge main,跳过 develop 中转:

```bash
# 1. 从 main 切 hotfix
git checkout main
git pull origin main
git checkout -b hotfix/<name>-<one-line-desc>

# 2. 修复 + 验证
# ... 改代码,在真机上跑一遍 ...

# 3. 提交 + push
git add -A
git commit -m "fix(<module>): <concise description>"
git push origin hotfix/<name>-<one-line-desc>

# 4. 开 PR,base 选 main,标题加 [HOTFIX] 前缀
gh pr create --base main --title "[HOTFIX] <description>" \
             --body "现场发现: <symptom>; 根因: <root cause>; 验证: <evidence>"

# 5. 等待 Thecnfor 紧急 review(电话/微信通知)
# 6. Thecnfor 合并(允许 squash)

# 7. cherry-pick 回 develop/ros2-sidecar,保持两条线同步
git checkout develop/ros2-sidecar
git pull origin develop/ros2-sidecar
git cherry-pick <hotfix-commit-sha>
git push origin develop/ros2-sidecar

# 8. 清理 hotfix 分支
git branch -d hotfix/<name>-<one-line-desc>
git push origin --delete hotfix/<name>-<one-line-desc>
```

**紧急修复的判定标准**(满足任一):
- 比赛动作无法完成(车不动 / arm_set 卡死 / 卸货失败)
- systemd 启动失败,boot_py.sh 不能拉起 qqq.py
- ZMQ 推理服务全挂(5001-5004 全部 timeout)
- 安全性问题(撞车、漏电风险)

普通 bug(显示不对、提示音错位)走普通 PR 流程,不当 hotfix 处理。

---

**最后更新**:2026-07-05 · 维护人:Thecnfor · 任何流程疑问在团队群讨论,重大调整需更新本文件并通知全员。
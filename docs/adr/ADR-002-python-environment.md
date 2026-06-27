# ADR-002: Python 环境管理策略

## 状态

**提议中** — 待团队讨论决定

## 日期

2026-06-27

## 背景

### 当前环境现状

```
系统 Python 3.8.10 (JetPack 预装)
pip 20.0.2 (非常旧，已弃用 pkg_resources)
465 个 pip 包全部装在全局环境
├── 系统基础包 (~200 个)
├── ROS Noetic 相关 (44 个)
├── PaddlePaddle GPU + CUDA 依赖 (~30 个)
├── 项目依赖 (~20 个)
├── 机器学习/科学计算 (~30 个)
├── ROS/catkin 构建工具 (~20 个)
└── 其他/未知 (~120 个)

虚拟环境: 无
conda: 未安装
uv: 未安装
pipx: 未安装
```

### 核心问题

| 问题 | 严重性 | 说明 |
|------|:---:|------|
| 全局环境污染 | 🔴 | `pip install` 可能破坏系统包或 ROS 包 |
| 无版本锁定 | 🔴 | 无 requirements.txt / pyproject.toml / poetry.lock |
| pip 版本过旧 | 🟠 | 20.0.2 不支持现代特性（resolver、PEP 668） |
| PySide2 + PySide6 共存 | 🟠 | 两套 Qt 绑定同时装，可能冲突 |
| 刷机后无法复现 | 🔴 | 465 个包靠记忆手动安装 |
| 无依赖隔离 | 🟠 | 项目包、ROS 包、系统包混在一起 |
| Python 3.8 EOL | 🟡 | 2024.10 已停止安全更新 |

### 约束条件

**PaddlePaddle GPU 不能轻易移动：**
- 依赖 CUDA 11.4 + cuDNN 8.6.0（系统级库）
- aarch64 专用 wheel，只在 Jetson 上能装
- 当前安装在 `/usr/local/lib/python3.8/dist-packages/`
- 移到虚拟环境需要 `--system-site-packages` 或手动链接 CUDA 库

**ROS 必须在系统环境：**
- ROS Noetic 的 Python 包装器（rospy、sensor_msgs 等）硬编码在 `/opt/ros/noetic/`
- `source /opt/ros/noetic/setup.bash` 修改 `PYTHONPATH`，不兼容标准虚拟环境
- ROS 节点必须能 import 系统级的 rospy

**PySide2/6 用于 HRI 显示：**
- JetPack 自带 PySide2（系统包）
- pip 又装了 PySide6（用户包）
- 两套共存，项目实际用 PySide2

## 方案对比

### 方案 A：保持现状（全局环境）

```
继续在系统 Python 3.8 上直接 pip install
```

| 优点 | 缺点 |
|------|------|
| 零改动 | `pip install` 随时可能破坏 ROS 或系统 |
| PaddlePaddle 和 ROS 无缝工作 | 465 个包无法复现 |
| 团队已习惯 | 刷机后靠记忆装包 |
| | 无法并行开发不同版本 |

### 方案 B：conda / miniforge

```
安装 miniforge → 创建 conda 环境 → 管理 Python 版本和依赖
```

| 优点 | 缺点 |
|------|------|
| 完整的环境隔离 | **PaddlePaddle GPU 在 conda 上 aarch64 支持差** |
| 自带 Python 版本管理 | conda 包和 pip 包混用容易冲突 |
| 成熟的包管理 | ROS 不兼容 conda 环境 |
| conda-forge 有大量 aarch64 包 | 需要 `conda install` + `pip install` 混用 |
| | 安装体积大（miniforge ~100MB） |
| | 学习成本：conda 命令与 pip 不同 |
| | **Jetson 上 PaddlePaddle 必须用 pip 装，conda 没有** |

### 方案 C：venv + system-site-packages ⭐ 推荐

```
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install <项目依赖>
```

| 优点 | 缺点 |
|------|------|
| **零额外安装** — Python 自带 venv | 不自带 Python 版本管理 |
| PaddlePaddle 通过 system-site-packages 可用 | ROS 包通过 system-site-packages 可用 |
| 项目依赖隔离在 `.venv/` | 如果不加 `--system-site-packages`，PaddlePaddle 不可用 |
| 刷机后 `pip install -r requirements.txt` 一键恢复 | |
| 轻量，无额外依赖 | |
| 团队只需学 `source .venv/bin/activate` | |

### 方案 D：uv（现代方案）

```
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv .venv --python 3.8
uv pip install -r requirements.txt
```

| 优点 | 缺点 |
|------|------|
| **极快**（比 pip 快 10-100 倍，Rust 实现） | **aarch64 支持不确定** |
| 内置 Python 版本管理（`uv python install 3.10`） | 较新工具（2024 年发布），社区经验少 |
| 兼容 pip / requirements.txt / pyproject.toml | PaddlePaddle GPU 的 aarch64 wheel 兼容性未验证 |
| 内置依赖锁定（`uv.lock`） | ROS 集成方式未验证 |
| 单二进制文件，无需安装 | 团队需要学习新工具 |
| 比 conda 轻量得多 | |

### 方案 E：venv + uv（混合方案）

```
用 uv 管理 Python 版本和依赖安装（快）
用 venv 创建虚拟环境（稳）
```

| 优点 | 缺点 |
|------|------|
| 两全其美 | 需要协调两个工具 |
| uv 的速度 + venv 的稳定性 | 复杂度略高 |

## 推荐方案：C（venv）+ 锁定文件

### 实施步骤

```bash
# 1. 创建虚拟环境（继承系统包，包括 PaddlePaddle 和 ROS）
cd /home/jetson/workspace/vehicle_wbt
python3 -m venv --system-site-packages .venv

# 2. 激活
source .venv/bin/activate

# 3. 生成当前依赖锁定文件
pip freeze > requirements-lock.txt

# 4. 创建干净的 requirements.txt（只列项目直接依赖）
cat > requirements.txt << 'EOF'
# 核心依赖
numpy>=1.23,<2.0
opencv-python>=4.5
pyserial>=3.5
pyzmq>=25.0
PyYAML>=5.3
psutil>=5.9
simple-pid>=2.0
jsonschema>=4.0
requests>=2.28

# 大模型
erniebot>=0.5
openai>=1.0

# 显示（PySide2 通过 system-site-packages 提供，不在此列）
# PySide2 — 系统自带
# paddlepaddle-gpu — 系统级安装，不在此列
EOF

# 5. 刷机后一键恢复
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### systemd 服务修改

```ini
# 修改 ExecStart 使用虚拟环境中的 Python
ExecStart=/home/jetson/workspace/vehicle_wbt/.venv/bin/python -u /home/jetson/workspace/vehicle_wbt/main/qqq.py
```

### 目录结构

```
vehicle_wbt/
├── .venv/                    # 虚拟环境（gitignored）
├── requirements.txt          # 项目直接依赖
├── requirements-lock.txt     # 完整锁定文件
├── .gitignore                # 添加 .venv/
└── ...
```

### .gitignore 添加

```
.venv/
__pycache__/
*.pyc
```

## 关于 uv 的补充建议

uv 是 2024 年发布的现代 Python 包管理器，速度极快（Rust 实现）。**建议团队关注但暂不采用**，原因：

1. aarch64 + PaddlePaddle GPU 的兼容性未验证
2. ROS 的 system-site-packages 集成方式未测试
3. 团队需要学习新工具

**如果未来 Python 升级到 3.10+，可以重新评估 uv。** 届时：
- uv 的 Python 版本管理可以自动安装 3.10
- uv 的依赖锁定比 pip freeze 更可靠
- uv 的 aarch64 支持应该已经成熟

## Python 版本升级路线

| 时间点 | 版本 | 方式 |
|--------|------|------|
| 当前 | 3.8.10 | JetPack 预装，不动 |
| JetPack 6.x 升级 | 3.10 或 3.11 | 系统自带 |
| 升级后 | 考虑 uv | 用 uv 管理 Python + 依赖 |

**不建议单独升级 Python 版本。** PaddlePaddle GPU 的 aarch64 wheel 绑定了特定 Python 版本，升级 Python 需要重新找兼容的 PaddlePaddle wheel。等 JetPack 升级时一并处理。

## 实施计划

### Phase 1: 立即（半天）

```bash
# 创建虚拟环境
python3 -m venv --system-site-packages .venv

# 生成锁定文件
source .venv/bin/activate
pip freeze > requirements-lock.txt

# 创建精简 requirements.txt（手动整理）

# 更新 .gitignore
echo ".venv/" >> .gitignore
```

### Phase 2: 清理（1 天）

```bash
# 检查 PySide2 vs PySide6 冲突
# 删除不用的 PySide6（如果项目只用 PySide2）
pip3 uninstall PySide6 PySide6-Addons PySide6-Essentials

# 更新 systemd 服务使用 .venv/bin/python
```

### Phase 3: 文档（半天）

```bash
# 更新 docs/system-environment.md 刷机步骤
# 添加 venv 创建和激活步骤
```

## 风险与缓解

| 风险 | 概率 | 影响 | 缓解措施 |
|------|:---:|:---:|---------|
| system-site-packages 漏掉某些包 | 低 | 🟡 | 激活后测试所有 import |
| venv 中 pip install 覆盖系统包 | 低 | 🟡 | pip 会警告，确认即可 |
| systemd 服务找不到 venv Python | 中 | 🔴 | 用绝对路径 `.venv/bin/python` |
| 团队忘记激活 venv | 中 | 🟡 | 在启动脚本中自动激活 |

## 结论

| 方案 | 工作量 | 隔离性 | PaddlePaddle | ROS | 推荐度 |
|------|:---:|:---:|:---:|:---:|:---:|
| A: 保持现状 | 0 | ❌ | ✅ | ✅ | ⭐ |
| B: conda | 1 天 | ✅ | ⚠️ 困难 | ❌ | ⭐⭐ |
| **C: venv** | **半天** | **✅** | **✅** | **✅** | **⭐⭐⭐⭐** |
| D: uv | 1 天 | ✅ | ⚠️ 未验证 | ⚠️ 未验证 | ⭐⭐⭐ |
| E: venv+uv | 1 天 | ✅ | ⚠️ | ⚠️ | ⭐⭐⭐ |

**推荐方案 C。** 零额外安装，Python 自带 venv，`--system-site-packages` 让 PaddlePaddle 和 ROS 无缝工作。配合 `requirements.txt` 解决刷机复现问题。等 JetPack 6.x 升级后再评估 uv。

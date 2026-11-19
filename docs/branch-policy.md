# 分支与合并策略

> 状态：建议方案。**当前 `main` 尚未开启 branch protection**，本文不自动修改仓库设置；
> 需要仓库 owner 在 GitHub Settings → Branches 手动启用，或用管理员 token 调 API。

## 1. 长期分支

| 分支 | 用途 | 规则 |
|------|------|------|
| `main` | 可交付基线 | 只接受 PR；必须通过 `ros-ci` / `sanitizer`；禁止 force push |
| `backup/*` | 修改前快照 | 只增不改；不作为开发分支；确认无未合并工作后可归档 |

## 2. 短期分支命名

| 前缀 | 用途 | 示例 |
|------|------|------|
| `fix/` | 缺陷修复 | `fix/p0-safety-20261119` |
| `perf/` | 性能优化 | `perf/p1-hotpath` |
| `refactor/` | 结构重构 | `refactor/split-core` |
| `ci/` | CI/构建改动 | `ci/debug-cmake` |
| `style/` | 纯格式/文档 | `style/readme-figures` |

合并后删除短期分支；`backup/*` 不在此列。

## 3. main 保护建议

启用后至少配置：

- Require a pull request before merging（至少 1 个 approval，单人仓库可勾选 "Allow specified actors to bypass"）
- Require status checks to pass：
  - `ROS 2 CI / colcon build`（按实际 job 名）
  - `覆盖率`
  - `ASan / UBSan`
- Require branches to be up to date before merging
- Do not allow force pushes / deletions
- 可选：Require linear history

`hardware-erc` 目前是 `continue-on-error`，**不要**设为 required，直到 ERC 能在 runner 稳定跑通且报告可归因。

## 4. 待清理分支（需 owner 确认）

以下分支已合并或为一次性调试分支，建议确认后删除；删除不可逆，先看 `git log main..<branch>` 是否为空：

- `ci/debug-cmake`
- `ci/split-sanitizer`
- `fix/docs-counts`
- `fix/docs-counts-2`
- `style/confidence-band`
- `style/readme-figures`

以下分支可能含未合并工作，**不要直接删**：

- `backup/pre-optimization-20261024`
- `backup/pre-optimization-20261025`
- `backup/pre-p0-20261119`
- `backup/pre-p1-20261119`
- `fix/p0-safety-and-tf`
- `fix/p1-control`
- `fix/p1-preview`
- `fix/realtime`
- `perf/p0-build-realtime`
- `perf/p1-hotpath`
- `refactor/remove-cmake-hacks`
- `refactor/split-core`
- 以及本轮 `fix/p0-safety-20261119`、`fix/p1-all-20261119`、`fix/p2-all-20261119`

## 5. 发布与标签

- 版本号以 `docs/project-facts.json` 为唯一真源。
- 打 tag 前确认 `docs/selftest-counts.json`、CI 步骤名、CHANGELOG 三处一致。
- 不在 `main` 上直接改版本号；版本提交走 PR。

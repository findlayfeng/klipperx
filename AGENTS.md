# AI 工作者约定（AGENTS.md）

给在本仓库工作的 AI 代理（pi、Claude Code、Codex 等）读的仓库级约定。开工前先读本文件，
再按任务读对应的文档（见文末「读什么」）。

## 任务与分支：一个任务一条分支

仓库支持**多个任务并行**。每个任务必须在自己的分支上做，不要把不同任务的改动混进同一条分支。

### 命名

分支名沿用 `agents/` 前缀 + 任务语义，按改动类型选前缀：

| 前缀 | 用于 | 示例 |
|------|------|------|
| `agents/docs-<主题>` | 文档改动 | `agents/docs-ai-branch-conventions` |
| `agents/fix-<模块>-<问题>` | 修复 | `agents/fix-mcu-identify-timeout` |
| `agents/feat-<功能>` | 新功能 | `agents/feat-heater-bed` |
| `agents/test-<范围>` | 测试 | `agents/test-upstream-gcode` |

### 规则

1. **基线**：从 `work`（当前开发主线）切出。不要直接在 `work` / `master` 上提交。
2. **并行任务用 worktree 隔离**：同一仓库同时开多个任务时，为每条分支建独立 worktree，
   避免互相污染检出状态；每个 worktree 内独立构建、独立提交：

   ```bash
   git worktree add ../klipperx.worktrees/<短名> -b agents/<分支名> work
   ```

   现有布局：主检出 `…/klipperx` → `work`；并行任务在 `…/klipperx.worktrees/<短名>`。
3. **开工前先看状态**：`git status` 确认工作区干净再切分支；发现已有未提交改动先报告，
   不要替用户 stash 或丢弃。
4. **任务完成后**：提交到自己的分支（提交信息遵循仓库现有风格，见下），**不自动合入 `work`**；
   合并由用户决定，或用户明确授权时再合。
5. **worktree 用完要收拾**：任务合并或放弃后，`git worktree remove <路径>` 清掉，
   避免 `.worktrees/` 下堆积僵尸目录。

### 提交信息风格

沿用现有历史：`<type>(<scope>): <中文简述>`，type 取 `feat` / `fix` / `docs` / `test` /
`refactor` 等；涉及工单编号的带前缀（如 `feat(C1a):`、`docs(H2/G4):`）。

pre-commit 钩子会自动跑 `cargo fmt --all` 并重新暂存 `.rs`（新检出后启用一次）：

```bash
git config core.hooksPath .githooks
```

## 改完必须同步手册

**每次修改完成后，必须在同一批改动里完成受影响的手册修改**，与代码一起提交——
手册不是“以后再补”的尾巴。手册滞后就是上次全面审计发现 16 类过时断言的根因，
不要制造下一批。

改什么，就同步哪本：

| 改动 | 同步到 |
|------|--------|
| `src/core/klippy/` 的行为、结构、模块增删 | [开发手册](docs/klippy/developer-manual/README.md)对应页（模块表 + 专题页） |
| G-Code 命令增删改、配置节/选项增删改 | [G-Code 命令参考](docs/klippy/user-manual/gcode-commands.md) / [配置文件参考](docs/klippy/user-manual/config.md) |
| 客户端交互、日志行为 | [客户端使用](docs/klippy/user-manual/client.md) / [日志与调试](docs/klippy/user-manual/logging.md) |
| `klippy-api` / `klippy-client`、线上形状、端点 | [第三方开发手册](docs/klippy/third-party-dev/README.md)（`api-reference.md` 与端点状态表要一起改） |
| 新增模块或测试、覆盖变化 | [测试](docs/klippy/developer-manual/testing.md)的覆盖清单 |
| 回归数字、忽略列表、转绿状态 | [回归测试](docs/klippy/developer-manual/regression-tests.md) |

三条硬要求：

1. **写死的数字与名字必须重核**：测试名、端点条数、计数、二进制大小、工单号——
   提交前逐个对回源码/实测（悬空的工单号与“已完成当待办”的引用同样是过时）。
2. **旧断言随实现一起改**：功能落地、工单完成、行为变更时，把手册与源码注释里
   对应的“尚未实现 / 待办 / 属某工单”一并改成现状，不要留下新旧两种说法。
3. **已完成的任务标号不得再被引用**：`TODO.md` 与 `docs/work-log/` 里的任务/问答
   标号（如 `D3`、`F8b`、`T3`、`Q2`、`H1-3`），一旦对应任务完成，就不得再出现在
   **其他文档与源码注释**中——引用处改为陈述现状，或注明“已归档”；连带的“见
   TODO X”“属 X”“X 待办”一并清理。**豁免范围**：`TODO.md` 与 `docs/work-log/`
   自身是台账与档案，其中的历史标号照旧保留，本条不适用于这两个位置。

提交前自查：手册里关于被改对象的每句话，现在还能对着源码成立吗？细则见
[测试 → 文档同步](docs/klippy/developer-manual/testing.md#文档同步)。

## 读什么

| 任务类型 | 先读 |
|----------|------|
| 任何任务 | 本文件 + [`docs/README.md`](docs/README.md) |
| 改 `src/core/klippy/`（消息、MCU、命令、事件、配置、G-Code…） | [开发手册](docs/klippy/developer-manual/README.md) |
| 改 `crates/klippy-api` / `klippy-client` 或对外 API 形状 | [第三方开发手册](docs/klippy/third-party-dev/README.md) + 开发手册的 `api` 一节 |
| 写测试 / 查覆盖 | [测试](docs/klippy/developer-manual/testing.md)、[`TESTING.md`](TESTING.md) |
| 推进上游回归 | [回归测试](docs/klippy/developer-manual/regression-tests.md) |
| 查看当前待办与工单 | [`TODO.md`](TODO.md) |
| 一次性分析/盘点背景 | [工作记录](docs/work-log/README.md)（非规范） |

改动后的同步义务见上一节[改完必须同步手册](#改完必须同步手册)。

## 常用命令

```bash
cargo test --workspace              # 全部单元测试（虚拟 workspace 必须带 --workspace 或 -p）
cargo test -p klipperx --lib <过滤> # 单包过滤
cargo fmt --all                     # 提交前格式化（钩子也会跑）
```

真机 / 外设验证不阻塞主线，约定见 [`TESTING.md`](TESTING.md)。

---
name: task-branch-flow
description: 本仓「一个任务一条分支」的实操：worktree 的开辟/构建环境/销毁、ff 还是 merge 的判定、abandoned 归档、并发构建。当要开新任务分支、合并一个已完成分支、或清理 worktree 时使用。
---

# 任务分支与 worktree 实操

> 政策（谁有权、禁止什么）在 [`AGENTS.md`](../../../AGENTS.md)「任务与分支」；本页只写**怎么做**与踩过的坑。

## 开一个新任务

```bash
git status                                    # 先确认工作区干净（有未提交改动先报告，不要 stash/丢弃）
git worktree add ../klipperx.worktrees/<短名> -b agents/<类型>-<主题> work
```

- 目录布局：主检出 `…/klipperx` → `work`；并行任务在 `…/klipperx.worktrees/<短名>`。
- **只有 main 建/销毁 worktree**；worker 只在自己那个目录里编辑、构建、提交。
- 每条分支**一个写者**（哪怕文件不相交：`cargo fmt --all`/`git status` 会让双方看到对方中间态）。
- 钩子（每份新检出启用一次）：`git config core.hooksPath .githooks`——pre-commit 会跑 `cargo fmt --all`
  并重新暂存 `.rs`。

### worktree 里的子模组：**保持空目录**

- 不 `git submodule update`（属禁止的远程交互），**也不拷**主检出的 `third_party/klipper`
  （拷贝会让子模组的相对 gitfile 失效，worktree 里任何 `git status` 都报「不是 Git 仓库」）。
- 需要语料/构建时用环境变量指向主检出：

  ```bash
  KLIPPERX_KLIPPER_DIR=<主检出>/third_party/klipper cargo test -p klipperx --lib <过滤>
  ```

  由 `crates/test-support`（`build.rs` 与 `klipper_dir()`）与 `src/core/klippy/upstream.rs` 统一解析；
  共享同一份检出是安全的（构建只写各自 `OUT_DIR`，`KCONFIG_CONFIG`/`OUT` 显式外移）。
- **板子固件用仓库根 `make`**（`make` / `make klipper` / `make katapult`，配置 `make menuconfig`）：
  它也写本检出/worktree 的 `out/firmware/` 与 `.config`，子模组只读；**不要**进子模组里 `make`。
  见 [构建 MCU 固件](../../../docs/klippy/user-manual/firmware-build.md)。
- `third_party/katapult` 同样保持空目录；读 katapult 检出的代码用
  `KLIPPERX_KATAPULT_DIR=<主检出>/third_party/katapult`（`katapult_dir()` 解析，同一约定）。
- 注意：worktree 的 `.cargo/config.toml` 里**不一定**有 `KLIPPERX_KLIPPER_DIR`（主检出的本地改动不带过去），
  所以每条 cargo 命令都显式带上这个前缀最稳。

### 要改子模组本身（固件）

给子模组单独开 worktree（兄弟目录），分支名同样 `agents/…`：

```bash
git -C third_party/klipper worktree add /abs/path/klipperx.worktrees/<短名> -b agents/<主题> HEAD
```

（`third_party/katapult` 要改时同理：`git -C third_party/katapult worktree add …`。）

- 销毁时**必须在属主仓库里执行**：`git -C third_party/klipper worktree remove <路径>`；
  在主检出里跑 `git worktree remove <子模组路径>` 会报「不是一个工作区」（踩过）。
- 子模组提交信息用英文前缀（`host:` / `mux:` / `uart:` / `build:` / `port:` …）；合并进 `klipperx`
  分支后再在主仓更新 gitlink。

## 合并一个已完成分支

判定顺序（踩过「本该 `--no-ff` 却默认 ff」的坑，已纠正过）：

```bash
git merge --no-ff --no-edit agents/<分支名>     # 多提交：保留合并提交
git merge --ff-only        agents/<分支名>     # 单提交：快进，不加合并提交
```

1. **先确认能干净合入**（无冲突）；有冲突先解决，再判断方式。
2. **相对合并基底只有一个提交 ⇒ `--ff-only`**：树内容与分支验收完全一致，**不必重跑全量测试**。
3. **多个提交（或多分支合并）⇒ `--no-ff` 保留合并提交**：树内容变了，要重跑对应验收。
4. 合入是 **main** 的动作；worker/子代理不得自行 `merge`/`rebase`/`cherry-pick`。

- 误用 ff 的补救（本地未推送时）：`git reset --hard <合并前>` 再 `git merge --no-ff`（先确认工作区干净）。
- 合并前跑闸门：`cargo test --workspace`（**虚拟 workspace 必须带 `--workspace` 或 `-p`**）。

## 放弃/搁置的改动

不删除、不遗弃在工作区：把它的头迁到 `abandoned/<短名>` 分支保存，并在
[`docs/abandoned.md`](../../../docs/abandoned.md) 登记来源、放弃原因、可挖掘点与日期。
`abandoned/*` 不参与主线；取料进主线要按正常流程重做新分支（可编译、带测试、手册同步），
提交信息注明来源分支。

## 构建与并发（避免互相踩）

```bash
scripts/cargo-locked.sh cargo test --workspace   # 探 cargo 自己的锁，会报「谁持有 + 等多久」
CARGO_TARGET_DIR=/tmp/t-<短名> cargo test …      # 要并发就各自独立 target 目录
```

- **同一个 `target/` 不要并发跑两个 cargo**：第二个在构建锁上排队，且 CPU 争用会让计时敏感的测试
  偶发卡住（实测遇到过一次 `futex_wait` 挂住）。
- 纯文档改动不必跑闸门；**改代码/固件后**、以及**合并前**必须跑。

## 收尾

```bash
git worktree remove <路径>     # 验收通过（已合入或已迁 abandoned）后销毁，别堆僵尸目录
git worktree list              # 主检出与子模组各看一次
```

跑过真机/测试后别忘了清残留进程与临时文件（见 [`flash-firmware`](../flash-firmware/SKILL.md) 收尾节）。

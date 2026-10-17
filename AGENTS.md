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
| `abandoned/<主题>` | **被放弃/搁置的改动**（只保存，不合入） | `abandoned/wip-main-leftovers` |

### 规则

1. **基线**：从 `work`（当前开发主线）切出。不要直接在 `work` / `master` 上提交。
2. **并行任务用 worktree 隔离**：同一仓库同时开多个任务时，为每条分支建独立 worktree，各自独立构建、
   独立提交。worktree 里 `third_party/klipper` 保持**空目录**——不 `git submodule update`（属规则 6 禁止的
   远程交互），**也不拷**主检出的子模组（相对 gitfile 会失效）；构建/语料用
   `KLIPPERX_KLIPPER_DIR=<主检出>/third_party/klipper` 指向主检出（共享同一份检出是安全的：只写各自
   `OUT_DIR`；**板子固件用仓库根 `make`**，写各自检出/worktree 的 `out/firmware/` 与 `.config`，子模组只读）；
   `third_party/katapult` 同理——保持空目录，读取用 `KLIPPERX_KATAPULT_DIR=<主检出>/third_party/katapult`。**命令、目录布局、子模组 worktree、并发构建与收尾见
   [`task-branch-flow`](.agents/skills/task-branch-flow/SKILL.md)。**此命令只由 main 执行，见规则 5。
3. **开工前先看状态**：`git status` 确认工作区干净再切分支；发现已有未提交改动先报告，
   不要替用户 stash 或丢弃。
4. **任务完成后**：提交到自己的分支（提交信息遵循仓库现有风格，见下），**不自动合入 `work`**；
   是否合并由用户决定（或用户明确授权时再合），**合并操作本身也由 main 执行**——
   worker/子代理不得自行 `git merge`/`rebase`/`cherry-pick` 到其他分支，只负责在自己的
   分支上提交。

   **合并方式：main 合入前先确认能干净合入，再按提交数判定**——相对合并基底**只有一个提交 ⇒ ff**
   （不产生合并提交，树未变、无需重跑全量测试）；**多个提交或多分支合并 ⇒ 保留普通合并提交**
   （树变了，重跑对应验收）。命令、误用 ff 的补救与并发构建见
   [`task-branch-flow`](.agents/skills/task-branch-flow/SKILL.md)。
5. **worktree 生命周期只归 main**：git 工作空间的**开辟**（`git worktree add`）
   与**验收后的销毁**（任务合并或放弃、验收通过后执行 `git worktree remove <路径>`，
   避免 `.worktrees/` 下堆积僵尸目录）都由 main（主代理）统一完成；**worker/子代理
   不得自行开辟、销毁或清理 git 工作空间**（含 `worktree add/remove/prune`），只在
   main 分配好的 worktree 内编辑、构建、提交。
6. **不操作任何远程仓库**：所有工作只在本地进行，**禁止对远程仓库做任何操作**——
   含 `git push` / `pull` / `fetch` / `clone`、`git remote add/remove/set-url`、
   子模块与镜像同步等一切远程交互；也不要把 worktree/分支推到远端。需要远端操作时
   先报告用户，由用户自己执行。
7. **文档更新由 main 代 worker 完成**：worker 完成代码改动后**不自行改手册/文档**——
   在产出中列出「受影响的文档 + 需同步的要点」，由 main 在同一分支上补齐文档并随
   代码一起提交（可追加提交或在验收前修入原提交）；worker 只负责代码与报告清单，
   「改完必须同步手册」的义务不变，只是执行人换成 main。
8. **放弃/搁置的改动进 `abandoned/` 分支集合**：被放弃、被替代、来源可疑或中断而未完成的
   改动（包括子代理写错检出、超时中断、方向被否的实现），**不得直接删除**——由 main 迁到
   `abandoned/<短名>` 分支保存，并在 [`docs/abandoned.md`](docs/abandoned.md) 登记来源、
   放弃原因、可挖掘点与日期。`abandoned/*` **不参与主线**：不合并、不在其上继续开发、
   不作为验收依据，只作素材来源；删除条目或从中取料继续做都要用户点头，执行仍由 main 完成。
   取料进主线时按正常流程重做新分支（可编译、带测试、手册同步），提交信息里注明来源分支。
9. **评审证据由 main 补齐**：评审/侦察子代理若声明「没有 shell」（它们只能读文件、跑不了 `git`），
   它的逐文件结论需要 main 补一次 `git show --stat <commit>` / `git show <commit> -- <file>`
   并把输出贴回去；别拿「内容比对」当文件清单证据。
10. **合入门槛**：评审裁定的 **P0/P1 必修完才合入**（修入原提交或紧接一个跟进提交）；P2 建议可随同批
   收口。评审“修复后可合入”时，先在分支上修完、重跑验收，再合并。

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

执行分工：**worker 不自行改文档**——改动代码后报告受影响文档与需同步要点，
由 **main 代 worker 完成**并随代码一起提交（见上文「任务与分支」规则 7）。

改什么就同步哪本、提交前自查、以及删档案前分流的分类去处：见
[`doc-sync`](.agents/skills/doc-sync/SKILL.md)（含改动→手册映射表）。

三条硬要求（细则与自查清单同见该 skill）：

1. **写死的数字与名字必须重核**：测试名、端点条数、计数、二进制大小、工单号——逐个对回源码/实测。
2. **旧断言随实现一起改**：手册与源码注释里的「尚未实现 / 待办 / 属某工单」一并改成现状，不留两种说法。
3. **已完成的任务标号不得再被引用**：`TODO.md`/`docs/work-log/` 里的 `D3`/`F8b`/`T3`/`Q2`/`H1-3` 之类
   一旦完成，不得再出现在其他文档与源码注释里（豁免这两个台账自身）。

提交前自查：手册里关于被改对象的每句话，现在还能对着源码成立吗？

**删档案前先分流**：删过程档案前，先把「唯它才有的规范信息」分类搬进规范文档，清掉所有引用、只留一句
非链接的溯源说明（分类去处见 [`doc-sync`](.agents/skills/doc-sync/SKILL.md)）。

## 派侦察（scout）的准则

- **规格未知才派**：写任务书前若不确定上游语义、本仓落点或依赖链，先派**只读 scout** 产出「可直接
  抄进任务书的规格」（带 file:line、公式/文案、本仓落点与缺口），并**与在跑的写手并行**——只读 lane
  不占「同一时刻 1 个写手」的约束，等写手落地再串行侦察是纯浪费。
- **不重复侦察**：写手自己要读的同一份文件（同一上游模块、同一 seam）**不再另派 scout**——两套事实
  要对账，且并行 prompt 必须在源缝隙/证据/决策上互不重叠。规格已知时，直接把要点写进任务书。
- **判断口径**：写手在跑**不相干**文件、而下一单元的规格**未知** → 侦察就该在这段等待期开出去。
  反之，若侦察产出的规格没人消费（下游未排期），也不要为「以防万一」而派。
- scout 任务书同样按六要素写全（目标/边界/约束/验收/输出/停止条件），约束至少含：只读、不改文件、
  不执行 git 写操作、不委派；产出**直接回传**，不落文件。

## Worker 执行纪律

- **禁止上下文压缩**：worker/子代理在执行任务期间**不得调用 `context_prune` 或任何形式的上下文
  压缩/摘要工具**——压缩会吞掉任务书里的规格细节与验收数字。上下文吃紧时：停止推进，在收口/
  阶段报告中说明「上下文吃紧 + 已完成进度 + 卡点」，由 main 裁棒换棒续做；不得自行压缩后继续。
- **超时必报三件套**：任务遇到任何超时（预算到点、单条命令 `timeout`/rc=124、复现超时）时，
  报告**必须同时**给出：① 当前进度（已提交/未提交状态、已证事实清单）；② 卡住的具体原因
  （卡在哪一步 + 关键日志尾部/证据）；③ 建议的下一步。不得只回「超时了/已停止」。
  **先分析后报告**：遇超时先自行定位首因（带界复现、看日志尾部/最后有效证据行、对照代码）；
  能定位的直接报首因 + 证据 + 修复计划，**确实不能定位的才按三件套上报**，且须附
  「已试方案与排除依据」（自查分析阶段不超过 15 分钟，不突破总预算）。
- **禁止破坏性 git 操作**：worker/子代理**不得**运行 `git checkout --` / `git restore` /
  `git stash` / `git clean`，也不得用任何会丢弃工作区改动的命令。看到**不属于自己**的改动
  （别的单元正在写、或来源不明）时：**只报告、不清理**。曾发生事故：一个 worker 跑
  `cargo fmt --all` 时看到并行单元的 `tui.rs` 改动，`git checkout --` 把它整份抹掉。
- **一个 worktree 一个写者**：main 不得把两个写者放进同一个 worktree——哪怕文件互不相交，
  `cargo fmt --all` / `git status` 这类命令也会让双方看到对方的中间态，进而误判或误清。
  并行单元一律各建 worktree；确需共用时，任务书里必须写明「本 worktree 只有你一个写者」
  并明确禁止上述破坏性操作。
- **任务书自带防超时三板斧**：给写手的任务书必须写「立即落笔（读完模板就写第一版）/ 10 分钟内
  必须首个提交（WIP 也算）/ 每步增量提交 / 只读错误点名的文件、禁探索式 grep」——曾有一个单元前
  三次尝试各 30 分钟**零产出**（侦察挤爆预算），加上这三句后 30 分钟内 2 个提交完成。
- **命令形式纪律**：bash 一律**单条命令**，禁 `2>&1`、`|`、`>`、`;`（要几步就几步）；改文件用
  `edit`/`write`，禁 heredoc / `sed -i`；跑测试用**窄过滤 + `timeout`**（如 `timeout 300 cargo test …`），
  普通测试禁开 `KLIPPERX_TRACE=1`（全量 trace 会灌爆 harness）。
- **测试必须整体有界**：新增/改造测试一律加总体 `tokio::time::timeout`（或等价兜底），**禁止**写出
  能挂住 CI 的用例；交互测试用 `--test-threads=1` 复现挂死。排查挂死先看线程状态：**全线程
  `futex_wait` + CPU≈0 = 在等**（不是死循环）。细则见 [测试](docs/klippy/developer-manual/testing.md)。
- **改 `third_party/`（上游子模组）或固件前先报告**：worker 不得擅自改子模组/固件的代码、`Kconfig`、
  `Makefile`（本仓对上游有偏移，需登记与验证口径），先把「要改什么、为什么必须改」报给 main 裁决；
  批准后仍只在自己的 worktree/分支内提交。
- **真机/刷板纪律**：刷固件、SWD 排障按 [`.agents/skills/flash-firmware`](.agents/skills/flash-firmware/SKILL.md)
  skill 执行（**SWD 频率必须保守、按设备调整，过高会静默刷写失败；读内存前必须 `halt`**）；
  真机跑完必须清理残留进程与临时文件（`pgrep -x klipper.elf` 自查——`kill -9` 夹具会漏子进程）。
- 本节各条同样适用于 worker 任务书：main 写任务书时必须把本节纪律写进约束节。main（主代理）不受此限。

## 技能（skills）

项目级 skill 放在 **`.agents/skills/<名>/SKILL.md`**（随仓库走，pi 从工作目录向上发现，到仓库根为止）；
需要时用 `/skill:<名>` 显式加载。当前七份：

| skill | 何时用 |
|-------|--------|
| [`flash-firmware`](.agents/skills/flash-firmware/SKILL.md) | 给板子构建/刷写固件、SWD 排障、验证「固件到底刷上没有」——**含频率/`halt`/回读校验的完整手法** |
| [`katapult-self-update`](.agents/skills/katapult-self-update/SKILL.md) | 不用编程器更新/开发 katapult 自身：deployer 自举的配置开关与符号语义、刷前三段核验、`flash` 自举命令与双复位窗口、读回硬证据、UART ROM 恢复 |
| [`subagent-rescue`](.agents/skills/subagent-rescue/SKILL.md) | 子代理卡住/超时/零产出、provider `content_filter` 误杀、测试挂死时的救援流程 |
| [`task-branch-flow`](.agents/skills/task-branch-flow/SKILL.md) | 开任务分支/worktree、子模组 worktree、`KLIPPERX_KLIPPER_DIR` 构建、ff vs `--no-ff` 判定、`abandoned/*` 归档、并发构建与收尾 |
| [`doc-sync`](.agents/skills/doc-sync/SKILL.md) | 改完同步手册的**清单**：改动→手册映射表、写死数字重核、旧断言/已完成标号清理、删档案前分流 |
| [`subagent-briefing`](.agents/skills/subagent-briefing/SKILL.md) | 写子代理任务书：六要素模板、纪律块、防超时三板斧、worker/scout/reviewer 分工与路由 |
| [`python-tools`](.agents/skills/python-tools/SKILL.md) | 加/用 Python 工具的固定流程：没有 `venv/` 先 `python3 -m venv venv` → 用 `venv/bin/pip` 装 → 之后一切 Python 只写 `venv/bin/python`、`venv/bin/<工具>` 的**绝对路径** |

新手法（工具/设备/流程）沉淀优先考虑做成 skill；**规则与纪律**（本节各节）仍写在 `AGENTS.md`。

## 读什么

| 任务类型 | 先读 |
|----------|------|
| 任何任务 | 本文件 + [`docs/README.md`](docs/README.md) |
| 改 `src/core/klippy/`（消息、MCU、命令、事件、配置、G-Code…） | [开发手册](docs/klippy/developer-manual/README.md) |
| 改 `crates/klippy-api` / `klippy-client` 或对外 API 形状 | [第三方开发手册](docs/klippy/third-party-dev/README.md) + 开发手册的 `api` 一节 |
| 写测试 / 查覆盖 | [测试](docs/klippy/developer-manual/testing.md)、[`TESTING.md`](TESTING.md) |
| 构建 / 刷写板子固件 | [构建 MCU 固件](docs/klippy/user-manual/firmware-build.md)、[`flash-firmware`](.agents/skills/flash-firmware/SKILL.md) |
| 推进上游回归 | [回归测试](docs/klippy/developer-manual/regression-tests.md) |
| 查看当前待办与工单 | [`TODO.md`](TODO.md) |
| 一次性分析/盘点背景 | [工作记录](docs/work-log/README.md)（非规范） |

- **上游对照只读仓库内 `third_party/klipper/`**：机器上另有 klipper 检出（例如 `/opt/klipper`）时，
  **不要**拿它当参照——它可能是**旧树**（实测缺 `trigger_analog` 等新特性、个别文件签名与语料不同源），
  与本仓语料/字典不一致；一切对照读 `third_party/klipper/`，**语料测试是最终仲裁**。

改动后的同步义务见上一节[改完必须同步手册](#改完必须同步手册)。

## 常用命令

```bash
make                                    # 板子固件：一次共享配置编 katapult + klipper 两份（见 docs/klippy/user-manual/firmware-build.md）
make klipper / make katapult            # 只编一份
make menuconfig                         # 改共享目标与两侧选项
cargo test --workspace                  # 全量闸门：单元测试 + 集成测试（虚拟 workspace 必须带 --workspace 或 -p）
cargo test -p klipperx --lib <过滤>     # 单包过滤——快，但不含 klippy-client 的集成测试
cargo build --workspace --all-targets   # 构建告警应为 0
cargo fmt --all                         # 提交前格式化（钩子也会跑）
```

- **同一 `target/` 不要并发跑两个 cargo**：第二个会在构建锁上排队，而且并发构建的 CPU 争用会让计时敏感的测试偶发卡住（实测遇到过一次 `futex_wait` 挂住）。要并发就各自 `CARGO_TARGET_DIR=<目录>`，或干脆分 worktree。
  **想知道自己是不是在排队**：用 `scripts/cargo-locked.sh <cargo 子命令…>` 代替 `cargo`——它探的是 **cargo 自己的锁**（`<target>/<profile>/.cargo-build-lock`，cargo 用 `flock(2)`），所以**裸 `cargo` 在跑它也能发现**；会先打
  `another cargo holds …; waiting (idle CPU here is expected, not a hang)` + 持有者 pid 与命令行，拿到锁后报 `acquired after <时长>`，退出码原样透传（实测：裸 `cargo build` 占着时，包装器报告并等了 11.5s）。
- **合并前用全量闸门**：`cargo test -p klipperx --lib` 很快，但**不含** `klippy-client` 的集成测试（历史上内建参数表过期就是这样漏掉的）；提交/合并前跑 `cargo test --workspace`。
- **Python 工具一律走仓库根的 `venv`**：装用 `venv/bin/pip`、跑用 `venv/bin/python`（工具同理 `venv/bin/<工具>`）；**不要**裸 `python`/`pip`（那四个都在 `/usr/bin/`，装出来不在 venv 里、按路径调就找不到），**也不要**依赖 `source venv/bin/activate`（激活是**状态**，一旦掉了就悄悄退回系统解释器）。没有 `venv/` 就先 `python3 -m venv venv`。三步细则与踩坑见 [`python-tools`](.agents/skills/python-tools/SKILL.md)。

真机 / 外设验证不阻塞主线，约定见 [`TESTING.md`](TESTING.md)。

# 工作记录

本目录存放一次性的**分析 / 盘点 / 审计记录**，不是规范，也不随实现推进而更新。
规范与执行清单以 `docs/klippy/` 与根目录的 `TODO.md` 为准。

正文记的是写作当天的盘点与判断，**不随后续实现改写**；每篇末尾的「实施记录」（§6）是唯一
随实现推进更新的回填节——余项与现状以它为准。

## 现行状态

**在册 1 份**：

| 记录 | 主题 | 何时可删 |
|------|------|----------|
| [`2026-10-18-voron-trident-gaps.md`](2026-10-18-voron-trident-gaps.md) | **阶段一**：让主机对接固件、在真机上安全开发调试还缺什么（主机侧 P0/P1），以及让现成 Moonraker 把 klipperx 当 klippy 用所需的一处状态补齐；含验收定义、配置层绕行、上板实测清单与文档债。打印相关已划到阶段二 | 阶段一验收达成后：先把独有的规范结论分流进手册与 `TODO.md`，再删 |

**已完结**的记录从本目录清理（最近一批：`2026-09-23-h2-notes.md`、
`2026-09-23-gcode-move-notes.md`，2026-10-18 收口——其独有的规范信息先分流进
[运行时](../klippy/developer-manual/klippy-runtime.md)、[G-Code 命令](../klippy/user-manual/gcode-commands.md)、
[配置手册](../klippy/user-manual/config.md) 与 `TODO.md` 的 H2/G4/H10 各节，然后才删）。

需要回溯时从 git 历史取：

```bash
git log --diff-filter=D -- docs/work-log/     # 被删的记录
git log -- docs/work-log/                    # 本目录的完整历史
```

**新记录怎么放**：一次性分析写进来（按 `<日期>-<主题>.md` 命名），任务收口时按
[`doc-sync`](../../.agents/skills/doc-sync/SKILL.md) 的删档条款先分流、再删——不要留下
「已完成但还在册」的记录，也不要让手册与台账各存一份同样的细节。

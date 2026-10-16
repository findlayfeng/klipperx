---
name: python-tools
description: 往本仓加/用一个 Python 工具时的固定流程：没有 venv 就先建、用 venv 的 pip 装、之后一切 Python 只用 venv 里的东西。当要安装或调用任何 Python 工具（CLI、脚本、库）时使用。
---

# Python 工具一律走仓库根的 `venv`

> 本页是** Python 解释器与包该从哪里来**的唯一出处。工具**怎么用**（例如刷板的频率与 `halt`）在各自的
> skill 里（[`flash-firmware`](../flash-firmware/SKILL.md) 就用 `./venv/bin/pyocd`）。
> 现状（已实测）：`venv/bin/python` 是 **3.14.8**，里面装着 SWD 那一套
> （`pylink-square`、`pyelftools`、`capstone`、`cmsis-pack-manager`、`libusb-package`、`psutil` …）。

## 三步

### 1. 没有 `venv/` 就先建

```bash
python3 -m venv venv
```

- 目录固定是**仓库根下的 `venv/`**（不是 `~/.venv`、不是每个工具一个），这样每份检出/每个 worktree 的
  工具来源一致，也便于 skill 里统一写路径。
- `python3` 与 `python` 本机都存在（都是 `/usr/bin/` 下的），**建 venv 用 `python3`**。
- venv 会自动生成一份 `venv/.gitignore`（内容就一个 `*`），仓库的 `.gitignore` 另有一条显式 `/venv/`
  兜底——**不要**因为你看到前者就把后者删掉：忽略规则不该依赖“目录是谁生成的”。

### 2. 用 **venv 的** pip 装工具

```bash
venv/bin/pip install <包名>
```

- **只走 `venv/bin/pip`**。不要用裸 `pip`/`pip3`，也不要 `python -m pip`——那四个都在 `/usr/bin/` 里，
  装出来的东西不在 venv 里，下次按 skill 的路径调用就找不到。
- **不要 `sudo`，不要 `--user`**：这个 venv 是用户级的，装包不需要任何特权。
- 要固定版本就写 `包名==版本`；一次装多个就写在同一条命令里（写进文档/提交信息时保持可复现）。

### 3. 之后所有 Python 内容**只用 venv 里的**

- 跑脚本：`venv/bin/python <脚本>`
- 跑工具：`venv/bin/<工具名>`（例如 `./venv/bin/pyocd`）
- **不写裸 `python`/`python3`/`<工具名>`**，即使它们在 `PATH` 里。

**为什么坚持绝对路径而不是 `source venv/bin/activate`**：激活是**状态**——一旦它掉了（新 shell、子代理、
`bash -lc`、任务书里另起一段命令），裸名字就悄悄退回系统解释器，而系统解释器里没有这些包，报错还常是
`ModuleNotFoundError` 这种指向别处的原因。路径写死则永远对，也让命令可以逐条抄进任务书。

## 常见坑

- **venv 不提交**：它含绝对路径（`pyvenv.cfg`、`bin/activate` 里的路径），换机器即失效，也没有跨平台意义。
  自查：`git ls-files venv` 必须是**空**；`.gitignore` 必须有 `/venv/`。
- **不要 `git add -f venv/...`** 去绕过忽略——那是把上一个坑做实。发现 venv 里有文件被跟踪了，按
  [`AGENTS.md`](../../../AGENTS.md)「先报告再处理」办，不要自行清理。
- **依赖清单**：本仓目前**没有** `requirements.txt`/`pyproject.toml`/`setup.py`（已核实不存在），装了什么就
  靠这一步的意图记录。要引入清单是新的一件事，别在装工具时顺手加。
- **新检出的工作区还没有 venv**：worktree 与主检出**不共享** `venv/`（它是磁盘上的目录，不是 git 内容）。
  在哪个检出里跑 Python 工具，就在哪个检出里按第 1、2 步建一份。

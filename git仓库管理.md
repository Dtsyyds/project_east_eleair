# Git 仓库管理

本文适用于东方电气项目仓库 `/home/fire/Desktop/project_east_eleair`，介绍如何创建短期功能分支、多次提交和推送、测试完成后合并到 `main`，以及合并后如何管理原分支。

推荐流程：从最新 `main` 创建功能分支 → 在功能分支上多次开发、提交、推送 → 测试 → 更新 `main` 并合并 → 再次测试 → 推送 `main`。

## 1. 分支命名规则

建议使用以下格式：

```text
类型/英文名-完整日期-功能说明
```

例如，开发者 `fire` 在 2026 年 11 月 9 日开始添加焊缝跟踪功能：

```text
feat/fire-20261109-seam-tracking
```

| 部分 | 含义 | 示例 |
| --- | --- | --- |
| 类型 | 新功能用 `feat`，修复用 `fix`，文档用 `docs` | `feat` |
| 英文名 | 开发者英文名，建议小写 | `fire` |
| 完整日期 | 创建分支的日期，使用 `YYYYMMDD`，避免跨年混淆 | `20261109` |
| 功能说明 | 简短英文说明，多个单词用 `-` 连接 | `seam-tracking` |

其他示例：

```text
feat/fire-20261109-camera-preview
fix/fire-20261110-ui-freeze
docs/fire-20261109-git-guide
```

一个分支对应一个相对独立的任务。任务跨天继续时，沿用原分支，不需要每天新建分支。同一天有两个独立任务时，用不同的功能说明区分。

下文统一使用 `feat/fire-20261109-seam-tracking`，实际操作时请将所有命令中的分支名替换成自己的分支名。

## 2. 从最新 main 创建短期功能分支

进入项目并检查当前状态：

```bash
cd /home/fire/Desktop/project_east_eleair
git status
git branch --show-current
```

先确认当前分支的修改已经提交，工作区干净。如果有尚未完成的修改，先在原分支妥善保存，再执行下面的切换操作。

```bash
git switch main &&
git pull --ff-only origin main &&
git switch -c feat/fire-20261109-seam-tracking
```

这些命令会依次切换到本地 `main`、获取并更新到远程 `main`、创建并切换到功能分支。`&&` 表示前一步成功后才执行下一步。

`--ff-only` 只允许快进更新。如果本地和远程 `main` 已经分叉，命令会停止，需要先检查双方提交并处理分叉。

确认创建结果：

```bash
git branch --show-current
```

预期输出：

```text
feat/fire-20261109-seam-tracking
```

## 3. 第一次 add、commit、push

在功能分支上修改代码。完成一个可以描述清楚的阶段后，先检查修改：

```bash
git status
git diff
```

确认需要提交的内容后，暂存修改：

```bash
git add .
git diff --cached
git status
```

在项目根目录执行 `git add .` 会暂存该目录下所有未被忽略的新增、修改和删除。提交前检查暂存内容，避免混入无关改动、日志、采集数据或本地配置；如果只需提交部分内容，可以将 `.` 换成具体文件或目录路径。

提交并首次推送：

```bash
git commit -m "feat: 添加焊缝跟踪功能"
git push -u origin feat/fire-20261109-seam-tracking
```

| 命令 | 作用 |
| --- | --- |
| `git add` | 把选定的工作区修改放入暂存区 |
| `git commit` | 把暂存区内容记录为当前分支上的一个本地提交 |
| `git push` | 把本地分支的提交推送到远程仓库 |
| 首次推送的 `-u` | 设置本地分支与远程分支的跟踪关系，方便后续直接执行 `git push` |

未提交的工作区修改不会被 `git push` 上传。

## 4. 后续开发可以多次提交和推送

只要仍然处于这个功能分支上，就可以反复执行以下流程：

```bash
git branch --show-current
git status
git diff

git add .
git diff --cached
git commit -m "fix: 修复焊缝短暂丢失时的显示问题"
git push
```

下一阶段继续修改后，再提交一次：

```bash
git add .
git diff --cached
git commit -m "feat: 完善跟踪参数与界面提示"
git push
```

每个 `commit` 都会形成一个独立提交；后一次提交不会覆盖前一次提交。一次 `push` 也可以上传多个尚未推送的提交，不要求每次提交后立即推送。

这些提交和推送都发生在功能分支上，远程 `main` 不会因此更新。是否影响 `main`，取决于你当前所在的分支以及推送的目标分支。

建议完成一项清楚的修改就提交一次，提交说明写清楚修改目的。例如：

```text
feat: 添加相机画面预览
fix: 修复参数保存后未生效的问题
docs: 补充功能测试步骤
```

## 5. 测试完成后合并到最新 main

### 5.1 提交并推送功能分支的最终修改

先在功能分支完成测试，并确认最终修改已经提交和推送：

```bash
git branch --show-current
git status
git push
```

如果 `git status` 仍显示需要保留的未提交修改，先按上一节的步骤提交，然后再继续。

### 5.2 更新 main 并合并

```bash
git switch main &&
git pull --ff-only origin main &&
git merge --no-ff feat/fire-20261109-seam-tracking \
  -m "merge: 合并 fire 20261109 焊缝跟踪功能"
```

这里使用 `--no-ff`：当功能分支有新提交需要合入时，即使能够快进，也会创建一个合并提交。这样可以从历史中看出这项功能何时整体合入 `main`。

如果功能分支的提交已经全部合入 `main`，Git 会提示已经是最新状态，不会重复创建合并提交。

### 5.3 在合并后的 main 上测试，再推送

如果开发期间其他人更新了 `main`，合并结果包含双方的修改，需要在合并后的 `main` 上重新运行与本次改动相关的构建、启动和功能测试。

测试通过后，检查历史并推送：

```bash
git status
git log --oneline --graph --decorate -15
git push origin main
```

`git merge` 先更新本地 `main`；执行 `git push origin main` 成功后，远程 `main` 才会更新。测试过程中如果修改了代码，也需要先检查、提交并重新测试，再推送。

如果远程仓库要求通过 Pull Request 合并，则推送功能分支后，在仓库平台创建目标为 `main` 的 Pull Request，并按仓库规则完成检查和合并；需要保留本文这种合并记录时，选择创建合并提交的方式（如果仓库允许）。

## 6. 合并会不会覆盖原来的 main 提交？

假设创建功能分支时，`main` 的最后一次提交是：

```text
1108---界面优化
```

功能分支随后产生三次提交，使用本文的 `--no-ff` 合并后，历史大致如下：

```text
*   M  merge: 合并 fire 20261109 焊缝跟踪功能  ← main
|\
| * E  feat: 完善跟踪参数与界面提示             ← 功能分支
| * D  fix: 修复焊缝短暂丢失时的显示问题
| * C  feat: 添加焊缝跟踪功能
|/
*   B  1108---界面优化
```

其中 `B`、`C`、`D`、`E`、`M` 是示意标签，不是实际提交哈希。

- 原来的 `1108---界面优化` 提交仍在历史中，不会被删除或覆盖。
- 功能分支上的三次提交也都会保留。
- `main` 新增一个合并提交 `M`，它同时连接合并前的 `main` 和功能分支末尾的提交。
- 合并后的文件内容会整合双方修改。如果功能分支修改了界面代码，最新界面内容可能变化；保留旧提交不代表旧文件内容永远不变。
- 如果开发期间 `main` 又新增了提交，正常合并也会保留并整合这些提交。

普通 `git merge` 在可以快进时，可能只移动分支位置，不新增合并提交。本文显式使用 `--no-ff`，便于保留清晰的功能合并记录。

## 7. 遇到合并冲突如何处理

如果合并提示冲突，先查看冲突文件：

```bash
git status
```

打开冲突文件，结合双方修改决定最终内容。文本冲突通常由 `<<<<<<< HEAD`、`=======` 和 `>>>>>>> 分支名` 三种标记分隔：前半部分是当前 `main` 一侧的内容，后半部分是功能分支一侧的内容。

编辑为需要保留的最终代码，删除冲突标记。新增、删除、重命名等冲突也要根据 `git status` 的提示逐项处理。

全部处理后：

```bash
git add .
git diff --cached
git merge --continue
```

如果打开提交信息编辑器，保存并退出即可完成合并。之后重新测试，再执行 `git push origin main`。

如果决定取消这次尚未完成的合并：

```bash
git merge --abort
```

日常合并使用普通的 `git merge --no-ff`。`-X theirs` 会在可自动处理的内容冲突中偏向另一侧，可能舍弃 `main` 中仍然需要的修改，不建议作为默认选项。`git push --force` 用于强制更新远程引用，也不是正常合并功能分支所需的操作。

如果推送 `main` 时提示远程已有新提交，可以获取并合并远程更新：

```bash
git fetch origin &&
git merge --no-edit origin/main
```

如有冲突，按本节处理；整合后重新测试，再执行 `git push origin main`，不要用强制推送跳过别人的提交。

## 8. 合并后，原来的功能分支还存在吗？

**存在。执行本文的合并和推送命令不会自动删除原分支。**

| 对象 | 合并并推送 main 后的状态 |
| --- | --- |
| 本地 `main` | 指向新的合并提交 |
| 远程 `main` | 推送成功后指向新的合并提交 |
| 本地功能分支 | 仍然存在，仍指向功能分支原来的最后一次提交 |
| 远程功能分支 | 如果此前已推送，也仍然存在 |

合并的含义是把功能分支的修改与提交历史整合到 `main`。它不会让功能分支自动消失，也不会让功能分支自动移动到 `main` 的新合并提交上。

可以保留功能分支用于查看；短期任务完成后，也可以手动删除。

### 8.1 可选：删除本地功能分支

确认合并结果已测试通过并推送到远程 `main`，切换到 `main` 后再删除：

```bash
git switch main
git branch -d feat/fire-20261109-seam-tracking
```

删除本地分支不会删除远程同名分支。如果 `-d` 提示尚未合并，先检查提交关系和合并结果。

### 8.2 可选：删除远程功能分支

如果远程也不再需要保留该分支：

```bash
git push origin --delete feat/fire-20261109-seam-tracking
git fetch origin --prune
```

删除远程分支不会自动删除其他开发者的本地分支。`--prune` 用于清理本地已经失效的远程跟踪引用。

按本文方式完成合并后，删除功能分支只是移除分支名称，已经合入 `main` 的提交仍然可以通过 `main` 的历史查看。

### 8.3 下一项功能重新建分支

开始另一个独立任务时，从更新后的 `main` 新建分支，例如：

```bash
git switch main &&
git pull --ff-only origin main &&
git switch -c feat/fire-20261110-camera-preview
```

## 9. 常用查看命令

```bash
# 查看当前分支
git branch --show-current

# 查看工作区和暂存区状态
git status

# 查看本地分支及远程跟踪关系
git branch -vv

# 查看本地已知的远程分支
git branch -r

# 查看最近的提交关系
git log --oneline --graph --decorate -20

# 查看双方分支的提交关系
git log --oneline --graph --decorate --all -30
```

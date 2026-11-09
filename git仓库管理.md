# Git 仓库管理

流程：更新 `main` → 创建功能分支 → 开发、提交、推送 → 功能测试 → 合并到最新 `main` → 再次测试 → 推送 `main` → 删除本地和远程功能分支。

## 1. 命名规则

- 分支名：`branch---月日英文名---功能说明`，例如 `branch---1109fire---更新仓库管理办法`。
- 提交说明：`月日英文名---修改说明`，例如 `1109fire---更新仓库管理办法`。
- 合并提交说明：`月日英文名---merge`，例如 `1109fire---merge`。
- `1109` 表示 11 月 9 日，`fire` 是开发者英文名。一个任务一个分支，跨天继续使用原分支，提交日期使用实际提交当天。

下文使用上述分支名，操作其他任务时替换即可。

## 2. 创建分支

先确认原分支的修改已提交、工作区干净，再从最新 `main` 创建：

```bash
cd /home/fire/Desktop/project_east_eleair
git status

git switch main &&
git pull --ff-only origin main &&
git switch -c branch---1109fire---更新仓库管理办法
```

## 3. 开发、提交、推送

一直在功能分支开发，每完成一个阶段就提交并推送：

```bash
git status
git add .
git diff --cached
git commit -m "1109fire---更新仓库管理办法"
git push -u origin branch---1109fire---更新仓库管理办法
```

首次推送使用 `-u`，以后重复 `add → commit → push`，推送只需 `git push`，每次更新提交说明。`git add .` 会暂存当前目录下所有未被忽略的改动，提交前检查内容。这些操作不会自动更新 `main`。

## 4. 测试通过后合并到 main

功能测试通过，且全部修改已提交、推送后，切换到 `main` 并合并：

```bash
git switch main &&
git pull --ff-only origin main &&
git merge --no-ff branch---1109fire---更新仓库管理办法 \
  -m "1109fire---merge"
```

有新提交需要合入时，`--no-ff` 会新增合并提交，原有历史保留。冲突需手动处理，再执行 `git add . && git merge --continue`；取消未完成的合并用 `git merge --abort`，此处的-m ""不必写清楚，因为仅有一个merge操作而已。

## 5. 再次测试并推送 main

在合并后的 `main` 上重新测试；如果又修改了代码，先提交并重新测试。通过后推送：

```bash
git push origin main
```

## 6. 删除本地和远程功能分支

**合并不会自动删除功能分支。确认 `main` 测试通过且推送成功后，再删除：**

```bash
git switch main

# 删除本地分支
git branch -d branch---1109fire---更新仓库管理办法

# 删除远程分支
git push origin --delete branch---1109fire---更新仓库管理办法
```

本地、远程分支需要分别删除，已合入 `main` 的提交仍保留。若 `-d` 提示未合并，先检查，不要直接强制删除。下一个任务从最新 `main` 重新创建分支。

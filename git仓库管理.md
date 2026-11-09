# Git 仓库管理

## 1. 命名规则

- 分支名：`branch---月日英文名---功能说明`，例如 `branch---1109fire---更新仓库管理办法`。
- 提交说明：`月日英文名---修改说明`，例如 `1109fire---更新仓库管理办法`。
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

第一次提交并推送：

```bash
git status
git add .
git diff --cached
git commit -m "1109fire---更新仓库管理办法"
git push -u origin branch---1109fire---更新仓库管理办法
```

`add` 暂存修改，`commit` 创建本地提交，`push` 上传提交；首次使用 `-u` 设置远程跟踪关系。`git add .` 会暂存当前目录下所有未被忽略的改动，提交前检查内容。

后续在同一分支反复执行，按实际修改填写提交说明：

```bash
git status
git add .
git diff --cached
git commit -m "1109fire---补充分支删除说明"
git push
```

这些提交都保留在当前分支上，不会自动更新 `main`。

## 4. 测试通过后合并到 main

先提交并推送功能分支的全部修改，再执行：

```bash
git switch main &&
git pull --ff-only origin main &&
git merge --no-ff branch---1109fire---更新仓库管理办法 \
  -m "1109fire---合并仓库管理办法分支"
```

合并后重新测试，通过再推送：

```bash
git push origin main
```

- 有新提交需要合入时，`--no-ff` 会新增一个合并提交，原有提交历史保留。
- **合并后，本地和远程的原功能分支仍然存在，不会自动删除。**
- 有冲突时，手动处理后执行 `git add . && git merge --continue`，然后重新测试、推送。
- 取消尚未完成的合并：`git merge --abort`。

## 5. 查看与删除分支

```bash
git branch -vv   # 查看本地分支及跟踪关系，* 表示当前分支
git branch -r    # 查看本地已知的远程分支
```

确认合并结果已测试并推送到 `main`，按需删除：

```bash
git switch main

# 删除本地分支
git branch -d branch---1109fire---更新仓库管理办法

# 删除远程分支（可选，与本地删除相互独立）
git push origin --delete branch---1109fire---更新仓库管理办法
```

删除本地分支不会删除远程分支，也不会删除已合入 `main` 的提交。若 `-d` 提示未合并，先检查；确认未合并提交不再需要时，才使用 `-D` 强制删除。

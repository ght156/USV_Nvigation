# Git 常用命令（本仓库速查）

> 工作区：`build/`、`install/`、`log/` 已被忽略，一般只提交 `src/`、`tools/` 等源码。

---

## 一、提交（最常用）

```bash
# 先看改了什么（建议每次提交前都跑一下）
git status

# 把要提交的文件加入暂存区
git add <文件或目录>       # 指定路径
git add .                  # 当前目录下所有改动（遵守 .gitignore）

# 正式保存一个版本
git commit -m "简要说明这次改了什么"

# 推到 GitHub（需要时已配置 origin）
git push
```

**补充**

```bash
git add -p                 # 按「块」选择暂存，大文件只提交一部分时很有用
git commit --amend         # 修改「上一次」提交（说明或漏文件；未 push 时更安全）
git commit --amend --no-edit   # 不改说明，只把新暂存并入上一次提交
```

---

## 二、查看与对比（最常用）

```bash
git status                 # 哪些改了、是否已暂存、分支名
git diff                   # 工作区相对「暂存区/上次提交」的差异（未暂存部分）
git diff --staged          # 已暂存、准备提交的内容和上次提交的差异
git log --oneline -20      # 最近 20 条提交，一行一条
git show                   # 最近一次提交的详情（补丁）
git show <提交hash>        # 某次提交的详情（hash 可用 log 里前几位）
```

---

## 三、撤销、删除与清理（重点：按场景选）

**取消暂存（文件改回「未暂存」，工作区内容保留）**

```bash
git restore --staged <文件>
```

**丢弃工作区对某文件的未提交修改（危险：未提交的改动会没）**

```bash
git restore <文件>
```

**同时：既取消暂存，又丢弃工作区修改（等价于「这文件回到上次提交」）**

```bash
git restore --staged <文件>
git restore <文件>
```

**删除 Git 已跟踪的文件（并从仓库里移除跟踪）**

```bash
git rm <文件>              # 删除磁盘文件并记入下次提交
git rm --cached <文件>      # 只停止跟踪，文件可留在磁盘（适合误加了大文件）
```

**未跟踪文件/空目录（例如误生成的垃圾）——先预览再删**

```bash
git clean -n               # 预览会被删的未跟踪文件
git clean -fd              # 删除未跟踪的文件和目录（确认无误再用）
```

---

## 四、远程与同步（简要）

```bash
git pull                   # 拉远端并合并到当前分支
git fetch origin           # 只拉信息，不合并
git push                   # 推送当前分支
git remote -v              # 查看远程地址
```

---

## 五、暂存现场（换分支前临时收起改动）

```bash
git stash
git stash list
git stash pop
```

---

## 六、分支（简要）

```bash
git branch
git switch <分支名>
git switch -c <新分支名>
```

---

*本文件仅作本地备忘，与 Git 版本无关的具体行为以 `git --help` 为准。*

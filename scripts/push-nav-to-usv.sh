#!/usr/bin/env bash
# 将本地 src/USV_NAV（含未提交改动）同步推送到 GitLab usv 仓库的 main 分支。
#
# 背景：本地仓库（USV_NX）与 GitLab usv 是两套不相干的历史，usv 是 monorepo。
#       每次同步 = 基于 usv/main 建临时分支，整体替换 src/USV_NAV，提交后快进 main。
#
# 用法：
#   bash scripts/push-nav-to-usv.sh           # 正常推送
#   bash scripts/push-nav-to-usv.sh --dry-run # 只检查差异，不推送
set -euo pipefail

DRY_RUN=0
[[ "${1:-}" == "--dry-run" ]] && DRY_RUN=1

REPO="$(git rev-parse --show-toplevel)"
cd "$REPO"

USV_REMOTE="usv"
SRC="src/USV_NAV"
ORIG="$(git branch --show-current)"

if [[ -z "$ORIG" ]]; then
  echo "错误：请先 checkout 到工作分支（如 USV_NX），再运行本脚本。"
  exit 1
fi

if ! git config "remote.${USV_REMOTE}.url" >/dev/null; then
  echo "错误：没有名为 ${USV_REMOTE} 的远程，请先执行："
  echo "  git remote add ${USV_REMOTE} http://192.168.51.6/wxhikrobot/usv.git"
  exit 1
fi

if [[ ! -d "$SRC" ]]; then
  echo "错误：当前分支缺少 $SRC 目录，请在正确的工作分支上运行。"
  exit 1
fi

echo "==> 1/6 拉取 ${USV_REMOTE}/main 最新状态"
git fetch "$USV_REMOTE"

BASE="$(git rev-parse --verify "${USV_REMOTE}/main")"
BRANCH="nav-sync-$(date +%Y%m%d-%H%M%S)"
WT_BASE="$(mktemp -d)"
WT="$WT_BASE/nav"

echo "==> 2/6 创建临时分支 $BRANCH（基于 ${BASE:0:7}）"
git worktree add -b "$BRANCH" "$WT" "$BASE"

cleanup_fail() {
  echo
  echo "出错了。临时分支 $BRANCH 与工作区 $WT 已保留，可手动修复后继续："
  echo "  cd $WT"
  echo "  git fetch $USV_REMOTE && git rebase $USV_REMOTE/main"
  echo "  git push $USV_REMOTE $BRANCH:main"
  echo "完成后清理："
  echo "  git worktree remove --force $WT"
  echo "  git branch -D $BRANCH"
}
trap cleanup_fail ERR

echo "==> 3/6 用本地内容整体替换 $SRC"
git -C "$WT" rm -r -q "$SRC"
git archive "$ORIG" "$SRC" | tar -x -C "$WT"

# 带上未提交的已跟踪修改、未跟踪新增文件；跳过被忽略的（map/*.pgm、__pycache__ 等）
while IFS= read -r f; do
  [[ -n "$f" ]] || continue
  mkdir -p "$WT/$(dirname "$f")"
  cp "$f" "$WT/$f"
done < <(git ls-files -m -o --exclude-standard -- "$SRC")

# 删除本地已删除（git rm）的跟踪文件
while IFS= read -r f; do
  [[ -n "$f" ]] || continue
  rm -f "$WT/$f"
done < <(git ls-files -d -- "$SRC")

# -f 必要：usv 仓库根 .gitignore 有 data/ 规则，YOLO 权重目录会被忽略
git -C "$WT" add -Af "$SRC"

if git -C "$WT" diff --cached --quiet; then
  echo "==> src/USV_NAV 与 ${USV_REMOTE}/main 没有差异，无需推送"
  git worktree remove --force "$WT"
  git branch -D "$BRANCH"
  rmdir "$WT_BASE" 2>/dev/null || true
  exit 0
fi

echo "==> 4/6 提交同步变更"
git -C "$WT" commit -m "feat(nav): 同步 src/USV_NAV 导航包到 usv 仓库"

if [[ "$DRY_RUN" == 1 ]]; then
  echo "==> 5/6 DRY-RUN：以下变更将推送（未实际推送）"
  git -C "$WT" show --stat --oneline HEAD
  git worktree remove --force "$WT"
  git branch -D "$BRANCH"
  rmdir "$WT_BASE" 2>/dev/null || true
  exit 0
fi

echo "==> 5/6 推送 $BRANCH -> ${USV_REMOTE}/main"
if ! git push "$USV_REMOTE" "$BRANCH:main"; then
  echo "推送被拒：main 上可能有新提交。按上面的提示在 $WT 里 rebase 后重推。"
  cleanup_fail
  exit 1
fi

echo "==> 6/6 清理临时分支"
git worktree remove --force "$WT"
git branch -D "$BRANCH"
rmdir "$WT_BASE" 2>/dev/null || true

echo "完成：src/USV_NAV 已同步到 GitLab ${USV_REMOTE}/main"

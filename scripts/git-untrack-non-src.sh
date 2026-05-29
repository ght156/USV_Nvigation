#!/usr/bin/env bash
# 从 Git 索引移除不应远程跟踪的路径（本地文件保留）。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

paths=(
  docs
  test
  rviz2.rviz
  tools
  map
)

for p in "${paths[@]}"; do
  if [[ -e "$p" ]] || git ls-files --error-unmatch "$p" &>/dev/null; then
    git rm -r --cached --ignore-unmatch "$p" 2>/dev/null || true
  fi
done

# src 内 docs / map / pgm
git ls-files 'src/**/docs/**' 'src/**/map/**' '*.pgm' 2>/dev/null | while read -r f; do
  git rm --cached --ignore-unmatch "$f" 2>/dev/null || true
done

echo "已从索引移除 docs/test/map 等（工作区文件未删除）。"
echo "请执行 git status 确认后 commit。"

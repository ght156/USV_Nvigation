#!/usr/bin/env bash
# 安装 pre-push：push 时仅允许 src/ 功能包（禁止 docs/test/map 等上大仓）。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
git -C "$ROOT" config core.hooksPath .githooks
chmod +x "$ROOT/.githooks/pre-push"
echo "已设置 core.hooksPath=.githooks（仓库: $ROOT）"
echo "推送规则: 仅 src/ 代码 + .gitignore / .githooks / scripts/install-git-hooks.sh"

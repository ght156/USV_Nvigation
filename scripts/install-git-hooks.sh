#!/usr/bin/env bash
# 安装 pre-push：允许 src/、docs/*.md、test/；禁止 map/pgm/归档等。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
git -C "$ROOT" config core.hooksPath .githooks
chmod +x "$ROOT/.githooks/pre-push"
echo "已设置 core.hooksPath=.githooks（仓库: $ROOT）"
echo "推送规则: src/ + docs/*.md + test/；禁止 map/*.pgm、*.tar.gz、src/perception/"

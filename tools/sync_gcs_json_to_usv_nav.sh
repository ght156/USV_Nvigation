#!/usr/bin/env bash
# 将 GROUND-CONTROL-STATION 保存的航点/颜色 JSON 同步到本仓库 workspace_nav/json（供 Nav2 与 USV_NAV 节点读取）
set -euo pipefail
GCS_ROOT="${GROUND_CONTROL_STATION:-$HOME/GROUND-CONTROL-STATION}"
SRC_JSON="$GCS_ROOT/backend/data"
DST_JSON="$(cd "$(dirname "$0")/.." && pwd)/src/USV_NAV/workspace_nav/json"

if [[ ! -d "$SRC_JSON" ]]; then
  echo "未找到 GCS 数据目录: $SRC_JSON" >&2
  echo "可设置: export GROUND_CONTROL_STATION=/你的/GROUND-CONTROL-STATION" >&2
  exit 1
fi

mkdir -p "$DST_JSON"
if [[ -f "$SRC_JSON/waypoints.json" ]]; then
  cp -v "$SRC_JSON/waypoints.json" "$DST_JSON/waypoints.json"
fi

# target_buoy 由 GCS 发 /color_code 后 target_buoy 节点写入；此处仅当 color 为绿/红/黑关键词时写占位
if [[ -f "$SRC_JSON/color_code.json" ]]; then
  col="$(python3 -c "import json; print(json.load(open('$SRC_JSON/color_code.json')).get('color_code','') or '')" 2>/dev/null || true)"
  case "$col" in
    green|red|black)
      tmp="$(mktemp)"
      printf '%s\n' "{\"target\": {\"color\": \"$col\", \"timestamp\": \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\"}}" > "$tmp"
      mv -f "$tmp" "$DST_JSON/target_buoy.json"
      echo "已根据 color_code 更新 target_buoy.json: $col"
      ;;
    *)
      echo "注意: GCS 中 color 为 '$col'，若为目标色请使用 green/red/black（小写）以便 target_buoy 识别。"
      ;;
  esac
fi

echo "已同步到: $DST_JSON"
echo "请执行: cd $(cd "$(dirname "$0")/.." && pwd) && colcon build --packages-select workspace_nav --merge-install"

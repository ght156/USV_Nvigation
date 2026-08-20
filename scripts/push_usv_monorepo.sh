#!/usr/bin/env bash
# push_usv_monorepo.sh — 把导航侧的改动同步推送到团队主仓
# （http://192.168.51.6/wxhikrobot/usv），不动别人的部分。
#
# 做法（吸取 2026-08-20 全量 rsync 误伤教训，选择性同步）：
#   1. m_common：只同步导航侧负责的接口文件白名单（新增围栏/任务接口），
#      其余文件（同事的 MQTT/decision 工作）一律不碰；
#   2. src/USV_NAV：默认只做差异报告（主仓可能更新，如实船调参），
#      确认本地更新时才加 --sync-usv-nav 推送；
#   3. 从最新 main 拉同步分支 → 提交 → 推分支，按输出链接开 MR。
#
# 用法：
#   scripts/push_usv_monorepo.sh ["提交说明"] [--sync-usv-nav]
# 环境变量：
#   USV_MONO_DIR  主仓克隆位置（默认 ~/usv_main_repo）

set -euo pipefail

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MONO_DIR="${USV_MONO_DIR:-$HOME/usv_main_repo}"
MSG="${1:-sync(m_common): 导航侧接口更新}"
SYNC_USV_NAV=0
for arg in "$@"; do
  [[ "$arg" == "--sync-usv-nav" ]] && SYNC_USV_NAV=1
done

REMOTE_URL="$(git -C "$WS_ROOT" config remote.usv.url || true)"
[[ -n "$REMOTE_URL" ]] || { echo "错误：本仓库未配置 usv 远端" >&2; exit 1; }

# 导航侧负责的 m_common 接口文件（新增/修改只通过这里进主仓）
SYNC_FILES=(
  src/m_common/action/NavigateTask.action
  src/m_common/msg/GeoPolygon.msg
  src/m_common/msg/NavZones.msg
  src/m_common/msg/GeoPoint.msg
  src/m_common/msg/GeoFence.msg
  src/m_common/msg/NavSafetyEvent.msg
  src/m_common/msg/MissionWaypoint.msg
  src/m_common/msg/NavStatus.msg
  src/m_common/srv/SetNavZones.srv
  src/m_common/srv/GetNavZones.srv
  src/m_common/srv/SetGeoFence.srv
  src/m_common/srv/GetGeoFence.srv
  src/m_common/urdf/usv_cf.xacro
)

echo "== 1/5 克隆/更新主仓: $MONO_DIR"
if [[ ! -d "$MONO_DIR/.git" ]]; then
  git clone --single-branch -b main "$REMOTE_URL" "$MONO_DIR"
fi
git -C "$MONO_DIR" fetch origin main

BRANCH="nav/sync-$(date +%Y%m%d-%H%M)"
echo "== 2/5 从 origin/main 拉分支: $BRANCH"
git -C "$MONO_DIR" checkout -B "$BRANCH" origin/main

echo "== 3/5 同步 m_common 白名单文件"
for f in "${SYNC_FILES[@]}"; do
  if [[ -f "$WS_ROOT/$f" ]]; then
    mkdir -p "$MONO_DIR/$(dirname "$f")"
    cp "$WS_ROOT/$f" "$MONO_DIR/$f"
  fi
done
# 历史遗留：误传到 msg/ 下的两个 srv，若存在则移除（正确位置在 srv/）
for f in src/m_common/msg/SetNavZones.srv src/m_common/msg/GetNavZones.srv; do
  [[ -f "$MONO_DIR/$f" ]] && git -C "$MONO_DIR" rm -q "$f" || true
done
# rosidl 清单检查：主仓 CMakeLists 缺条目时给出提示（不自动改，避免误伤）
for f in "${SYNC_FILES[@]}"; do
  base="$(basename "$f")"
  if [[ -f "$MONO_DIR/$f" ]] && ! grep -q "\"$base\"" "$MONO_DIR/src/m_common/CMakeLists.txt"; then
    echo "  提示：主仓 CMakeLists.txt 缺少 \"$base\" 条目，请手动加入 rosidl_generate_interfaces" >&2
  fi
done

if [[ "$SYNC_USV_NAV" == "1" ]]; then
  echo "== 4/5 同步 src/USV_NAV（rsync，不带 --delete）"
  rsync -a --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
        --exclude='*.zip' --exclude='*.tar.gz' \
        "$WS_ROOT/src/USV_NAV/" "$MONO_DIR/src/USV_NAV/"
else
  echo "== 4/5 src/USV_NAV 差异报告（默认不同步；加 --sync-usv-nav 才推送）"
  TMP_SYNC="$(mktemp -d)"
  rsync -a --delete --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
        --exclude='*.zip' --exclude='*.tar.gz' \
        "$MONO_DIR/src/USV_NAV/" "$TMP_SYNC/main/"
  rsync -a --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
        --exclude='*.zip' --exclude='*.tar.gz' \
        "$WS_ROOT/src/USV_NAV/" "$TMP_SYNC/local/"
  if diff -rq "$TMP_SYNC/main" "$TMP_SYNC/local" > /dev/null 2>&1; then
    echo "  本地与主仓 src/USV_NAV 一致"
  else
    echo "  存在差异的文件："
    diff -rq "$TMP_SYNC/main" "$TMP_SYNC/local" | head -20 | sed 's/^/    /'
    echo "  注意确认哪边更新（主仓可能有实船调参），再决定是否加 --sync-usv-nav"
  fi
  rm -rf "$TMP_SYNC"
fi

echo "== 5/5 提交并推送"
git -C "$MONO_DIR" add src/m_common src/USV_NAV
if git -C "$MONO_DIR" diff --cached --quiet; then
  echo "与主仓 main 无差异，无需推送。"
  exit 0
fi
git -C "$MONO_DIR" --no-pager diff --cached --stat | tail -5
git -C "$MONO_DIR" commit -m "$MSG"
git -C "$MONO_DIR" push -u origin "$BRANCH"

echo
echo "完成。请到 GitLab 开 MR 合入 main："
echo "  http://192.168.51.6/wxhikrobot/usv/-/merge_requests/new?merge_request%5Bsource_branch%5D=${BRANCH}"
echo "提示：开 MR 前看一眼 diff，确认没有误删/误改同事的文件。"

#!/usr/bin/env bash
# Run dock_mission pytest suite after sourcing ROS 2 Humble.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_DIR}/../.." && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "ERROR: /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "${WS_ROOT}/install/setup.bash"
  set -u
fi

cd "${PKG_DIR}"
# anyio 与 pytest 版本不兼容，必须禁用插件自动加载
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
exec python3 -m pytest test/ -v "$@"

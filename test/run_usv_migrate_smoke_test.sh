#!/usr/bin/env bash
# 实船 mission 迁移 mock 冒烟测试（在仿真工作区执行，source 实船 USV_NAV install）
set -eo pipefail

SIM_WS="$(cd "$(dirname "$0")/.." && pwd)"
USV_WS="${USV_WS:-/home/ght/USV_NAV}"

if [[ ! -f "${USV_WS}/install/setup.bash" ]]; then
  echo "ERROR: 请先编译实船工作区: cd ${USV_WS} && colcon build --packages-select m_common workspace_nav"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${USV_WS}/install/setup.bash"
set -u

export USV_WS
exec python3 "${SIM_WS}/test/usv_migrate_smoke_test.py" "$@"

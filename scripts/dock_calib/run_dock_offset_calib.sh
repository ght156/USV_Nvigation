#!/usr/bin/env bash
# 船端运行入口：加载 ROS 2 与兴飞安装空间后执行 dock_offset_calib.py。
# 用法：
#   ./run_dock_offset_calib.sh --print
#   ./run_dock_offset_calib.sh --dry-run
#   ./run_dock_offset_calib.sh --write
#   ./run_dock_offset_calib.sh --verify
# 注意：ROS 2 setup.bash 在 set -u 下会引用未定义变量，这里不要开 -u。
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
USV_INSTALL="${USV_INSTALL:-$HOME/usv-test/install/arm}"

if [[ -r "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  echo "[ERROR] 找不到 ROS 2 环境文件：$ROS_SETUP" >&2
  echo "        本机只想跑自测/看配置时，直接：python3 $SCRIPT_DIR/dock_offset_calib.py --self-test" >&2
  exit 1
fi

if [[ -r "$USV_INSTALL/local_setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$USV_INSTALL/local_setup.bash"
else
  echo "[WARN] 找不到兴飞安装空间：$USV_INSTALL/local_setup.bash" >&2
  echo "       如果只是查看配置可以继续；采样会找不到 m_common / usv_rs485_driver 服务。" >&2
fi

# 兴飞栈默认 ROS_DOMAIN_ID=5；保留调用者显式设置的值。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"

exec python3 "$SCRIPT_DIR/dock_offset_calib.py" "$@"

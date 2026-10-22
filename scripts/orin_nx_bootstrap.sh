#!/usr/bin/env bash
# =============================================================================
# orin_nx_bootstrap.sh —— 新 Orin NX 上的导航 / 归港 / 速度桥 / 地图工具依赖
#
# 只安装公共依赖、做自检，并生成 ~/usv_env.sh。
#
# 新机器推荐：
#   ./scripts/orin_nx_bootstrap.sh --install-ros
#
# 已有 ROS 2 Humble 的机器：
#   ./scripts/orin_nx_bootstrap.sh
#
# 只检查，不改系统：
#   ./scripts/orin_nx_bootstrap.sh --check-only
#
# 关键选项：
#   --install-ros         缺少 ROS 2 Humble 时，配置 ROS apt 源并安装 ros-base
#   --workspace PATH      本机工作区的环境文件位置，默认 ~/usv_nav_ws
#   --skip-python         不执行 pip 安装 utm
#   --check-only          只检查，不执行 sudo/apt/pip/文件写入
#   --help                显示帮助
#
# 说明：
#   * 全栈固定 FastDDS：ROS_DOMAIN_ID=5、RMW_IMPLEMENTATION=rmw_fastrtps_cpp。
#   * 地图工具网页本身不需要 Node/npm。NX 端依赖是 rosbridge_server；
#     PCD 转图时才从 CDN 懒加载 Three.js，网页不联网时加载已有 yaml/pgm 不受影响。
#   * MAVROS（含 GetMapOrigin）和 m_common 由同事提供，本脚本不检查、不安装。
#   * usv_localization、RTK、Livox 和感知/相机链同样由对应工作区提供。
# =============================================================================
set -Eeuo pipefail

ROS_DISTRO="humble"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE="${HOME}/usv_nav_ws"
INSTALL_ROS=0
SKIP_PYTHON=0
CHECK_ONLY=0

APT_PACKAGES=(
  build-essential
  ca-certificates
  cmake
  curl
  git
  gnupg
  iproute2
  jq
  lsb-release
  net-tools
  pkg-config
  python3-colcon-common-extensions
  python3-dev
  python3-numpy
  python3-pip
  python3-rosdep
  python3-setuptools
  python3-transforms3d
  python3-yaml
  rsync
  tmux
  unzip
  wget
)

ROS_PACKAGES=(
  "ros-${ROS_DISTRO}-ament-cmake"
  "ros-${ROS_DISTRO}-ament-cmake-python"
  "ros-${ROS_DISTRO}-geometry-msgs"
  "ros-${ROS_DISTRO}-nav-msgs"
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-robot-state-publisher"
  "ros-${ROS_DISTRO}-rosbridge-server"
  "ros-${ROS_DISTRO}-rmw-fastrtps-cpp"
  "ros-${ROS_DISTRO}-sensor-msgs"
  "ros-${ROS_DISTRO}-std-msgs"
  "ros-${ROS_DISTRO}-teleop-twist-keyboard"
  "ros-${ROS_DISTRO}-tf2-geometry-msgs"
  "ros-${ROS_DISTRO}-tf2-ros"
  "ros-${ROS_DISTRO}-tf2-tools"
  "ros-${ROS_DISTRO}-xacro"
)

if [[ -t 1 ]]; then
  C_INFO=$'\033[1;36m'
  C_WARN=$'\033[1;33m'
  C_ERROR=$'\033[1;31m'
  C_OK=$'\033[1;32m'
  C_RESET=$'\033[0m'
else
  C_INFO=""
  C_WARN=""
  C_ERROR=""
  C_OK=""
  C_RESET=""
fi

log() { printf '%s[orin-bootstrap]%s %s\n' "$C_INFO" "$C_RESET" "$*"; }
warn() { printf '%s[orin-bootstrap] WARN:%s %s\n' "$C_WARN" "$C_RESET" "$*" >&2; }
ok() { printf '%s[orin-bootstrap] OK:%s %s\n' "$C_OK" "$C_RESET" "$*"; }
die() { printf '%s[orin-bootstrap] ERROR:%s %s\n' "$C_ERROR" "$C_RESET" "$*" >&2; exit 1; }

usage() {
  sed -n '3,/^# =============================================================================$/p' \
    "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

need_value() {
  [[ $# -ge 2 && -n "${2:-}" ]] || die "选项 $1 缺少参数"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-ros) INSTALL_ROS=1; shift ;;
    --workspace)
      need_value "$@"
      WORKSPACE="$2"
      shift 2
      ;;
    --skip-python) SKIP_PYTHON=1; shift ;;
    --check-only) CHECK_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数：$1（使用 --help 查看用法）" ;;
  esac
done

WORKSPACE="${WORKSPACE/#\~/${HOME}}"

if [[ ${EUID} -eq 0 ]]; then
  die "请不要用 sudo/root 运行本脚本；脚本会在需要时自行调用 sudo"
fi

check_platform() {
  local os_id="" os_version="" arch
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    os_id="${ID:-}"
    os_version="${VERSION_ID:-}"
  fi

  if [[ "$os_id" != "ubuntu" || "$os_version" != "22.04" ]]; then
    die "目标系统必须是 Ubuntu 22.04 (${ROS_DISTRO} 支持范围)；当前：${os_id:-unknown} ${os_version:-unknown}"
  fi

  arch="$(uname -m)"
  if [[ "$arch" != "aarch64" ]]; then
    warn "当前架构是 ${arch}，本脚本的目标平台是 Orin NX (aarch64)；在开发机上只建议使用 --check-only"
  else
    ok "平台：Ubuntu ${os_version} / ${arch}"
  fi
}

ros_available() {
  [[ -r "$ROS_SETUP" ]]
}

configure_ros_apt_repo() {
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  local ros_list="/etc/apt/sources.list.d/ros2.list"
  local mirror="${ROS_APT_MIRROR:-http://packages.ros.org/ros2/ubuntu}"
  local codename arch

  log "安装 ROS 软件源前置工具"
  sudo apt-get update
  sudo apt-get install -y curl gnupg lsb-release software-properties-common
  sudo add-apt-repository universe -y

  if [[ ! -s "$keyring" ]]; then
    log "导入 ROS 2 apt 公钥"
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | sudo tee "$keyring" >/dev/null
    sudo chmod 0644 "$keyring"
  fi

  # shellcheck disable=SC1091
  source /etc/os-release
  codename="${UBUNTU_CODENAME:-jammy}"
  arch="$(dpkg --print-architecture)"
  printf 'deb [arch=%s signed-by=%s] %s %s main\n' \
    "$arch" "$keyring" "$mirror" "$codename" \
    | sudo tee "$ros_list" >/dev/null
  ok "ROS apt 源：${mirror} (${codename}/${arch})"
}

install_ros_if_requested() {
  if ros_available; then
    ok "已找到 ROS 2 ${ROS_DISTRO}：${ROS_SETUP}"
    return
  fi

  if [[ "$INSTALL_ROS" -ne 1 ]]; then
    die "未找到 ${ROS_SETUP}。新 Orin NX 可用 --install-ros 自动安装，或先手工安装 ROS 2 Humble"
  fi

  log "安装 ROS 2 ${ROS_DISTRO} ros-base（arm64）"
  configure_ros_apt_repo
  sudo apt-get update
  sudo apt-get install -y "ros-${ROS_DISTRO}-ros-base"
}

install_apt_dependencies() {
  log "更新 apt 索引"
  sudo apt-get update

  log "安装系统、编译、Python 与 ROS 依赖"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    "${APT_PACKAGES[@]}" "${ROS_PACKAGES[@]}"
}

install_python_dependencies() {
  if [[ "$SKIP_PYTHON" -eq 1 ]]; then
    warn "按 --skip-python 跳过 utm"
    return
  fi

  if python3 -c 'import utm' >/dev/null 2>&1; then
    ok "Python utm 已安装"
    return
  fi

  log "安装 Python utm（导航 waypoint / mission_bridge 必需）"
  python3 -m pip install --user --disable-pip-version-check "utm==0.8.1"
  python3 -c 'import utm' >/dev/null 2>&1 || die "utm 安装后仍无法 import"

  # Ubuntu 22.04 自带 transforms3d 0.3.1 与较新的 NumPy 不兼容；只在确实
  # import 失败时升级，避免无谓改动用户现有的 Python 环境。
  if ! python3 -c 'import transforms3d' >/dev/null 2>&1; then
    log "修复 Python transforms3d 兼容性"
    python3 -m pip install --user --disable-pip-version-check \
      --upgrade "transforms3d>=0.4.2"
    python3 -c 'import transforms3d' >/dev/null 2>&1 \
      || die "transforms3d 安装后仍无法 import"
  fi
}

write_env_file() {
  local env_file="${WORKSPACE}/usv_env.sh"
  mkdir -p "$WORKSPACE"
  if [[ -f "$env_file" ]]; then
    warn "保留已有环境文件：${env_file}"
    return
  fi

  {
    cat <<EOF
# 由 USV_NAV/scripts/orin_nx_bootstrap.sh 生成
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI

source ${ROS_SETUP}
if [ -f "${WORKSPACE}/install/arm/setup.bash" ]; then
  source "${WORKSPACE}/install/arm/setup.bash"
fi
EOF
  } > "$env_file"
  chmod 0644 "$env_file"
  ok "已生成 ${env_file}（不会被自动 source；需要时手工 source）"
}

ros_pkg_check() {
  local pkg="$1"
  (
    set +u
    source "$ROS_SETUP"
    ros2 pkg prefix "$pkg"
  ) >/dev/null 2>&1
}

check_dependencies() {
  local missing=0
  local cmd pkg module

  log "开始依赖自检"
  for cmd in colcon tmux rsync python3 sudo; do
    if command -v "$cmd" >/dev/null 2>&1; then
      ok "命令 ${cmd}"
    else
      warn "缺少命令：${cmd}"
      missing=$((missing + 1))
    fi
  done

  if ros_available; then
    ok "ROS 2 ${ROS_DISTRO}：${ROS_SETUP}"
    for pkg in \
      nav2_bringup \
      rosbridge_server \
      rmw_fastrtps_cpp \
      tf2_ros \
      tf2_geometry_msgs \
      robot_state_publisher \
      xacro; do
      if ros_pkg_check "$pkg"; then
        ok "ROS 包 ${pkg}"
      else
        warn "缺少 ROS 包：${pkg}"
        missing=$((missing + 1))
      fi
    done
  else
    warn "缺少 ROS 2 ${ROS_DISTRO}：${ROS_SETUP}"
    missing=$((missing + 1))
  fi

  for module in yaml numpy transforms3d utm; do
    if python3 -c "import ${module}" >/dev/null 2>&1; then
      ok "Python 模块 ${module}"
    else
      warn "缺少 Python 模块：${module}"
      missing=$((missing + 1))
    fi
  done

  if [[ "$missing" -eq 0 ]]; then
    ok "全部必需依赖检查通过"
    return 0
  fi

  warn "共发现 ${missing} 项缺失"
  return 1
}

print_next_steps() {
  cat <<EOF

下一步：
  1) 加载环境：
       source ${WORKSPACE}/usv_env.sh

  2) 本脚本不处理以下组件：
       MAVROS / mavros_msgs(GetMapOrigin)
       m_common
       ~/usv             整船代码、RTK bw_gi320_driver
       Livox 驱动        MID360 / MID360s
       感知链            AprilTag / 后视相机

  3) 本脚本只安装依赖，不复制源码，也不执行 colcon build。

  地图工具：
    - 本次已安装 rosbridge_server，地址使用 ws://<NX_IP>:9090。
    - 网页无需安装 Node/npm；可在 ${HOME}/ros_map_tool 下用
        python3 -m http.server 8001 --bind 0.0.0.0
      临时提供页面。
EOF
}

main() {
  check_platform

  if [[ "$CHECK_ONLY" -eq 1 ]]; then
    check_dependencies
    print_next_steps
    exit $?
  fi

  command -v sudo >/dev/null 2>&1 || die "缺少 sudo"
  sudo -v

  install_ros_if_requested
  install_apt_dependencies
  install_python_dependencies

  mkdir -p "$WORKSPACE"
  write_env_file

  check_dependencies
  print_next_steps
}

main "$@"

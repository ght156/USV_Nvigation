#!/usr/bin/env bash
# 确保 gi320 / rs485 / mavros 三个驱动在跑（幂等，可重复执行）
export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="eth1"/></Interfaces></General></Domain></CycloneDDS>'
set +u
source /opt/ros/humble/setup.bash
source "$HOME/usv-main/install/arm/setup.bash"
set -u

# XML 内含双引号，嵌套 bash -c 会被剥掉导致 CycloneDDS 解析失败；写成 env 文件再 source
ENV_FILE="$HOME/bridge_calib/.cyclone_env"
mkdir -p "$HOME/bridge_calib"
printf "export CYCLONEDDS_URI='%s'\n" "$CYCLONEDDS_URI" > "$ENV_FILE"

ensure() { # name | pattern | cmd...
  local name="$1" pat="$2"; shift 2
  if pgrep -f "$pat" >/dev/null 2>&1; then
    echo "SKIP $name 已在运行"
  else
    setsid nohup bash -c "export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source $ENV_FILE; source /opt/ros/humble/setup.bash; source $HOME/usv-main/install/arm/setup.bash; exec $*" \
      > "$HOME/bridge_calib/${name}.log" 2>&1 < /dev/null &
    echo "OK   $name 已启动"
  fi
}

ensure gi320 'bw_gi320_driver.launch.py' \
  "ros2 launch bw_gi320_driver bw_gi320_driver.launch.py"
ensure rs485 'usv_rs485_driver.launch.py' \
  "ros2 launch usv_rs485_driver usv_rs485_driver.launch.py"
ensure mavros 'mavros apm.launch' \
  "ros2 launch mavros apm.launch fcu_url:=udp://:15001@192.168.2.14:15001"

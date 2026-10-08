#!/bin/bash
# 编译 set_lidar_ip —— 给 Livox MID-360 / MID-360S 改 IP 的独立小工具（不依赖 ROS）。
#
# 本机（Livox SDK2 装在 /usr/local）：
#   bash scripts/livox/build_set_lidar_ip.sh          # 产出 ./set_lidar_ip（x86-64）
#
# 船端 NX（在 docker 里编 ARM64，用自己编的 SDK）：
#   docker run --rm -v $HOME/ws_livox:/ws -w /ws/src/livox_ros_driver2/tools \
#     usv-runtime:jp6-cu126-humble bash -lc \
#     'SDK_PREFIX=/ws/sdk_install OUT=set_lidar_ip_arm bash build_set_lidar_ip.sh'
set -e
cd "$(dirname "$0")"

SDK_PREFIX="${SDK_PREFIX:-/usr/local}"
OUT="${OUT:-set_lidar_ip}"

g++ -std=c++14 -O2 -o "$OUT" set_lidar_ip.cpp \
    -I"$SDK_PREFIX/include" -L"$SDK_PREFIX/lib" \
    -llivox_lidar_sdk_shared -Wl,-rpath,"$SDK_PREFIX/lib" -lpthread

echo "built: $(pwd)/$OUT"

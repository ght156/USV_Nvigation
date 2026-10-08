#!/usr/bin/env bash
# 后视相机曝光模式切换（背光 tag 场景）
# 用法:
#   cam_backlight_mode.sh on    # 手动短曝光（背光板点亮时，黑底亮 tag，抗逆光）
#   cam_backlight_mode.sh off   # 恢复自动曝光
#   cam_backlight_mode.sh       # 查看当前值
# 注意:
#   1. 设备号不固定！按 /sys 里的设备名自动找 "USB Camera"（2026-10-06：重启后
#      后视相机从 video0 变成 video8，写错设备会污染 Orbbec 深度相机）
#   2. exposure_auto=3(自动) 下直接写 exposure_absolute 会 EACCES，必须先切手动
#   3. 设置不持久，相机重连/船重启后需重设
set -u

DEV=""
for v in /sys/class/video4linux/video*/name; do
  if grep -q "USB Camera" "$v" 2>/dev/null; then
    DEV="/dev/$(basename "$(dirname "$v")")"
    break
  fi
done
if [[ -z "$DEV" ]]; then
  echo "找不到 USB Camera 设备"; exit 1
fi
echo "后视相机: $DEV"

set_ctrl() { # cid val
  python3 - "$DEV" "$1" "$2" <<'PYEOF'
import fcntl, os, struct, sys
dev, cid, val = sys.argv[1], int(sys.argv[2], 0), int(sys.argv[3])
fd = os.open(dev, os.O_RDWR)
fcntl.ioctl(fd, 0xC008561C, struct.pack("Ii", cid, val))
out = fcntl.ioctl(fd, 0xC008561B, struct.pack("Ii", cid, 0), True)
print("  回读:", struct.unpack("Ii", out)[1])
os.close(fd)
PYEOF
}

EXPOSURE_AUTO=0x009A0901
EXPOSURE_ABS=0x009A0902

case "${1:-}" in
  on)
    echo "切手动 + 短曝光 ${2:-50}:"
    set_ctrl $EXPOSURE_AUTO 1
    set_ctrl $EXPOSURE_ABS "${2:-50}"
    ;;
  off)
    echo "恢复自动曝光:"
    set_ctrl $EXPOSURE_AUTO 3
    ;;
  *)
    echo "当前值:"
    set_ctrl $EXPOSURE_AUTO 0 2>/dev/null || true
    python3 - "$DEV" <<'PYEOF'
import fcntl, os, struct, sys
fd = os.open(sys.argv[1], os.O_RDWR)
for name, cid in [("exposure_auto", 0x009A0901), ("exposure_absolute", 0x009A0902), ("gain", 0x00980913)]:
    try:
        out = fcntl.ioctl(fd, 0xC008561B, struct.pack("Ii", cid, 0), True)
        print(f"  {name} = {struct.unpack('Ii', out)[1]}")
    except OSError as e:
        print(f"  {name} = <errno {e.errno}>")
os.close(fd)
PYEOF
    ;;
esac

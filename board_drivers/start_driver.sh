#!/usr/bin/env bash
set -euo pipefail

if [ "$USER" != "root" ];then
    echo "Error: Require root"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Enabling registers for IO boards..."
setpci -d 1221:8605 command=001
setpci -d 1221:8623 command=001
#setpci -d 1221:86d3 command=001 AOボード16ch
setpci -d 1221:86c3 command=001 #AOボード8ch

echo "Loading board driver modules..."
# modprobe AI1616LLPE/AI1616L
# modprobe AO1608LLPE/AO1608L
insmod "${SCRIPT_DIR}/AI1616LLPE/AI1616L.ko"
insmod "${SCRIPT_DIR}/AO1608LLPE/AO1608L.ko"
#insmod /home/kklab/board_drivers/CNT3204MTLPE/CNT3204MT.ko カウンタボード

echo "Creating device files..."
rm -f /dev/AI /dev/AO
mknod /dev/AI c 85 0
mknod /dev/AO c 200 0
#mknod /dev/CNT c 87 0

chmod 666 /dev/AI
chmod 666 /dev/AO
#chmod 666 /dev/CNT

echo "Board driver initialized."

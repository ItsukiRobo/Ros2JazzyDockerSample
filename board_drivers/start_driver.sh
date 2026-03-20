#!/usr/bin/env bash
set -euo pipefail

if [ "$USER" != "root" ];then
    echo "Error: Require root"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

load_module_if_needed() {
    local module_name="$1"
    local module_path="$2"

    if lsmod | awk '{print $1}' | grep -Fxq "$module_name"; then
        echo "${module_name} is already loaded. Skipping."
        return 0
    fi

    insmod "$module_path"
}

echo "Enabling registers for IO boards..."
setpci -d 1221:8605 command=001
setpci -d 1221:8623 command=001
#setpci -d 1221:86d3 command=001 AOボード16ch
setpci -d 1221:86c3 command=001 #AOボード8ch

echo "Loading board driver modules..."
# modprobe AI1616LLPE/AI1616L
# modprobe AO1608LLPE/AO1608L
load_module_if_needed "AI1616L" "${SCRIPT_DIR}/AI1616LLPE/AI1616L.ko"
load_module_if_needed "AO1608L" "${SCRIPT_DIR}/AO1608LLPE/AO1608L.ko"
load_module_if_needed "CNT3204MT" "${SCRIPT_DIR}/CNT3204MTLPE/CNT3204MT.ko"

echo "Creating device files..."
rm -f /dev/AI /dev/AO /dev/CNT
mknod /dev/AI c 85 0
mknod /dev/AO c 200 0
mknod /dev/CNT c 87 0

chmod 666 /dev/AI
chmod 666 /dev/AO
chmod 666 /dev/CNT

echo "Board driver initialized."

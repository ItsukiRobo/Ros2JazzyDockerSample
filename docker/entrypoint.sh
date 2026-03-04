#!/usr/bin/env bash
set -Eeuo pipefail
export TZ=Asia/Tokyo

# setup.bash は未定義変数を参照することがあるので、source時だけ nounset を外す
set +u
source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
set -u

BRINGUP_PKG="sample_project_bringup"
BRINGUP_LAUNCH="bringup.launch.py"

ARTIFACTS_ROOT="${ARTIFACTS_ROOT:-/artifacts}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${ARTIFACTS_ROOT}/${RUN_ID}"

mkdir -p "${OUT_DIR}"

LAUNCH_PID=""
BAG_ALL_PID=""
BAG_STD_PID=""

STD_TOPICS=(
  /tf
  /tf_static
  /clock
  /parameter_events
  /rosout
)

######### 拡張機能 ##########
# Paramter保存の有効/無効（デフォルトは有効）
ENABLE_PARAMS_DUMP="${ENABLE_PARAMS_DUMP:-1}"
# rosbag2_std -> ros1bag 変換の有効/無効（デフォルトは有効）
ENABLE_ROS1_CONVERT="${ENABLE_ROS1_CONVERT:-1}"
#############################

is_true() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

generate_std_topics() {
  # どの msg package を std とみなすか（空白区切り）
  local allow_pkgs="${STD_ALLOW_PKGS:-std_msgs geometry_msgs sensor_msgs nav_msgs tf2_msgs diagnostic_msgs rcl_interfaces builtin_interfaces trajectory_msgs}"
  local out_txt="${1:?out_txt required}"

  echo "[entrypoint] generating STD_TOPICS from live graph..."
  echo "[entrypoint] allow msg packages: ${allow_pkgs}"

  # topic list -t が取れるまで少し待つ（起動直後は空のことがある）
  local tries=15
  local sleep_s=0.3
  local tmp="/tmp/topic_list_t.txt"
  : > "${tmp}"

  for i in $(seq 1 "${tries}"); do
    if ros2 topic list -t > "${tmp}" 2>/dev/null; then
      if grep -q "\[" "${tmp}"; then
        break
      fi
    fi
    sleep "${sleep_s}"
  done

  # Pythonで確実にパースし、allowlistでフィルタして出力
  python3 - <<'PY' "${tmp}" "${out_txt}" "${allow_pkgs}"
import sys, re
src = sys.argv[1]
dst = sys.argv[2]
allow = set(sys.argv[3].split())

# ros2 topic list -t の典型:
# /rosout [rcl_interfaces/msg/Log]
# /tf [tf2_msgs/msg/TFMessage]
topic_re = re.compile(r'^\s*(\S+)\s+\[([^\]]+)\]\s*$')

topics = []
with open(src, "r", encoding="utf-8", errors="ignore") as f:
  for line in f:
    m = topic_re.match(line.strip())
    if not m:
      continue
    topic = m.group(1)
    types = [t.strip() for t in m.group(2).split(",") if t.strip()]
    # 1つでも allow package の型を含めば採用（通常は型は1つ）
    ok = False
    for t in types:
      # "pkg/msg/Type" を想定
      pkg = t.split("/", 1)[0]
      if pkg in allow:
        ok = True
        break
    if ok:
      topics.append(topic)

# ノイズを減らしたければここで除外したいトピックを追加できる
# 例: topics = [t for t in topics if not t.startswith("/_")]

# 重複排除＆ソート
topics = sorted(set(topics))

with open(dst, "w", encoding="utf-8") as f:
  for t in topics:
    f.write(t + "\n")

print(f"[entrypoint] STD topics selected: {len(topics)}")
for t in topics[:30]:
  print("  " + t)
if len(topics) > 30:
  print("  ...")
PY

  # フォールバック（空なら最低限）
  if [[ ! -s "${out_txt}" ]]; then
    echo "[entrypoint] WARNING: STD topics empty; using fallback set"
    cat > "${out_txt}" <<'EOF'
/rosout
/parameter_events
/tf
/tf_static
EOF
  fi

  # 読み込んで配列へ（ros2 bag record に渡すため）
  mapfile -t STD_TOPICS < "${out_txt}"
  echo "[entrypoint] STD_TOPICS loaded: ${#STD_TOPICS[@]}"
}

fix_artifacts_owner() {
  # ホストのUID/GIDを環境変数で受け取る（無ければ何もしない）
  if [[ -z "${HOST_UID:-}" || -z "${HOST_GID:-}" ]]; then
    # echo "[entrypoint] HOST_UID/HOST_GID not set -> skip chown"
    return 0
  fi

  if [[ "$(id -u)" != "0" ]]; then
    # echo "[entrypoint] not running as root -> skip chown"
    return 0
  fi

  echo "[entrypoint] chown artifacts to ${HOST_UID}:${HOST_GID}"
  chown -R "${HOST_UID}:${HOST_GID}" "${OUT_DIR}" 2>/dev/null || true
}

cleanup() {
  if [[ "${CLEANUP_DONE:-0}" == "1" ]]; then
    return
  fi
  CLEANUP_DONE=1

  echo "[entrypoint] cleanup entered"

  # 1) まず SIGINT
  for pid in "${BAG_STD_PID:-}" "${BAG_ALL_PID:-}" "${LAUNCH_PID:-}"; do
    [[ -n "${pid}" ]] && kill -INT "${pid}" 2>/dev/null || true
  done

  # 2) 最大10秒だけ待つ
  deadline=$((SECONDS + 10))
  for pid in "${BAG_STD_PID:-}" "${BAG_ALL_PID:-}" "${LAUNCH_PID:-}"; do
    [[ -z "${pid}" ]] && continue
    while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
      sleep 0.2
    done
  done

  # 3) まだ生きてたら SIGTERM
  for pid in "${BAG_STD_PID:-}" "${BAG_ALL_PID:-}" "${LAUNCH_PID:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      echo "[entrypoint] pid ${pid} still alive -> SIGTERM"
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  # 4) さらに最大5秒待つ
  deadline=$((SECONDS + 5))
  for pid in "${BAG_STD_PID:-}" "${BAG_ALL_PID:-}" "${LAUNCH_PID:-}"; do
    [[ -z "${pid}" ]] && continue
    while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
      sleep 0.2
    done
  done

  # 5) それでも生きてたら SIGKILL
  for pid in "${BAG_STD_PID:-}" "${BAG_ALL_PID:-}" "${LAUNCH_PID:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      echo "[entrypoint] pid ${pid} still alive -> SIGKILL"
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done

  # 6) wait は「可能なら」回収（ここでブロックしない）
  for pid in "${BAG_STD_PID:-}" "${BAG_ALL_PID:-}" "${LAUNCH_PID:-}"; do
    [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null || true
  done

  if is_true "${ENABLE_ROS1_CONVERT}"; then
    # 7) rosbag2_std -> ros1bag 変換
    echo "[entrypoint] converting rosbag2_std -> ros1bag..."
    mkdir -p "${OUT_DIR}/ros1bag"

    # 変換に使った入力情報も artifacts に残す（後で再現しやすい）
    echo "${STD_TOPICS[@]}" | tr ' ' '\n' > "${OUT_DIR}/std_topics_used.txt"

    set +e
    rosbags-convert --src "${OUT_DIR}/rosbag2_std/run" \
      --dst "${OUT_DIR}/ros1bag/run.bag" \
      > "${OUT_DIR}/convert_rosbags_stdout.log" \
      2> "${OUT_DIR}/convert_rosbags_stderr.log"
    RC=$?
    set -e

    echo "[entrypoint] rosbags-convert exit code: ${RC}"
    if [[ "${RC}" -ne 0 ]]; then
      echo "[entrypoint] WARNING: conversion failed. See convert_rosbags_stderr.log"
    fi
  else
    echo "[entrypoint] ros1 conversion disabled"
  fi

  # 8) artifacts の所有者をホストのユーザに変更
  fix_artifacts_owner

  echo "[entrypoint] cleanup finished"
}

# INT/TERM で cleanup → exit
on_signal() {
  echo "[entrypoint] signal received"
  cleanup
  exit 0
}

trap on_signal INT TERM
trap cleanup EXIT

echo "[entrypoint] PID1 is $$"

# bringup 起動
echo "[entrypoint] starting bringup..."
ros2 launch "${BRINGUP_PKG}" "${BRINGUP_LAUNCH}" &
LAUNCH_PID=$!

sleep 3

# params dump
if is_true "${ENABLE_PARAMS_DUMP}"; then
  echo "[entrypoint] dumping params..."
  python3 -m sample_project_tools.dump_params --out "${OUT_DIR}/params_dump.txt" || true
else
  echo "[entrypoint] params dump disabled"
fi

# bag record（全てのTopic)
echo "[entrypoint] recording rosbag2_all..."
mkdir -p "${OUT_DIR}/rosbag2_all"
ros2 bag record -a -o "${OUT_DIR}/rosbag2_all/run" &
BAG_ALL_PID=$!
# bag record (std_msgs などの標準的なTopicのみ)
echo "[entrypoint] recording rosbag2_std..."
mkdir -p "${OUT_DIR}/rosbag2_std"
# 生成結果を artifacts に残す（後で何を録ったか分かる）
STD_TOPICS_TXT="${OUT_DIR}/std_topics_used.txt"
generate_std_topics "${STD_TOPICS_TXT}"
# std は sqlite3 推奨（変換安定）
ros2 bag record -s sqlite3 -o "${OUT_DIR}/rosbag2_std/run" --topics "${STD_TOPICS[@]}" &
BAG_STD_PID=$!

while kill -0 "${LAUNCH_PID}" 2>/dev/null; do
  sleep 0.5
done

echo "[entrypoint] launch exited"
exit 0

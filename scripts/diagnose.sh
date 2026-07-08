#!/usr/bin/env bash
# diagnose.sh — one-shot health check for dev box + Jetson + DDS.
#
# Run from the dev box when:
#   - "nothing works" — find which layer is broken
#   - before a competition — confirm all green
#   - after a Jetson reboot — verify it came back up
#
# Usage:
#   bash scripts/diagnose.sh                 # full check (default)
#   bash scripts/diagnose.sh --no-remote     # dev box only, skip Jetson
#   bash scripts/diagnose.sh --target=192.168.3.69   # custom target
#   bash scripts/diagnose.sh --json         # machine-readable output
#   bash scripts/diagnose.sh --quiet        # only show failures
#
# Output is a 15-row check table + summary. Exit code:
#   0 = all pass
#   1 = one or more FAIL
#   2 = one or more WARN
set -euo pipefail

SCRIPT_NAME="diagnose.sh"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JETSON_HOST="${JETSON_HOST:-orin}"
TARGET="${TARGET_HOST:-}"
DO_REMOTE=1
JSON=0
QUIET=0
ROS_DISTRO=""

usage() {
  sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
  cat <<'EOF'

Options:
  --no-remote          Skip Jetson checks (dev box only)
  --target=HOST        Jetson host (default: $JETSON_HOST or 'orin')
  --json               Output as JSON
  -q, --quiet          Only show failures + summary
  -h, --help           Show this help

Environment:
  JETSON_HOST          default: orin
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-remote)   DO_REMOTE=0 ;;
    --target=*)     TARGET="${1#*=}"; JETSON_HOST="$TARGET" ;;
    --json)         JSON=1 ;;
    -q|--quiet)     QUIET=1 ;;
    -h|--help)      usage; exit 0 ;;
    *)              echo "[$SCRIPT_NAME] unknown arg: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

# Collect results: arrays of (id, name, status, detail)
declare -a R_ID=() R_NAME=() R_STATUS=() R_DETAIL=()
PASS=0; FAIL=0; WARN=0

# Source ROS if available (so we can call ros2 commands)
for d in humble jazzy lyrical iron; do
  if [[ -f "/opt/ros/${d}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${d}/setup.bash" >/dev/null 2>&1
    set -u
    ROS_DISTRO="$d"
    break
  fi
done
[[ -z "$ROS_DISTRO" ]] && ROS_DISTRO="none"

# ─── helpers ──────────────────────────────────────────────────────────────
add_result() {
  R_ID+=("$1"); R_NAME+=("$2"); R_STATUS+=("$3"); R_DETAIL+=("$4")
  case "$3" in
    PASS) PASS=$((PASS+1)) ;;
    FAIL) FAIL=$((FAIL+1)) ;;
    WARN) WARN=$((WARN+1)) ;;
  esac
}

remote() {
  if [[ $DO_REMOTE -eq 0 ]]; then
    return 0
  fi
  if [[ -z "$TARGET" ]]; then
    ssh -o ConnectTimeout=5 -o BatchMode=yes "$JETSON_HOST" "$@" 2>&1
  else
    ssh -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" "$@" 2>&1
  fi
}

# ─── 4 dev-box checks ─────────────────────────────────────────────────────
check_dev() {
  # 01: cyclonedds config
  if [[ -f "${HOME}/.ros/cyclonedds.xml" ]]; then
    add_result "01" "dev:cyclonedds.xml" "PASS" "exists"
  else
    add_result "01" "dev:cyclonedds.xml" "WARN" "missing — run: bash scripts/start_team_rviz.sh"
  fi

  # 02: ROS distro
  if [[ "$ROS_DISTRO" != "none" ]]; then
    add_result "02" "dev:ros_distro" "PASS" "$ROS_DISTRO"
  else
    add_result "02" "dev:ros_distro" "FAIL" "no /opt/ros/<distro> — see docs/development/dev-machine-setup.md"
  fi

  # 03: ROS_DOMAIN_ID
  if [[ "${ROS_DOMAIN_ID:-}" == "42" ]]; then
    add_result "03" "dev:domain_id" "PASS" "42"
  else
    add_result "03" "dev:domain_id" "FAIL" "got '${ROS_DOMAIN_ID:-<unset>}' (need 42)"
  fi

  # 04: SSH passwordless
  if ssh -o ConnectTimeout=3 -o BatchMode=yes "$JETSON_HOST" true 2>/dev/null; then
    add_result "04" "ssh:passwordless" "PASS" "ok"
  else
    add_result "04" "ssh:passwordless" "FAIL" "run: bash scripts/setup_ssh_key.sh"
  fi
}

# ─── 7 Jetson checks ─────────────────────────────────────────────────────
check_jetson() {
  if [[ $DO_REMOTE -eq 0 ]]; then
    return 0
  fi

  # 05: orin user
  local who
  who=$(remote "whoami" 2>/dev/null | head -1 | tr -d '\r' || true)
  if [[ -n "$who" ]]; then
    add_result "05" "jetson:user" "PASS" "$who"
  else
    add_result "05" "jetson:user" "FAIL" "ssh failed"
    return 0
  fi

  # 06: Humble on Jetson
  if remote "test -f /opt/ros/humble/setup.bash" >/dev/null 2>&1; then
    add_result "06" "jetson:ros_humble" "PASS" "installed"
  else
    add_result "06" "jetson:ros_humble" "FAIL" "/opt/ros/humble/setup.bash missing"
  fi

  # 07: colcon
  if remote "command -v colcon" >/dev/null 2>&1; then
    add_result "07" "jetson:colcon" "PASS" "$(remote "colcon --version" 2>/dev/null | head -1 | tr -d '\r' || true)"
  else
    add_result "07" "jetson:colcon" "FAIL" "colcon not on PATH"
  fi

  # 08: workspace built
  if remote "test -f ~/ros2_ws/install/setup.bash" >/dev/null 2>&1; then
    add_result "08" "jetson:workspace" "PASS" "built"
  else
    add_result "08" "jetson:workspace" "WARN" "not built — ssh orin 'cd ros2_ws && colcon build'"
  fi

  # 09: full_system running
  if remote "pgrep -f full_system.launch.py" >/dev/null 2>&1; then
    add_result "09" "jetson:sidecar" "PASS" "running"
  else
    add_result "09" "jetson:sidecar" "WARN" "not running — ssh orin 'ros2 launch vehicle_wbt_platform_cpp full_system.launch.py ...'"
  fi

  # 10: Jetson disk
  local disk
  disk=$(remote "df -h / | tail -1 | awk '{print \$4}'" 2>/dev/null | tr -d '\r' || true)
  if [[ -n "$disk" ]]; then
    add_result "10" "jetson:disk_free" "PASS" "${disk} free"
  else
    add_result "10" "jetson:disk_free" "WARN" "could not query"
  fi

  # 11: Jetson memory
  local mem
  mem=$(remote "free -m | awk '/Mem:/ {print \$7}'" 2>/dev/null | tr -d '\r' || true)
  if [[ -n "$mem" && "$mem" -gt 500 ]]; then
    add_result "11" "jetson:mem_free" "PASS" "${mem}MB"
  elif [[ -n "$mem" ]]; then
    add_result "11" "jetson:mem_free" "WARN" "only ${mem}MB free"
  else
    add_result "11" "jetson:mem_free" "WARN" "could not query"
  fi
}

# ─── 4 DDS / network checks ──────────────────────────────────────────────
check_dds() {
  if [[ "$ROS_DISTRO" == "none" ]]; then
    add_result "12" "dds:node_list" "WARN" "ros2 not sourced — skipping"
    add_result "13" "dds:image_topic" "WARN" "ros2 not sourced — skipping"
    add_result "14" "dds:tf_static" "WARN" "ros2 not sourced — skipping"
    add_result "15" "dds:ros2_daemon" "WARN" "ros2 not sourced — skipping"
    return 0
  fi

  # 12: dev sees Jetson nodes
  # ros2 daemon cache is often stale (returns empty even when DDS is working).
  # Cross-check with topic-level signals: if image_compressed is publishing,
  # the daemon cache is just stale — downgrade FAIL to WARN.
  if timeout 5 ros2 node list 2>/dev/null | grep -qE 'camera_(arm|front)'; then
    add_result "12" "dds:node_list" "PASS" "Jetson nodes visible"
  else
    add_result "12" "dds:node_list" "WARN" "daemon cache empty — run: ros2 daemon stop && ros2 daemon start (DDS may still work)"
  fi

  # 13: image_compressed publishing — this is the AUTHORITATIVE signal:
  # if we can measure ~30Hz, DDS is working regardless of daemon cache state.
  local rate
  rate=$(timeout 4 ros2 topic hz /vehicle_wbt/v1/sensors/camera/front/image_compressed 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1 | awk '{print $3}' || true)
  if [[ -n "$rate" && "${rate%.*}" -ge 5 ]]; then
    add_result "13" "dds:image_topic" "PASS" "${rate} Hz (DDS working)"
  elif [[ -n "$rate" ]]; then
    add_result "13" "dds:image_topic" "WARN" "low rate ${rate} Hz"
  else
    add_result "13" "dds:image_topic" "FAIL" "no message in 4s — check Jetson sidecar"
  fi

  # 14: tf_static has both camera optical frames
  local tf_check
  tf_check=$(timeout 4 ros2 topic echo --once /tf_static 2>/dev/null | grep -E "child_frame_id" | head -3 || true)
  local n_frames
  n_frames=$(echo "$tf_check" | grep -c "camera_optical_frame" || true)
  if [[ $n_frames -ge 2 ]]; then
    add_result "14" "dds:tf_static" "PASS" "$n_frames camera frames"
  elif [[ $n_frames -ge 1 ]]; then
    add_result "14" "dds:tf_static" "WARN" "only $n_frames camera frame"
  else
    add_result "14" "dds:tf_static" "FAIL" "no camera_optical_frame in /tf_static"
  fi

  # 15: ros2 daemon alive
  if ros2 daemon status 2>&1 | grep -q "running"; then
    add_result "15" "dds:ros2_daemon" "PASS" "running"
  else
    add_result "15" "dds:ros2_daemon" "WARN" "not running — results may be stale"
  fi
}

# ─── output ───────────────────────────────────────────────────────────────
emit_row() {
  local sym="❓"
  case "$3" in
    PASS) sym="✅" ;;
    FAIL) sym="❌" ;;
    WARN) sym="⚠️ " ;;
  esac
  printf "%s [%s] %-25s %s\n" "$sym" "$1" "$2" "$4"
}

emit_text() {
  echo "═══════════════════════════════════════════════════════════════"
  echo "  vehicle_wbt diagnose — $(date '+%Y-%m-%d %H:%M:%S')"
  echo "  target: ${TARGET:-$JETSON_HOST}    remote: $([[ $DO_REMOTE -eq 1 ]] && echo yes || echo no)"
  echo "═══════════════════════════════════════════════════════════════"
  for i in "${!R_ID[@]}"; do
    if [[ $QUIET -eq 1 && "${R_STATUS[$i]}" == "PASS" ]]; then continue; fi
    emit_row "${R_ID[$i]}" "${R_NAME[$i]}" "${R_STATUS[$i]}" "${R_DETAIL[$i]}"
  done
  echo ""
  echo "📊 Summary: $PASS pass / $FAIL fail / $WARN warn"
  if [[ $FAIL -gt 0 || $WARN -gt 0 ]]; then
    echo ""
    echo "🚨 Action items:"
    for i in "${!R_ID[@]}"; do
      case "${R_STATUS[$i]}" in
        FAIL|WARN) printf "   %s [%s] %s\n" "${R_STATUS[$i]}" "${R_ID[$i]}" "${R_DETAIL[$i]}" ;;
      esac
    done
  fi
  echo "═══════════════════════════════════════════════════════════════"
}

emit_json() {
  printf '{"timestamp":"%s","target":"%s","results":[' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${TARGET:-$JETSON_HOST}"
  for i in "${!R_ID[@]}"; do
    [[ $i -gt 0 ]] && printf ","
    printf '{"id":"%s","name":"%s","status":"%s","detail":"%s"}' \
      "${R_ID[$i]}" "${R_NAME[$i]}" "${R_STATUS[$i]}" "${R_DETAIL[$i]}"
  done
  printf '],"summary":{"pass":%d,"fail":%d,"warn":%d}}\n' "$PASS" "$FAIL" "$WARN"
}

# ─── main ────────────────────────────────────────────────────────────────
check_dev
check_jetson
check_dds

if [[ $JSON -eq 1 ]]; then
  emit_json
else
  emit_text
fi

[[ $FAIL -gt 0 ]] && exit 1
[[ $WARN -gt 0 ]] && exit 2
exit 0

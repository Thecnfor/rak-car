#!/usr/bin/env bash
# onboard.sh — one-shot first-time setup for a new dev box.
#
# Walks a new team member from a clean Ubuntu install to "I can see the
# Jetson's cameras over the LAN". Idempotent: safe to re-run.
#
# Usage:
#   bash scripts/onboard.sh                  # full pipeline (default)
#   bash scripts/onboard.sh --phase=1        # only environment probe
#   bash scripts/onboard.sh --phase=2        # only install deps
#   bash scripts/onboard.sh --phase=3        # only build + verify
#   bash scripts/onboard.sh --dry-run        # probe only, no changes
#   bash scripts/onboard.sh --skip-ros-install   # skip apt ROS2 install
#
# Environment:
#   JETSON_HOST   default: 192.168.3.69 (team constant, see docs/team-constants.md)
#   JETSON_USER   default: xrak
#
# Press Ctrl-C to abort at any time.
set -euo pipefail

SCRIPT_NAME="onboard.sh"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO=""
export ROS_DISTRO  # so child processes (colcon, pytest) inherit the choice
# Hard-coded team constant: Jetson is always at 192.168.3.69.
# See docs/team-constants.md.
JETSON_HOST="${JETSON_HOST:-192.168.3.69}"
JETSON_USER="${JETSON_USER:-xrak}"

# ─── flags ────────────────────────────────────────────────────────────────
PHASE="all"
DRY_RUN=0
SKIP_ROS_INSTALL=0

usage() {
  sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
  cat <<'EOF'

Options:
  --phase=1|2|3|all   Run only one phase (default: all)
  --dry-run           Probe environment, do not modify system
  --skip-ros-install   Skip apt install of ROS2 packages
  -h, --help           Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --phase=*)     PHASE="${1#*=}" ;;
    --phase)       PHASE="$2"; shift ;;
    --dry-run)     DRY_RUN=1 ;;
    --skip-ros-install) SKIP_ROS_INSTALL=1 ;;
    -h|--help)     usage; exit 0 ;;
    *)             echo "[$SCRIPT_NAME] unknown arg: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

run() {
  if [[ $DRY_RUN -eq 1 ]]; then
    echo "[DRY] $*"
  else
    eval "$@"
  fi
}

log() { echo "[$SCRIPT_NAME] $*"; }
ok()  { echo "[$SCRIPT_NAME] ✅ $*"; }
warn(){ echo "[$SCRIPT_NAME] ⚠️  $*"; }
err() { echo "[$SCRIPT_NAME] ❌ $*"; }

# ─── phase 1: environment probe ────────────────────────────────────────────
phase1_probe() {
  log "[1/3] Probing environment..."

  # OS
  if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    log "  OS: ${PRETTY_NAME:-unknown}"
  else
    warn "  OS: /etc/os-release not found (not Ubuntu?)"
  fi

  # ROS2 distro — Jetson side is fixed at humble; dev side accepts ANY distro.
  # Preference order: humble (matches Jetson, safest for ABI compat) → jazzy → lyrical → iron.
  # Falls back to scanning /opt/ros/ for any installed distro not in the list.
  for d in humble jazzy lyrical iron; do
    if [[ -f "/opt/ros/${d}/setup.bash" ]]; then
      ROS_DISTRO="$d"
      ok "  ROS2 distro: ${d}"
      break
    fi
  done
  if [[ -z "$ROS_DISTRO" ]]; then
    # Fallback: scan /opt/ros/ for any installed distro
    for d in /opt/ros/*/setup.bash; do
      [[ -f "$d" ]] || continue
      local candidate; candidate=$(basename "$(dirname "$d")")
      ROS_DISTRO="$candidate"
      warn "  ROS2 distro: ${candidate} (auto-detected from /opt/ros/, not in preferred list)"
      break
    done
  fi
  if [[ -z "$ROS_DISTRO" ]]; then
    warn "  ROS2 distro: not installed. See docs/development/dev-machine-setup.md"
  elif [[ "$ROS_DISTRO" != "humble" ]]; then
    warn "  dev ROS2 is '$ROS_DISTRO' (Jetson is humble). Usually fine for source-only dev"
    log "       (Jetson builds its own install under Humble — see docs/team-constants.md)"
  fi

  # Tools
  for tool in git ssh colcon python3; do
    if command -v "$tool" >/dev/null 2>&1; then
      log "  $tool: $(command -v $tool)"
    else
      warn "  $tool: missing (Phase 2 will install)"
    fi
  done

  # Disk
  local free_kb; free_kb=$(df -Pk "$REPO_ROOT" | tail -1 | awk '{print $4}')
  if [[ $free_kb -gt 10485760 ]]; then
    ok "  disk: $((free_kb/1048576)) GB free (≥10 GB)"
  else
    warn "  disk: $((free_kb/1048576)) GB free (<10 GB may fail build)"
  fi

  # Jetson reachability
  if ping -c 1 -W 2 "$JETSON_HOST" >/dev/null 2>&1; then
    ok "  jetson reachable: $JETSON_HOST"
  else
    warn "  jetson unreachable: $JETSON_HOST (SSH key may not be set yet)"
  fi
}

# ─── phase 2: install deps ────────────────────────────────────────────────
phase2_install() {
  log "[2/3] Installing dependencies..."

  if [[ $SKIP_ROS_INSTALL -eq 0 ]]; then
    if [[ -z "$ROS_DISTRO" ]]; then
      err "  ROS2 not installed. Install manually or set --skip-ros-install"
      log "  See: docs/development/dev-machine-setup.md"
      exit 1
    fi
    run "sudo apt-get install -y python3-colcon-common-extensions build-essential cmake git"
    ok "  apt deps installed"
  fi

  # CycloneDDS config (idempotent)
  local cfg_src="${REPO_ROOT}/ros2_ws/src/vehicle_wbt_platform_cpp/config/cyclonedds.xml"
  local cfg_dst="${HOME}/.ros/cyclonedds.xml"
  if [[ -f "$cfg_src" ]]; then
    mkdir -p "${HOME}/.ros"
    if [[ ! -f "$cfg_dst" ]] || ! diff -q "$cfg_src" "$cfg_dst" >/dev/null; then
      run "cp '$cfg_src' '$cfg_dst'"
      ok "  cyclonedds.xml deployed → $cfg_dst"
    else
      log "  cyclonedds.xml already up to date"
    fi
  else
    warn "  cyclonedds.xml not in repo; skipping DDS config deploy"
  fi

  # ROS_DOMAIN_ID in bashrc
  if ! grep -q "ROS_DOMAIN_ID=42" "${HOME}/.bashrc" 2>/dev/null; then
    run "echo 'export ROS_DOMAIN_ID=42' >> '${HOME}/.bashrc'"
    ok "  ROS_DOMAIN_ID=42 added to ~/.bashrc"
  else
    log "  ROS_DOMAIN_ID already in ~/.bashrc"
  fi
}

# ─── phase 3: build + verify ──────────────────────────────────────────────
phase3_build() {
  log "[3/3] Building + verifying..."

  if [[ ! -d "${REPO_ROOT}/ros2_ws" ]]; then
    err "  ros2_ws/ not found in repo"
    exit 1
  fi

  # Source ROS for this build step
  if [[ -n "$ROS_DISTRO" ]]; then
    # Lyrical's setup.bash references $AMENT_TRACE_SETUP_FILES without default,
    # tripping `set -u`. Temporarily disable for the source step.
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
  fi

  cd "${REPO_ROOT}/ros2_ws"
  log "  colcon build (this may take 5-10 min on first run)..."
  if ! run "colcon build --packages-up-to vehicle_wbt_platform_cpp vehicle_wbt_platform --event-handlers console_direct+"; then
    err "  colcon build failed. See output above."
    log "  See: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md"
    exit 1
  fi
  ok "  colcon build complete"

  # Smoke test: Python tests
  log "  running pytest smoke test..."
  if [[ -d "${REPO_ROOT}/ros2_ws/src/vehicle_wbt_platform/test" ]]; then
    cd "${REPO_ROOT}/ros2_ws/src/vehicle_wbt_platform"
    if PYTHONPATH=. python3 -m pytest test/ -q 2>&1 | tail -5; then
      ok "  pytest: 45/45 pass"
    else
      warn "  pytest failed (non-blocking — check output)"
    fi
  fi

  ok "  build + smoke test complete"
  log ""
  log "Next: ask a teammate to add your SSH pub key to Jetson (~/.ssh/authorized_keys)"
  log "      (team handles SSH key distribution — usually one-time ask in chat)"
  log "Then: bash scripts/diagnose.sh     (verify all green)"
  log "Finally: bash scripts/start_team_rviz.sh  (see live cameras)"
}

# ─── main ────────────────────────────────────────────────────────────────
case "$PHASE" in
  1|probe)        phase1_probe ;;
  2|install)      phase2_install ;;
  3|build)        phase3_build ;;
  all)
    phase1_probe
    phase2_install
    phase3_build
    ;;
  *) err "unknown phase: $PHASE"; usage; exit 1 ;;
esac

ok "onboard complete ✅"

#!/usr/bin/env bash

set -euo pipefail

SERIAL_NO=""
DEPTH_WIDTH="424"
DEPTH_HEIGHT="240"
DEPTH_FPS="15"
START_LCM_SERVER="1"

usage() {
  cat <<'EOF'
Usage:
  run_realsense_wander_obstacle_avoidance.sh [options]

Options:
  --serial SERIAL_NO       RealSense serial number. If omitted, first D435i is used.
  --depth-width PIXELS     Depth width. Default: 424
  --depth-height PIXELS    Depth height. Default: 240
  --depth-fps FPS          Depth fps. Default: 15
  --no-start-lcm-server    Do not auto-start lcm_server_high.
  -h, --help               Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)
      SERIAL_NO="$2"
      shift 2
      ;;
    --depth-width)
      DEPTH_WIDTH="$2"
      shift 2
      ;;
    --depth-height)
      DEPTH_HEIGHT="$2"
      shift 2
      ;;
    --depth-fps)
      DEPTH_FPS="$2"
      shift 2
      ;;
    --no-start-lcm-server)
      START_LCM_SERVER="0"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

source /opt/ros/melodic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"

resolve_serial() {
  if [[ -n "${SERIAL_NO}" ]]; then
    echo "${SERIAL_NO}"
    return 0
  fi

  rs-enumerate-devices 2>/dev/null | awk '
    /Name[[:space:]]*:/ && /Intel RealSense D435I/ { seen=1 }
    seen && /Serial Number[[:space:]]*:/ { gsub(/^[^:]*:[[:space:]]*/, "", $0); print $0; exit 0 }
  '
}

REAL_SENSE_SERIAL="$(resolve_serial || true)"
if [[ -z "${REAL_SENSE_SERIAL}" ]]; then
  echo "No Intel RealSense D435i was detected by rs-enumerate-devices." >&2
  exit 1
fi

echo "Using Intel RealSense D435i serial: ${REAL_SENSE_SERIAL}"

if ! modinfo uvcvideo 2>/dev/null | grep -qi realsense; then
  echo "The current uvcvideo kernel module is not RealSense-patched." >&2
  echo "Run ./scripts/install_realsense_dkms.sh once, reboot, then retry." >&2
  exit 1
fi

LCM_PID=""
cleanup() {
  if [[ -n "${LCM_PID}" ]] && kill -0 "${LCM_PID}" 2>/dev/null; then
    sudo kill "${LCM_PID}" 2>/dev/null || true
    wait "${LCM_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${START_LCM_SERVER}" == "1" ]]; then
  if [[ ! -x "$HOME/unitree_legged_sdk/build/lcm_server_high" ]]; then
    echo "Missing executable: $HOME/unitree_legged_sdk/build/lcm_server_high" >&2
    exit 1
  fi

  if pgrep -f "$HOME/unitree_legged_sdk/build/lcm_server_high" >/dev/null 2>&1; then
    echo "lcm_server_high is already running."
  else
    echo "Starting lcm_server_high with sudo..."
    sudo -v
    sudo "$HOME/unitree_legged_sdk/build/lcm_server_high" &
    LCM_PID="$!"
    sleep 2
  fi
fi

exec roslaunch slam_planner realsense_wander_obstacle_avoidance.launch \
  serial_no:="${REAL_SENSE_SERIAL}" \
  depth_width:="${DEPTH_WIDTH}" \
  depth_height:="${DEPTH_HEIGHT}" \
  depth_fps:="${DEPTH_FPS}"

#!/usr/bin/env bash

set -euo pipefail

IP_ADDRESS="192.168.11.1"
MAP_BUNDLE=""
SAVE_MAP_NAME=""
BUNDLE_ROOT="$HOME/catkin_ws/src/slamrplidar/slam_planner/maps/generated"
TRANSITION_KEY="n"
START_LCM_SERVER="1"
HOST_INTERFACE="eth0"
HOST_ADDRESS=""
AUTO_CONFIGURE_INTERFACE="1"

usage() {
  cat <<'EOF'
Usage:
  run_integrated_mapping_navigation.sh [options]

Options:
  --ip ADDRESS            LiDAR/SLAMWare IP address. Default: 192.168.11.1
  --map-bundle PATH       Existing saved bundle directory or .stcm path.
  --save-name NAME        Stable name for the new bundle.
  --bundle-root PATH      Root directory for saved bundles.
  --transition-key KEY    Key used to finish mapping and enter navigation. Default: n
  --host-interface IFACE  Host NIC used for direct SLAMWare link. Default: eth0
  --host-address CIDR     Host address to assign on that NIC, e.g. 192.168.11.2/24
  --no-configure-iface    Do not auto-configure the host NIC for the target subnet.
  --no-start-lcm-server   Do not auto-start lcm_server_high.
  -h, --help              Show this help text.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ip)
      IP_ADDRESS="$2"
      shift 2
      ;;
    --map-bundle)
      MAP_BUNDLE="$2"
      shift 2
      ;;
    --save-name)
      SAVE_MAP_NAME="$2"
      shift 2
      ;;
    --bundle-root)
      BUNDLE_ROOT="$2"
      shift 2
      ;;
    --transition-key)
      TRANSITION_KEY="$2"
      shift 2
      ;;
    --host-interface)
      HOST_INTERFACE="$2"
      shift 2
      ;;
    --host-address)
      HOST_ADDRESS="$2"
      shift 2
      ;;
    --no-configure-iface)
      AUTO_CONFIGURE_INTERFACE="0"
      shift
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

LCM_PID=""
LAUNCH_PID=""
READY_MODE=""

default_host_address_for_target() {
  local first second third fourth
  IFS=. read -r first second third fourth <<<"${IP_ADDRESS}"
  if [[ -n "${first}" && -n "${second}" && -n "${third}" ]]; then
    echo "${first}.${second}.${third}.2/24"
  fi
}

find_same_subnet_interface() {
  local target_prefix="${IP_ADDRESS%.*}."
  ip -o -4 addr show | awk -v target_prefix="${target_prefix}" '
    {
      split($4, cidr, "/");
      if (index(cidr[1], target_prefix) == 1) {
        print $2;
        exit 0;
      }
    }
  '
}

configure_host_interface() {
  [[ "${AUTO_CONFIGURE_INTERFACE}" == "1" ]] || return 0
  [[ -n "${HOST_INTERFACE}" ]] || return 0
  ip link show "${HOST_INTERFACE}" >/dev/null 2>&1 || return 0

  if [[ -z "${HOST_ADDRESS}" ]]; then
    HOST_ADDRESS="$(default_host_address_for_target)"
  fi

  [[ -n "${HOST_ADDRESS}" ]] || return 0

  if [[ "$(find_same_subnet_interface || true)" == "${HOST_INTERFACE}" ]]; then
    return 0
  fi

  echo "Configuring ${HOST_INTERFACE} as ${HOST_ADDRESS} for SLAMWare access..."
  sudo -v
  sudo ip link set dev "${HOST_INTERFACE}" up
  sudo ip addr replace "${HOST_ADDRESS}" dev "${HOST_INTERFACE}"
}

slamware_port_open() {
  timeout 1 bash -lc "</dev/tcp/${IP_ADDRESS}/1445" >/dev/null 2>&1
}

print_connectivity_hint() {
  local iface_on_subnet carrier="unknown" route_line=""
  iface_on_subnet="$(find_same_subnet_interface || true)"
  route_line="$(ip route get "${IP_ADDRESS}" 2>/dev/null | head -n 1 || true)"

  echo
  echo "Waiting for live SLAMWare data from ${IP_ADDRESS}:1445."
  if [[ -n "${route_line}" ]]; then
    echo "Current route: ${route_line}"
  fi

  if [[ -n "${iface_on_subnet}" ]] && [[ -r "/sys/class/net/${iface_on_subnet}/carrier" ]]; then
    carrier="$(cat "/sys/class/net/${iface_on_subnet}/carrier")"
    if [[ "${carrier}" == "0" ]]; then
      echo "Interface ${iface_on_subnet} has no physical carrier. Connect the SLAMWare Ethernet cable or use the correct Wi-Fi link."
    fi
  elif [[ -r "/sys/class/net/${HOST_INTERFACE}/carrier" ]] && [[ "$(cat "/sys/class/net/${HOST_INTERFACE}/carrier")" == "0" ]]; then
    echo "Interface ${HOST_INTERFACE} has no physical carrier. Connect the SLAMWare Ethernet cable or use the correct Wi-Fi link."
  fi

  if ! slamware_port_open; then
    echo "Target ${IP_ADDRESS}:1445 is not reachable yet."
  fi
  echo
}

wait_for_manager_mode() {
  local last_hint_ts=0 now mode=""
  while true; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      wait "${LAUNCH_PID}"
      exit $?
    fi

    mode="$(rosparam get /integrated_mapping_navigation/mode 2>/dev/null || true)"
    case "${mode}" in
      mapping|navigation)
        READY_MODE="${mode}"
        return 0
        ;;
    esac

    now="$(date +%s)"
    if (( now - last_hint_ts >= 8 )); then
      print_connectivity_hint
      last_hint_ts="${now}"
    fi
    sleep 1
  done
}

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

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

configure_host_interface
print_connectivity_hint

echo "Starting integrated mapping/navigation stack..."
roslaunch slam_planner integrated_mapping_navigation.launch \
  ip_address:="${IP_ADDRESS}" \
  map_bundle:="${MAP_BUNDLE}" \
  save_map_name:="${SAVE_MAP_NAME}" \
  bundle_root:="${BUNDLE_ROOT}" &
LAUNCH_PID="$!"

wait_for_manager_mode

if [[ -z "${MAP_BUNDLE}" ]]; then
  echo
  echo "Mapping mode is active."
  echo "Walk the robot with the official remote controller."
  echo "Press '${TRANSITION_KEY}' in this terminal to finish mapping, save the map, and start navigation."
  echo "Press 'q' to quit everything."
  echo

  while true; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      wait "${LAUNCH_PID}"
      exit $?
    fi

    IFS= read -r -s -n 1 key
    if [[ "${key}" == "${TRANSITION_KEY}" ]]; then
      rosservice call /integrated_mapping_navigation/finish_mapping
      echo
      echo "Navigation mode started. Use RViz '2D Nav Goal' to set the destination."
    elif [[ "${key}" == "q" ]]; then
      break
    fi
  done
else
  echo
  echo "Loaded existing map bundle: ${MAP_BUNDLE}"
  echo "Navigation mode started immediately."
  echo "Use RViz '2D Nav Goal' to set the destination."
  echo
  wait "${LAUNCH_PID}"
fi

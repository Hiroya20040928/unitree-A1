#!/usr/bin/env bash
set -u

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$HOME/a1_logs/$RUN_ID"
mkdir -p "$LOG_DIR"
mkdir -p "$HOME/a1_logs"
ln -sfn "$LOG_DIR" "$HOME/a1_logs/latest"

BODY_FILTER_MODE="${BODY_FILTER_MODE:-active}"

FRONT_STOP_M="${FRONT_STOP_M:-0.70}"
FRONT_TURN_ONLY_M="${FRONT_TURN_ONLY_M:-0.85}"
FRONT_SLOW_M="${FRONT_SLOW_M:-1.05}"
FRONT_SECTOR_HALF_DEG="${FRONT_SECTOR_HALF_DEG:-35}"

BODY_LENGTH_M="${BODY_LENGTH_M:-0.50}"
BODY_WIDTH_M="${BODY_WIDTH_M:-0.30}"
LIDAR_X_FROM_BODY_CENTER_M="${LIDAR_X_FROM_BODY_CENTER_M:-0.20}"
DYNAMIC_FRONT_EXTRA_M="${DYNAMIC_FRONT_EXTRA_M:-0.12}"
DYNAMIC_REAR_EXTRA_M="${DYNAMIC_REAR_EXTRA_M:-0.22}"
DYNAMIC_SIDE_EXTRA_M="${DYNAMIC_SIDE_EXTRA_M:-0.08}"
SAFETY_MARGIN_M="${SAFETY_MARGIN_M:-0.05}"

MAX_VX="${MAX_VX:-0.16}"
MAX_WZ="${MAX_WZ:-0.45}"

FILTER="$HOME/a1_lidar_footprint_filter_v6_8_2.py"
CAMERA_SERVER="$HOME/a1_follow_lowlatency_depth_server_raw_v6_5.py"
DRIVER_BIN="$HOME/unitree_legged_sdk/build/a1_high_follow_driver"
SCAN_LAUNCH="$HOME/a1_slamware_scan_only.launch"

echo "[A1 START v6.8.3] run_id=$RUN_ID"
echo "[A1 START v6.8.3] logs=$LOG_DIR"
echo "[A1 START v6.8.3] BODY_FILTER_MODE=$BODY_FILTER_MODE"
echo "[A1 START v6.8.3] scan provider=slamware scan-only launch"
echo "[A1 START v6.8.3] footprint body=${BODY_LENGTH_M}x${BODY_WIDTH_M} lidar_x=${LIDAR_X_FROM_BODY_CENTER_M}"
echo "[A1 START v6.8.3] front stop=${FRONT_STOP_M} turn_only=${FRONT_TURN_ONLY_M} slow=${FRONT_SLOW_M} half_deg=${FRONT_SECTOR_HALF_DEG}"

for f in "$FILTER" "$CAMERA_SERVER" "$DRIVER_BIN" "$SCAN_LAUNCH"; do
  if [ ! -e "$f" ]; then
    echo "[A1 START v6.8.3] ERROR: missing $f" >&2
    exit 1
  fi
done

echo "[A1 START v6.8.3] sudo check"
sudo -n true 2>/dev/null || sudo true

run_ros() {
  bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; $*"
}

echo "[A1 START v6.8.3] stopping old processes"
sudo pkill -9 -f '[a]1_high_follow_driver' 2>/dev/null || true
sudo pkill -9 -f '[a]1_high_follow_driver_v2' 2>/dev/null || true

pkill -9 -f '[a]1_lidar_footprint_filter_v6_8' 2>/dev/null || true
pkill -9 -f '[a]1_body_footprint_filter' 2>/dev/null || true
pkill -9 -f '[a]1_follow_lowlatency_depth_server' 2>/dev/null || true
pkill -9 -f '[a]1_cmd_passthrough_loop' 2>/dev/null || true

pkill -9 -f '[r]oslaunch.*a1_slamware_scan_only.launch' 2>/dev/null || true
pkill -9 -f '[r]oslaunch.*slam_planner_online.launch' 2>/dev/null || true
pkill -9 -f '[b]ase_controller_node' 2>/dev/null || true
pkill -9 -f '[s]lam_planner_node' 2>/dev/null || true
pkill -9 -f '[s]lamware_ros_sdk_server_node' 2>/dev/null || true
pkill -9 -f '[r]osmaster' 2>/dev/null || true
pkill -9 -f '[r]osout' 2>/dev/null || true
sleep 1

echo "[A1 START v6.8.3] reset eth0"
sudo ip addr flush dev eth0 || true
sudo ip link set eth0 down || true
sudo ip link set eth0 up || true
sudo ip addr add 192.168.11.100/24 dev eth0 || true
sudo ip addr add 192.168.123.162/24 dev eth0 || true
sleep 2
ping -c 2 192.168.123.161 || true

echo "[A1 START v6.8.3] start scan-only provider"
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; roslaunch $SCAN_LAUNCH" > "$LOG_DIR/scan_provider.log" 2>&1 &
echo $! > "$LOG_DIR/scan_provider.pid"

echo "[A1 START v6.8.3] waiting for /scan"
ok=0
for i in $(seq 1 30); do
  if run_ros "rostopic echo -n 1 /scan/header >/tmp/a1_scan_header_check 2>/dev/null"; then
    ok=1
    break
  fi
  sleep 0.5
done
if [ "$ok" != "1" ]; then
  echo "[A1 START v6.8.3] ERROR: /scan did not appear" >&2
  tail -160 "$LOG_DIR/scan_provider.log" >&2 || true
  exit 1
fi

echo "[A1 START v6.8.3] check /scan streaming"
run_ros "timeout 6 rostopic hz /scan" > "$LOG_DIR/scan_hz.log" 2>&1 || true
cat "$LOG_DIR/scan_hz.log"
if ! grep -q "average rate" "$LOG_DIR/scan_hz.log"; then
  echo "[A1 START v6.8.3] ERROR: /scan exists but is not streaming" >&2
  tail -160 "$LOG_DIR/scan_provider.log" >&2 || true
  exit 1
fi

echo "[A1 START v6.8.3] verify no base_controller_node"
if pgrep -f '[b]ase_controller_node' >/dev/null 2>&1; then
  echo "[A1 START v6.8.3] ERROR: base_controller_node is running and will conflict with Unitree driver" >&2
  ps aux | grep -E 'base_controller_node|slam_planner_node|slamware_ros_sdk_server_node|roslaunch' | grep -v grep >&2 || true
  exit 1
fi

echo "[A1 START v6.8.3] initialize command files"
python3 - <<'PY'
import time
for p in ["/tmp/a1_follow_cmd_raw", "/tmp/a1_follow_cmd"]:
    with open(p, "w") as f:
        f.write("0 0.00000 0.00000 0.00000 %.6f\n" % time.time())
with open("/tmp/a1_obstacle_front_m", "w") as f:
    f.write("999.0000\n")
PY

MONITOR_ONLY="false"
if [ "$BODY_FILTER_MODE" = "monitor" ]; then
  MONITOR_ONLY="true"
fi

echo "[A1 START v6.8.3] start body footprint filter monitor_only=$MONITOR_ONLY"
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; python $FILTER \
  _scan_topic:=/scan \
  _monitor_only:=$MONITOR_ONLY \
  _front_stop_m:=$FRONT_STOP_M \
  _front_turn_only_m:=$FRONT_TURN_ONLY_M \
  _front_slow_m:=$FRONT_SLOW_M \
  _front_sector_half_deg:=$FRONT_SECTOR_HALF_DEG \
  _body_length_m:=$BODY_LENGTH_M \
  _body_width_m:=$BODY_WIDTH_M \
  _lidar_x_from_body_center_m:=$LIDAR_X_FROM_BODY_CENTER_M \
  _dynamic_front_extra_m:=$DYNAMIC_FRONT_EXTRA_M \
  _dynamic_rear_extra_m:=$DYNAMIC_REAR_EXTRA_M \
  _dynamic_side_extra_m:=$DYNAMIC_SIDE_EXTRA_M \
  _safety_margin_m:=$SAFETY_MARGIN_M \
  _side_hard_clearance_m:=0.03 \
  _side_soft_clearance_m:=0.10 \
  _desired_side_clearance_m:=0.15 \
  _rear_hard_clearance_m:=0.04 \
  _rear_soft_clearance_m:=0.12 \
  _max_vx:=$MAX_VX \
  _max_wz:=$MAX_WZ \
  _max_wz_near:=0.25 \
  _max_vx_side_soft:=0.10 \
  _max_vx_side_hard:=0.05 \
  _vy_bias:=0.000 \
  _vy_sign:=1.0 \
  _wz_bias:=0.000 \
  _wz_sign:=1.0 \
  _rear_swing_protect:=true" > "$LOG_DIR/body_filter.log" 2>&1 &
echo $! > "$LOG_DIR/body_filter.pid"

sleep 1

echo "[A1 START v6.8.3] start camera/follow HTTP server"
nohup python3 -u "$CAMERA_SERVER" > "$LOG_DIR/camera_server.log" 2>&1 &
echo $! > "$LOG_DIR/camera_server.pid"

echo "[A1 START v6.8.3] waiting for camera HTTP"
ok=0
for i in $(seq 1 30); do
  if curl -s --max-time 1 "http://127.0.0.1:8090/snapshot.jpg" >/dev/null 2>&1; then
    ok=1
    break
  fi
  sleep 0.5
done
if [ "$ok" != "1" ]; then
  echo "[A1 START v6.8.3] ERROR: camera HTTP did not start" >&2
  tail -160 "$LOG_DIR/camera_server.log" >&2 || true
  exit 1
fi

echo "[A1 START v6.8.3] start Unitree driver: $DRIVER_BIN"
nohup bash --noprofile --norc -c "cd $HOME/unitree_legged_sdk; export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$(pwd)/lib; printf '\n' | sudo -n -E '$DRIVER_BIN'" > "$LOG_DIR/high_follow_driver.log" 2>&1 &
echo $! > "$LOG_DIR/high_follow_driver.pid"
sleep 2

if ! pgrep -f '[a]1_high_follow_driver' >/dev/null 2>&1; then
  echo "[A1 START v6.8.3] ERROR: Unitree driver is not running" >&2
  tail -120 "$LOG_DIR/high_follow_driver.log" >&2 || true
  ps aux | grep -E 'base_controller_node|a1_high_follow_driver|slamware_ros_sdk_server_node|roslaunch' | grep -v grep >&2 || true
  exit 1
fi

echo "[A1 START v6.8.3] process summary"
ps aux | grep -E 'roslaunch|slamware_ros_sdk_server_node|slam_planner_node|base_controller_node|a1_lidar_footprint_filter_v6_8_2|a1_follow_lowlatency_depth_server|a1_high_follow_driver' | grep -v grep || true

echo "[A1 START v6.8.3] status files"
echo "  logs: $LOG_DIR"
echo "  raw cmd: /tmp/a1_follow_cmd_raw"
echo "  final cmd: /tmp/a1_follow_cmd"
echo "  front distance: /tmp/a1_obstacle_front_m"
echo "  body debug: /tmp/a1_body_footprint_debug"

echo "[A1 START v6.8.3] DONE. Start PC client next."

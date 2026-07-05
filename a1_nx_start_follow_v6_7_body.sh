#!/usr/bin/env bash
set -euo pipefail

LOG_ROOT="$HOME/a1_logs"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

BODY_FILTER_MODE="${BODY_FILTER_MODE:-active}"   # active / monitor / pass
BODY_VY_BIAS="${BODY_VY_BIAS:-0.000}"
BODY_VY_SIGN="${BODY_VY_SIGN:-1.0}"
BODY_WZ_BIAS="${BODY_WZ_BIAS:-0.000}"
BODY_WZ_SIGN="${BODY_WZ_SIGN:-1.0}"

# Footprint parameters. LIDAR_X_FROM_BODY_CENTER_M must match the actual mount.
BODY_LENGTH_M="${BODY_LENGTH_M:-0.50}"
BODY_WIDTH_M="${BODY_WIDTH_M:-0.30}"
LIDAR_X_FROM_BODY_CENTER_M="${LIDAR_X_FROM_BODY_CENTER_M:-0.20}"
DYNAMIC_FRONT_EXTRA_M="${DYNAMIC_FRONT_EXTRA_M:-0.16}"
DYNAMIC_REAR_EXTRA_M="${DYNAMIC_REAR_EXTRA_M:-0.26}"
DYNAMIC_SIDE_EXTRA_M="${DYNAMIC_SIDE_EXTRA_M:-0.12}"
SAFETY_MARGIN_M="${SAFETY_MARGIN_M:-0.07}"

FRONT_STOP_M="${FRONT_STOP_M:-0.85}"
FRONT_TURN_ONLY_M="${FRONT_TURN_ONLY_M:-1.15}"
FRONT_SLOW_M="${FRONT_SLOW_M:-1.40}"
FRONT_SECTOR_HALF_DEG="${FRONT_SECTOR_HALF_DEG:-75}"

SIDE_HARD_CLEARANCE_M="${SIDE_HARD_CLEARANCE_M:-0.06}"
SIDE_SOFT_CLEARANCE_M="${SIDE_SOFT_CLEARANCE_M:-0.18}"
DESIRED_SIDE_CLEARANCE_M="${DESIRED_SIDE_CLEARANCE_M:-0.28}"
REAR_HARD_CLEARANCE_M="${REAR_HARD_CLEARANCE_M:-0.08}"
REAR_SOFT_CLEARANCE_M="${REAR_SOFT_CLEARANCE_M:-0.22}"

MAX_VX="${MAX_VX:-0.10}"
MAX_WZ="${MAX_WZ:-0.38}"
MAX_WZ_NEAR="${MAX_WZ_NEAR:-0.18}"
MAX_VX_SIDE_SOFT="${MAX_VX_SIDE_SOFT:-0.055}"
MAX_VX_SIDE_HARD="${MAX_VX_SIDE_HARD:-0.030}"

# Known working scan provider in this A1 image. This starts the RPLIDAR/Slamware ROS server only.
# The follow program consumes only /scan. It does not consume /map, /odom, goals, or planner output.
SCAN_IP="${SCAN_IP:-192.168.11.1}"
SCAN_LAUNCH_PKG="${SCAN_LAUNCH_PKG:-slam_planner}"
SCAN_LAUNCH_FILE="${SCAN_LAUNCH_FILE:-slam_rplidar_start.launch}"

DRIVER_BIN="$HOME/unitree_legged_sdk/build/a1_high_follow_driver"
CAMERA_SERVER="$HOME/a1_follow_lowlatency_depth_server_raw_v6_5.py"
BODY_FILTER="$HOME/a1_body_footprint_filter_v6_7.py"
OBSTACLE_WRITER="$HOME/a1_laserscan_obstacle_writer.py"

run_ros() {
  bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; $*"
}

require_file() {
  if [ ! -f "$1" ]; then
    echo "[A1 START v6.7] ERROR: missing $1" >&2
    exit 1
  fi
}

require_file "$OBSTACLE_WRITER"
require_file "$BODY_FILTER"
require_file "$CAMERA_SERVER"
if [ ! -x "$DRIVER_BIN" ]; then
  echo "[A1 START v6.7] ERROR: driver not executable: $DRIVER_BIN" >&2
  echo "[A1 START v6.7] check: ls -lh $DRIVER_BIN ; file $DRIVER_BIN" >&2
  exit 1
fi

printf '[A1 START v6.7] run_id=%s\n' "$RUN_ID"
printf '[A1 START v6.7] logs=%s\n' "$LOG_DIR"
printf '[A1 START v6.7] BODY_FILTER_MODE=%s\n' "$BODY_FILTER_MODE"
printf '[A1 START v6.7] scan provider=%s %s ip=%s\n' "$SCAN_LAUNCH_PKG" "$SCAN_LAUNCH_FILE" "$SCAN_IP"
printf '[A1 START v6.7] footprint body=%sx%s lidar_x=%s dyn_front=%s dyn_rear=%s dyn_side=%s margin=%s\n' "$BODY_LENGTH_M" "$BODY_WIDTH_M" "$LIDAR_X_FROM_BODY_CENTER_M" "$DYNAMIC_FRONT_EXTRA_M" "$DYNAMIC_REAR_EXTRA_M" "$DYNAMIC_SIDE_EXTRA_M" "$SAFETY_MARGIN_M"
printf '[A1 START v6.7] front stop=%s turn_only=%s slow=%s half_deg=%s\n' "$FRONT_STOP_M" "$FRONT_TURN_ONLY_M" "$FRONT_SLOW_M" "$FRONT_SECTOR_HALF_DEG"

sed -i 's/\bsouce\b/source/g' "$HOME/.bash_profile" "$HOME/.bashrc" 2>/dev/null || true

echo "[A1 START v6.7] sudo check"
sudo -v

echo "[A1 START v6.7] stopping old processes"
sudo pkill -9 -f '[a]1_high_follow_driver_v2' 2>/dev/null || true
sudo pkill -9 -f '[a]1_high_follow_driver' 2>/dev/null || true
pkill -9 -f '[r]oslaunch' 2>/dev/null || true
pkill -9 -f '[r]oscore' 2>/dev/null || true
pkill -9 -f '[b]ase_controller_node' 2>/dev/null || true
pkill -9 -f '[l]cm_server_high' 2>/dev/null || true
pkill -9 -f '[e]xample_walk' 2>/dev/null || true
pkill -9 -f '[a]1_laserscan_obstacle_writer.py' 2>/dev/null || true
pkill -9 -f '[a]1_body_safety_filter_footprint_v2.py' 2>/dev/null || true
pkill -9 -f '[a]1_body_footprint_filter_v6_6.py' 2>/dev/null || true
pkill -9 -f '[a]1_body_footprint_filter_v6_7.py' 2>/dev/null || true
pkill -9 -f '[a]1_cmd_passthrough_loop.sh' 2>/dev/null || true
pkill -9 -f '[a]1_cmd_hard_safety_passthrough.py' 2>/dev/null || true
pkill -9 -f '[a]1_.*fallback.*node' 2>/dev/null || true
pkill -9 -f '[a]1_follow_lowlatency_depth_server_raw' 2>/dev/null || true
pkill -9 -f '[s]lamware_ros_sdk_server_node' 2>/dev/null || true
sleep 1

echo "[A1 START v6.7] reset eth0"
sudo ip addr flush dev eth0 || true
sudo ip link set eth0 down || true
sudo ip link set eth0 up
sudo ip addr add 192.168.11.100/24 dev eth0 || true
sudo ip addr add 192.168.123.162/24 dev eth0 || true
sleep 3
ping -c 2 192.168.123.161 | tee "$LOG_DIR/ping_192.168.123.161.log" || true

echo "[A1 START v6.7] start scan provider"
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; roslaunch $SCAN_LAUNCH_PKG $SCAN_LAUNCH_FILE ip_address:=$SCAN_IP" > "$LOG_DIR/scan_provider.log" 2>&1 &
echo $! > "$LOG_DIR/scan_provider.pid"

echo "[A1 START v6.7] waiting for /scan"
for i in $(seq 1 60); do
  if run_ros "rostopic list 2>/dev/null | grep -qx /scan"; then
    break
  fi
  sleep 1
  if [ "$i" = "60" ]; then
    echo "[A1 START v6.7] ERROR: /scan did not appear" >&2
    tail -160 "$LOG_DIR/scan_provider.log" >&2 || true
    exit 1
  fi
done

# Stop any default walking/controller node launched by the image. This follow stack owns HighCmd.
pkill -9 -f '[b]ase_controller_node' 2>/dev/null || true
pkill -9 -f '[l]cm_server_high' 2>/dev/null || true
pkill -9 -f '[e]xample_walk' 2>/dev/null || true
sleep 1

echo "[A1 START v6.7] check /scan message"
run_ros "timeout 8 rostopic echo -n 1 /scan" > "$LOG_DIR/scan_first_msg.log" 2>&1 || {
  echo "[A1 START v6.7] ERROR: /scan exists but no message" >&2
  tail -120 "$LOG_DIR/scan_first_msg.log" >&2 || true
  exit 1
}

echo "[A1 START v6.7] start obstacle writer"
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; python $OBSTACLE_WRITER _scan_topic:=/scan _front_deg:=$FRONT_SECTOR_HALF_DEG" > "$LOG_DIR/obstacle_writer.log" 2>&1 &
echo $! > "$LOG_DIR/obstacle_writer.pid"

echo "[A1 START v6.7] initialize command files"
rm -f /tmp/a1_follow_cmd /tmp/a1_follow_cmd_raw /tmp/a1_body_footprint_debug /tmp/a1_body_safety_debug /tmp/a1_obstacle_front_m
printf '0 0.00000 0.00000 0.00000 %.6f\n' "$(date +%s.%N)" > /tmp/a1_follow_cmd_raw
printf '0 0.00000 0.00000 0.00000 %.6f\n' "$(date +%s.%N)" > /tmp/a1_follow_cmd

if [ "$BODY_FILTER_MODE" = "pass" ]; then
  echo "[A1 START v6.7] BODY_FILTER_MODE=pass: direct passthrough, not recommended"
  cat > "$HOME/a1_cmd_passthrough_loop.sh" <<'EOF'
#!/usr/bin/env bash
while true; do
  if [ -s /tmp/a1_follow_cmd_raw ]; then
    cp /tmp/a1_follow_cmd_raw /tmp/a1_follow_cmd.tmp
    mv /tmp/a1_follow_cmd.tmp /tmp/a1_follow_cmd
  fi
  sleep 0.02
done
EOF
  chmod +x "$HOME/a1_cmd_passthrough_loop.sh"
  nohup bash "$HOME/a1_cmd_passthrough_loop.sh" > "$LOG_DIR/passthrough.log" 2>&1 &
  echo $! > "$LOG_DIR/passthrough.pid"
else
  MONITOR_ONLY="false"
  if [ "$BODY_FILTER_MODE" = "monitor" ]; then
    MONITOR_ONLY="true"
  fi
  echo "[A1 START v6.7] start body footprint filter monitor_only=$MONITOR_ONLY"
  nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; python $BODY_FILTER \
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
    _side_hard_clearance_m:=$SIDE_HARD_CLEARANCE_M \
    _side_soft_clearance_m:=$SIDE_SOFT_CLEARANCE_M \
    _desired_side_clearance_m:=$DESIRED_SIDE_CLEARANCE_M \
    _rear_hard_clearance_m:=$REAR_HARD_CLEARANCE_M \
    _rear_soft_clearance_m:=$REAR_SOFT_CLEARANCE_M \
    _max_vx:=$MAX_VX \
    _max_wz:=$MAX_WZ \
    _max_wz_near:=$MAX_WZ_NEAR \
    _max_vx_side_soft:=$MAX_VX_SIDE_SOFT \
    _max_vx_side_hard:=$MAX_VX_SIDE_HARD \
    _vy_bias:=$BODY_VY_BIAS \
    _vy_sign:=$BODY_VY_SIGN \
    _wz_bias:=$BODY_WZ_BIAS \
    _wz_sign:=$BODY_WZ_SIGN \
    _rear_swing_protect:=true" > "$LOG_DIR/body_footprint_filter.log" 2>&1 &
  echo $! > "$LOG_DIR/body_footprint_filter.pid"
fi

echo "[A1 START v6.7] start camera/follow HTTP server"
nohup bash --noprofile --norc -c "python3 -u $CAMERA_SERVER" > "$LOG_DIR/camera_server.log" 2>&1 &
echo $! > "$LOG_DIR/camera_server.pid"

echo "[A1 START v6.7] waiting for camera HTTP"
CAM_OK=0
for i in $(seq 1 20); do
  if curl -fsS http://127.0.0.1:8090/status > "$LOG_DIR/camera_status_initial.txt" 2>&1; then
    CAM_OK=1
    break
  fi
  sleep 1
done
if [ "$CAM_OK" != "1" ]; then
  echo "[A1 START v6.7] ERROR: camera HTTP server not responding" >&2
  tail -160 "$LOG_DIR/camera_server.log" >&2 || true
  exit 1
fi

echo "[A1 START v6.7] start Unitree driver: $DRIVER_BIN"
nohup bash --noprofile --norc -c "cd $HOME/unitree_legged_sdk; export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$(pwd)/lib; printf '\n' | sudo -n -E '$DRIVER_BIN'" > "$LOG_DIR/high_follow_driver.log" 2>&1 &
echo $! > "$LOG_DIR/high_follow_driver.pid"
sleep 2
if ! pgrep -f '[a]1_high_follow_driver' >/dev/null 2>&1; then
  echo "[A1 START v6.7] ERROR: Unitree driver is not running" >&2
  tail -120 "$LOG_DIR/high_follow_driver.log" >&2 || true
  exit 1
fi

echo "[A1 START v6.7] process summary"
ps aux | grep -E 'slamware_ros_sdk_server_node|a1_laserscan_obstacle_writer|a1_body_footprint_filter_v6_7|a1_follow_lowlatency_depth_server|a1_high_follow_driver|a1_cmd_passthrough_loop' | grep -v grep || true

echo "[A1 START v6.7] status files"
echo "  logs: $LOG_DIR"
echo "  raw cmd: /tmp/a1_follow_cmd_raw"
echo "  final cmd: /tmp/a1_follow_cmd"
echo "  front distance: /tmp/a1_obstacle_front_m"
echo "  body debug: /tmp/a1_body_footprint_debug"
echo "[A1 START v6.7] DONE. Start PC client next."

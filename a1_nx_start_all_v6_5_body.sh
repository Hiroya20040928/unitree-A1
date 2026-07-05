#!/usr/bin/env bash
set -euo pipefail

# Robust NX-side launcher for A1 follow + fresh SLAM.
# Run from PC via: ssh -t unitree@192.168.12.1 "bash ~/a1_nx_start_all_v6_1.sh"

LOG_ROOT="$HOME/a1_logs"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

BODY_VY_BIAS="${BODY_VY_BIAS:--0.035}"
BODY_VY_SIGN="${BODY_VY_SIGN:-1.0}"
BODY_WZ_BIAS="${BODY_WZ_BIAS:-0.000}"
BODY_WZ_SIGN="${BODY_WZ_SIGN:-1.0}"
NO_ROUTE_FALLBACK="${NO_ROUTE_FALLBACK:-0}"

echo "[A1 START v6.5] run_id=$RUN_ID"
echo "[A1 START v6.5] logs=$LOG_DIR"
echo "[A1 START v6.5] body_filter: BODY_VY_BIAS=$BODY_VY_BIAS BODY_VY_SIGN=$BODY_VY_SIGN BODY_WZ_BIAS=$BODY_WZ_BIAS BODY_WZ_SIGN=$BODY_WZ_SIGN"

# Remove old typo noise if present. This is safe if files do not exist.
sed -i 's/\bsouce\b/source/g' "$HOME/.bash_profile" "$HOME/.bashrc" 2>/dev/null || true

run_ros() {
  bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; $*"
}

require_file() {
  local f="$1"
  if [ ! -f "$f" ]; then
    echo "[A1 START v6.5] ERROR: required file missing: $f" >&2
    exit 1
  fi
}

require_exec_or_file() {
  local f="$1"
  if [ ! -e "$f" ]; then
    echo "[A1 START v6.5] ERROR: required file missing: $f" >&2
    exit 1
  fi
}

echo "[A1 START v6.5] checking required files"
require_file "$HOME/a1_laserscan_obstacle_writer.py"
require_file "$HOME/a1_follow_lowlatency_depth_server_raw_v6_5.py"
require_file "$HOME/a1_slam_route_fallback_node_v6_5_passive.py"
require_file "$HOME/a1_body_safety_filter_footprint_v2.py"

DRIVER_BIN="$HOME/unitree_legged_sdk/build/a1_high_follow_driver"
if [ ! -x "$DRIVER_BIN" ]; then
  echo "[A1 START v6.5] ERROR: original safe driver not found: $DRIVER_BIN" >&2
  echo "[A1 START v6.5] This version intentionally uses the original driver because its obstacle stop was already confirmed." >&2
  exit 1
fi

echo "[A1 START v6.5] sudo check"
sudo -v

echo "[A1 START v6.5] stopping old processes"
echo "[A1 START v6.5] force kill old Unitree drivers"
sudo pkill -9 -f '[a]1_high_follow_driver_v2' 2>/dev/null || true
sudo pkill -9 -f '[a]1_high_follow_driver' 2>/dev/null || true
sleep 1
pkill -f roslaunch 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
pkill -f base_controller_node 2>/dev/null || true
pkill -f lcm_server_high 2>/dev/null || true
pkill -f '[a]1_high_follow_driver' 2>/dev/null || true
pkill -f '[a]1_high_follow_driver_v2' 2>/dev/null || true
pkill -f a1_laserscan_obstacle_writer.py 2>/dev/null || true
pkill -f a1_slam_route_fallback_node 2>/dev/null || true
pkill -f a1_body_safety_filter_footprint_v2.py 2>/dev/null || true
pkill -f a1_follow_lowlatency_depth_server_raw_v6_5.py 2>/dev/null || true
pkill -f example_walk 2>/dev/null || true
sleep 1

echo "[A1 START v6.5] reset eth0"
sudo ip addr flush dev eth0 || true
sudo ip link set eth0 down || true
sudo ip link set eth0 up
sudo ip addr add 192.168.11.100/24 dev eth0 || true
sudo ip addr add 192.168.123.162/24 dev eth0 || true
sleep 3
ping -c 2 192.168.123.161 | tee "$LOG_DIR/ping_192.168.123.161.log" || true

echo "[A1 START v6.5] start SLAM fresh"
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; roslaunch slam_planner slam_planner_online.launch" > "$LOG_DIR/slam_planner_online.log" 2>&1 &
echo $! > "$LOG_DIR/slam_roslaunch.pid"

echo "[A1 START v6.5] waiting for /scan"
for i in $(seq 1 75); do
  if run_ros "rostopic list 2>/dev/null | grep -qx /scan"; then
    break
  fi
  sleep 1
  if [ "$i" = "75" ]; then
    echo "[A1 START v6.5] ERROR: /scan did not appear" >&2
    tail -120 "$LOG_DIR/slam_planner_online.log" >&2 || true
    exit 1
  fi
done

echo "[A1 START v6.5] kill competing A1 control nodes"
pkill -f base_controller_node 2>/dev/null || true
pkill -f lcm_server_high 2>/dev/null || true
pkill -f example_walk 2>/dev/null || true
sleep 1

echo "[A1 START v6.5] clear current map if service exists"
run_ros "rosservice list 2>/dev/null | grep -q '/slamware_ros_sdk_server_node/clear_map' && rosservice call /slamware_ros_sdk_server_node/clear_map '{}'" >> "$LOG_DIR/clear_map.log" 2>&1 || true

echo "[A1 START v6.5] check /scan message"
run_ros "timeout 8 rostopic echo -n 1 /scan" > "$LOG_DIR/scan_first_msg.log" 2>&1 || {
  echo "[A1 START v6.5] ERROR: /scan exists but no message" >&2
  tail -80 "$LOG_DIR/scan_first_msg.log" >&2 || true
  exit 1
}

echo "[A1 START v6.5] start obstacle writer"
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; python $HOME/a1_laserscan_obstacle_writer.py _scan_topic:=/scan _front_deg:=25" > "$LOG_DIR/obstacle_writer.log" 2>&1 &
echo $! > "$LOG_DIR/obstacle_writer.pid"

echo "[A1 START v6.5] start body safety filter"
rm -f /tmp/a1_follow_cmd /tmp/a1_follow_cmd_raw /tmp/a1_body_safety_debug
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; python $HOME/a1_body_safety_filter_footprint_v2.py _scan_topic:=/scan _front_stop_m:=0.70 _front_slow_m:=1.05 _body_length_m:=0.50 _body_width_m:=0.30 _lidar_x_from_body_center_m:=0.20 _dynamic_front_extra_m:=0.17 _dynamic_rear_extra_m:=0.25 _dynamic_side_extra_m:=0.14 _safety_margin_m:=0.08 _side_hard_clearance_m:=0.08 _side_soft_clearance_m:=0.22 _rear_hard_clearance_m:=0.10 _rear_soft_clearance_m:=0.26 _desired_side_clearance_m:=0.30 _k_side_vy:=0.075 _max_side_vy:=0.050 _vy_bias:=${BODY_VY_BIAS} _vy_sign:=${BODY_VY_SIGN} _wz_bias:=${BODY_WZ_BIAS} _wz_sign:=${BODY_WZ_SIGN} _max_vx_when_side_close:=0.090 _max_vx_when_narrow:=0.055" > "$LOG_DIR/body_safety_filter.log" 2>&1 &
echo $! > "$LOG_DIR/body_safety_filter.pid"

if [ "$NO_ROUTE_FALLBACK" != "1" ]; then
ROUTE_NODE="$HOME/a1_slam_route_fallback_node_v6_5_passive.py"
echo "[A1 START v6.5] start route fallback node: $ROUTE_NODE"
nohup bash --noprofile --norc -c "source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1; python '$ROUTE_NODE' _scan_topic:=/scan _odom_topic:=/odom _max_lost_time:=2.0 _max_lost_dist:=0.45 _route_max_vx:=0.030 _route_align_vx:=0.015 _route_max_wz:=0.08 _k_heading:=0.18 _desired_side_m:=0.70 _k_side_vy:=0.000 _k_wall_yaw:=0.00 _vy_bias:=0.000 _cmd_alpha:=0.12 _hard_front_m:=0.70 _slow_front_m:=1.05 _absolute_stop_m:=0.70" > "$LOG_DIR/route_fallback.log" 2>&1 &
echo $! > "$LOG_DIR/route_fallback.pid"

echo "[A1 START v6.5] waiting for route fallback HTTP"
ROUTE_OK=0
for i in $(seq 1 15); do
  if curl -fsS http://127.0.0.1:8091/status > "$LOG_DIR/route_status_initial.txt" 2>&1; then
    ROUTE_OK=1
    break
  fi
  sleep 1
done
if [ "$ROUTE_OK" != "1" ]; then
  echo "[A1 START v6.5] ERROR: route fallback HTTP not responding" >&2
  tail -120 "$LOG_DIR/route_fallback.log" >&2 || true
  exit 1
fi
else
  echo "[A1 START v6.5] NO_ROUTE_FALLBACK=1: route fallback node not started"
  pkill -9 -f a1_slam_route_fallback_node_v6_5_passive.py 2>/dev/null || true
fi

echo "[A1 START v6.5] start camera/follow HTTP server"
nohup bash --noprofile --norc -c "python3 -u $HOME/a1_follow_lowlatency_depth_server_raw_v6_5.py" > "$LOG_DIR/camera_server.log" 2>&1 &
echo $! > "$LOG_DIR/camera_server.pid"

echo "[A1 START v6.5] waiting for camera HTTP"
CAM_OK=0
for i in $(seq 1 20); do
  if curl -fsS http://127.0.0.1:8090/status > "$LOG_DIR/camera_status_initial.txt" 2>&1; then
    CAM_OK=1
    break
  fi
  sleep 1
done
if [ "$CAM_OK" != "1" ]; then
  echo "[A1 START v6.5] ERROR: camera HTTP server not responding" >&2
  echo "[A1 START v6.5] camera log:" >&2
  tail -160 "$LOG_DIR/camera_server.log" >&2 || true
  echo "[A1 START v6.5] device summary:" >&2
  ls -l /dev/video* 2>/dev/null >&2 || true
  python3 - <<'PY' >&2 || true
import sys
for m in ["cv2", "numpy"]:
    try:
        mod=__import__(m)
        print(m, "OK", getattr(mod, "__version__", ""))
    except Exception as e:
        print(m, "NG", repr(e))
PY
  exit 1
fi

echo "[A1 START v6.5] start Unitree driver: $DRIVER_BIN"
nohup bash --noprofile --norc -c "cd $HOME/unitree_legged_sdk; export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$(pwd)/lib; printf '\n' | sudo -n -E '$DRIVER_BIN'" > "$LOG_DIR/high_follow_driver.log" 2>&1 &
echo $! > "$LOG_DIR/high_follow_driver.pid"

sleep 2
if ! pgrep -f '[a]1_high_follow_driver' >/dev/null 2>&1; then
  echo "[A1 START v6.5] ERROR: Unitree driver is not running" >&2
  tail -120 "$LOG_DIR/high_follow_driver.log" >&2 || true
  exit 1
fi

echo "[A1 START v6.5] process summary"
ps aux | grep -E "slamware_ros_sdk_server_node|slam_planner_node|base_controller_node|a1_laserscan_obstacle_writer|a1_slam_route_fallback|a1_body_safety_filter|a1_follow_lowlatency|a1_high_follow_driver" | grep -v grep || true

echo "[A1 START v6.5] ROS topics"
run_ros "rostopic list | grep -Ei 'scan|map|odom'" || true

echo "[A1 START v6.5] status files"
echo "  logs: $LOG_DIR"
echo "  obstacle: /tmp/a1_obstacle_front_m"
echo "  route debug: /tmp/a1_route_debug"
echo "  follow cmd: /tmp/a1_follow_cmd"
echo "[A1 START v6.5] DONE. Start PC client next."

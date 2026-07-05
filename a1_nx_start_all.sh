#!/usr/bin/env bash
set -euo pipefail

LOG_ROOT="$HOME/a1_logs"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

echo "[A1 START] run_id=$RUN_ID"
echo "[A1 START] logs=$LOG_DIR"

echo "[A1 START] sudo check"
sudo -v

echo "[A1 START] stopping old processes"
pkill -f roslaunch 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
pkill -f base_controller_node 2>/dev/null || true
pkill -f lcm_server_high 2>/dev/null || true
pkill -f a1_high_follow_driver 2>/dev/null || true
pkill -f a1_high_follow_driver_v2 2>/dev/null || true
pkill -f a1_laserscan_obstacle_writer.py 2>/dev/null || true
pkill -f a1_slam_route_fallback_node 2>/dev/null || true
pkill -f a1_follow_lowlatency_depth_server.py 2>/dev/null || true
pkill -f example_walk 2>/dev/null || true
sleep 1

echo "[A1 START] reset eth0"
sudo ip addr flush dev eth0 || true
sudo ip link set eth0 down || true
sudo ip link set eth0 up
sudo ip addr add 192.168.11.100/24 dev eth0 || true
sudo ip addr add 192.168.123.162/24 dev eth0 || true
sleep 3
ping -c 2 192.168.123.161 | tee "$LOG_DIR/ping_192.168.123.161.log" || true

# ROS env used by every background shell
ROS_ENV='source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1'

echo "[A1 START] start SLAM fresh"
nohup bash -lc "$ROS_ENV; roslaunch slam_planner slam_planner_online.launch" > "$LOG_DIR/slam_planner_online.log" 2>&1 &
echo $! > "$LOG_DIR/slam_roslaunch.pid"

# Wait for /scan and /map.
echo "[A1 START] waiting for /scan"
for i in $(seq 1 60); do
  if bash -lc "$ROS_ENV; rostopic list 2>/dev/null | grep -qx /scan"; then
    break
  fi
  sleep 1
  if [ "$i" = "60" ]; then
    echo "[A1 START] ERROR: /scan did not appear" >&2
    tail -80 "$LOG_DIR/slam_planner_online.log" >&2 || true
    exit 1
  fi
done

# This launch starts base_controller_node; it competes with our Unitree driver.
echo "[A1 START] kill competing A1 control nodes"
pkill -f base_controller_node 2>/dev/null || true
pkill -f lcm_server_high 2>/dev/null || true
pkill -f example_walk 2>/dev/null || true
sleep 1

# Try to reset current SLAM map each run if service exists.
echo "[A1 START] clear current map if service exists"
bash -lc "$ROS_ENV; rosservice list 2>/dev/null | grep -q '/slamware_ros_sdk_server_node/clear_map' && rosservice call /slamware_ros_sdk_server_node/clear_map '{}'" >> "$LOG_DIR/clear_map.log" 2>&1 || true

# Ensure /scan really contains data.
bash -lc "$ROS_ENV; timeout 6 rostopic echo -n 1 /scan" > "$LOG_DIR/scan_first_msg.log" 2>&1 || {
  echo "[A1 START] ERROR: /scan exists but no message" >&2
  exit 1
}

echo "[A1 START] start obstacle writer"
nohup bash -lc "$ROS_ENV; python ~/a1_laserscan_obstacle_writer.py _scan_topic:=/scan _front_deg:=25" > "$LOG_DIR/obstacle_writer.log" 2>&1 &
echo $! > "$LOG_DIR/obstacle_writer.pid"

# Route fallback. Use v2base if present; otherwise fallback to old file.
ROUTE_NODE="$HOME/a1_slam_route_fallback_node_v2base.py"
if [ ! -f "$ROUTE_NODE" ]; then
  ROUTE_NODE="$HOME/a1_slam_route_fallback_node.py"
fi

echo "[A1 START] start route fallback node: $ROUTE_NODE"
nohup bash -lc "$ROS_ENV; python '$ROUTE_NODE' _scan_topic:=/scan _odom_topic:=/odom _max_lost_time:=2.8 _max_lost_dist:=0.75 _route_max_vx:=0.055 _route_align_vx:=0.025 _route_max_wz:=0.28 _k_heading:=0.90 _desired_side_m:=0.70 _k_side_vy:=0.045 _k_wall_yaw:=0.05 _vy_bias:=-0.035 _cmd_alpha:=0.25" > "$LOG_DIR/route_fallback.log" 2>&1 &
echo $! > "$LOG_DIR/route_fallback.pid"

sleep 1
if curl -fsS http://127.0.0.1:8091/status > "$LOG_DIR/route_status_initial.txt" 2>&1; then
  echo "[A1 START] route fallback HTTP ok"
else
  echo "[A1 START] WARNING: route fallback HTTP not responding yet"
fi

echo "[A1 START] start camera/follow HTTP server"
nohup bash -lc "python3 ~/a1_follow_lowlatency_depth_server.py" > "$LOG_DIR/camera_server.log" 2>&1 &
echo $! > "$LOG_DIR/camera_server.pid"

sleep 2
curl -fsS http://127.0.0.1:8090/status > "$LOG_DIR/camera_status_initial.txt" 2>&1 || {
  echo "[A1 START] ERROR: camera HTTP server not responding" >&2
  tail -80 "$LOG_DIR/camera_server.log" >&2 || true
  exit 1
}

# Start Unitree HighLevel driver. Prefer v2 if built; fallback to original.
DRIVER_BIN="$HOME/unitree_legged_sdk/build/a1_high_follow_driver_v2"
if [ ! -x "$DRIVER_BIN" ]; then
  DRIVER_BIN="$HOME/unitree_legged_sdk/build/a1_high_follow_driver"
fi
if [ ! -x "$DRIVER_BIN" ]; then
  echo "[A1 START] ERROR: no high follow driver binary found" >&2
  echo "Expected $HOME/unitree_legged_sdk/build/a1_high_follow_driver_v2 or a1_high_follow_driver" >&2
  exit 1
fi

echo "[A1 START] start Unitree driver: $DRIVER_BIN"
# Driver waits for Enter. Pipe newline. sudo uses credential from sudo -v.
nohup bash -lc "cd ~/unitree_legged_sdk; export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$(pwd)/lib; printf '\n' | sudo -n -E '$DRIVER_BIN'" > "$LOG_DIR/high_follow_driver.log" 2>&1 &
echo $! > "$LOG_DIR/high_follow_driver.pid"

sleep 2

echo "[A1 START] process summary"
ps aux | grep -E "slamware_ros_sdk_server_node|slam_planner_node|base_controller_node|a1_laserscan_obstacle_writer|a1_slam_route_fallback|a1_follow_lowlatency|a1_high_follow_driver" | grep -v grep || true

echo "[A1 START] ROS topics"
bash -lc "$ROS_ENV; rostopic list | grep -Ei 'scan|map|odom'" || true

echo "[A1 START] status files"
echo "  logs: $LOG_DIR"
echo "  obstacle: /tmp/a1_obstacle_front_m"
echo "  route debug: /tmp/a1_route_debug"
echo "  follow cmd: /tmp/a1_follow_cmd"
echo "[A1 START] DONE. Start PC client next."

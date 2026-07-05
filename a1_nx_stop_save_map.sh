#!/usr/bin/env bash
set -euo pipefail

ROS_ENV='source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=192.168.12.1'
MAP_DIR="$HOME/maps"
mkdir -p "$MAP_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
MAP_BASE="$MAP_DIR/a1_map_$STAMP"

echo "[A1 STOP] send stop command"
echo "0 0.00000 0.00000 0.00000 $(date +%s.%N)" > /tmp/a1_follow_cmd || true
curl -fsS "http://127.0.0.1:8090/follow_stop" >/dev/null 2>&1 || true
curl -fsS "http://127.0.0.1:8091/route_state?mode=stop&xerr=0&area=0" >/dev/null 2>&1 || true

# Save map before killing ROS.
echo "[A1 STOP] saving map to $MAP_BASE"
if bash -lc "$ROS_ENV; timeout 3 rostopic echo -n 1 /map >/dev/null"; then
  if bash -lc "$ROS_ENV; rosrun map_server map_saver -f '$MAP_BASE'"; then
    echo "[A1 STOP] saved: ${MAP_BASE}.pgm / ${MAP_BASE}.yaml"
  else
    echo "[A1 STOP] WARNING: map_saver failed" >&2
  fi
else
  echo "[A1 STOP] WARNING: /map unavailable; skip map save" >&2
fi

echo "[A1 STOP] killing processes"
pkill -f a1_high_follow_driver_v2 2>/dev/null || true
pkill -f a1_high_follow_driver 2>/dev/null || true
pkill -f a1_laserscan_obstacle_writer.py 2>/dev/null || true
pkill -f a1_slam_route_fallback_node 2>/dev/null || true
pkill -f a1_follow_lowlatency_depth_server.py 2>/dev/null || true
pkill -f base_controller_node 2>/dev/null || true
pkill -f lcm_server_high 2>/dev/null || true
pkill -f example_walk 2>/dev/null || true
pkill -f roslaunch 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
sleep 1

echo "[A1 STOP] maps:"
ls -lh "$MAP_DIR" | tail -20 || true

echo "[A1 STOP] DONE"

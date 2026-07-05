#!/usr/bin/env bash
set -u

LOG_DIR="$HOME/a1_logs/v2_min_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$HOME/a1_logs/latest"

CAMERA_SERVER="$HOME/a1_follow_lowlatency_depth_server_raw_v6_5.py"
DRIVER_BIN="$HOME/unitree_legged_sdk/build/a1_high_follow_driver"

echo "[A1 v2-min] logs=$LOG_DIR"

for f in "$CAMERA_SERVER" "$DRIVER_BIN"; do
  if [ ! -e "$f" ]; then
    echo "[A1 v2-min] ERROR: missing $f" >&2
    exit 1
  fi
done

echo "[A1 v2-min] sudo check"
sudo -n true 2>/dev/null || sudo true

echo "[A1 v2-min] kill old processes"
sudo pkill -9 -f '[a]1_high_follow_driver' 2>/dev/null || true
sudo pkill -9 -f '[a]1_high_follow_driver_v2' 2>/dev/null || true
pkill -9 -f '[a]1_lidar_footprint_filter' 2>/dev/null || true
pkill -9 -f '[a]1_body_footprint_filter' 2>/dev/null || true
pkill -9 -f '[a]1_follow_lowlatency_depth_server' 2>/dev/null || true
pkill -9 -f '[a]1_cmd_passthrough_loop' 2>/dev/null || true

# Critical: these conflict with Unitree UDP
pkill -9 -f '[r]oslaunch.*slam_planner' 2>/dev/null || true
pkill -9 -f '[r]oslaunch.*a1_slamware' 2>/dev/null || true
pkill -9 -f '[b]ase_controller_node' 2>/dev/null || true
pkill -9 -f '[s]lam_planner_node' 2>/dev/null || true
pkill -9 -f '[s]lamware_ros_sdk_server_node' 2>/dev/null || true
pkill -9 -f '[r]osmaster' 2>/dev/null || true
pkill -9 -f '[r]osout' 2>/dev/null || true
sleep 1

echo "[A1 v2-min] reset eth0"
sudo ip addr flush dev eth0 || true
sudo ip link set eth0 down || true
sudo ip link set eth0 up || true
sudo ip addr add 192.168.123.162/24 dev eth0 || true
sleep 2
ping -c 2 192.168.123.161 || true

echo "[A1 v2-min] init cmd files"
python3 - <<'PY'
import time
for p in ["/tmp/a1_follow_cmd_raw", "/tmp/a1_follow_cmd"]:
    with open(p, "w") as f:
        f.write("0 0.00000 0.00000 0.00000 %.6f\n" % time.time())
with open("/tmp/a1_obstacle_front_m", "w") as f:
    f.write("999.0000\n")
PY

echo "[A1 v2-min] start camera/follow HTTP server"
nohup python3 -u "$CAMERA_SERVER" > "$LOG_DIR/camera_server.log" 2>&1 &
echo $! > "$LOG_DIR/camera_server.pid"

echo "[A1 v2-min] waiting for camera HTTP"
ok=0
for i in $(seq 1 30); do
  if curl -s --max-time 1 "http://127.0.0.1:8090/snapshot.jpg" >/dev/null 2>&1; then
    ok=1
    break
  fi
  sleep 0.5
done
if [ "$ok" != "1" ]; then
  echo "[A1 v2-min] ERROR: camera HTTP did not start" >&2
  tail -120 "$LOG_DIR/camera_server.log" >&2 || true
  exit 1
fi

echo "[A1 v2-min] start raw -> final passthrough"
nohup bash -c '
while true; do
  if [ -s /tmp/a1_follow_cmd_raw ]; then
    cp /tmp/a1_follow_cmd_raw /tmp/a1_follow_cmd
  fi
  sleep 0.03
done
' > "$LOG_DIR/passthrough.log" 2>&1 &
echo $! > "$LOG_DIR/passthrough.pid"

echo "[A1 v2-min] verify no conflicting controller"
if pgrep -f '[b]ase_controller_node|[s]lam_planner_node|[s]lamware_ros_sdk_server_node' >/dev/null 2>&1; then
  echo "[A1 v2-min] ERROR: conflicting ROS controller is still running" >&2
  ps aux | grep -E 'base_controller_node|slam_planner_node|slamware_ros_sdk_server_node|roslaunch' | grep -v grep >&2 || true
  exit 1
fi

echo "[A1 v2-min] start Unitree driver"
nohup bash --noprofile --norc -c "cd $HOME/unitree_legged_sdk; export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$(pwd)/lib; printf '\n' | sudo -n -E '$DRIVER_BIN'" > "$LOG_DIR/high_follow_driver.log" 2>&1 &
echo $! > "$LOG_DIR/high_follow_driver.pid"
sleep 2

if ! pgrep -f '[a]1_high_follow_driver' >/dev/null 2>&1; then
  echo "[A1 v2-min] ERROR: Unitree driver is not running" >&2
  tail -120 "$LOG_DIR/high_follow_driver.log" >&2 || true
  ps aux | grep -E 'base_controller_node|slam_planner_node|slamware_ros_sdk_server_node|a1_high_follow_driver' | grep -v grep >&2 || true
  exit 1
fi

echo "[A1 v2-min] process summary"
ps aux | grep -E 'a1_follow_lowlatency_depth_server|a1_high_follow_driver|base_controller_node|slam_planner_node|slamware_ros_sdk_server_node|roslaunch' | grep -v grep || true

echo "[A1 v2-min] DONE"
echo "logs: $LOG_DIR"

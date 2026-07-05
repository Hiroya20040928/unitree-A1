#!/usr/bin/env bash
set -euo pipefail

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/home/unitree/a1_nav_logs/${TS}"
mkdir -p "${LOG_DIR}"

echo "[A1NAV] log_dir=${LOG_DIR}"

pkill -f rostopic || true
pkill -f roslaunch || true
pkill -f roscore || true
pkill -f move_base || true
pkill -f slam_planner_node || true
pkill -f slamware_ros_sdk_server_node || true
pkill -f base_controller_node || true
pkill -f a1_drift_guard_node.py || true
pkill -f a1_lateral_guard_node.py || true
pkill -f a1_cmd_vel_bias.py || true
pkill -f lcm_server_high || true
sleep 2

a1clean 2>/dev/null || true
pkill -f lcm_server_high || true

sudo ip addr flush dev eth0
sudo ip link set eth0 down
sudo ip link set eth0 up
sudo ip addr add 192.168.11.100/24 dev eth0
sudo ip addr add 192.168.123.162/24 dev eth0

{
  echo "===== date ====="
  date
  echo "===== ip addr ====="
  ip addr show eth0
  echo "===== route ====="
  ip route get 192.168.11.1 || true
  ip route get 192.168.123.161 || true
  echo "===== ping slamware ====="
  ping -c 2 192.168.11.1 || true
  echo "===== slamware tcp ====="
  nc -vz 192.168.11.1 1445 || true
  echo "===== ping atom ====="
  ping -c 2 192.168.123.161 || true
} | tee "${LOG_DIR}/preflight.txt"

source /opt/ros/melodic/setup.bash
source /home/unitree/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1

cp -a /home/unitree/catkin_ws/src/slamrplidar/slam_planner/launch "${LOG_DIR}/launch_snapshot"
cp -a /home/unitree/catkin_ws/src/slamrplidar/slam_planner/params "${LOG_DIR}/params_snapshot"
cp -a /home/unitree/catkin_ws/src/slamrplidar/slam_planner/scripts "${LOG_DIR}/scripts_snapshot"

echo "${LOG_DIR}" > /home/unitree/a1_nav_logs/latest_path.txt
rm -f /home/unitree/a1_nav_logs/latest
ln -s "${LOG_DIR}" /home/unitree/a1_nav_logs/latest

roslaunch slam_planner a1_move_base_teb_logged.launch log_dir:="${LOG_DIR}" 2>&1 | tee "${LOG_DIR}/roslaunch_console.log"

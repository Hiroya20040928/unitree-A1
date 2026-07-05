#!/usr/bin/env bash
set -x

echo "[A1NAV_PURE] script started"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/home/unitree/a1_nav_logs/${TS}"
mkdir -p "${LOG_DIR}"

echo "[A1NAV_PURE] log_dir=${LOG_DIR}"
echo "${LOG_DIR}" > /home/unitree/a1_nav_logs/latest_path.txt

pkill -f roslaunch || true
pkill -f roscore || true
pkill -f move_base || true
pkill -f amcl || true
pkill -f map_server || true
pkill -f slamware_ros_sdk_server_node || true
pkill -f base_controller_node || true
pkill -f a1_odom_to_tf.py || true
pkill -f a1_lateral_guard_node.py || true
pkill -f a1_drift_guard_node.py || true
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

ip addr show eth0 | tee "${LOG_DIR}/ip_addr.txt"
ip route get 192.168.11.1 | tee "${LOG_DIR}/route_slamware.txt"
ip route get 192.168.123.161 | tee "${LOG_DIR}/route_atom.txt"

ping -c 2 192.168.11.1 | tee "${LOG_DIR}/ping_slamware.txt"
nc -vz 192.168.11.1 1445 2>&1 | tee "${LOG_DIR}/nc_slamware_1445.txt"
ping -c 2 192.168.123.161 | tee "${LOG_DIR}/ping_atom.txt"

source /opt/ros/melodic/setup.bash
source /home/unitree/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1

echo "[A1NAV_PURE] starting roslaunch"

roslaunch slam_planner a1_saved_map_nav_pure.launch 2>&1 | tee "${LOG_DIR}/roslaunch_console.log"

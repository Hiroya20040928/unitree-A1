#!/usr/bin/env bash
set +e

echo "[A1 STOP v6.8.2] send stop command"
curl -fsS "http://127.0.0.1:8090/follow_stop" >/dev/null 2>&1 || true
printf '0 0.00000 0.00000 0.00000 %.6f\n' "$(date +%s.%N)" > /tmp/a1_follow_cmd_raw
printf '0 0.00000 0.00000 0.00000 %.6f\n' "$(date +%s.%N)" > /tmp/a1_follow_cmd
sleep 0.3

echo "[A1 STOP v6.8.2] killing follow stack"
sudo pkill -9 -f '[a]1_high_follow_driver' 2>/dev/null || true
sudo pkill -9 -f '[a]1_high_follow_driver_v2' 2>/dev/null || true
pkill -9 -f '[a]1_lidar_footprint_filter_v6_8.py' 2>/dev/null || true
pkill -9 -f '[a]1_lidar_footprint_filter_v6_8_2.py' 2>/dev/null || true
pkill -9 -f '[a]1_body_footprint_filter' 2>/dev/null || true
pkill -9 -f '[a]1_cmd_passthrough_loop.sh' 2>/dev/null || true
pkill -9 -f '[a]1_follow_lowlatency_depth_server_raw' 2>/dev/null || true
pkill -9 -f '[b]ase_controller_node' 2>/dev/null || true
pkill -9 -f '[l]cm_server_high' 2>/dev/null || true
pkill -9 -f '[e]xample_walk' 2>/dev/null || true
pkill -9 -f '[r]oslaunch' 2>/dev/null || true
pkill -9 -f '[r]oscore' 2>/dev/null || true
pkill -9 -f '[s]lamware_ros_sdk_server_node' 2>/dev/null || true

echo "[A1 STOP v6.8.2] DONE"

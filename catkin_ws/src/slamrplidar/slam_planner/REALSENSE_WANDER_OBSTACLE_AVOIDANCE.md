# RealSense Wander Obstacle Avoidance

## What was actually detected

This machine does **not** have a USB `RPLIDAR A1` serial device attached.

What is actually present on USB is:

- `Intel RealSense D435i`
- serial: `044322070839`
- physical port: `2-3.3-6`

The program below uses that USB depth camera directly for obstacle avoidance and wandering.

## Current blocker on this machine

The currently loaded `uvcvideo` module is **not** RealSense-patched.

- `modinfo uvcvideo` does **not** contain `realsense`
- `/dev/video*` nodes are missing
- the D435i enumerates, but depth frames time out

So the first one-time fix is to install the RealSense DKMS package and reboot.

## Behavior

- Moves forward when the center depth ROI is clear.
- Starts turning when the forward path gets tighter than `0.40 m`.
- Treats `0.30 m` as emergency stop-turn distance.
- Uses left / center / right depth sectors and small random turns so it does not just go straight forever.

## One-command start

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
./scripts/run_realsense_wander_obstacle_avoidance.sh
```

If the driver is still unpatched, the script stops immediately and tells you to run the installer below.

## One-time driver repair

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
chmod +x scripts/install_realsense_dkms.sh
./scripts/install_realsense_dkms.sh
sudo reboot
```

## Direct launch

Terminal 1:

```bash
sudo ~/unitree_legged_sdk/build/lcm_server_high
```

Terminal 2:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_planner realsense_wander_obstacle_avoidance.launch serial_no:=044322070839
```

## After reboot from zero

1. Login on the robot.
2. Build once if the workspace changed:

```bash
cd ~/catkin_ws
catkin_make
```

3. Start the program:

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
./scripts/run_realsense_wander_obstacle_avoidance.sh
```

## Stop

- `Ctrl-C` in the launch terminal.
- The node publishes zero velocity on shutdown.

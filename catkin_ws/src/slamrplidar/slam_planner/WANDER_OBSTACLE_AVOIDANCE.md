# Wander Obstacle Avoidance

## What it does

`wander_obstacle_avoidance.py` uses only `/scan` and publishes `/cmd_vel`.

- If the front is clear, it walks forward.
- If obstacles get close, it turns away from the denser side.
- It occasionally injects small random turns so it does not just drive straight forever.
- If `/scan` stops arriving, it publishes zero velocity.
- Default clearances are tuned to allow approach down to about `0.30 m`.

## Full start command

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
chmod +x scripts/wander_obstacle_avoidance.py
sudo ~/unitree_legged_sdk/build/lcm_server_high
```

In another terminal:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_planner wander_obstacle_avoidance.launch ip_address:=192.168.11.1 start_rviz:=true
```

## Start on an already-running SLAMWare ROS session

If `/slamware_ros_sdk_server_node` is already running, do not start it again:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_planner wander_obstacle_avoidance.launch start_slamware:=false start_rviz:=false
```

## Required network condition

The SLAMWare device must actually be reachable and publishing `/scan`.

Typical host-side setup for direct Ethernet:

```bash
sudo ip link set dev eth0 up
sudo ip addr replace 192.168.11.2/24 dev eth0
ping -c 3 192.168.11.1
```

## Stop

- `Ctrl-C` in the launch terminal.
- The node publishes a zero `Twist` on shutdown.

# TEB Via-Points Teleop

## What was added

- `src/teb_via_teleop.cpp`
  - subscribes to `/teb_via_teleop/cmd_vel`
  - converts operator twist into a short-horizon `nav_msgs/Path`
  - publishes that path to `/move_base/TebLocalPlannerROS/via_points`
  - sends the path endpoint as a rolling `move_base` goal
- `scripts/teb_via_keyboard.py`
  - keyboard publisher for `/teb_via_teleop/cmd_vel`
- `params/teb_via_teleop_params.yaml`
  - TEB parameters for ordered custom via-points
- `launch/teb_via_teleop.launch`
  - starts SLAMWare server, `move_base`, `base_controller_node`, and `teb_via_teleop`

## Important runtime note

This workspace now builds `slam_planner`, but the actual `teb_local_planner` runtime package is still blocked on this machine by external network access to `packages.ros.org`.

- The cloned `~/catkin_ws/src/teb_local_planner` was marked with `CATKIN_IGNORE` so the rest of the workspace can build.
- The teleop node itself is built and ready.
- The final launch will only work after `teb_local_planner` is actually installable or buildable on this machine.

## Verified local facts

- `teb_local_planner` subscribes to `~/via_points`
- message type is `nav_msgs/Path`
- custom via-points are ignored if `global_plan_viapoint_sep > 0`

The implementation here is aligned to that contract:

- topic: `/move_base/TebLocalPlannerROS/via_points`
- parameter: `global_plan_viapoint_sep: -1.0`
- parameter: `via_points_ordered: true`

## Build

```bash
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws
catkin_make --pkg slam_planner
source ~/catkin_ws/devel/setup.bash
```

## Full start commands

### 1. Configure the SLAMWare Ethernet link

```bash
sudo ip link set dev eth0 up
sudo ip addr replace 192.168.11.2/24 dev eth0
ping -c 3 192.168.11.1
```

### 2. Start SLAMWare + move_base + Unitree base controller + via-points teleop bridge

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_planner teb_via_teleop.launch ip_address:=192.168.11.1 start_rviz:=true
```

Do not start `~/unitree_legged_sdk/build/lcm_server_high` separately for this launch.
`base_controller_node` already opens the Unitree high-level LCM channel, so starting both causes the bind failure:

```text
Error: Bind client ip&port failed.
```

### 3. Start keyboard teleoperation in a separate terminal

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun slam_planner teb_via_keyboard.py _cmd_vel_topic:=/teb_via_teleop/cmd_vel
```

The Unitree handheld remote is not part of this control path.
This stack only reacts to ROS commands published to `/teb_via_teleop/cmd_vel`.
If you drive the robot with the physical remote, neither TEB nor `move_base` can intervene.

The current default local planner is `dwa_local_planner/DWAPlannerROS`, because `teb_local_planner` is not installed on this machine.
That makes the launch usable immediately with `roslaunch slam_planner teb_via_teleop.launch`.

## Optional TEB mode when the plugin is installed

If `move_base` dies with:

```text
Failed to create the teb_local_planner/TebLocalPlannerROS planner
```

you can switch back to TEB with:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch slam_planner teb_via_teleop.launch ip_address:=192.168.11.1 start_rviz:=true local_planner:=teb_local_planner/TebLocalPlannerROS
```

In the current default DWA mode, the rolling-goal teleop bridge remains active, but `via_points` are ignored because DWA does not subscribe to them.

## Keyboard controls

- `w`: forward
- `s`: backward
- `a`: rotate left
- `d`: rotate right
- `x` or `Space`: stop
- `Ctrl-C`: quit

## Stop

- `Ctrl-C` in the keyboard terminal
- `Ctrl-C` in the launch terminal

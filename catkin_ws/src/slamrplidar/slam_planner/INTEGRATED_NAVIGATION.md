# Integrated Mapping And Navigation

## What this starts

`run_integrated_mapping_navigation.sh` starts one integrated workflow:

1. `lcm_server_high`
2. `slamware_ros_sdk_server_node`
3. RViz with the live map and `2D Nav Goal`
4. `integrated_mapping_navigation_manager.py`

By default it also configures `eth0` to the target SLAMWare subnet if needed.  
Example: target `192.168.11.1` becomes host `192.168.11.2/24`.

If no existing map bundle is given, it starts in **mapping mode**.  
If an existing map bundle is given, it starts in **navigation mode** immediately.

## Mapping mode

- Walk the robot with the official Unitree remote controller.
- RViz shows the map being built continuously.
- If the SLAMWare is not reachable yet, the manager stays alive and keeps waiting.
- When mapping is finished, press `n` in the terminal that launched the program.
- The manager then:
  - freezes the SLAM map
  - saves `map.stcm`
  - saves `map.yaml` and `map.pgm`
  - saves the current robot pose into `metadata.yaml`
  - starts the navigation stack

After that, use RViz `2D Nav Goal` to set the destination.

## Navigation mode

- The navigation stack uses the frozen map as the global map.
- `/scan` is still used for local obstacle avoidance.
- `base_controller_node` sends `/cmd_vel` to the robot.

## Saved bundle format

Each saved bundle directory contains:

- `map.stcm`
- `map.yaml`
- `map.pgm`
- `metadata.yaml`

`map.stcm` is used for restart with automatic pose restoration.  
`map.yaml` and `map.pgm` are kept as standard 2D map exports.

## Start command

Create a new map:

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
./scripts/run_integrated_mapping_navigation.sh
```

Load an existing map bundle and start directly in navigation mode:

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
./scripts/run_integrated_mapping_navigation.sh --map-bundle ~/catkin_ws/src/slamrplidar/slam_planner/maps/generated/my_map
```

Save to a stable name:

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
./scripts/run_integrated_mapping_navigation.sh --save-name office_map
```

Use a different SLAMWare IP:

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
./scripts/run_integrated_mapping_navigation.sh --ip 192.168.153.1
```

Force a specific host NIC and address:

```bash
cd ~/catkin_ws/src/slamrplidar/slam_planner
./scripts/run_integrated_mapping_navigation.sh --ip 192.168.11.1 --host-interface eth0 --host-address 192.168.11.2/24
```

## Notes

- The transition key is `n`.
- The quit key is `q`.
- If you see `Interface eth0 has no physical carrier`, the Ethernet cable is not linked.
- The official remote controller is used only during mapping mode.
- After navigation mode starts, stop manual walking and set the goal from RViz.

# Project Instructions

This repository contains a ROS 2 Jazzy tracked-robot project. The source workspace lives under `ros_ws/src`, and the robot runtime workspace on the Raspberry Pi is `/home/yy/ros2_ws`.

Use this file as the first project context for future coding agents.

## Current Environment

- Local repo: `/home/zjy/Desktop/ros_project_git_2026-05-20`
- Local ROS workspace: `/home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws`
- Robot host: `yy@192.168.43.21`
- Robot source directory: `/home/yy/ros2_ws/src`
- ROS distro: `jazzy`
- Main branch remote: `git@github.com:forwardzz/ros_project.git`

Always treat the git tree as potentially dirty. Do not revert user changes unless the user explicitly asks for that exact rollback.

## Main Packages

- `mapping_bringup`: launch files and robot-specific Python nodes.
  - `launch/mapping.launch.py`: mapping pipeline.
  - `launch/navigation.launch.py`: navigation pipeline.
  - `mapping_bringup/tracked_motor_driver.py`: GPIO motor driver subscribing to `/cmd_vel`.
  - `mapping_bringup/mission_manager.py`: RViz mission-point handling and mission execution.
  - `config/nav2_params.yaml`: Nav2 controller, planner, costmap, AMCL, and smoother parameters.
  - `config/slam.yaml`: `slam_toolbox` mapping parameters.
- `sllidar_ros2`: SLLIDAR/RPLIDAR driver publishing `/scan`.
- `rf2o_laser_odometry`: laser odometry publishing `/odom` and `odom -> base_link` TF.
- `robot_control_ui`: Tkinter desktop UI, normally run on the host PC, not on the Raspberry Pi display.
- `robot_monitor_interfaces`: mission, gas, and UI-related custom messages/services.
- `robot_mission_utils`: helper planning utilities used by mission preview/validation.
- `imu_ros2_device` and `YbImuLib`: retained in the repo, but IMU is not part of the default mapping/navigation chain.

## Runtime Architecture

The current default robot stack is lidar-first and does not use IMU fusion.

Mapping mode starts:

- `sllidar_ros2`
- static `base_link -> laser`
- `rf2o_laser_odometry`
- `tracked_motor_driver`
- `slam_toolbox`

Navigation mode starts:

- `sllidar_ros2`
- static `base_link -> laser`
- `rf2o_laser_odometry`
- `tracked_motor_driver`
- `mission_manager`
- Nav2: `map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `smoother_server`, `velocity_smoother`, lifecycle manager

Do not run mapping and navigation at the same time. `slam_toolbox` and `map_server + amcl` both participate in the `map` chain and can create TF/localization conflicts if run together.

## Build Commands

From the local or robot workspace:

```bash
cd /home/yy/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Common selective builds:

```bash
cd /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mapping_bringup
colcon build --packages-select robot_control_ui
colcon build --packages-select robot_monitor_interfaces mapping_bringup robot_control_ui
```

If building on the robot, run from `/home/yy/ros2_ws`, not `/home`; otherwise `colcon` may fail creating `log/`.

## Launch Commands

Mapping on robot:

```bash
source /opt/ros/jazzy/setup.bash
source /home/yy/ros2_ws/install/setup.bash
ros2 launch mapping_bringup mapping.launch.py
```

Navigation on robot:

```bash
source /opt/ros/jazzy/setup.bash
source /home/yy/ros2_ws/install/setup.bash
ros2 launch mapping_bringup navigation.launch.py map:=/home/yy/ros2_ws/map_name.yaml
```

Host UI:

```bash
source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch robot_control_ui ui.launch.py \
  remote_user:=yy \
  remote_host:=192.168.43.21 \
  workspace_path:=/home/yy/ros2_ws \
  map_path:=/home/yy/ros2_ws/map_name.yaml
```

RViz recommended config:

```bash
source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
rviz2 -d /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/src/sllidar_ros2/rviz/sllidar_ros2.rviz
```

## RViz Mission Workflow

Use RViz for mission points. The UI should only confirm and start the mission.

- `Publish Point` publishes `/clicked_point` and adds a mission point.
- `2D Goal Pose (Nav)` publishes `/goal_pose` and immediately starts normal Nav2 navigation.
- `2D Goal Pose (Mission Heading)` publishes `/mission_goal_pose` and only updates the nearest mission point heading.
- `Clear RViz Points` clears mission-manager cached RViz points and preview markers.

Mission execution must respect the operator-provided point order. Do not silently reorder RViz mission points during execution.

## UI Notes

The UI is a Tkinter application intended to run on the host PC. It subscribes to robot ROS topics over DDS and uses SSH to launch/stop processes on the Raspberry Pi.

Current UI layout is intentionally preserved by user request. Be careful when changing `robot_control_ui.py`; layout regressions are easy because status cards, runtime log, thermal panel, map, and manual drive all compete for vertical space.

If the UI appears unchanged after a code edit, kill stale UI processes before retesting:

```bash
pkill -f robot_control_ui
pkill -f "ros2 launch robot_control_ui"
```

Then re-source the intended local workspace and launch again.

## Networking And QoS

- Host PC and Raspberry Pi must share the same ROS domain.
- Use `ROS_DOMAIN_ID=0` unless the user says otherwise.
- Use `ROS_LOCALHOST_ONLY=0` for cross-machine discovery.
- `/map` subscribers should use transient-local durability and reliable QoS.
- `/scan` should use sensor-data QoS or best-effort-compatible settings.

ROS 2 graph tools on the host will show robot-side nodes because DDS discovers the whole network graph. Seeing all nodes in `rqt_graph` does not mean they run locally.

## Hardware Notes

- Lidar is expected on a stable by-path serial device, not plain `ttyUSB0`.
- Gas sensor should not fall back to the lidar serial port.
- Thermal and gas sensor monitoring are separate from mapping/navigation and should be launched only when needed.
- The motor driver is open-loop GPIO/PWM; there is no encoder feedback in the current code.
- The driver board voltage display is not automatically available to ROS unless there is a telemetry interface, ADC, CAN, UART, I2C, or another readable data path.

## Navigation Tuning Notes

Important files:

- `ros_ws/src/mapping_bringup/config/nav2_params.yaml`
- `ros_ws/src/mapping_bringup/mapping_bringup/mission_manager.py`
- `ros_ws/src/mapping_bringup/launch/navigation.launch.py`

Known sensitive parameters:

- `FollowPath.use_rotate_to_heading`
- `FollowPath.rotate_to_heading_min_angle`
- `FollowPath.allow_reversing`
- `general_goal_checker.xy_goal_tolerance`
- `local_costmap` and `global_costmap` `footprint`, `resolution`, `inflation_radius`, `cost_scaling_factor`

YAML booleans must be real booleans (`true` / `false`), not quoted strings or misspellings. A typo such as `flase` will crash `controller_server` during lifecycle configure.

## Validation Checklist

Before saying a change is done, prefer at least one of:

```bash
cd /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select <changed_package>
```

For robot-side changes, sync to `yy@192.168.43.21:/home/yy/ros2_ws/src/...` and build on the robot when the user expects immediate testing.

Useful runtime checks:

```bash
ros2 topic list
ros2 topic info /map -v
ros2 topic info /scan -v
ros2 topic echo /cmd_vel
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

## Git And Generated Files

Do not commit build outputs:

- `ros_ws/build/`
- `ros_ws/install/`
- `ros_ws/log/`

Avoid touching deleted legacy `robot_monitor_ws_src` files unless the user specifically asks to restore or inspect them.

Keep commits focused. If there are unrelated dirty files, leave them alone and report that they were not included.

# Project Instructions

ROS 2 Jazzy tracked-robot project. Robot runs on a Raspberry Pi 5; the desktop UI runs on the host PC and controls the robot over DDS + SSH. Read this file first.

## Environment

- Local repo (host PC): `/home/zjy/Desktop/ros_project` — paths in older docs pointing at `ros_project_git_2026-05-20` are stale; use this one.
- Local ROS workspace: `/home/zjy/Desktop/ros_project/ros_ws`
- Robot host: `yy@<robot_ip>` (documented as `192.168.43.21`; the UI launch file's default is `192.168.43.16` — always override with the real robot IP via `remote_host:=`)
- Robot workspace: `/home/yy/ros2_ws` (sources mirrored under `/home/yy/ros2_ws/src`)
- ROS distro: `jazzy`
- Remote: `https://github.com/forwardzz/ros_project.git`

Treat the git tree as potentially dirty. Do not revert user changes unless explicitly asked for that rollback.

## Packages

- `mapping_bringup`: launch files + robot Python nodes (the project entrypoint).
  - `launch/mapping.launch.py`, `launch/navigation.launch.py`, `launch/sensor_monitor.launch.py`
  - `mapping_bringup/tracked_motor_driver.py`: open-loop GPIO/PWM motor driver on `/cmd_vel`.
  - `mapping_bringup/mission_manager.py`: RViz mission-point handling and execution.
  - `mapping_bringup/safety_monitor.py`: undervoltage / collision / no-progress guard; latches fault and cancels nav (runs in navigation chain only).
  - `mapping_bringup/actual_path_recorder.py`: records executed path.
  - `mapping_bringup/gas_sensor_node.py`, `mapping_bringup/thermal_camera_node.py`: sensors, launched only via `sensor_monitor.launch.py`.
  - `config/nav2_params.yaml`, `config/slam.yaml`, `config/ekf.yaml`, `config/imu_filter_param.yaml`, `config/navigate_to_pose_no_spin.xml`, `config/navigate_through_poses_no_spin.xml`
- `sllidar_ros2`: RPLIDAR driver → `/scan`.
- `rf2o_laser_odometry`: laser odometry. Now publishes **`/laser_odom`** with `publish_tf: false`; it no longer owns `/odom` or the TF.
- `robot_control_ui`: Tkinter + rclpy desktop UI; runs on the host PC, not the Pi.
- `robot_monitor_interfaces`: custom mission/gas/UI msgs and services.
- `robot_mission_utils`: mission planning/preview helpers.
- `robot_localization` (system pkg): EKF fuses `/laser_odom` + IMU, owns `/odom` and `odom -> base_link` TF.
- `imu_ros2_device` + `YbImuLib`: YB IMU driver (`ybimu_node`) — **now part of both mapping and navigation chains** (older docs say "not used" — that is stale).

## Runtime Architecture

Both chains are now lidar + IMU fused through EKF (the old "lidar-first, no IMU" description is outdated — trust the launch files).

Mapping (`mapping.launch.py`): `sllidar` → static `base_link->laser`, `base_link->imu` → `ybimu_node` → `rf2o` (`/laser_odom`) → `ekf` (`/odom`, TF) → `tracked_motor_driver` → `slam_toolbox`.

Navigation (`navigation.launch.py`): same sensing stack, plus `mission_manager`, `actual_path_recorder`, `safety_monitor`, and the Nav2 lifecycle set: `map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator` (custom **no-spin** BT XMLs), `behavior_server`, `smoother_server`, `velocity_smoother`, `lifecycle_manager_navigation`.

Velocity wiring: `controller_server` remaps `/cmd_vel`→`/cmd_vel_nav`; `velocity_smoother` takes `/cmd_vel`→`/cmd_vel_nav` and outputs `/cmd_vel_smoothed`→`/cmd_vel`, which the motor driver consumes.

Never run mapping and navigation at the same time — `slam_toolbox` and `map_server + amcl` both own the `map` chain and will fight over TF/localization.

## Build

```bash
cd /home/zjy/Desktop/ros_project/ros_ws   # host; on the Pi use /home/yy/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Selective:

```bash
colcon build --packages-select mapping_bringup
colcon build --packages-select robot_control_ui
colcon build --packages-select robot_monitor_interfaces mapping_bringup robot_control_ui
```

On the robot, build from `/home/yy/ros2_ws`, not `/home` (colcon fails to create `log/` there). Note: the workspace dir above is `ros_ws` (typo-free); if older docs write `ros_w`, that's wrong.

## Launch

Mapping (on robot):

```bash
source /opt/ros/jazzy/setup.bash
source /home/yy/ros2_ws/install/setup.bash
ros2 launch mapping_bringup mapping.launch.py
```

Navigation (on robot):

```bash
source /opt/ros/jazzy/setup.bash
source /home/yy/ros2_ws/install/setup.bash
ros2 launch mapping_bringup navigation.launch.py map:=/home/yy/ros2_ws/map_name.yaml
```

Save a map: `ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/map_name`

Host UI (override `remote_host`!):

```bash
source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch robot_control_ui ui.launch.py \
  remote_user:=yy \
  remote_host:=<robot_ip> \
  workspace_path:=/home/yy/ros2_ws \
  map_path:=/home/yy/ros2_ws/map_name.yaml
```

Host RViz:

```bash
source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project/ros_ws/install/setup.bash
rviz2 -d /home/zjy/Desktop/ros_project/ros_ws/src/sllidar_ros2/rviz/sllidar_ros2.rviz
```

## Networking and QoS

- Host and Pi must share the same ROS domain: `ROS_DOMAIN_ID=0` and `ROS_LOCALHOST_ONLY=0`.
- `/map`: subscribers must use `Reliable` + `Transient Local` durability, or they miss the already-published map on reconnect/refresh.
- `/scan`: use `sensor_data` (best-effort compatible) QoS.
- `ros2 node/topic list` and `rqt_graph` on the host show Pi-side nodes via DDS discovery — seeing them locally does **not** mean they run locally.
- Both launch files set `CYCLONEDDS_URI` inline; mixing DDS implementations across machines breaks discovery.

## RViz Mission Workflow

Mission points come from RViz; the UI only confirms and starts.

- `Publish Point` → `/clicked_point` → adds a mission point (also auto-recalculates order + preview).
- `2D Goal Pose (Nav)` → `/goal_pose` → immediate Nav2 navigation (does not touch mission points).
- `2D Goal Pose (Mission Heading)` → `/mission_goal_pose` → updates heading of the nearest existing RViz mission point only; rejected if not near one; never triggers motion.
- `Clear RViz Points` (in UI) clears cached mission points, markers, and `/mission_preview_path`.

Two path topics differ on purpose: `/mission_preview_path` is the offline planning preview; `/plan` is Nav2's actual global path. They diverging during exec/avoidance is normal.

Preserve operator-provided point order. Do not silently reorder mission points during execution.

`Start Mission` does pre-flight checks: points can't be in/unknown/unreachable cells, first point can't be too close to the robot, and an order must yield a collision-free preview — otherwise the UI rejects it without sending to Nav2.

If a safety fault latches, `Start Mission` is blocked until UI `Reset Safety`.

## Hardware Notes

- Lidar must be on the stable by-path serial device `platform-xhci-hcd.0-usb-0:2:1.0-port0` (declared as the launch default), not plain `ttyUSB0`.
- IMU is on `/dev/ttyAMA0`.
- Gas/thermal are independent of mapping/navigation; launch them only via `sensor_monitor.launch.py` (or UI `Start Thermal`, which starts both). They must not fall back to the lidar serial port.
- Motor driver is open-loop PWM; no encoder feedback exists in the code.
- Board voltage is not exposed to ROS unless a telemetry path is added.

## Navigation Tuning

Sensitive files: `config/nav2_params.yaml`, `mapping_bringup/mission_manager.py`, `launch/navigation.launch.py`.

Sensitive params: `FollowPath.use_rotate_to_heading`, `FollowPath.rotate_to_heading_min_angle`, `FollowPath.allow_reversing`, `general_goal_checker.xy_goal_tolerance`, costmap `footprint`/`resolution`/`inflation_radius`/`cost_scaling_factor`.

Nav uses **no-spin** custom BT XMLs (`navigate_to_pose_no_spin.xml`, `navigate_through_poses_no_spin.xml`); spin-in-place is intentionally avoided for this tracked chassis.

YAML booleans must be real `true`/`false` (not quoted, not misspelled). A typo like `flase` crashes `controller_server` during lifecycle configure.

`use_rviz` launch arg is declared but not wired into either launch file (known limitation).

EKF config (`ekf.yaml`): currently fuses IMU yaw rate only — magnetometer yaw and linear acceleration are deliberately excluded until mounting/drift are field-checked. Don't blindly enable them.

`lidar_yaw` (default `3.1415926`) must be identical between mapping and navigation; changing it requires rebuilding the map.

## Validation

Before marking done, at minimum build the changed package:

```bash
cd /home/zjy/Desktop/ros_project/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select <changed_package>
```

For robot-side changes, sync to `yy@<robot_ip>:/home/yy/ros2_ws/src/...` and build on the Pi when the user expects to test live.

Runtime checks:

```bash
ros2 topic list
ros2 topic info /map -v
ros2 topic info /scan -v
ros2 topic echo /cmd_vel
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

There is no unit-test suite in this repo; the colcon build is the primary automated check.

## Git and Generated Files

`.gitignore` already covers `build/`, `install/`, `log/`, `dist/`, `*.egg-info/`, `__pycache__/`. Do not commit those.

Keep commits focused; leave unrelated dirty files alone and report they were excluded.

`robot_monitor_ws_src` legacy files are deleted — avoid restoring/inspecting unless explicitly asked.

## UI Gotchas

`robot_control_ui.py` layout is intentionally preserved by user request; status cards, runtime log, thermal panel, map, and manual drive all compete for vertical space — layout regressions are easy.

If the UI looks unchanged after an edit, stale processes are likely cached. Kill before retesting:

```bash
pkill -f robot_control_ui
pkill -f "ros2 launch robot_control_ui"
```

Then re-source the intended local workspace and relaunch.

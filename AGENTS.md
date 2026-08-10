# Project Instructions

This repository contains a ROS 2 Jazzy tracked-robot project. The source workspace lives under `ros_ws/src`, and the robot runtime workspace on the Raspberry Pi is `/home/yy/ros2_ws`.

Use this file as the first project context for future coding agents.

## Current Environment

- Local repo: `/home/zjy/Desktop/ros_project_git_2026-05-20`
- Local ROS workspace: `/home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws`
- Robot host: `yy@192.168.43.30`
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

## Launch Command

The supported project entry point is only:

```bash
cd /home/zjy/Desktop/ros_project_git_2026-05-20
./start.sh
```

`start.sh` builds the affected local packages, enforces a single local console, and starts the host Tk UI plus RViz. It does not automatically start mapping or navigation on the robot. Start and stop the selected robot mode through the UI. Do not launch a second UI/RViz manually.

## RViz Mission Workflow

Use RViz for mission points. The UI should only confirm and start the mission.

- `Publish Point` publishes `/clicked_point` and adds a mission point.
- `2D Goal Pose (Nav)` publishes `/goal_pose` and immediately starts normal Nav2 navigation.
- `2D Goal Pose (Mission Heading)` publishes `/mission_goal_pose` and only updates the nearest mission point heading.
- `Clear RViz Points` clears mission-manager cached RViz points and preview markers.

Mission ordering is explicit in the Tk UI. `TSP` defaults on and optimizes ordinary points and regions; when it is off, execution must preserve the operator-provided point/region order. Never reorder without reflecting the selected mode in the UI and mission status.

## UI Notes

The UI is a Tkinter application intended to run on the host PC. It subscribes to robot ROS topics over DDS and uses SSH to launch/stop processes on the Raspberry Pi.

Current UI layout is intentionally preserved by user request. Be careful when changing `robot_control_ui.py`; layout regressions are easy because status cards, runtime log, thermal panel, map, and manual drive all compete for vertical space.

If the UI appears unchanged after a code edit, close the current `start.sh` console and run `./start.sh` again. Its exact local cleanup avoids duplicate UI/RViz processes.

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

- `planner_server.GridBased` SmacPlanner2D parameters and `allow_unknown`
- `global_costmap` live laser obstacle layer, resolution, and inflation settings
- `navigate_to_pose_no_spin.xml` / `navigate_through_poses_no_spin.xml` path-validity replanning flow
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

For robot-side changes, sync to `yy@192.168.43.30:/home/yy/ros2_ws/src/...` and build on the robot when the user expects immediate testing.

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

## UI 网络与状态监控（2026-08 改造）

- 默认机器人 SSH 地址为 `yy@192.168.43.30`，可用 `REMOTE_HOST=<addr> ./start.sh` 覆盖。
- Mission Control 的 `Scan LAN` / `Refresh` / `Apply IP` 执行局域网尽力发现（邻居表 + TCP 22 探测，不要求管理员权限，不依赖 nmap）；任务运行中禁止切换地址。
- `Robot Status` 的系统卡片通过 `RemoteHealthProbe`（后台 SSH，`BatchMode=yes` + `ConnectTimeout=2`）采集温度/CPU/内存/负载/运行时间/欠压，断线保留末值并标记过期。
- 电压边界：`vcgencmd measure_volts` 是 CPU 核心电压；5V 输入电压在无 ADC/遥测路径时一律显示 `N/A (no ADC)`。
- ROS 话题表结合运行模式（idle/mapping/navigation）分类；`/map` 按"是否收到 + 是否有发布者"判定，不再用两秒消息超时。
- 所有扫描与 SSH 探测在后台线程执行，控件更新经 `root.after` 回到主线程。

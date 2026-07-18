# Repository Guidelines

## Project Structure & Module Organization

The repository contains one ROS 2 Jazzy workspace under `inspection_migrated_ws/`. Package sources live in `inspection_migrated_ws/src/`: `inspection_robot_bringup` owns launch, Nav2, EKF, behavior-tree, map, and RViz configuration; `inspection_robot_mission` and `robot_mission_utils` implement mission logic and planners; `inspection_robot_gui` provides the PyQt interface; hardware and safety nodes are separated into `inspection_robot_hardware` and `inspection_robot_safety`. C++ controllers and drivers use `include/` and `src/`; Python packages keep modules in a same-named directory. Interfaces are defined in `robot_monitor_interfaces/{msg,srv}`. Treat `YbImuLib`, `sllidar_ros2`, and `rf2o_laser_odometry` as imported driver code and avoid broad refactors there.

## Build, Test, and Development Commands

Run commands from the workspace:

```bash
cd inspection_migrated_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

Use `colcon build --packages-select <package>` and the matching `colcon test` option for focused changes. After building, `./start.sh gui`, `./start.sh rviz`, or `./start.sh all` launches local tools. See `start.md` for remote robot arguments and networking variables.

## Coding Style & Naming Conventions

Python uses four spaces, PEP 8 imports, `snake_case` functions and modules, and `CapWords` classes. C++ follows the existing ROS 2 style: two-space indentation, braces on separate lines, and headers under `include/<package>/`. Keep ROS package, node, topic, parameter, launch, and YAML names lowercase with underscores. Preserve compiler warnings (`-Wall -Wextra -Wpedantic`) and existing ament lint configuration.

## Testing Guidelines

Place Python tests in `<package>/test/test_*.py`; use `pytest` and ament linters (`ament_flake8`, `ament_pep257`) where configured. Add package-level tests for changed planners, safety decisions, and controller behavior. There is no formal coverage threshold, so PRs should state what was exercised and identify hardware-only paths that remain unverified.

## Commit & Pull Request Guidelines

Follow the history’s concise, imperative subjects, such as `Add safety monitor` or `Preserve mission waypoint order`. Keep each commit scoped to one behavior. PRs should describe affected packages and runtime impact, link relevant issues, list build/test commands, and include screenshots for GUI or RViz changes. Document any robot-side validation and rollback steps.

## Safety & Configuration

Read `HARDWARE_SAFETY_REVIEW.md` before real-hardware tests. Never bypass `velocity_safety_gate`, fabricate sensor data, or energize motors without verified device paths, GPIO permissions, transforms, and emergency-stop behavior. Do not commit host-specific credentials, SSH keys, or generated `build/`, `install/`, and `log/` directories.

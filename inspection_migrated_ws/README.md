# inspection_migrated_ws

这是从 `inspection_sim_ws` 的任务/算法逻辑与 `ros_project` 的实车接口组合出的独立 ROS 2 Jazzy 工作空间。

- 原项目目录不在此工作空间中，迁移过程不覆盖原文件。
- `inspection_robot_bringup` 只启动实车雷达、YB IMU、RF2O、EKF、GPIO 电机、Nav2 和安全链路。
- `/cmd_vel_nav`、`/cmd_vel_auto`、`/cmd_vel_teleop` 经过 `velocity_safety_gate` 后才到 `/cmd_vel`。
- 仿真 Gazebo、ros_gz_bridge、SDF/world、`/sim/imu/data_raw` 和轮速里程计接口不属于本工程。

构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

完整的实车启动、建图、导航、GUI、安全停机和回滚流程见 [`start.md`](start.md)。

实车启动前请核对串口 by-path、IMU 安装方向、雷达安装外参、GPIO 权限和急停策略；没有实车联调前不要直接给电机上电。

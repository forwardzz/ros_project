# ros_project

基于 ROS 2 的履带小车项目，运行在树莓派 5 上，当前方案以 `RPLIDAR + RF2O + Nav2 + slam_toolbox` 为主，不再依赖 IMU 参与建图和导航定位。

## 项目概览

当前系统采用纯雷达定位路径：

- `sllidar_ros2` 发布 `/scan`
- `rf2o_laser_odometry` 基于激光数据生成 `/odom`
- `slam_toolbox` 负责在线建图
- `Nav2` 负责地图导航、路径规划和速度平滑
- `mapping_bringup/tracked_motor_driver` 订阅 `/cmd_vel` 驱动履带底盘

## 目录结构

```text
ros_ws/
├── map_name.pgm
├── map_name.yaml
└── src/
    ├── mapping_bringup/
    ├── sllidar_ros2/
    ├── rf2o_laser_odometry/
    ├── imu_ros2_device/
    └── YbImuLib/
```

说明：

- 实际运行主链路目前使用 `mapping_bringup`、`sllidar_ros2`、`rf2o_laser_odometry`
- `imu_ros2_device` 和 `YbImuLib` 仍保留在仓库中，但当前建图/导航流程默认不启用

## 主要功能

- 雷达扫描采集
- 基于 RF2O 的二维激光里程计
- 在线建图
- 基于已有地图的自主导航
- `cmd_vel` 底盘运动控制
- Nav2 局部/全局路径规划
- 速度平滑与基础避障

## 环境依赖

建议环境：

- Raspberry Pi 5
- Ubuntu + ROS 2
- `colcon`
- `slam_toolbox`
- `nav2` 相关包
- `robot_localization`（当前主流程不依赖，但可保留）
- `RPi.GPIO`

如果是从源码工作区编译，确保系统已安装本项目依赖的 ROS 2 包。

## 编译

进入工作区后编译：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

如果只想编译核心包，可以按需选择：

```bash
RPLIDAR ROS 2 驱动，负责发布 `/scan`。
colcon build --packages-select sllidar_ros2 rf2o_laser_odometry mapping_bringup
```

如果要编译图形控制界面，一并加入：

```bash
colcon build --packages-select sllidar_ros2 rf2o_laser_odometry mapping_bringup robot_control_ui
```

## 启动方式

### 1. 建图

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch mapping_bringup mapping.launch.py
```

默认会启动：

- 雷达驱动
- `base_link -> laser` 静态 TF
- `rf2o_laser_odometry`
- 电机驱动
- `slam_toolbox`

建图完成后可保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/map_name
```

### 2. 导航

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch mapping_bringup navigation.launch.py map:=/home/yy/ros2_ws/map_name.yaml
```

默认会启动：

- 雷达驱动
- `base_link -> laser` 静态 TF
- `rf2o_laser_odometry`
- 电机驱动
- `map_server`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `behavior_server`
- `velocity_smoother`

### 3. 图形控制界面

编译完成后可启动桌面 UI：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_control_ui ui.launch.py
```

可选参数：

```bash
ros2 launch robot_control_ui ui
source install/setup.bash
ros2 launch robot_control_ui ui.launch.py
```

可选参数：.launch.py \
  workspace_path:=/home/yy/ros2_ws \
  map_path:=/home/yy/ros2_ws/map_name.yaml
```

当前 UI 集成了这些功能：

- 启动建图
- 启动导航
- 保存地图
- 停止当前 launch 任务
- 通过 SSH 在树莓派上执行上述任务
- 手动前进、后退、左转、右转、停止
- 键盘控制 `W/A/S/D`、方向键、空格停止
- 内嵌地图显示 `/map`、机器人位置和轨迹
- 日志状态灯显示 `SSH / scan / odom / map`
- 显示 `/scan` 和 `/odom` 在线状态
- 显示当前机器人位置
- 显示运行日志

## 手动控制

### 1. 键盘控制小车

如果系统里已经安装 `teleop_twist_keyboard`，可以直接运行：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用按键：

- `i`：前进
- `,`：后退
- `j`：左转
- `l`：右转
- `k`：停止
- `u` `o` `m` `.`：斜向或带转向运动

如果没有安装，可以先安装：

```bash
sudo apt install ros-<your-distro>-teleop-twist-keyboard
```

例如 Humble：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

### 2. 直接发送速度指令

前进：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

后退：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

左转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.50}}"
```

右转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.50}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

说明：

- 手动控制时，确保 `tracked_motor_driver` 已经启动
- 手动调试优先直接发 `/cmd_vel`
- 如果 Nav2 正在运行，注意不要同时让导航和手动控制一起发速度

## 关键话题

- `/scan`：雷达扫描
- `/odom`：RF2O 输出里程计
- `/cmd_vel`：底盘控制输入
- `/cmd_vel_nav`：Nav2 控制器输出，经平滑后回到 `/cmd_vel`

## 关键包说明

### `mapping_bringup`

项目主入口，包含：

- 建图启动文件 `mapping.launch.py`
- 导航启动文件 `navigation.launch.py`
- Nav2 参数 `config/nav2_params.yaml`
- SLAM 参数 `config/slam.yaml`
- 底盘控制节点 `tracked_motor_driver.py`

### `sllidar_ros2`

RPLIDAR ROS 2 驱动，负责发布 `/scan`。

### `rf2o_laser_odometry`

基于激光扫描匹配的二维里程计，用于生成 `/odom`。

### `imu_ros2_device`

旧 IMU 驱动包，当前默认不参与主流程。

### `robot_control_ui`

基于 `tkinter + rclpy` 的桌面控制界面，用来统一操作建图、导航、地图保存和手动运动控制。

## 已知限制

- 当前底盘控制是开环 PWM，没有编码器闭环反馈
- 纯雷达定位对环境特征较敏感，空旷区域、强动态干扰场景下稳定性会下降
- `imu_ros2_device` 仍在仓库中，但没有从仓库物理清理
- `use_rviz` 参数当前未接入启动逻辑

## 建议排查项

运行前优先确认：

- 雷达设备是否存在：`/dev/rplidar`
- 雷达是否正常发布 `/scan`
- TF 是否连续：`base_link -> laser`
- RF2O 是否正常发布 `/odom`
- 底盘是否正确响应 `/cmd_vel`

## 后续可改进方向

- 加编码器，补齐闭环速度控制
- 清理仓库中未使用的 IMU 相关包和配置
- 补充 RViz 启动与调试文档
- 增加一键构建和一键启动脚本

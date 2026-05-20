# ros_project

基于 ROS 2 Jazzy 的履带小车项目，运行主体在树莓派 5，当前主链路为：

- `sllidar_ros2` 发布 `/scan`
- `rf2o_laser_odometry` 生成 `/odom`
- `slam_toolbox` 负责在线建图
- `Nav2 + AMCL + map_server` 负责地图导航
- `mapping_bringup/tracked_motor_driver` 订阅 `/cmd_vel` 驱动底盘
- `robot_control_ui` 在电脑上显示界面，并通过 SSH 控制树莓派启动/停止任务

当前默认不再使用 IMU 参与建图和导航。

## 目录结构

```text
ros_ws/
├── map_name.pgm
├── map_name.yaml
└── src/
    ├── mapping_bringup/
    ├── sllidar_ros2/
    ├── rf2o_laser_odometry/
    ├── robot_control_ui/
    ├── imu_ros2_device/
    └── YbImuLib/
```

说明：

- 主运行链路目前只依赖 `mapping_bringup`、`sllidar_ros2`、`rf2o_laser_odometry`、`robot_control_ui`
- `imu_ros2_device` 和 `YbImuLib` 仍保留在仓库中，但当前默认不启用

## 主要功能

- 雷达扫描采集
- 基于 RF2O 的二维激光里程计
- 在线建图
- 基于已有地图的自主导航
- 手动 `cmd_vel` 控制
- Nav2 全局/局部路径规划
- 速度平滑
- 电脑端图形控制界面

## 环境依赖

- Raspberry Pi 5
- Ubuntu + ROS 2 Jazzy
- `colcon`
- `slam_toolbox`
- Nav2 相关包
- `RPi.GPIO`

如果从源码工作区编译，确保系统已经安装这些 ROS 2 依赖。

## 编译

树莓派或本地电脑都可以按工作区编译：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

只编译核心包：

```bash
colcon build --packages-select sllidar_ros2 rf2o_laser_odometry mapping_bringup
```

带图形控制界面一起编译：

```bash
colcon build --packages-select sllidar_ros2 rf2o_laser_odometry mapping_bringup robot_control_ui
```

## 建图

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mapping_bringup mapping.launch.py
```

默认会启动：

- 雷达驱动
- `base_link -> laser` 静态 TF
- `rf2o_laser_odometry`
- 电机驱动
- `slam_toolbox`

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/map_name
```

## 导航

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
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

重要说明：

- 不要同时运行建图和导航
- 建图时是 `slam_toolbox` 在管理 `map`
- 导航时是 `map_server + amcl` 在管理 `map`
- 两套同时运行会导致 TF 冲突、初始位姿异常、坐标轴乱跳

## 本地电脑显示 UI

推荐模式：

- 树莓派运行雷达、里程计、建图/导航、底盘驱动
- 电脑运行 `robot_control_ui`
- 电脑通过局域网订阅树莓派 ROS 2 数据
- UI 按钮通过 SSH 到树莓派执行启动/停止命令

### 1. 树莓派环境

树莓派上：

```bash
source /opt/ros/jazzy/setup.bash
source /home/yy/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

### 2. 电脑环境

电脑上：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

两边要求：

- ROS 2 版本一致，当前使用 Jazzy
- `ROS_DOMAIN_ID` 一致
- `ROS_LOCALHOST_ONLY=0`
- 电脑和树莓派在同一局域网

### 3. 启动 UI

```bash
ros2 launch robot_control_ui ui.launch.py \
  remote_user:=yy \
  remote_host:=192.168.43.16 \
  workspace_path:=/home/yy/ros2_ws \
  map_path:=/home/yy/ros2_ws/map_name.yaml
```

UI 当前集成：

- 启动建图
- 启动导航
- 保存地图
- 停止当前机器人任务
- 通过 SSH 在树莓派执行上述命令
- 手动前进、后退、左转、右转、停止
- 键盘控制 `W/A/S/D`、方向键、空格停止
- 内嵌地图显示 `/map`、机器人位置和轨迹
- 状态灯显示 `SSH / scan / odom / map`
- 显示运行日志

## 手动控制

### 1. 键盘控制

如果系统里安装了 `teleop_twist_keyboard`：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用按键：

- `i`：前进
- `,`：后退
- `j`：左转
- `l`：右转
- `k`：停止

### 2. 直接发送速度

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

## 通信与显示细节

### 1. 为什么电脑上能看到所有节点

这是 ROS 2 网络发现的正常现象。只要电脑和树莓派在同一个 ROS 图里，电脑上的：

- `ros2 node list`
- `ros2 topic list`
- `rqt_graph`
- `rviz2`

都会看到整套网络节点，不代表这些节点都跑在电脑上。

### 2. `/map` Reset 后消失的原因

导航时 `/map` 由 `map_server` 发布，QoS 是：

- `Reliable`
- `Transient Local`

如果订阅端用错 QoS，界面重置或重新订阅后可能拿不到已发布地图。

当前 `robot_control_ui` 已修正：

- `/map` 使用 `Transient Local + Reliable`
- `/scan` 使用 `sensor_data` QoS

如果用 RViz，建议 `Map` display 也手动设置：

- Topic：`/map`
- Reliability：`Reliable`
- Durability：`Transient Local`

### 3. 激光朝向

`mapping.launch.py` 和 `navigation.launch.py` 现在都使用统一的 `lidar_yaw` 参数。

默认值当前为：

```text
3.1415926
```

如果实际雷达安装方向变化，建图和导航必须保持同一个 `lidar_yaw`，并重新建图。

### 4. 停止任务后图里还有残留

如果用过 `pkill -f ros2` 这类粗暴清理，DDS 发现图可能短时间残留。建议电脑和树莓派都执行：

```bash
ros2 daemon stop
ros2 daemon start
```

## 关键话题

- `/scan`：雷达扫描
- `/odom`：RF2O 输出里程计
- `/map`：静态地图 / 定位地图
- `/cmd_vel`：底盘控制输入
- `/cmd_vel_nav`：Nav2 控制器输出，经平滑后回到 `/cmd_vel`
- `/initialpose`：导航初始位姿

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

### `robot_control_ui`

基于 `tkinter + rclpy` 的桌面控制界面，用来统一操作建图、导航、地图保存和手动控制。

## 已知限制

- 当前底盘控制是开环 PWM，没有编码器闭环反馈
- 纯雷达定位对环境特征较敏感，空旷区域、强动态干扰场景下稳定性会下降
- `imu_ros2_device` 仍在仓库中，但没有物理清理
- `use_rviz` 参数当前未接入启动逻辑

## 建议排查项

运行前优先确认：

- 雷达设备存在：`/dev/rplidar`
- `/scan` 有发布者
- `/odom` 有发布者
- 导航时 `/map_server` 和 `/amcl` 处于 `active`
- 建图和导航不要同时运行

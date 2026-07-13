# inspection_sim_ws → inspection_migrated_ws 迁移分析与执行计划

更新时间：2026-07-13  
工作目录：`/home/zjy/Desktop/project_transform`

本文件记录本次迁移的实际结果和后续上车步骤。源目录 `inspection_sim_ws/` 与 `ros_project/` 保持只读；所有新文件均位于 `inspection_migrated_ws/`。目标不是把实车工程改造成 Gazebo 仿真，而是把仿真的任务和控制算法放到一个新的实车工作空间，并把真实传感器和 GPIO 底层作为适配边界。

## 0. 已确认的迁移边界

| 决策 | 采用方案 |
| --- | --- |
| 新工作空间 | `inspection_migrated_ws` |
| 控制逻辑 | 任务、区域扫描、Nav2 调用和自定义 DWA 以仿真版本为基线 |
| 电机边界 | 仿真侧速度/状态控制逻辑 + 实车侧 GPIO/PWM 电机适配器 |
| 里程计 | `/laser_odom` + `/imu/data_raw` → EKF → `/odom`；不假设实车有 `/wheel_odom` |
| 速度安全 | `velocity_safety_gate` 是唯一写入 `/cmd_vel` 的节点；安全状态缺失/超过 2 s、输入超时或故障时输出零，电机驱动只订阅最终输出 |
| 区域任务 | 仿真区域规划和“旋转—低速直行—旋转”底盘原语；保留激光、TF、超时和障碍安全检查 |
| GUI | PyQt5 仿真任务界面 + 实车 Tkinter UI 中的 SSH、气体、热成像和安全状态能力 |
| 自定义 DWA | 迁移并设为 `FollowPath` 默认控制器 |
| 服务接口 | `StartNavigation.srv` 增加 `waypoint_pause_sec`，客户端和服务端同时重编译 |
| 实车几何初值 | `wheel_radius=0.025 m`、`track_width=0.155 m`、约 `0.27 × 0.22 m` footprint；必须现场标定 |

## 一、两个原项目的技术栈和目录结构对比

### 1.1 构建和 ROS 版本

两个源项目均为 ROS 2 Jazzy，使用 `colcon`，不是 ROS 1 `catkin`。两者都混合使用 `ament_cmake` 和 `ament_python`：

| 项目 | 工作空间根 | 包数量/主要包 | 构建方式 | 运行时钟 |
| --- | --- | --- | --- | --- |
| `inspection_sim_ws` | `inspection_sim_ws/` | 7 个：`inspection_sim_bringup`、`inspection_sim_dwa_controller`、`inspection_sim_gui`、`inspection_sim_mission`、`rf2o_laser_odometry`、`robot_mission_utils`、`robot_monitor_interfaces` | `colcon build --symlink-install`；bringup/DWA/RF2O 为 CMake，其余多为 Python | Gazebo `/clock`，参数中大量 `use_sim_time: true` |
| `ros_project` | `ros_project/ros_ws/` | 7 个 ROS 包；`mapping_bringup` 内还包含电机、任务、安全、气体、热成像和路径记录节点；另有 `sllidar_ros2`、`imu_ros2_device`、`YbImuLib` | `colcon build`；`mapping_bringup` 为 Python，雷达/RF2O 为 CMake | 系统时间；目标板为 Raspberry Pi，DDS 通过网络与上位机连接 |
| 新工程 | `inspection_migrated_ws/` | 12 个可构建条目，包含实车驱动、仿真任务/DWA、GUI、安全和接口 | `colcon build --symlink-install` | 所有 bringup 参数为 `use_sim_time: false` |

`YbImuLib` 没有 ROS `package.xml`，但其 `setup.py` 会被 colcon 作为 Python 依赖构建；在目标板上仍需确认 Python 安装路径和串口权限。

### 1.2 目录、依赖和配置资产

仿真工程的 `inspection_sim_bringup` 同时包含 launch、Nav2/SLAM/EKF 配置、RViz、behavior tree、URDF/Xacro、Gazebo SDF model/world；实车工程的 `mapping_bringup` 没有 URDF/Xacro 或 Gazebo world，依靠 launch 中的 `base_link→laser`、`base_link→imu_link` 静态 TF。

主要依赖如下：

- C++：`rclcpp`、`nav2_core`、`nav2_costmap_2d`、`pluginlib`、`tf2`、Eigen；对应自定义 DWA、RF2O 和 SLLIDAR SDK。
- ROS Python：`rclpy`、`nav2_msgs`、`nav_msgs`、`sensor_msgs`、`geometry_msgs`、`tf2_ros`、`visualization_msgs`、`robot_localization`、`slam_toolbox`。
- Python 第三方：`numpy`、`PyYAML`、`psutil`、PyQt5；气体节点使用 `pyserial`，热成像节点按需使用 `board/busio/adafruit_mlx90640`。
- 实车专用：`RPi.GPIO`（可选导入，实际接电机时必须可用）、YB IMU 串口库、SLLIDAR by-path 串口和 udev 规则。

### 1.3 节点和功能结构

仿真侧的核心启动链为 Gazebo/ros-gz bridge → `/scan`、`/sim/imu/data_raw`、`/wheel_odom`、`/cmd_vel`；RF2O 和 EKF 后进入 SLAM/Nav2。实车侧的核心链为 SLLIDAR → `/scan`、YB IMU → `/imu/data_raw`、RF2O → `/laser_odom`、EKF → `/odom`、GPIO 电机订阅 `/cmd_vel`。

新工程将任务和控制层从仿真侧重命名为：

- `inspection_robot_mission`：仿真任务管理器、区域生成/执行、任务状态和实车路径记录。
- `inspection_robot_dwa_controller`：仿真自定义 DWA 插件，作为 Nav2 `FollowPath` 默认控制器。
- `inspection_robot_gui`：PyQt5 实车控制台，通过 SSH 启停目标板 launch，同时订阅真实状态。
- `inspection_robot_hardware`：GPIO 电机、气体串口、MLX90640 节点。
- `inspection_robot_safety`：电源/导航安全监视器和最终速度仲裁门。
- `inspection_robot_bringup`：只含实车 mapping/navigation launch、参数、RViz、BT 和运行时地图目录。

## 二、三层架构划分

### 2.1 上层巡检业务逻辑

`inspection_robot_mission/mission_manager.py` 和 `mission_regions.py` 来自仿真工程，负责：

- RViz 点位接收、顺序任务、`NavigateToPose` 调用、每点停留时间；
- 区域角点、扫描线和跨区域路线预览；
- 旋转—直行—旋转底盘原语，TF 不可用、雷达超时、通道障碍时跳过区域；
- 异常/中止、区域状态、Marker/Path 可视化和 `/mission_status`；
- 新增 `/mission_status_typed`，供安全监视器和 GUI 使用。

该层不直接访问 GPIO、串口或 Gazebo；它只向 `/cmd_vel_nav` 输出自主控制输入。

### 2.2 通用机器人功能

包括 `robot_mission_utils`（A*、双向 A*、Lazy Theta*、TSP、路径平滑、栅格膨胀/碰撞检查）、RF2O 激光里程计、自定义接口、Nav2/SLAM 参数、无 spin 行为树、实际路径记录和 GUI 的地图/任务工具。这些代码不应依赖仿真时间或 Gazebo 消息。

### 2.3 平台相关接口

包括 `sllidar_ros2`/SDK/udev、`imu_ros2_device` + `YbImuLib`、GPIO/PWM 电机、气体传感器 Modbus 串口、MLX90640 I2C、实车安全电源监测、静态传感器 TF、SSH 远程 launch。该层拥有硬件权限和急停责任，不能被仿真控制器替换。

## 三、功能对比表

| 功能 | `inspection_sim_ws` 中的位置 | `ros_project` 中的位置 | 迁移方式 | 风险 |
| --- | --- | --- | --- | --- |
| ROS/colcon 基础 | 各包 `package.xml`、CMake/setup | `ros_project/ros_ws` | 新工程统一为 Jazzy/colcon | 环境 overlay 顺序错误 |
| Gazebo 机器人模型 | `inspection_sim_bringup/models/inspection_tracked_robot/model.sdf`、`urdf/*.xacro` | 无实车等价物 | 不迁移；新工程不生成 RSP/Gazebo | 误启动仿真接口、TF 重复 |
| 仿真 launch/bridge | `inspection_sim_bringup/launch/sim.launch.py`、`teleop.launch.py` | 无 | 不迁移 | `/clock`、`/sim/imu`、`/wheel_odom` 语义只在仿真成立 |
| 激光雷达 | Gazebo GPU lidar，`/scan` | `sllidar_ros2`，by-path 串口 | 保留实车驱动；统一 `/scan` 和可配置 `laser` frame | 扫描频率、量程、反向安装、QoS |
| IMU | Gazebo `/sim/imu/data_raw` + `sim_imu_adapter.py` | `imu_ros2_device`/`YbImuLib` `/imu/data_raw` | 删除仿真 adapter，直接使用 YB 节点 | 轴向、ENU/NED、单位、串口权限 |
| 激光里程计 | `rf2o_laser_odometry` | 同名且源码一致 | 直接复用；新 launch 输出 `/laser_odom` | 真实地面反光、遮挡和低纹理 |
| 轮速里程计 | SDF diff-drive `/wheel_odom` | GPIO 驱动没有编码器反馈 | 不迁移；EKF 不订阅该 topic | 若误加入会产生虚假约束/TF |
| EKF | 仿真 `laser_odom + wheel_odom + imu`，50 Hz | 实车原有配置；当前采用 `laser_odom + imu`，30 Hz | 新 `config/ekf.yaml` 明确去掉 `/wheel_odom` | IMU yaw rate 偏置和时间戳 |
| SLAM | 仿真 bringup + `slam_toolbox` | 实车 `mapping_bringup` | 新 mapping launch 保留 SLAM，`use_sim_time=false` | 不得和 map_server/AMCL 同时运行 |
| 定位/导航 | 仿真 Nav2、DWB/RotationShim | 实车 Nav2 原配置 | 新 navigation launch 使用仿真自定义 DWA + 实车几何参数 | DWA 参数、footprint、控制频率 |
| 自定义 DWA | `inspection_sim_dwa_controller` | 原实车无该插件 | 重命名为 `inspection_robot_dwa_controller`，设为默认 FollowPath | pluginlib、Nav2 ABI、现场调参 |
| 任务点巡检 | `inspection_sim_mission/mission_manager.py` | 原 `mapping_bringup/mission_manager.py` 为较小 RViz/ThroughPoses 逻辑 | 以仿真任务管理器为准，保留 real-only 传感器语义 | 任务接口变化、导航 action 状态 |
| 区域巡检 | 仿真 mission/regions | 原实车没有同等区域执行 | 直接迁移业务逻辑；速度走安全门 | 实车狭窄空间、障碍误判 |
| 自定义消息/服务 | `robot_monitor_interfaces` | 同名包已有 Gas/Safety/mission 接口 | 以实车接口为基线，新增 typed `MissionStatus`，扩展 `StartNavigation` | 所有客户端必须重编译 |
| 气体传感器 | 无真实硬件；仅业务数据结构 | `mapping_bringup/gas_sensor_node.py` | 放入 `inspection_robot_hardware`，原串口帧解析保留 | 串口 fallback 不能误用雷达端口 |
| 热成像 | 无 Gazebo 等价物 | `mapping_bringup/thermal_camera_node.py` | 放入 `inspection_robot_hardware`，无设备时只报错不造 mock 数据 | I2C 库、帧维度和温度单位 |
| 安全监视 | 仿真无硬件电源安全；目标有 `safety_monitor.py` | `mapping_bringup/safety_monitor.py` 会直接发布 `/cmd_vel` | 新安全监视器只发布 `/safety_stop`/状态，速度由 gate 统一输出 | 安全状态丢失必须 fail-safe |
| 手动 GUI | 仿真 PyQt5，直接发布 `/cmd_vel` | 实车 Tkinter，含 SSH/气体/热成像/安全 | 新 PyQt5 GUI：手动 `/cmd_vel_teleop`，加入实车状态和 SSH | DDS 网络、远程路径、误操作 |
| 实际路径 | 仿真无或不依赖硬件 | `actual_path_recorder.py` | 放入新 mission 包并在 navigation 启动 | AMCL/action 状态 QoS |
| udev/设备权限 | 不需要 | SLLIDAR rules、串口权限、GPIO 用户组 | 保留实车规则到新工程，部署时单独安装 | 规则未安装导致节点假死 |

## 四、接口映射表

| 仿真接口 | 实车接口 | 类型 | 是否兼容 | 需要的适配 |
| --- | --- | --- | --- | --- |
| `/scan`（Gazebo GPU lidar，frame `laser`） | `/scan`（SLLIDAR，frame 默认 `laser`） | `sensor_msgs/msg/LaserScan` | 消息兼容，语义需核对 | launch 传入 by-path 串口、frame、yaw、QoS；现场确认量程/频率 |
| `/sim/imu/data_raw` | `/imu/data_raw` | `sensor_msgs/msg/Imu` | topic 不兼容、消息兼容 | 移除 `sim_imu_adapter`；使用 YB 驱动原始单位和 `imu_link` 静态 TF |
| `/wheel_odom` | 无可靠实车等价物 | `nav_msgs/msg/Odometry` | 不兼容 | 新 EKF 不订阅；GUI 将其标记为“实车未提供” |
| `/laser_odom` | `/laser_odom` | `nav_msgs/msg/Odometry` | 兼容 | RF2O 统一输入 `/scan`，关闭其 TF 发布，由 EKF 负责 `odom→base_link` |
| `/odom`（仿真 EKF） | `/odom`（实车 EKF） | `nav_msgs/msg/Odometry` | 名称兼容、来源不同 | 仅将 EKF 输出作为导航/任务位姿，不把它当编码器里程计 |
| `odom→base_link`（Gazebo diff-drive/EKF） | `odom→base_link`（EKF） | TF | 不能双发布 | 关闭 RF2O TF，禁用 Gazebo diff-drive；只让 EKF 发布 |
| `base_link→laser`（SDF/URDF） | `base_link→laser`（static_transform_publisher） | TF | 名称兼容、外参不自动兼容 | 默认 z=0.15、yaw=π；必须现场测量并只保留一个发布者 |
| `base_link→imu_link`（SDF/URDF） | `base_link→imu_link`（static_transform_publisher） | TF | 名称兼容、外参不自动兼容 | 默认 z=0.05；核对 YB 安装方向，禁止同时运行 RSP/静态重复 TF |
| `/cmd_vel`（Gazebo bridge） | `/cmd_vel`（GPIO driver） | `geometry_msgs/msg/Twist` | 消息兼容，安全语义不同 | gate 唯一发布；输入分为 `/cmd_vel_nav`、`/cmd_vel_auto`、`/cmd_vel_teleop`，并做超时/限幅 |
| Nav2 `/cmd_vel` | `/cmd_vel_nav` | `Twist` | 通过 remap 兼容 | controller_server 输出 remap 到 `/cmd_vel_nav` |
| velocity smoother `/cmd_vel_smoothed` | `/cmd_vel_auto` | `Twist` | 通过 remap 兼容 | smoother 不再直连电机；gate 统一仲裁 |
| GUI `/cmd_vel` | GUI `/cmd_vel_teleop` | `Twist` | 通过配置适配 | 手动控制不绕过 gate；停机时发送 teleop 零速度并由 watchdog 兜底 |
| `/thermal_frame` | `/thermal_frame` | `std_msgs/msg/Float32MultiArray` | 兼容 | 真实 MLX90640 发布 32×24；GUI 只显示真实帧，不注入 mock |
| `/gas_data` | `/gas_data` | `robot_monitor_interfaces/msg/GasData` | 兼容 | 保留 Modbus 13 字节解析和稳定串口路径 |
| `/robot_safety_status` | `/robot_safety_status` | `RobotSafetyStatus` | 兼容 | safety monitor 保留电源/导航检查，gate 订阅状态；另发 `/safety_stop` |
| `/mission_status` | `/mission_status` | `std_msgs/msg/String` | 向后兼容 | 保留旧文本；新增 `/mission_status_typed` 用于机器判断 |
| 无 | `/mission_status_typed` | `MissionStatus` | 新接口 | 字段包括 state/mode/message/active/safety_warning/current/total |
| `StartNavigation.waypoints` | 同名字段 | `robot_monitor_interfaces/srv/StartNavigation` | 部分兼容 | 新增 `float64 waypoint_pause_sec`；GUI 请求和 mission manager 同步更新 |
| `/navigate_to_pose` | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 兼容 | 真实 `/odom`、`/scan`、AMCL/TF 必须先稳定 |
| `/navigate_through_poses` | `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 兼容 | mission/safety/actual recorder 同时监听 action status |
| `/map`（slam_toolbox） | `/map`（slam_toolbox 或 map_server） | `nav_msgs/msg/OccupancyGrid` | 兼容但所有权互斥 | mapping 只运行 SLAM；navigation 只运行 map_server+AMCL |
| Gazebo `/clock` | Linux system clock | `rosgraph_msgs/msg/Clock` | 不兼容 | 新参数全为 `use_sim_time=false`，禁止仿真 bridge |

## 五、文件迁移表

以下是已在新工作空间执行的文件级动作；原路径没有覆盖。

| 源文件/目录 | 目标位置 | 操作方式 | 修改内容 | 依赖 |
| --- | --- | --- | --- | --- |
| `inspection_sim_ws/src/inspection_sim_mission/` | `inspection_migrated_ws/src/inspection_robot_mission/` | 复制并重命名包 | 改包名；删除 `sim_imu_adapter.py`；加入 typed status、实车路径记录 | rclpy、Nav2、TF、接口包 |
| `inspection_sim_ws/src/inspection_sim_dwa_controller/` | `inspection_migrated_ws/src/inspection_robot_dwa_controller/` | 复制并重命名 | CMake、plugin XML、include guard、plugin class 名称路径一致化 | Nav2 C++、pluginlib |
| `inspection_sim_ws/src/inspection_sim_gui/` | `inspection_migrated_ws/src/inspection_robot_gui/` | 复制并重命名 | PyQt GUI；去除 scene/world 仿真 UI；手动速度改发 `/cmd_vel_teleop` | PyQt5、rclpy、接口包 |
| `inspection_sim_ws/src/robot_mission_utils/` | 新同名包 | 复制仿真通用算法 | 保留仿真侧 `world_to_grid` 修正；不覆盖实车已有安全校验逻辑 | numpy、nav_msgs |
| `inspection_sim_ws/src/rf2o_laser_odometry/` | 新同名包 | 直接复制 | 源码与实车版本一致；统一输出 `/laser_odom` | Eigen、sensor_msgs、tf2 |
| `inspection_sim_ws/src/robot_monitor_interfaces/` + 实车同名包 | 新同名包 | 以实车接口为基线合并 | 保留 Gas/Safety/InspectionPoint/现有服务；新增 `MissionStatus.msg`，扩展 `StartNavigation.srv` | rosidl、std_msgs、geometry_msgs |
| `ros_project/ros_ws/src/sllidar_ros2/` | 新同名包 | 直接复制 | 保留 SDK、launch、udev rules | 串口权限、SLLIDAR SDK |
| `ros_project/ros_ws/src/imu_ros2_device/` + `YbImuLib/` | 新同名包/库 | 直接复制 | 保留 YB 串口驱动；launch 使用 `/dev/ttyAMA0` 默认值 | pyserial/设备权限/YbImuLib |
| `ros_project/ros_ws/src/mapping_bringup/tracked_motor_driver.py` | `inspection_robot_hardware/tracked_motor_driver.py` | 复制到硬件适配包 | 保留 GPIO BCM pin、PWM、加速度限制、0.5 s watchdog；输入固定最终 `/cmd_vel` | RPi.GPIO、geometry_msgs |
| `ros_project/ros_ws/src/mapping_bringup/gas_sensor_node.py` | `inspection_robot_hardware/gas_sensor_node.py` | 复制 | 保留 Modbus 请求、13 字节解析、by-path fallback；不使用雷达端口 fallback | pyserial、GasData |
| `ros_project/ros_ws/src/mapping_bringup/thermal_camera_node.py` | `inspection_robot_hardware/thermal_camera_node.py` | 复制 | 保留 MLX90640 实帧发布；缺少库时不造 mock 帧 | I2C/Adafruit 可选库 |
| `ros_project/ros_ws/src/mapping_bringup/safety_monitor.py` | `inspection_robot_safety/safety_monitor.py` | 复制后接口适配 | 删除直接 `/cmd_vel` publisher；发布 `/safety_stop`，同时监听两种 Nav2 action 和 typed mission status | RobotSafetyStatus、Trigger |
| 新增 | `inspection_robot_safety/velocity_safety_gate.py` | 新建 | fault/stop/stale input/NaN 时输出零；自主优先、手动 fallback；限幅到 0.18 m/s、0.55 rad/s | Twist、Bool、RobotSafetyStatus |
| `ros_project/.../actual_path_recorder.py` | `inspection_robot_mission/actual_path_recorder.py` | 复制 | 保留 `/actual_path` 和 `/mission_actual_path`；在 navigation launch 启动 | AMCL、action_msgs、nav_msgs |
| `inspection_sim_ws/src/inspection_sim_bringup/config/*.yaml` | `inspection_robot_bringup/config/` | 选择性复制/改参数 | 只保留 EKF/Nav2/SLAM/BT/RViz；`use_sim_time=false`、真实 footprint/速度、DWA plugin | Nav2、robot_localization、slam_toolbox |
| `inspection_sim_ws/src/inspection_sim_bringup/launch/*.launch.py` | `inspection_robot_bringup/launch/` | 重写实车 launch | mapping/navigation 直接启动真实雷达/IMU/RF2O/EKF/GPIO/safety；不 include sim.launch | launch_ros、实车包 |
| `inspection_sim_ws` 的 SDF/URDF/world/ros-gz bridge | 无 | 明确不复制 | 不在新工程安装 | Gazebo 专用，禁止上车 |

## 六、A–E 分类结果

### A. 可以直接复用的通用代码

- `robot_mission_utils` 中的栅格、A*、Lazy Theta*、TSP、路径平滑和碰撞工具。
- RF2O 激光里程计 C++ 源码（两个项目逐文件一致）。
- 仿真 DWA 的轨迹采样、代价评估、局部/全局路径发布和 pluginlib 实现；仅包名和参数需适配。
- `mission_regions.py`、`ros_utils.py`、QoS 辅助、Marker/Path 生成和任务状态机的硬件无关部分。
- `InspectionPoint`、`GasData`、`RobotSafetyStatus` 等消息，以及现有 Localize/Trigger 类服务。
- 仿真 PyQt GUI 的地图视图、任务/区域按钮、初始位姿和导航 action 客户端。
- 实车实际路径记录器（它只依赖 AMCL 和 action status，不依赖 GPIO）。

### B. 修改接口后可以复用的代码

- `mission_manager.py`：删除仿真 IMU 适配；速度改发 `/cmd_vel_nav`；状态增加 typed message；区域原语保持，但必须经过安全 gate。
- `StartNavigation.srv`：增加 `waypoint_pause_sec` 并同步 GUI/服务端。
- Nav2 参数：自定义 DWA plugin、footprint、速度/加速度、真实 `/scan` 与 `/odom`，关闭仿真时间。
- EKF：从仿真三源改为实车 RF2O+YB IMU；只让 EKF 发布 `odom→base_link`。
- GUI RosAdapter/LaunchManager：`/cmd_vel_teleop`、远程 SSH、实车状态订阅、远程 workspace/setup 路径。
- safety monitor：保留实车电源/导航检测，输出 stop 信号，不再与 gate/电机争用 `/cmd_vel`。
- mapping/navigation launch：保留仿真 Nav2/SLAM 结构，替换 Gazebo 节点为实车传感器/驱动。

### C. 仅适用于仿真的代码

- `inspection_sim_bringup/launch/sim.launch.py`、`teleop.launch.py` 及 Gazebo 启动逻辑。
- `models/inspection_tracked_robot/model.sdf`、`worlds/*.sdf`、`urdf/*.xacro`、Gazebo plugin 和 `ros_gz_bridge` 参数桥。
- `/clock`、`/sim/imu/data_raw`、Gazebo diff-drive 产生的 `/wheel_odom` 和 `wheel_odom_tf`。
- `inspection_sim_mission/sim_imu_adapter.py`。
- 仿真传感器噪声、mock/bridge 数据和任何以 `use_sim_time=true` 为前提的 launch/config。
- 仿真场景 catalog/world/spawn 参数。新 GUI 已删除 scene/world 控件，LaunchManager 的仿真入口只保留拒绝提示，不会启动 Gazebo。

### D. `ros_project` 已存在、不应该重复迁移的代码

- GPIO/PWM 电机驱动的真实 pin、方向、dead-zone、加速度斜坡和 watchdog。
- SLLIDAR C++ SDK、驱动 launch、稳定 by-path 串口和 udev rules。
- YB IMU 驱动、`YbImuLib`、`/dev/ttyAMA0` 默认接口。
- 气体 Modbus 解析、热成像 MLX90640 采集、设备 fallback 和权限说明。
- 实车安全监视器的 undervoltage/throttled 电源检测、复位服务和任务 abort 逻辑（仅调整速度接口）。
- 实车 `actual_path_recorder`、实车 no-spin BT XML、现有 RViz 配置和已有 `rf2o_laser_odometry`。
- 实车中已经工作的串口/CAN/GPIO/SDK/设备权限和安全启动内容；新工程只是独立副本/适配边界，不覆盖原工程。

### E. 与实机底层驱动存在冲突的代码

- Gazebo diff-drive plugin 与真实 GPIO 电机：二者都可能把 `/cmd_vel` 解释为轮速，绝不能同时启动。
- 仿真 `/wheel_odom` 与实车无编码器事实：不能将仿真 odom 当作真实反馈写入 EKF。
- 仿真 `/sim/imu/data_raw` adapter 与 YB IMU 原始数据：会重复发布/改变单位，已移除 adapter。
- 仿真模型静态 TF/robot_state_publisher 与实车 launch 静态 TF：会产生重复 `base_link→laser/imu_link`。
- 原实车 safety monitor 直接发布 `/cmd_vel` 与新 gate、电机：已改成 stop/status 结构，确保唯一最终发布者。
- 原实车 Tkinter GUI 直接发布 `/cmd_vel` 与 gate：新 GUI 改为 `/cmd_vel_teleop`，旧 GUI 不得与新工程同时控制。

## 七、建议的硬件抽象层和适配层

当前新工程已经实现最小安全边界，建议后续继续按下列边界演进：

1. **速度安全门（已实现）**：接收 `/cmd_vel_auto`、`/cmd_vel_teleop`，检查 safety level、`/safety_stop`、输入新鲜度、NaN/Inf 和物理速度上限，唯一发布 `/cmd_vel`。
2. **底盘适配器（已实现）**：`tracked_motor_driver` 只负责 Twist→差速轮 RPM→GPIO PWM，保留真实 pin、breakout PWM、加速度和 watchdog；不得在任务层直接写 GPIO。
3. **传感器适配器（已实现）**：SLLIDAR/YB IMU/气体/热成像各自保持 ROS 标准消息；任务层通过 topic 和 typed 状态消费，不知道串口协议。
4. **参数化外参层（建议确认）**：将雷达/IMU xyz/rpy、footprint、轮半径、履带宽度、速度/加速度集中到实车 YAML，并在上车前用测量值覆盖默认值。
5. **健康状态层（建议新增）**：为 scan/IMU/gas/thermal/电机 watchdog 发布统一诊断状态；gate 对关键传感器超时可配置为停止或仅告警。
6. **急停层（必须由硬件确认）**：ROS stop 只能作为软件保护；物理急停应切断电机使能或驱动板电源，并与 GPIO 的默认安全电平核对。

## 八、分阶段实施、独立编译和回滚

每阶段只改新工作空间，可通过不选择对应 launch 或恢复新工程中的单个包目录回滚；原工程不参与回滚。

| 阶段 | 内容 | 独立验证 | 回滚边界 |
| --- | --- | --- | --- |
| 0 基线 | 记录两个原仓库状态、包/接口/TF/topic 清单；建立新 workspace 和 `.gitignore` | `git -C inspection_sim_ws status`、`git -C ros_project status` 均为空 | 删除整个新 workspace，不影响原目录 |
| 1 接口/通用库 | 构建 `robot_monitor_interfaces`、`robot_mission_utils`、RF2O；确认 MissionStatus/StartNavigation | `colcon build --packages-select robot_monitor_interfaces robot_mission_utils rf2o_laser_odometry`；`ros2 interface show` | 仅移除新接口/工具包 |
| 2 DWA/Nav2 配置 | 构建 DWA；加载真实 footprint、速度、EKF/SLAM/Nav2 参数 | `colcon build --packages-select inspection_robot_dwa_controller inspection_robot_bringup`；launch `--show-args` | navigation launch 改回无 DWA 的验证配置 |
| 3 任务层 | 启动 mission manager/actual recorder（不接电机），验证服务、Marker、typed status、区域文件 | `colcon build --packages-select inspection_robot_mission`；mock/只读 topic 服务测试 | 不启动 mission node，保留 Nav2 基础链路 |
| 4 硬件/安全 | 单独启动雷达、IMU、GPIO 驱动和 safety/gate；先不使能电机，检查 watchdog 和 stop 初值 | `colcon build --packages-select inspection_robot_hardware inspection_robot_safety`；现场 `ros2 topic echo` | 只运行传感器，不运行 motor/gate；物理急停保持断开 |
| 5 mapping | 真机低速、无导航任务运行 SLAM；确认 `/scan`、`/laser_odom`、`/odom`、TF 无重复 | `ros2 launch inspection_robot_bringup mapping.launch.py`；检查 `/map` 和 TF | 停止 mapping，恢复原 `mapping_bringup` launch |
| 6 navigation | 已保存地图运行 AMCL/Nav2/DWA；先手动短距离，再单点，再多点 | `ros2 launch ... navigation.launch.py map:=... use_rviz:=false`；检查 lifecycle 和 `/cmd_vel` 唯一 publisher | 停止新 navigation，原实车导航仍可独立使用 |
| 7 区域/GUI | PyQt 远程启动、任务点、区域任务、气体/热成像状态和安全故障演练 | GUI 服务调用、障碍/TF/串口超时演练 | 停止 GUI/mission，保留基础导航和物理急停 |
| 8 部署 | 将新 workspace 安装到目标板确认路径，安装 udev/用户组，固定 ROS_DOMAIN_ID/DDS | 目标板本地 build、冷启动、断电恢复测试 | 保留原 `/home/yy/ros2_ws`，切换启动脚本即可回退 |

## 九、主要风险和控制措施

1. **ROS 版本差异**：当前两个工程均为 Jazzy；仍需在目标板实际 `source /opt/ros/jazzy/setup.bash` 后重新构建，不能只依赖本机 install。
2. **消息类型不一致**：`StartNavigation.srv` 增加字段后，旧 UI/旧 mission manager 不能混用；必须同一 overlay 重编译并用 `ros2 interface show` 核对。
3. **topic/namespace 冲突**：所有 launch 使用绝对 topic。mapping 与 navigation 不得同时运行；旧 Tkinter GUI、旧 safety monitor 和新 gate 不得同时发布控制命令。
4. **TF 冲突**：RF2O 关闭 TF；EKF 唯一发布 `odom→base_link`；静态 launch 唯一发布 `base_link→laser/imu_link`；不要额外启动仿真 RSP 或旧静态发布者。
5. **控制频率和单位**：仿真轮半径 0.035 m/轮距 0.23 m，实车驱动 0.025 m/0.155 m；线速度 m/s、角速度 rad/s、IMU gyro rad/s 必须在现场用实测运动验证。gate 上限为 0.18/0.55，电机内部仍有独立 clamp。
6. **仿真时间与系统时间**：新工程所有 EKF/Nav2/SLAM 参数设为 `false`；不得在目标板残留 `/clock` bridge 或 `use_sim_time=true` overlay。
7. **传感器噪声和频率**：Gazebo 数据理想且规则，真实 SLLIDAR 会有反光/遮挡，YB IMU 有偏置；EKF 初期只融合 IMU yaw rate，后续再根据标定决定是否加入其他轴。
8. **底盘速度接口**：仿真 bridge 可直接接受 Twist，实车 PWM 存在死区和响应延迟；gate、velocity smoother、motor driver 三处限幅/加速度参数必须一致或明确分层，不能绕过 gate。
9. **实机安全**：gate 初始 fault/stop 为真，未收到安全状态时输出零；safety monitor 故障会请求 `/abort_mission`。但软件 stop 不能替代物理急停，首次上电必须轮子离地、限流、低 PWM、有人值守。
10. **串口和设备权限**：SLLIDAR、气体传感器、YB IMU 的 by-path 设备必须逐一确认；气体 fallback 不得包含雷达设备；udev 规则和用户组要在目标板安装。
11. **网络/DDS/SSH**：GUI 默认 `yy@192.168.43.21`，新远程 workspace 默认 `/home/yy/inspection_migrated_ws`；ROS_DOMAIN_ID、`ROS_LOCALHOST_ONLY`、CycloneDDS URI、目标板实际安装路径必须一致。
12. **地图和 AMCL 初始位姿**：navigation 的 `map:=` 不能为空；新工程默认地图仅是路径占位，必须由实车建图保存并在 RViz/GUI 设置真实初始位姿。
13. **依赖可选性**：RPi.GPIO、MLX90640、pyserial、PyQt5 和 YbImuLib 在目标板/上位机的安装方式不同；节点应在缺少可选硬件时安全报错，但不能将缺失数据伪装为有效传感器。

## 十、已执行验证

- `git -C inspection_sim_ws status --short`：无输出。
- `git -C ros_project status --short`：无输出。
- 对新工程所有 Python、YAML、`package.xml` 做 AST/YAML/XML 静态检查：通过。
- 新工程 `colcon build --symlink-install`：12 个条目全部完成；SLLIDAR SDK 只有既有的 zero-size array/unused parameter 编译警告，无失败。
- `ros2 launch inspection_robot_bringup mapping.launch.py --show-args`：通过。
- `ros2 launch inspection_robot_bringup navigation.launch.py --show-args`：通过。
- `ros2 launch inspection_robot_gui gui.launch.py --show-args`：通过。
- `ros2 interface show` 已确认 `MissionStatus.msg` 和扩展后的 `StartNavigation.srv`。
- 尚未接入真实 Raspberry Pi、GPIO、电机、电源和传感器，因此没有宣称完成现场运行验证。

## 十一、需要用户确认的关键决策

1. 目标板最终部署路径是否为 `/home/yy/inspection_migrated_ws`（当前 GUI 默认），还是继续使用 `/home/yy/ros2_ws`？
2. 雷达真实安装外参是否确认为 `laser`、z=0.15 m、yaw=π；IMU 是否确认为 `imu_link`、z=0.05 m、零 yaw？
3. 实车 footprint、轮半径 0.025 m、履带宽 0.155 m、最大速度 0.18/0.55 是否需要根据实测修改？
4. YB IMU 是否允许在 EKF 中只融合 yaw rate；是否需要先增加轴向/符号校准节点？
5. `/cmd_vel_teleop` 与自主 `/cmd_vel_auto` 的优先级是否保持“自主新鲜时优先、否则手动”，还是改为人工始终优先？
6. 物理急停信号是否已有 GPIO/CAN 输入；若有，应接入 `velocity_safety_gate` 的 `/safety_stop` 或独立硬件使能，而不是只依赖 ROS topic。
7. 气体传感器、热成像是否在 mapping/navigation 默认启动，还是只由 GUI 的“启动传感器”按需启动？当前方案为按需启动。
8. 是否允许在目标板安装 PyQt5 并运行 GUI，还是 GUI 固定运行在上位机、目标板只运行 ROS 节点？当前方案支持上位机 PyQt5 + SSH，默认不在目标板强制启动 GUI。
9. 现场验收时是否继续使用自定义 DWA 为默认控制器，还是先保留一个 Nav2 原生控制器作为紧急回退配置？当前默认已设为自定义 DWA。
10. 是否需要把 `actual_path`、热成像温升和气体阈值进一步纳入巡检任务完成/失败判定？当前只采集、显示和保留安全状态，不改变任务成功条件。

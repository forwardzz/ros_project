# ros_project

ROS 2 Jazzy 履带机器人项目。源码工作区在 `ros_ws/src`，树莓派运行工作区默认是 `/home/yy/ros2_ws`。

当前默认运行链路是雷达优先：RPLIDAR 发布 `/scan`，RF2O 生成 `/odom`，建图使用 `slam_toolbox`，导航使用 `map_server + AMCL + Nav2`。IMU 包仍保留，但默认不参与建图或导航融合。

## 项目结构

```text
.
├── AGENTS.md
├── README.md
├── START.md
└── ros_ws/
    └── src/
        ├── mapping_bringup/
        │   ├── config/                 # Nav2、SLAM、行为树参数
        │   ├── launch/                 # mapping/navigation/sensor launch
        │   └── mapping_bringup/        # 机器人运行节点
        ├── robot_control_ui/           # 电脑端 Qt 控制界面
        ├── robot_monitor_interfaces/   # 自定义 msg/srv
        ├── robot_mission_utils/        # 任务点、路径、碰撞检查工具
        ├── sllidar_ros2/               # RPLIDAR 驱动和 RViz 配置
        ├── rf2o_laser_odometry/        # 激光里程计
        ├── imu_ros2_device/            # 保留的 IMU 驱动
        └── YbImuLib/                   # 保留的 IMU 依赖库
```

不要提交 `ros_ws/build/`、`ros_ws/install/`、`ros_ws/log/`。

## 关键包

- `mapping_bringup`
  - `launch/mapping.launch.py`：建图链路
  - `launch/navigation.launch.py`：导航链路
  - `mapping_bringup/tracked_motor_driver.py`：GPIO/PWM 底盘驱动，订阅 `/cmd_vel`
  - `mapping_bringup/mission_manager.py`：RViz 任务点、区域巡检、直接导航桥接
  - `mapping_bringup/actual_path_recorder.py`：发布 `/actual_path`
  - `mapping_bringup/safety_monitor.py`：导航安全监控
  - `mapping_bringup/velocity_safety_gate.py`：唯一 `/cmd_vel` 发布者，统一仲裁手动与自主速度
- `robot_control_ui`
  - 电脑端 UI，通过 ROS 2 DDS 读状态，通过 SSH 控制树莓派启动/停止任务
- `robot_monitor_interfaces`
  - 任务点、气体、安全状态、UI 服务等接口
- `robot_mission_utils`
  - 地图栅格、任务点校验、A*/Theta* 路径预览和碰撞检查
- `sllidar_ros2`
  - 雷达驱动，推荐 RViz 配置在 `rviz/sllidar_ros2.rviz`
- `rf2o_laser_odometry`
  - 发布 `/odom`，导航链路中参与 `odom -> base_link`

## 运行架构

建图模式启动：

- `sllidar_ros2`
- 静态 `base_link -> laser`
- `rf2o_laser_odometry`
- `tracked_motor_driver`
- `slam_toolbox`

导航模式启动：

- `sllidar_ros2`
- 静态 `base_link -> laser`
- `rf2o_laser_odometry`
- `tracked_motor_driver`
- `mission_manager`
- `actual_path_recorder`
- `safety_monitor`
- Nav2：`map_server`、`amcl`、`planner_server`、`controller_server`、`bt_navigator`、`behavior_server`、`smoother_server`、`velocity_smoother`、lifecycle manager

不要同时运行建图和导航。`slam_toolbox` 与 `map_server + amcl` 都参与 `map` 链路，同时运行会造成 TF 或定位冲突。

## 编译

树莓派：

```bash
cd /home/yy/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

本地电脑：

```bash
cd /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

常用选择性构建：

```bash
colcon build --packages-select mapping_bringup
colcon build --packages-select robot_control_ui
colcon build --packages-select mapping_bringup robot_control_ui robot_mission_utils sllidar_ros2
```

## 启动

以后只从项目根目录运行：

```bash
cd /home/zjy/Desktop/ros_project_git_2026-05-20
./start.sh
```

`start.sh` 会在正确的 `ros_ws` 目录编译受影响包，清理本机旧 UI，然后只启动唯一的 Qt 控制台。RViz 不再默认启动，可在界面顶部点击“启动 RViz”按需打开；它不会自动让树莓派进入建图或导航。使用 UI 中的 `Start Mapping` / `Start Navigation` 选择模式，重复执行会被单实例锁拒绝。

## RViz 操作

导航启动后，可在 Qt 界面的 `Localization and Initial Pose` 输入 X/Y/Yaw 并点击 `Set Initial Pose`；也保留 RViz `2D Pose Estimate` 方式。

RViz 工具用途：

- `Publish Point`：添加普通任务点；区域巡检模式下每两个点生成一个矩形区域
- `2D Goal Pose (Mission Heading)`：只修改最近普通任务点的朝向，发布 `/mission_goal_pose`
- `2D Goal Pose (Direct Nav)`：直接导航到目标点，发布 `/goal_pose`，由 `mission_manager` 转成 Nav2 `/navigate_to_pose` action

任务执行规则：

- Qt 界面中 `TSP` 默认开启：普通多点任务按地图可通行路径代价优化顺序；关闭后严格保留 RViz 点击/请求顺序
- 10 个及以下目标使用精确开放式 TSP，更多目标使用最近邻加 2-opt；区域 TSP 同时选择区域顺序和正/反扫方向
- `Return to Start` 默认关闭；开启后仅在全部巡检成功时返回任务启动瞬间记录的 AMCL 位姿
- 只有 UI 启用 `Region Mode` 时才执行区域巡检，已保存区域不会抢占普通多点任务
- 区域巡检使用轴对齐矩形，默认 `sweep_spacing=0.10m`、`region_margin=0.23m`
- 区域方案默认保存到 `/home/yy/ros2_ws/config/inspection_regions.yaml`

RViz 主要显示：

- `/mission_points_markers`：普通任务点、朝向箭头、区域边框和区域预览 marker
- `/mission_preview_path`：任务级预览路径
- `/plan`：Nav2 当前全局规划路径
- `/actual_path`：机器人实际行驶轨迹

全局路径由 `SmacPlanner2D` 在静态地图与实时激光障碍组成的全局代价地图上生成。行为树以 1Hz 检查当前路径，仅在目标变化或路径失效时重新规划；实车继续使用 RPP 跟踪路径，并禁用 Spin 恢复。

## UI 功能

- 启动/停止建图和导航
- 设置 AMCL 初始位姿并显示确认结果
- 保存地图
- 按住按钮或键盘持续手动控制底盘，松开立即停车
- 查看 `/scan`、`/odom`、`/map`、安全状态、热成像和气体数据
- RViz 任务启动、TSP/返航选择、定位检查、任务点清空
- 区域巡检模式开关、保存、加载、清空
- 安全故障复位

本机 UI 旧进程由 `start.sh` 自动清理。由界面启动的 RViz 采用单实例管理并在主窗口关闭时退出，不影响其他方式启动的 RViz。

## 网络与 QoS

- 电脑和树莓派使用同一 ROS domain，默认 `ROS_DOMAIN_ID=0`
- 跨机器发现需要 `ROS_LOCALHOST_ONLY=0`
- `/map` 订阅端使用 `Transient Local + Reliable`
- `/scan` 使用 sensor data / best effort 兼容 QoS
- 电脑上看到树莓派节点是 DDS 网络发现结果，不代表节点运行在电脑本机

## 常用检查

```bash
ros2 topic list
ros2 topic info /map -v
ros2 topic info /scan -v
ros2 topic info /goal_pose -v
ros2 topic info /mission_goal_pose -v
ros2 action list -t
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

## 硬件与限制

- 雷达建议使用稳定的 `/dev/serial/by-path/...`，不要依赖易变的 `ttyUSB0`
- 气体传感器不要回退到雷达串口
- 底盘当前是开环 GPIO/PWM 控制，没有编码器闭环
- 驱动板电压不会自动进入 ROS，除非有 ADC/CAN/UART/I2C 等可读通道
- 纯雷达定位对环境特征敏感，空旷区域或动态干扰会降低稳定性

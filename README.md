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

## RViz 任务点模式

现在支持直接在 RViz 里选任务点，再由你确认后执行。这个模式适合巡检或多点导航。

### 1. 启动导航

先启动导航，而不是建图：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mapping_bringup navigation.launch.py map:=/home/yy/ros2_ws/map_name.yaml
```

然后在 RViz 里先用 `2D Pose Estimate` 设定机器人初始位姿。

### 2. 打开推荐 RViz 配置

本机推荐直接使用项目里的 RViz 配置：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
rviz2 -d /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/src/sllidar_ros2/rviz/sllidar_ros2.rviz
```

这个配置里已经包含：

- `Publish Point` 工具，发布到 `/clicked_point`
- `2D Goal Pose (Nav)` 工具，发布到 `/goal_pose`
- `2D Goal Pose (Mission Heading)` 工具，发布到 `/mission_goal_pose`
- `/mission_points_markers` 任务点标记
- `/mission_preview_path` 任务预览路径
- `/plan` Nav2 实际全局路径

两种 `2D Goal Pose` 的用途：

- 仅导航：使用 `2D Goal Pose (Nav)`，机器人会按 Nav2 立即前往目标
- 任务朝向编辑：使用 `2D Goal Pose (Mission Heading)`，只更新任务点朝向，不触发运动

### 3. 添加任务点

在 RViz 顶部工具栏选择 `Publish Point`，在地图上逐个点击任务点。

每点击一次：

- `mission_manager` 会记录一个 RViz 任务点
- RViz 中会显示编号标记
- 系统会自动重新计算访问顺序
- 绿色 `Mission Preview Path` 会显示任务级预览路线
- 蓝色 `Nav2 Global Path` 会在真正导航时显示 Nav2 当前实际全局路径

### 4. 给任务点指定朝向

如果某个任务点需要指定朝向：

- 选择 RViz 的 `2D Goal Pose (Mission Heading)` 工具
- 在目标任务点附近按下并拖动箭头
- `mission_manager` 会把这个朝向写入离该位置最近的 RViz 任务点

约束：

- 只有离现有 RViz 任务点足够近的 `/mission_goal_pose` 才会被接受
- 普通 `2D Goal Pose (Nav)` 只用于导航，不会再改任务点朝向
- RViz 里会显示蓝色朝向箭头和文本角度

### 5. 确认并执行

打开本机的 `robot_control_ui` 后：

- 如果 UI 本地点位列表为空，点 `Start Mission`
- UI 会询问是否使用 RViz 中已选择的任务点
- 确认后，机器人会按规划顺序执行这些点

### 6. 清空 RViz 任务点

在 `robot_control_ui` 里点 `Clear RViz Points`，会清空：

- 机器人端缓存的 RViz 任务点
- RViz 中的任务点标记
- 任务预览路径

两条路径的区别：

- `/mission_preview_path` 是 `mission_manager` 基于任务点和地图计算出的离线路径预览，不是 Nav2 的真实执行路径
- `/plan` 是 Nav2 当前全局规划器给出的真实全局路径，更接近机器人接下来要走的路线
- 两者不一致是正常现象，尤其在局部避障、重规划、控制器平滑之后会更明显

## UI 本地点位模式

除了 RViz 模式，UI 里原来的任务点模式仍然保留：

- `Add Current Pose` 把当前机器人位姿加入任务列表
- `Optimize Order` 基于地图障碍计算访问顺序
- `Current Order` 保持当前顺序，只刷新路径预览
- `Sync Points` 把 UI 任务点同步到机器人
- `Start Mission` 用 UI 里的任务点执行任务

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
  remote_host:=192.168.43.21 \
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
- 状态灯显示 `SSH / scan / odom / map / safety`
- 显示运行日志

传感器说明：

- `Start Mapping` 和 `Start Navigation` 现在只启动建图/导航主链路
- 红外热成像和气体传感器不会跟着建图或导航一起启动
- 需要时单独点 UI 里的 `Start Thermal`，会同时启动热成像和气体传感器
- `Stop Thermal` 会同时停止这两个传感器节点

安全保护说明：

- 导航链路现在会启动 `safety_monitor`
- UI 会显示 `Safety` 状态灯和安全状态卡片
- 如果检测到掉压、连续碰撞恢复或连续无进展，系统会自动取消导航并发送零速度
- 安全故障锁存时，UI 会阻止新的 `Start Mission`
- 故障排除后，需要点 UI 里的 `Reset Safety` 才能重新开始任务

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

### 5. 导航安全保护

`Start Mission` 现在会先做启动前检查：

- 任务点不能落在障碍物或未知区
- 任务点不能太贴近障碍物
- 第一个点不能离机器人当前位置过近
- 当前任务点顺序必须存在一条无碰撞预览路径

如果不满足，UI 会直接弹出原因，不会把任务发给 Nav2。

导航运行中还会持续监控：

- 掉压 `undervoltage`
- 连续 `collision ahead`
- 连续 `failed to make progress`

触发后行为是：

- 自动取消当前导航任务
- 发送零速度到 `/cmd_vel`
- UI 弹出 `Safety Fault`
- 任务锁存，直到手动 `Reset Safety`

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
- 当前运行时安全保护主要挂在导航链路；建图阶段如果主板直接掉压重启，UI 来不及弹出安全故障

## 建图掉电说明

如果建图时出现“雷达停转、SSH 断开、主板只能重启”，优先按供电问题处理，不要先怀疑 OOM。

这台车当前已经确认过：

- 出现过系统级 `Undervoltage detected!`
- 最近几次异常结束没有正常关机记录
- 异常时内存和温度都正常，因此不像 OOM 或过热关机

建图链路当前只启动：

- `sllidar_node`
- `rf2o_laser_odometry`
- `tracked_motor_driver`
- `slam_toolbox`

也就是说，建图掉电通常是“雷达 + 树莓派 + slam_toolbox 负载”把供电余量吃掉了，而不是红外或气体传感器被误启动。

建议优先排查：

- 树莓派 5V 电源是否足够硬
- 供电线是否过长、过细
- 电池和降压模块在雷达启动、CPU 升载时是否掉压
- 雷达是否直接从树莓派 USB 口取电并拉低 5V 轨

如果硬件暂时不改，软件侧可先降建图负载验证：

- `slam.yaml` 的 `resolution` 从 `0.03` 降到 `0.05`
- `throttle_scans` 从 `1` 改到 `2`
- `enable_interactive_mode` 从 `true` 改到 `false`

## 建议排查项

运行前优先确认：

- 雷达设备存在：`/dev/rplidar`
- `/scan` 有发布者
- `/odom` 有发布者
- 导航时 `/map_server` 和 `/amcl` 处于 `active`
- 建图和导航不要同时运行

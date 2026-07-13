# inspection_migrated_ws 启动命令

以下命令只负责环境配置、PyQt GUI 和 RViz 启动。建图/导航节点由 GUI 中的远程启动按钮控制，或单独在实车端启动。

## 1. 环境配置

本机 GUI/ RViz：

```bash
cd /home/zjy/Desktop/project_transform/inspection_migrated_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

实车目标板：

```bash
cd /home/yy/inspection_migrated_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

上位机和目标板必须使用相同的 `ROS_DOMAIN_ID`，并保持 `ROS_LOCALHOST_ONLY=0`。

## 2. 启动 PyQt GUI

GUI 默认运行在上位机，通过 SSH 控制目标板启动/停止实车 launch：

```bash
cd /home/zjy/Desktop/project_transform/inspection_migrated_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch inspection_robot_gui gui.launch.py \
  workspace:=/home/yy/inspection_migrated_ws \
  map:=/home/yy/inspection_migrated_ws/maps/inspection_map.yaml \
  ros_setup:=/opt/ros/jazzy/setup.bash \
  remote_user:=yy \
  remote_host:=192.168.43.21
```

GUI 启动后，在界面中使用“开始建图”“开始导航”“启动传感器”等按钮。默认目标板路径、用户名和 IP 可按现场修改。

## 3. 启动 RViz

如果实车导航链已经运行，只启动 RViz 可使用：

```bash
cd /home/zjy/Desktop/project_transform/inspection_migrated_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

rviz2 -d install/inspection_robot_bringup/share/inspection_robot_bringup/rviz/sllidar_ros2.rviz
```

也可以从实车端启动导航并同时启动 RViz：

```bash
cd /home/yy/inspection_migrated_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch inspection_robot_bringup navigation.launch.py \
  map:=/home/yy/inspection_migrated_ws/maps/inspection_map.yaml \
  use_rviz:=true
```

不要同时启动两份 navigation、mapping 或 RViz；停止当前终端进程使用 `Ctrl+C`。

## 4. 一键启动

```bash
cd /home/zjy/Desktop/project_transform/inspection_migrated_ws
./start.sh all
```

脚本模式：

```bash
./start.sh gui    # 只启动 GUI
./start.sh rviz   # 只启动 RViz
./start.sh all    # 同时启动 GUI 和 RViz
```

可通过环境变量覆盖默认参数：

```bash
ROS_DOMAIN_ID=0 \
REMOTE_USER=yy \
REMOTE_HOST=192.168.43.21 \
ROBOT_WS=/home/yy/inspection_migrated_ws \
MAP_FILE=/home/yy/inspection_migrated_ws/maps/inspection_map.yaml \
./start.sh all
```

脚本只启动 GUI 和 RViz，不会绕过 GUI 或导航栈的安全速度门，也不会自动伪造传感器数据。

# 快速启动

## 树莓派环境

```bash
source /opt/ros/jazzy/setup.bash
source /home/yy/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

## 电脑环境

```bash
source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

## 树莓派建图

```bash
ros2 launch mapping_bringup mapping.launch.py
```

## 树莓派导航

```bash
ros2 launch mapping_bringup navigation.launch.py map:=/home/yy/ros2_ws/map_name.yaml
```

## 电脑端 UI

```bash
ros2 launch robot_control_ui ui.launch.py \
  remote_user:=yy \
  remote_host:=192.168.43.21 \
  workspace_path:=/home/yy/ros2_ws \
  map_path:=/home/yy/ros2_ws/map_name.yaml
```

## 电脑端 RViz

```bash
rviz2 -d /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/src/sllidar_ros2/rviz/sllidar_ros2.rviz
```

## RViz 工具

- `Publish Point`：添加任务点；区域模式下两点成框
- `2D Goal Pose (Mission Heading)`：设置最近任务点朝向
- `2D Goal Pose (Direct Nav)`：直接导航到目标

## 常用清理

```bash
pkill -f robot_control_ui
pkill -f "ros2 launch robot_control_ui"
ros2 daemon stop
ros2 daemon start
```

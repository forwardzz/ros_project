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

source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0


两边要求：

- ROS 2 版本一致，当前使用 Jazzy
- `ROS_DOMAIN_ID` 一致
- `ROS_LOCALHOST_ONLY=0`
- 电脑和树莓派在同一局域网

### 3. 启动 UI

source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
ros2 launch robot_control_ui ui.launch.py \
    remote_user:=yy \
    remote_host:=192.168.43.21 \
    workspace_path:=/home/yy/ros2_ws \
    map_path:=/home/yy/ros2_ws/map_name.yaml
 


### 2. 打开推荐 RViz 配置

source /opt/ros/jazzy/setup.bash
source /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/install/setup.bash
rviz2 -d /home/zjy/Desktop/ros_project_git_2026-05-20/ros_ws/src/sllidar_ros2/rviz/sllidar_ros2.rviz

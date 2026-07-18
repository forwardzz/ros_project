# inspection_migrated_ws 部署与启动

上位机只运行 PyQt GUI 与 RViz；树莓派只运行传感器、底盘、安全、定位、SLAM/Nav2 和任务节点。两台机器固定使用 `ROS_DOMAIN_ID=0`、`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 和 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；启动链会清除 Jazzy 已弃用的 `ROS_LOCALHOST_ONLY`。

## 安全门槛

实机上电前必须完成独立 5 V/5 A 树莓派供电改造，隔离扩展板向 GPIO 5V 引脚反供，两个 5V 电源不得并联，只保留公共地。物理急停必须能独立切断电机侧。电机端口确认前保持 `motor_pair:=disabled actuation_enabled:=false`。

## 部署和目标板构建

从上位机执行：

```bash
cd /home/zjy/Desktop/ros_project_migration/inspection_migrated_ws
./deploy_robot.sh
```

脚本同步到 `yy@192.168.43.24:/home/yy/inspection_migrated_ws`，排除 `.git/build/install/log`，并保留远端 `maps/`（包括地图和 `inspection_regions.yaml`）。随后跳过 GUI，按依赖顺序逐包单任务构建。只同步不构建可使用 `DEPLOY_ONLY=1 ./deploy_robot.sh`。

## 上位机启动

```bash
cd /home/zjy/Desktop/ros_project_migration/inspection_migrated_ws
./start.sh all
```

`all` 同时启动本地 GUI 和 RViz；`gui`、`rviz` 分别只启动一项。GUI 的 SSH 导航命令不会在树莓派启动 RViz。默认远端参数可覆盖：

```bash
REMOTE_USER=yy \
REMOTE_HOST=192.168.43.24 \
ROBOT_WS=/home/yy/inspection_migrated_ws \
MAP_FILE=/home/yy/inspection_migrated_ws/maps/inspection_map.yaml \
./start.sh all
```

`start.sh` 默认在上位机使用 Fast DDS 和同网段自动发现，`LaunchManager` 会把同一个 RMW 与 discovery range 传播给目标板 SSH 进程。不要让上位机使用 Fast DDS、目标板沿用系统默认 Cyclone DDS：实测这种混合配置下 topic 看似正常，但 service/action 会超时，并可能出现 RTPS payload size 错误。若覆盖 RMW 或 discovery range，两端必须保持一致；远程运行不得使用 `LOCALHOST`/`OFF`。

GUI 的“刷新地图”通过 SSH 枚举 `${ROBOT_WS}/maps`，不是在上位机本地查找该路径；正常连接时日志应显示 `[MAP] ... found 1 map yaml file(s)`。建图日志必须包含 `motor_pair:=disabled ... actuation_enabled:=false`，若仍看到缺少这些参数的旧日志，应完全关闭旧 GUI 进程后重新启动当前 workspace。

“保存地图”仅在 GUI 管理的建图进程运行且 `/map` 数据新鲜时可用，输出固定到远端 `${ROBOT_WS}/maps/<名称>.yaml`，未写扩展名时自动补 `.yaml`。同名 YAML 或 PGM 已存在时必须确认覆盖；新地图会先写入远端临时目录，校验两份文件后再替换，失败时保留旧地图。保存成功后 GUI 会刷新远端地图列表并显示完整路径；不要在导航模式下用该按钮重写静态地图。

GUI 中 `电机端口` 默认为 `disabled`，且每次 GUI 启动都会取消勾选“启用电机输出”。本车选择 `cd` 时使用旧实机已验证的直接 GPIO 接口：C/左=`GPIO18 PWM + GPIO22/GPIO27`，D/右=`GPIO23 PWM + GPIO25/GPIO24`，100 Hz。架空轮逐侧及双侧实测已确认通道对应正确，正向极性为默认的 `left_inverted=false`、`right_inverted=false`；前进、后退、左转和右转均符合 ROS 符号约定。未命令侧在 PWM 为零时可能被机械传动轻微带动。`ab` 是未验证的 PCA 备用路径。只有架空履带并由现场人员守住物理急停后，才可显式启用，不得放宽速度上限。

手动行进不是仅启动 GUI 后即可使用：必须先选择 `cd`、勾选“启用电机输出（仅架空轮）”，再点击“开始建图”或“开始导航”，等待 `/cmd_vel_teleop` 安全门、SAFE 状态和新鲜雷达扫描就绪。“启动传感器”按钮只启动红外/气体监测，不启动底盘。方向按钮和 W/A/S/D 键采用按住运行、松开立即归零；默认架空测试速度为 `0.02 m/s`、`0.10 rad/s`。若 GUI 提示底层未启动、电机未启用、安全状态或雷达超时，应按提示排查，禁止绕过检查。

上位机入口已实测：GUI 可启动，远端地图刷新可找到 `inspection_map.yaml`，传感器按钮可收到 32×24 热帧，禁用电机的导航按钮可完成定位、服务和同位姿 action。终端 Ctrl+C 会清理目标板 launch/传感器进程和本地 ROS adapter，并干净退出；若看到 traceback 或远端残留，说明运行的不是当前构建。

GUI 会按屏幕可用区域调整初始尺寸。本机 2880×1800、2×缩放环境使用 1200×720 逻辑窗口，不会被自动最大化；左右栏均避免横向裁切，较长的热成像、手动控制和状态内容通过各栏纵向滚动访问。

启动导航前填写的“初始位姿”同时作为可选返航的固定起点。GUI 会把该位姿和远端 `${ROBOT_WS}/maps/inspection_regions.yaml` 传给任务管理器；勾选“任务完成后返回起点”只在全部任务正常完成后返航，任务失败、区域持续阻塞或人工取消时保持停车。修改初始位姿后必须重新启动导航，不能只重新点击“开始任务”。

## 目标板手动诊断（电机禁用）

```bash
ssh yy@192.168.43.24
cd /home/yy/inspection_migrated_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch inspection_robot_bringup navigation.launch.py \
  map:=/home/yy/inspection_migrated_ws/maps/inspection_map.yaml \
  motor_pair:=disabled actuation_enabled:=false
```

导航启动后必须从 GUI 发布初始位姿（消息时间戳保持为零，让 AMCL 使用最新 TF），再确认 `map -> odom -> base_link` 和 Nav2 managed nodes 均已激活。全局代价地图以 `robot_radius=0.18 m` 包络车体，局部代价地图保留真实多边形，膨胀半径均为 `0.25 m`。Nav2 Jazzy 1.3.11 的 `SmacPlanner2D` 在圆形模式下仍会打印一条 inflation 错误；这是该版本先检查零 `possible_collision_cost`、后判断圆形模式造成的日志误报，不能通过减小安全尺寸来消除。远端现有 `inspection_map` 是静止诊断地图，不得直接用于地面自主运行。

禁用电机复测已确认任务目标可进入 Nav2 action，控制命令按 `/cmd_vel_nav -> /cmd_vel_auto -> /cmd_vel` 路由；“停止所有任务”后各层在 0.10 秒内归零。若任务尚未发布初始位姿、没有任务点或 managed node 未全部 active，GUI 不应强行启动任务。此软件闭环不代表地面路径跟随已经验收。

传感器独立启动默认启用 MLX90640、禁用不存在的气体传感器：

```bash
ros2 launch inspection_robot_hardware sensor_monitor.launch.py
```

当前目标板实测雷达路径为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，已作为建图和导航默认值；更换 USB 串口设备后必须重新核对，不能沿用旧 by-path。

电机后端禁用时可验证锁存软件急停：

```bash
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
ros2 service call /reset_safety_monitor std_srvs/srv/Trigger "{}"
```

急停后应看到 `/safety_stop=true`、安全状态 `FAULT/EMERGENCY_STOP` 和 `/cmd_vel` 零速度；传入 `{data: false}` 不得解除锁存。现场复测急停归零为 0.021 秒，只有显式复位后才恢复放行。软件急停不能替代切断电机侧的物理急停。

SLLIDAR 多次快速启停后可能出现“健康状态正常、扫描模式已启动，但暂时没有 `/scan`”的状态。驱动会记录取帧超时，并在连续三次失败后停转 3 秒再启动扫描；未收到真实帧时最多连续恢复两次。没有新鲜 `/scan` 时速度门持续禁止运动，扫描 watchdog 硬限制为不大于 0.40 秒，实测最后扫描帧到零速度为 0.403337 秒。若自动恢复仍无数据，应停止全部机器人节点，现场检查或重新插拔雷达 USB/电源，禁止放宽超时或绕过速度门。

GUI 停止按钮会通过 SSH 清理建图/导航及其 IMU、EKF、TF、任务、安全和 Nav2 子进程。停止后可用 `ros2 node list` 与 `ps` 确认无残留；若 GUI 或网络异常退出，重新启动前必须先做该检查，不能依赖同名节点警告继续运行。

RF2O 的逐帧耗时和位姿只在 DEBUG 级别输出；正常 INFO 日志不应再以雷达频率滚动。若出现持续日志洪泛，应核对目标板是否已重新构建当前 `rf2o_laser_odometry` 源码。

完整的实机检查顺序和立即停止条件见 `HARDWARE_SAFETY_REVIEW.md`。

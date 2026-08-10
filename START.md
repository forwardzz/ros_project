# 快速启动

项目统一入口是根目录的 `start.sh`：

```bash
cd /home/zjy/Desktop/ros_project_git_2026-05-20
./start.sh
```

`start.sh` 会：

- 在 `ros_ws` 中编译界面、任务和安全链相关包
- 设置 `ROS_DOMAIN_ID=0` 和 `ROS_LOCALHOST_ONLY=0`
- 清理本机残留的 Tk UI/RViz
- 启动唯一的 Tk 控制台和项目 RViz 配置
- 在 UI 或 RViz 退出时收尾另一个本机进程

`start.sh` 不会自动启动树莓派底盘。界面打开后：

1. 使用 `Start Mapping` 或 `Start Navigation` 启动树莓派对应后端，两者不要同时运行。
2. 导航模式下，在 `Localization and Initial Pose` 输入 X/Y/Yaw，点击 `Set Initial Pose`。
3. RViz 中用 `Publish Point` 添加任务点，用 `Mission Heading` 修改点位朝向。
4. 在 Tk 界面确认点位停留时间、`TSP` 和 `Return to Start`；TSP 默认开启，返航默认关闭，再点击 `Start Mission`。
5. 区域任务需先启用 `Region Mode`，每两个 `Publish Point` 定义一个矩形。

如需更换机器人地址，可仅对当次启动覆盖环境变量：

```bash
REMOTE_HOST=192.168.43.30 ./start.sh
```

不要再直接运行 `ros2 launch robot_control_ui ...` 或另起 `rviz2`；重复执行 `start.sh` 会显示“control console is already running”并退出。

## 状态监控说明

- `Scan LAN` 扫描当前子网并列出设备（IP/主机名/MAC/SSH 端口），`Apply IP` 应用所选地址。
- 系统卡片显示树莓派温度、CPU、内存、运行时间与欠压状态；无 ADC 时 5V 输入电压显示 `N/A`。
- `/map` 以"收到地图 + 存在发布者"判定可用，避免静态地图被误判掉线。

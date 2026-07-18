# 实机部署前软件安全审查

审查日期：2026-07-13；实机复核：2026-07-18
审查对象：`../inspection_migrated_ws`（迁移后的实车工作空间）及其与本目录实车项目的接口边界。  
审查方式：静态代码、参数、launch 图和离线构建检查；2026-07-18 在目标树莓派上补充了传感器、GPIO、安全门、失效停车和架空履带测试。

## 结论

迁移工作空间已经具备一条受限的单出口速度链，并已修复本次发现的若干明显软件故障点：命令超时、无激光数据放行、手动接管、节点退出停车、重复底盘启动、Nav2 行为服务器旁路和软件急停。目标板构建与测试通过。现场已确认独立电机急停、树莓派独立供电隔离和架空履带条件；雷达、IMU、MLX90640、C/D GPIO 底盘、安全门及失效停车均完成实机复核。

2026-07-18 在修复 Nav2 行为服务器旁路后重新验收：`/cmd_vel` 已恢复为安全门一个 publisher、底盘一个 subscriber；雷达约 9.22 Hz、IMU 10.00 Hz、32×24 热帧约 1.97 Hz；命令输入超时和安全监控失联分别在 0.304 s、0.257 s 停车。人工关闭雷达时，从最后一帧 `/scan` 到 `/cmd_vel=0` 实测 0.5199 s，超过严格的 0.500 s 验收线，因此本次重新验收判定未完全通过，并按立即停止条件跳过了后续架空履带动作。

针对该结果，扫描 watchdog 的默认值和硬上限已从 0.50 s 下调为 0.40 s，速度门检查周期硬上限为 0.05 s，离线名义最坏预算不大于 0.45 s；输入命令和安全心跳的参数上限也分别锁定为 0.35 s、0.30 s。修复 SLLIDAR 快速重启后的扫描恢复后，目标板在电机后端禁用条件下重新实测：从最后一帧 `/scan` 到 `/cmd_vel=0` 为 **0.403337 s**，从停雷达请求到零速度为 0.482052 s，均满足 0.500 s 验收线；重新启动雷达后 3.175717 s 恢复扫描，首帧后 0.003986 s 重新放行测试输入。

架空履带已确认 `linear.x` 正/负分别对应前进/后退，`angular.z` 正/负分别对应逆时针左转/顺时针右转，默认 `left_inverted=false`、`right_inverted=false`。这仍不等同于地面自主导航验收：实际轮距/轮径、开环死区、制动距离、雷达外参和地图定位仍需在隔离场地低速验证。

同日使用 GUI 的 SSH 启动链、保存的静止诊断地图和禁用电机后端完成 Nav2 复测：GUI 零时间戳初始位姿可使 AMCL 建立 `map -> odom`，map server、AMCL、planner、controller、BT navigator、behavior、smoother 和 velocity smoother 均进入 active；`/localize_robot` 成功，空闲状态调用 `/abort_mission` 可安全返回成功。全局代价地图采用包络车体的 `robot_radius=0.18 m`，局部代价地图保留 `0.27 × 0.22 m` 多边形，两者膨胀半径均为 `0.25 m`。该地图仅用于静止链路诊断，不可作为地面导航验收地图。

修复 GUI 远端地图枚举后，再次由当前 `LaunchManager` 同时启动建图和 MLX90640，仍保持 `motor_pair=disabled`、`actuation_enabled=false`。5 秒采样得到 `/scan` 9.32 Hz、IMU 9.91 Hz、`/laser_odom` 9.32 Hz、EKF `/odom` 9.91 Hz、`/map` 1.19 Hz、安全状态 10.90 Hz、热帧 1.98 Hz；扫描为 1080 点，IMU 四元数范数为 1，热帧为 768 点且温度 31.44–36.57 °C，安全状态为 SAFE。GUI 自动停止后 ROS 节点、相关进程及雷达/IMU 串口占用均为零，目标板 `get_throttled=0x0`。

同一禁用电机链路完成软件急停闭环：持续发布 `0.02 m/s` 测试输入时，`/emergency_stop=true` 后 **0.021 s** 内 `/cmd_vel` 归零并锁存 `FAULT/EMERGENCY_STOP` 与 `/safety_stop=true`；以 `false` 尝试解锁被拒绝，只有 `/reset_safety_monitor` 成功后才恢复放行。停止输入后 **0.360 s** 自动归零。建图模式没有任务管理器时，中止任务请求已改为非阻塞检查，急停期间安全心跳最大间隔为 0.103 s，低于 0.30 s 硬上限。

禁用电机的导航任务闭环也已通过：经当前 `LaunchManager` 启动导航，发布诊断初始位姿后八个 Nav2 managed node 全部 active；任务服务正确拒绝空任务，并接受地图内 0.422 m 的安全测试目标。实测非零软件命令依次出现在 `/cmd_vel_nav`、`/cmd_vel_auto` 和最终 `/cmd_vel`；主动调用 `/abort_mission` 后三者分别在 **0.002/0.087/0.100 s** 内归零并持续为零，typed mission status 退出 active，Nav2 action 不再活动。目标板实际参数仍为 `motor_pair=disabled`、`actuation_enabled=false`，最终话题保持安全门一个 publisher、禁用驱动一个 subscriber。该试验只证明任务/action/取消和速度路由，不证明机器人已完成地面路径跟随。

随后从上位机实际运行 GUI 的 `RosAdapter` 完成跨主机复测。最初上位机使用 Fast DDS、目标板使用 Cyclone DDS 时，普通 topic 可以互通，但 service/action 超时，并出现 RTPS history payload size 错误；两端统一为 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 后故障消失。上位机收到 1080 点 `laser` 扫描、`imu_link` IMU、82×90 地图、32×24 热帧和 SAFE 状态；初始位姿到 AMCL 更新为 0.126 s，定位服务成功，空任务被正确拒绝，同位姿 `NavigateToPose` 成功，空闲中止服务成功。复测时 `bt_navigator` 与 `velocity_smoother` 均为 active，ROS 图有 33 个节点，最终 `/cmd_vel` 为一个 publisher、禁用驱动一个 subscriber。目标板未运行 GUI/RViz，且全程禁用电机；该结果证明 PC↔目标板的数据、服务和 action 链路，不替代地面路径跟随验收。

又从用户实际入口 `./start.sh gui` 复核 GUI 本身：脚本权限为 755，界面可启动；“刷新地图”从目标板找到一份 `inspection_map.yaml`；“启动传感器”收到 32×24、768 点 MLX90640 热帧；“开始导航”在禁用电机条件下完成 1080 点扫描、IMU、地图、安全状态、AMCL、定位、空任务拒绝、同位姿 action 和空闲中止闭环。启动链现使用 Jazzy 的 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 并清除已弃用的 `ROS_LOCALHOST_ONLY`。终端 Ctrl+C 会依次停止远端传感器/bringup、关闭 ROS adapter 并以退出码 0 结束，不再留下 traceback 或远端进程。

## 本次确认的运动指令链

```text
GUI 手动控制 ──> /cmd_vel_teleop ─┐
                                   ├─> velocity_safety_gate ─> /cmd_vel
Nav2 controller ─┐
Nav2 behaviors ──┼─> /cmd_vel_nav ─> velocity_smoother ─> /cmd_vel_auto ─┘
巡检区域底盘原语 ─┘
                                                            ↓
                                      tracked_motor_driver (V4.0 PCA/GPIO)
```

最终输出门 `velocity_safety_gate` 是唯一设计上的 `/cmd_vel` 发布者；底盘驱动只订阅 `/cmd_vel`。2026-07-18 复测曾发现 `behavior_server` 的五个行为插件直接发布 `/cmd_vel`，修复重映射后实测 `/cmd_vel` 恢复为安全门一个 publisher、底盘一个 subscriber；五个行为端点均进入 `/cmd_vel_nav`。手动通道优先于自动通道，且 GUI 发送手动命令时会取消 Nav2 目标并请求中止巡检任务。两套硬件 bringup 之间新增了主机锁；底盘驱动和速度门也各自有进程锁，第二个实例保持不执行/不发布。

运行前仍必须实测确认：

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_auto -v
ros2 topic info /cmd_vel_nav -v
```

通过条件是 `/cmd_vel` 只有 `velocity_safety_gate` 一个 publisher、`tracked_motor_driver` 只有一个 subscriber；出现未预期 publisher 时立即停止 bringup。

## 本次修复的明显软件安全问题

| 问题 | 修复位置 | 修复结果 |
|---|---|---|
| 激光断流时仍可能放行旧的运动命令 | `inspection_robot_safety/velocity_safety_gate.py` | 默认要求 `/scan` 在 0.40 s 内更新，检查周期不大于 0.05 s；超时、未收到、安全状态超时、故障或外部停止均发布零速度。 |
| GUI 手动控制不能立即抢占自主导航 | `inspection_robot_gui/main_window.py`、速度门 | 手动通道优先；手动操作取消导航并中止任务，避免松开手动键后自主命令恢复。 |
| GUI 未启动底层时把可选任务服务缺失误报为手动控制故障，按钮又只发送一次 Twist | `inspection_robot_gui/{main_window,ros_adapter,config}.py` | 手动非零命令前检查 GUI bringup、电机显式启用、安全门订阅、SAFE 心跳和新鲜扫描；任务服务缺失不再弹错误。按钮/键盘按住时每 0.10 s 续发，松开立即归零，默认架空速度降为 0.02 m/s、0.10 rad/s。 |
| 电机驱动 watchdog 仅每 0.5 s 清目标且继续缓停 | `inspection_robot_hardware/tracked_motor_driver.py` | 0.05 s watchdog；命令超过 0.35 s、NaN/Inf、节点退出均立即将 PWM 和方向脚置零。 |
| 参数可把速度、PWM 或失效超时配置到危险值 | 电机驱动、速度门 | 最终边界硬限制：0.18 m/s、0.55 rad/s、70% PWM、0.35 m/s²、0.70 rad/s²；命令、安全心跳和扫描超时上限为 0.35/0.30/0.40 s，检查周期上限 0.05 s；非法/非有限参数回退安全值。 |
| 两个 launch 可能重复启动电机驱动/速度门 | `mapping.launch.py`、`navigation.launch.py`、电机驱动、速度门 | 共享 bringup 文件锁拒绝第二套 mapping/navigation；电机板与 `/cmd_vel` 输出另有独占锁。 |
| Nav2 behavior server 绕过最终速度门 | `navigation.launch.py`、bringup launch 回归测试 | 将 controller 和五个 behavior 插件统一重映射到 `/cmd_vel_nav`，再经 velocity smoother 与 `/cmd_vel_auto` 进入安全门；目标板电机禁用复测确认 `/cmd_vel` 仅剩安全门一个 publisher。 |
| 迁移后 C/D 的 PCA/GPIO 混合接口未能驱动实机 | `inspection_robot_hardware/{motor_board,tracked_motor_driver}.py` | 按旧实机项目恢复已验证接口：C/左=`GPIO18 PWM + GPIO22/GPIO27`，D/右=`GPIO23 PWM + GPIO25/GPIO24`，100 Hz；架空轮逐侧及双侧测试确认通道对应正确、两侧正向均无需软件反向，默认 `left_inverted=false`、`right_inverted=false`。前进、后退、左转和右转均符合 ROS 符号约定。未命令侧出现的轻微反向带动发生在其 PWM 为零时，按机械耦合处理。仍保留启动归零、异常锁死、0.35 s watchdog、限幅和进程锁。A/B PCA 仅作未选备用；默认端口未选择且执行禁用。 |
| 任务管理器退出时未明确发送停车 | `mission_manager.py` | `destroy_node()` 先发布 `/cmd_vel_nav` 零速度并停止区域控制 timer。 |
| 监控节点退出后停止状态未锁存 | `safety_monitor.py` | 退出前发布锁存的 FAULT/`/safety_stop`；速度门的安全状态失联 watchdog 也会停车。 |
| 没有人工软件急停入口 | `safety_monitor.py`、GUI | 新增锁存服务 `/emergency_stop`（`std_srvs/SetBool true`），GUI 有“紧急停止（锁存）”按钮；只可经确认后的 `/reset_safety_monitor` 复位。 |
| 建图模式急停时等待不存在的任务中止服务会阻塞安全心跳 | `safety_monitor.py`、安全超时回归测试 | `/abort_mission` 改为即时可用性检查；目标板复测急停心跳最大间隔 0.103 s，速度在 0.021 s 内归零。 |
| GUI 的 SSH 命令、地图枚举和停止清理未正确作用于远端 | `inspection_robot_gui/{launch_manager,main_window}.py` | 将整个 `bash -lc` 命令作为一个引用后的 SSH 参数传递；地图列表改为通过 SSH 查询远端 workspace；`pkill` 使用自规避表达式并覆盖 IMU、EKF、静态 TF、任务及 Nav2 子进程。实测可找到保存地图、启动单套建图/传感器链，停止后相关进程为零。 |
| 上位机与目标板使用不同 RMW 时 topic 看似正常但 service/action 不可靠 | `start.sh`、`deploy_robot.sh`、`inspection_robot_gui/launch_manager.py` | 默认并传播 `rmw_fastrtps_cpp`，确保上位机 GUI 与目标板进程使用同一 RMW；实际 `RosAdapter` 跨主机数据、服务和 action 复测通过。 |
| SLLIDAR 快速重启后健康正常但无扫描帧 | `sllidar_ros2` | 显式传递 `Sensitivity` 模式；取帧超时可见，连续三次失败时停转 3 s 后恢复扫描，连续恢复最多两次。人工 `/stop_motor` 不会被自动恢复覆盖。 |
| 空闲任务中止返回失败且关机时可能在失效 ROS context 发布 | `inspection_robot_mission/mission_manager.py` | 空闲 `/abort_mission` 现在返回成功并发送零速度；销毁时先检查 context，SIGTERM 复测无 traceback。 |
| RF2O 每帧输出三条 INFO 且可能在 TF 尚未就绪时初始化 | `rf2o_laser_odometry` | 逐帧耗时/位姿降为 DEBUG；20 Hz timer 的正常空转不再产生 WARN；首次扫描必须取得 laser-to-base TF，默认初始姿态使用有效单位四元数。目标板单包单线程重编译并复测，GUI 日志洪泛消失。 |

这些修复位于 `inspection_migrated_ws`；本目录既有实机代码未被改写，只有本审查报告新增。

## 低速地面测试前的剩余约束

1. **物理急停每次测试前仍须复核。** 现场已报告急停可独立切断电机侧，但软件 `/emergency_stop` 仍依赖 ROS、CPU、GPIO 和 H 桥，不能替代硬件急停。地面测试前必须再次确认急停、断线和掉电均为安全状态。
2. **履带极性和转向符号已完成架空验证。** `linear.x > 0` 前进、`linear.x < 0` 后退、`angular.z > 0` 逆时针左转、`angular.z < 0` 顺时针右转均通过 0.8 s 短脉冲验证；任何接线或电机端口变更后必须重新执行。
3. **几何与制动能力未经标定。** `wheel_radius=0.025 m`、`track_width=0.155 m`、`max_rpm=80` 为开环假设；没有编码器闭环或真实轮速反馈。若死区 PWM 导致突跳或停车距离过长，禁止地面测试。
4. **激光坐标安装未实测。** launch 声明 `base_link -> laser` yaw 为 `+3.1415926 rad`。若物理雷达朝向不同，障碍物会在错误方向，可能导致导航撞向障碍物。必须在 RViz 验证前方障碍显示在 `base_link` 正前方。
5. **不得将旧实机项目和迁移 workspace 混合运行。** 旧项目 UI 曾直接使用 `/cmd_vel`；与迁移链同时 source/launch 会破坏“单出口”假设。实车测试仅 source `inspection_migrated_ws/install/setup.bash`，并先清理旧节点。
6. **树莓派独立供电接线已由现场确认，但仍需负载观察。** Raspberry Pi 5 官方推荐 27 W USB-C 电源，规格表给出的推荐电流能力为 5 A。现场已报告隔离扩展板到 GPIO 5V 的反供、两个 5V 电源不并联且只保留公共地；测试期间 `vcgencmd get_throttled=0x0`。仍需在完整导航负载下持续观察。见 [Raspberry Pi 官方供电说明](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#power-supply)。

## 架空轮测试前应完成/确认的问题

| 项目 | 要求 |
|---|---|
| 物理电气 | 断电时电机不转；急停独立切断驱动；确认保险丝、电源极性、H 桥散热和公共地。 |
| 软件版本 | 在树莓派上重新构建本次迁移 workspace；不得使用旧 `ros_project` 的底盘/GUI launch。 |
| 电机端口 | 根据 PCB 丝印或清晰照片确认使用 A/B 或 C/D；确认前 `motor_pair=disabled`、`actuation_enabled=false`。 |
| 独占启动 | 只启动 mapping 或 navigation 之一，不能并行；确认 `/tmp/inspection_robot_bringup.lock` 生效。 |
| 速度图 | `/cmd_vel` 仅安全门发布；`/cmd_vel_auto` 仅 velocity smoother 发布；`/cmd_vel_nav` publisher 仅限 controller、mission manager 和 behavior server，且唯一 subscriber 为 velocity smoother。 |
| TF 图 | `map -> odom` 在导航时仅由 AMCL、建图时仅由 SLAM Toolbox 发布；`odom -> base_link` 仅由 EKF 发布；`base_link -> laser` 与 `base_link -> imu_link` 仅由静态发布器发布。不要同时启动独立 RF2O/IMU 展示 launch。 |
| 传感器门 | 速度门未收到新鲜 `/scan`、`/robot_safety_status` 时必须持续输出零速度。确认激光数据频率大于 2 Hz。 |
| 急停 | 在未使电机上电的情况下验证 GUI 与 `ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"` 会使 `/safety_stop=true`、安全状态为 `FAULT`、`/cmd_vel` 为零；仅在检查后调用 `/reset_safety_monitor`。 |

## 可在低速地面测试中验证的问题

- 以不超过 `0.02 m/s`、`0.10 rad/s` 验证前进/后退、原地左/右转符号和最小可控 PWM；任一方向错误立即急停。
- 以不超过 `0.05 m/s` 验证 watchdog：停止发布 `/cmd_vel_teleop` 后，电机应在 0.5 s 内无驱动力；激光断流/拔除的等效测试后速度门应立即为零。
- 验证 RF2O + EKF 的 `odom -> base_link` 连续性及 AMCL 的 `map -> odom`；定位跳变、TF 超时或激光前方障碍方向异常时停止。
- 验证 Nav2 失败、目标取消、局部代价地图无数据、控制器无进展时，任务会取消且 `/cmd_vel_auto` 归零。安全门独立要求新鲜扫描，但当前没有“IMU 丢失即硬停”的策略；IMU 丢失时应停止自主导航并排查。
- 验证串口断开行为：雷达断开应由扫描 watchdog 禁止运动；IMU/RF2O/气体和红外断开不得产生伪造运动命令。迁移工作空间未发现 CAN 控制链；气体串口仅用于监测，不得作为安全停车的唯一依据。

## 单位、方向和接口审计

| 接口/参数 | 代码单位 | 当前结论 |
|---|---|---|
| `geometry_msgs/Twist.linear.x` | m/s | GUI、Nav2、速度门、电机驱动一致；最终硬上限 0.18 m/s。 |
| `Twist.angular.z` | rad/s | GUI、Nav2、速度门、电机驱动一致；最终硬上限 0.55 rad/s。 |
| GUI 初始航向显示 | degree | 仅 GUI 输入/显示使用 degree，发布前转换为 rad。 |
| TF/姿态/路径 yaw | rad | Nav2、EKF、静态 TF 按 ROS 标准 rad。 |
| 电机内部 | rad/s -> RPM -> PWM % | 开环换算；`max_rpm` 是 RPM，`max_pwm` 是百分比，不是速度单位。 |
| encoder tick | 不使用 | 当前底盘无编码器闭环；`/wheel_odom` 是可选接口，不可假定存在或准确。 |

计划 TF 树为：导航模式 `map -> odom -> base_link -> {laser, imu_link}`；建图模式没有 `map -> odom` 的 AMCL 边。RF2O 配置 `publish_tf: false`，避免与 EKF 竞争；EKF 配置 `publish_tf: true`。`base_footprint` 当前不使用，所有 Nav2 base frame 都是 `base_link`，不得临时增加第二条静态链。

## 仿真时间和仿真依赖

- `nav2_params.yaml`、`slam.yaml` 和 launch 传参均为 `use_sim_time: false`。`slam_toolbox` 自身显示的默认值为 true，但 bringup 显式覆盖为 false。
- DWA 参数 `sim_time: 2.5` 是轨迹前瞻时长，不是 ROS/Gazebo 时钟。
- 迁移运行链未发现 Gazebo、`ros_gz`、`/clock`、Gazebo plugin/service 依赖。不要额外启动源仿真 workspace 的 Gazebo launch。

## 推荐测试顺序、通过条件和立即停止条件

| 阶段 | 操作（未来实测） | 通过条件 | 立即停止条件 |
|---|---|---|---|
| 0. 断电审查 | 电机动力断开，仅检查接线、急停和软件版本 | 急停独立断开驱动；旧节点已清理 | 无物理急停、线序/电源不明。 |
| 1. 软件静态启动 | 不给电机上电，启动单一 bringup，查看 topic/TF | `/cmd_vel` 单 publisher；安全状态新鲜；TF 为计划树 | 任意重复 driver/gate、TF 同 child 多父、`use_sim_time=true`。 |
| 2. 软件急停/看门狗 | 发布一次极低 Twist 后停止发布；触发软件急停 | 0.5 s 内 `/cmd_vel=0`；`/safety_stop=true` 后不再放行 | 非零输出持续、复位无需确认、急停不锁存。 |
| 3. 架空轮 | 电机上电、履带离地，以 0.02 m/s / 0.10 rad/s 点动 | 前进、后退、左右转符合 REP-103；停止后无爬行 | 方向相反、突跳、PWM 不归零、异常发热/异响。 |
| 4. 传感器失效 | 在架空轮状态模拟扫描/安全监控/定位中断 | 雷达中断 0.50 s 内禁止运动；任务取消后零速度 | 激光或安全状态丢失仍转动；TF 不连续。 |
| 5. 定位与导航 | 无人站在运动路径，低速 0.05 m/s 验证初始位姿、障碍层、取消目标 | 前方障碍位置正确；取消/失败时停车 | 地图漂移、激光朝向错误、控制器重复碰撞/无进展。 |
| 6. 受监护地面巡检 | 开阔隔离区域、限速、操作员守住物理急停 | 制动距离和转向可预测，所有失效测试已通过 | 任一传感器异常、失控、不可预测轨迹、人员/障碍进入安全区。 |

## 残余风险与部署约束

- 软件急停、GUI 停止、任务取消和 ROS watchdog 都不是功能安全认证机制；物理急停拥有最高优先级。
- PCA9685 I²C 写异常会触发尽力归零并锁死后续软件动作；若 I²C 总线完全失效，软件无法证明已清除板上最后一次 PWM，因此物理急停仍是必需项。
- `safety_monitor` 的欠压检测依赖树莓派可用接口，且启动文件当前将 `fault_on_undervoltage_seen` 设为 false。部署前应确认真实欠压输入、阈值和策略；不要把树莓派 core voltage 当作电机电池电压。
- 碰撞/无进展监测目前依赖 Nav2 日志和任务中止；它是辅助防护，不能替代激光距离阈值、安全激光或硬件防撞。
- 默认雷达设备路径、IMU 串口和 `lidar_yaw` 都是硬件相关参数；在目标车上必须以 udev 固定路径和实测安装值替换/确认，不能凭同名设备推断。
- SLLIDAR 多次快速启停后曾出现健康检查和扫描模式成功但 `/scan` 无数据；速度门在整个异常期间保持阻断。驱动现会在连续三次取帧失败后执行有限次数的停转/重启，实测恢复后 `/scan` 稳定约 9.21 Hz。若两次自动恢复仍失败，必须停止全部机器人节点并现场检查 USB/供电，禁止放宽扫描超时或绕过速度门。
- 目标板 Nav2 Jazzy 1.3.11 的 `SmacPlanner2D` 会在圆形检查模式下传入零 `possible_collision_cost`，而同版本碰撞检查器在判断圆形模式前记录一条“inflation insufficient”错误。实测参数为半径 0.18 m、膨胀 0.25 m 且规划器正常 active；这是上游日志误报，不应通过缩小车体或本地复制 Nav2 来规避。
- 本报告只说明离线软件审查结果，不构成对制动性能、EMC、电源完整性、机械强度或人身安全的验收。

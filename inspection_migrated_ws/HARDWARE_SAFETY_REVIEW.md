# 实机部署前软件安全审查

审查日期：2026-07-13  
审查对象：`../inspection_migrated_ws`（迁移后的实车工作空间）及其与本目录实车项目的接口边界。  
审查方式：静态代码、参数、launch 图和离线构建检查；**未连接串口/CAN/GPIO，也未启动或控制任何真实硬件。**

## 结论

迁移工作空间已经具备一条受限的单出口速度链，并已修复本次发现的若干明显软件故障点：命令超时、无激光数据放行、手动接管、节点退出停车、重复底盘启动和软件急停。离线编译通过。

但它**尚不允许直接在地面实车测试**。独立于 ROS、能切断电机使能/电源的物理急停尚未在代码或接线资料中得到证实；并且履带方向、实际轮距/半径、激光安装朝向、制动距离均未通过架空轮验证。上述项目未确认前，只能做断电或架空轮测试。

## 本次确认的运动指令链

```text
GUI 手动控制 ──> /cmd_vel_teleop ─┐
                                   ├─> velocity_safety_gate ─> /cmd_vel
Nav2 controller ─> /cmd_vel_nav ─> velocity_smoother ─> /cmd_vel_auto ─┘
巡检区域底盘原语 ─> /cmd_vel_nav ───────────────────────────────────────┘
                                                            ↓
                                              tracked_motor_driver (GPIO/PWM)
```

最终输出门 `velocity_safety_gate` 是唯一设计上的 `/cmd_vel` 发布者；底盘驱动只订阅 `/cmd_vel`。手动通道优先于自动通道，且 GUI 发送手动命令时会取消 Nav2 目标并请求中止巡检任务。两套硬件 bringup 之间新增了主机锁；底盘驱动和速度门也各自有进程锁，第二个实例保持不执行/不发布。

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
| 激光断流时仍可能放行旧的运动命令 | `inspection_robot_safety/velocity_safety_gate.py` | 默认要求 `/scan` 在 0.50 s 内更新；超时、未收到、安全状态超时、故障或外部停止均发布零速度。 |
| GUI 手动控制不能立即抢占自主导航 | `inspection_robot_gui/main_window.py`、速度门 | 手动通道优先；手动操作取消导航并中止任务，避免松开手动键后自主命令恢复。 |
| 电机驱动 watchdog 仅每 0.5 s 清目标且继续缓停 | `inspection_robot_hardware/tracked_motor_driver.py` | 0.05 s watchdog；命令超过 0.35 s、NaN/Inf、节点退出均立即将 PWM 和方向脚置零。 |
| 参数可把速度/PWM 配置到危险值 | 电机驱动、速度门 | 最终边界硬限制：0.18 m/s、0.55 rad/s、70% PWM、0.35 m/s²、0.70 rad/s²；非法/非有限参数回退安全值。 |
| 两个 launch 可能重复启动 GPIO 驱动/速度门 | `mapping.launch.py`、`navigation.launch.py`、电机驱动、速度门 | 共享 bringup 文件锁拒绝第二套 mapping/navigation；GPIO 与 `/cmd_vel` 输出另有独占锁。 |
| 任务管理器退出时未明确发送停车 | `mission_manager.py` | `destroy_node()` 先发布 `/cmd_vel_nav` 零速度并停止区域控制 timer。 |
| 监控节点退出后停止状态未锁存 | `safety_monitor.py` | 退出前发布锁存的 FAULT/`/safety_stop`；速度门的安全状态失联 watchdog 也会停车。 |
| 没有人工软件急停入口 | `safety_monitor.py`、GUI | 新增锁存服务 `/emergency_stop`（`std_srvs/SetBool true`），GUI 有“紧急停止（锁存）”按钮；只可经确认后的 `/reset_safety_monitor` 复位。 |

这些修复位于 `inspection_migrated_ws`；本目录既有实机代码未被改写，只有本审查报告新增。

## 阻止实车地面测试的严重问题

1. **物理急停尚未证实。** 软件 `/emergency_stop` 依赖 ROS、CPU、GPIO 和 H 桥仍可工作，不能替代硬件急停。必须确认红蘑菇急停可独立切断电机驱动器使能或动力电源，且断线/掉电为安全状态。未确认前禁止地面测试。
2. **履带极性和转向符号未经实测。** 当前约定符合 REP-103：`linear.x > 0` 前进，`angular.z > 0` 左转；计算结果为左履带较慢/反转、右履带较快/正转。L298N/TB6612 实际 IN 引脚、左右电机线序可能相反。必须架空轮以不超过 `0.02 m/s` 验证。
3. **几何与制动能力未经标定。** `wheel_radius=0.025 m`、`track_width=0.155 m`、`max_rpm=80` 为开环假设；没有编码器闭环或真实轮速反馈。若死区 PWM 导致突跳或停车距离过长，禁止地面测试。
4. **激光坐标安装未实测。** launch 声明 `base_link -> laser` yaw 为 `+3.1415926 rad`。若物理雷达朝向不同，障碍物会在错误方向，可能导致导航撞向障碍物。必须在 RViz 验证前方障碍显示在 `base_link` 正前方。
5. **不得将旧实机项目和迁移 workspace 混合运行。** 旧项目 UI 曾直接使用 `/cmd_vel`；与迁移链同时 source/launch 会破坏“单出口”假设。实车测试仅 source `inspection_migrated_ws/install/setup.bash`，并先清理旧节点。

## 架空轮测试前应完成/确认的问题

| 项目 | 要求 |
|---|---|
| 物理电气 | 断电时电机不转；急停独立切断驱动；确认保险丝、电源极性、H 桥散热和公共地。 |
| 软件版本 | 在树莓派上重新构建本次迁移 workspace；不得使用旧 `ros_project` 的底盘/GUI launch。 |
| 独占启动 | 只启动 mapping 或 navigation 之一，不能并行；确认 `/tmp/inspection_robot_bringup.lock` 生效。 |
| 速度图 | 检查 `/cmd_vel` publisher 唯一性；`/cmd_vel_auto` 和 `/cmd_vel_nav` 出现多个自动 publisher 时停止。 |
| TF 图 | `map -> odom` 仅由 AMCL（导航）发布；`odom -> base_link` 仅由 EKF 发布；`base_link -> laser` 与 `base_link -> imu_link` 仅由静态发布器发布。不要同时启动独立 RF2O/IMU 展示 launch。 |
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
- `safety_monitor` 的欠压检测依赖树莓派可用接口，且启动文件当前将 `fault_on_undervoltage_seen` 设为 false。部署前应确认真实欠压输入、阈值和策略；不要把树莓派 core voltage 当作电机电池电压。
- 碰撞/无进展监测目前依赖 Nav2 日志和任务中止；它是辅助防护，不能替代激光距离阈值、安全激光或硬件防撞。
- 默认雷达设备路径、IMU 串口和 `lidar_yaw` 都是硬件相关参数；在目标车上必须以 udev 固定路径和实测安装值替换/确认，不能凭同名设备推断。
- 本报告只说明离线软件审查结果，不构成对制动性能、EMC、电源完整性、机械强度或人身安全的验收。

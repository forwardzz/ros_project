import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import serial
import os
import yaml

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from robot_monitor_interfaces.msg import GasData, InspectionPoint
from robot_monitor_interfaces.srv import (
    LoadMap, Localize, StartNavigation, ConfirmInspectionPoints
)

WARNING_THRESHOLD = {
    "氢气浓度": {"1级": 50, "3级": 100},
    "CO浓度": {"1级": 20, "3级": 100},
    "VOC浓度": {"1级": 10, "3级": 30},
    "烟雾浓度": {"1级": 15, "3级": 40},
}

MLX90640_WIDTH = 32
MLX90640_HEIGHT = 24
UPDATE_INTERVAL_MS = 250

PWMA, AIN1, AIN2 = 18, 22, 27
PWMB, BIN1, BIN2 = 23, 25, 24
MOTOR_SPEED = 50

SEND_FRAME = bytes([0x01, 0x03, 0x00, 0x01, 0x00, 0x04, 0x15, 0xC9])


def try_import_gpio():
    try:
        import RPi.GPIO as GPIO
        return True, GPIO
    except ImportError:
        print("未检测到RPi.GPIO库，电机控制功能将禁用")
        return False, None


def try_init_mlx90640():
    try:
        import board
        import busio
        import adafruit_mlx90640

        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        mlx = adafruit_mlx90640.MLX90640(i2c)
        mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
        frame = np.zeros((MLX90640_HEIGHT * MLX90640_WIDTH,))
        print("✅ MLX90640硬件初始化成功，将仅读取真实热成像数据")
        return True, mlx, frame
    except (ImportError, RuntimeError) as e:
        print(f"MLX90640硬件初始化失败：{e}")
        print("无硬件支持，热成像区域将无法显示数据")
        return False, None, None


class ROS2Adapter:
    def __init__(self, args=None):
        rclpy.init(args=args)
        self.node = rclpy.create_node('ui_ros_adapter')
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        self.gas_sub = self.node.create_subscription(
            GasData, '/gas_data', self.gas_callback, 10
        )
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 1
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.manual_control_pub = self.node.create_publisher(
            Bool, '/manual_control_active', 10
        )
        self.cmd_vel_pub = self.node.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.load_map_client = self.node.create_client(LoadMap, '/load_map')
        self.localize_client = self.node.create_client(Localize, '/localize_robot')
        self.navigation_client = self.node.create_client(StartNavigation, '/start_navigation')
        self.confirm_points_client = self.node.create_client(ConfirmInspectionPoints, '/confirm_inspection_points')

        self.gas_data = {
            "氢气浓度": 0.0,
            "CO浓度": 0.0,
            "VOC浓度": 0.0,
            "烟雾浓度": 0.0,
        }
        self.robot_pos = {"x": 0.0, "y": 0.0}
        self.map_data = None
        self.is_localized = False
        self.services_ready = False
        
        self._wait_for_services()

    def _wait_for_services(self):
        self.node.get_logger().info('检查ROS服务状态...')
        
        max_wait = 10
        wait_count = 0
        
        while wait_count < max_wait:
            services_ready = (
                self.load_map_client.wait_for_service(timeout_sec=0.5) and
                self.localize_client.wait_for_service(timeout_sec=0.5) and
                self.confirm_points_client.wait_for_service(timeout_sec=0.5) and
                self.navigation_client.wait_for_service(timeout_sec=0.5)
            )
            
            if services_ready:
                self.services_ready = True
                self.node.get_logger().info('所有ROS服务已就绪')
                return
            
            wait_count += 1
            self.node.get_logger().warn(f'等待ROS服务... ({wait_count}/{max_wait})')
        
        self.node.get_logger().warn('部分ROS服务未就绪，UI将以受限模式运行')
        self.services_ready = False

    def gas_callback(self, msg):
        self.gas_data["氢气浓度"] = msg.hydrogen_concentration
        self.gas_data["CO浓度"] = msg.co_concentration
        self.gas_data["VOC浓度"] = msg.voc_concentration
        self.gas_data["烟雾浓度"] = msg.smoke_concentration

    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        self.robot_pos["x"] = msg.pose.pose.position.x
        self.robot_pos["y"] = msg.pose.pose.position.y

    def publish_manual_control(self, active):
        msg = Bool()
        msg.data = active
        self.manual_control_pub.publish(msg)

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def call_load_map(self, map_file_path):
        req = LoadMap.Request()
        req.map_file_path = map_file_path
        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def call_localize(self):
        req = Localize.Request()
        future = self.localize_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()
        if result and result.success:
            self.is_localized = True
        return result

    def call_confirm_points(self, points):
        req = ConfirmInspectionPoints.Request()
        req.points = points
        future = self.confirm_points_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def call_start_navigation(self, waypoints):
        req = StartNavigation.Request()
        req.waypoints = waypoints
        future = self.navigation_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()


class DataMonitorPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("机器人环境监测系统（ROS2）")
        self.root.geometry("1400x900")

        self.GPIO_AVAILABLE, self.GPIO = try_import_gpio()
        self.HARDWARE_AVAILABLE, self.mlx, self.frame = try_init_mlx90640()

        self.serial_port = "/dev/ttyUSB0"
        self.ser = None
        self.SERIAL_AVAILABLE = False
        self.init_serial()

        self.ros_adapter = ROS2Adapter()

        self.thermal_data = np.zeros((MLX90640_HEIGHT, MLX90640_WIDTH))
        self.min_temp = self.max_temp = self.avg_temp = 0.0

        self.gas_data = self.ros_adapter.gas_data.copy()
        self.robot_pos = self.ros_adapter.robot_pos.copy()
        self.map_data = None

        self.start_time = time.time()
        self.prev_avg_temp = None
        self.temp_change_per_min = 0.0
        self.has_enough_data = False

        self.L_Motor = None
        self.R_Motor = None
        if self.GPIO_AVAILABLE:
            self.init_gpio()

        self.cbar = None
        self.inspection_points = []
        self.is_navigating = False
        self.is_mapping = False

        self._create_ui()
        self.update_data()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def init_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
            )
            self.SERIAL_AVAILABLE = True
            print(f"✅ 串口/{self.serial_port} 初始化成功，等待读取气体数据...")
        except Exception as e:
            self.SERIAL_AVAILABLE = False
            print(f"❌ 串口/{self.serial_port} 初始化失败：{e}")

    def read_gas_data_from_serial(self):
        if not (self.SERIAL_AVAILABLE and self.ser and self.ser.is_open):
            return self.gas_data

        try:
            self.ser.flushInput()
            self.ser.write(SEND_FRAME)
            time.sleep(0.1)
            response = self.ser.read(13)

            if len(response) != 13:
                return self.gas_data

            co = (response[3] << 8) + response[4]
            h2 = (response[5] << 8) + response[6]
            voc = (response[7] << 8) + response[8]
            smoke = 40 if (response[10] & 0x08) else 0.0

            return {"氢气浓度": h2, "CO浓度": co, "VOC浓度": voc, "烟雾浓度": smoke}
        except Exception as e:
            print(f"❌ 串口读取失败：{e}，保持当前气体数据")
            return self.gas_data

    def init_gpio(self):
        G = self.GPIO
        G.setwarnings(False)
        G.setmode(G.BCM)
        for pin in [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB]:
            G.setup(pin, G.OUT)
        self.L_Motor = G.PWM(PWMA, 100)
        self.R_Motor = G.PWM(PWMB, 100)
        self.L_Motor.start(0)
        self.R_Motor.start(0)

    def _drive(self, left_forward, right_forward, speed=MOTOR_SPEED):
        if not self.GPIO_AVAILABLE:
            action = {
                (True, True): "前进",
                (False, False): "后退",
                (False, True): "左转",
                (True, False): "右转",
            }.get((left_forward, right_forward), "停止")
            print(f"模拟电机：{action}")
            self.ros_adapter.publish_cmd_vel(
                linear_x=0.5 if left_forward else -0.5,
                angular_z=0.0
            )
            return

        G = self.GPIO
        self.L_Motor.ChangeDutyCycle(speed)
        self.R_Motor.ChangeDutyCycle(speed)
        G.output(AIN1, left_forward)
        G.output(AIN2, not left_forward)
        G.output(BIN1, right_forward)
        G.output(BIN2, not right_forward)

    def motor_up(self):
        self.ros_adapter.publish_manual_control(True)
        self._drive(True, True)

    def motor_down(self):
        self.ros_adapter.publish_manual_control(True)
        self._drive(False, False)

    def motor_left(self):
        self.ros_adapter.publish_manual_control(True)
        self._drive(False, True)

    def motor_right(self):
        self.ros_adapter.publish_manual_control(True)
        self._drive(True, False)

    def motor_stop(self):
        self.ros_adapter.publish_manual_control(False)
        if not self.GPIO_AVAILABLE:
            print("模拟电机：停止")
            self.ros_adapter.publish_cmd_vel(0.0, 0.0)
            return
        G = self.GPIO
        self.L_Motor.ChangeDutyCycle(0)
        self.R_Motor.ChangeDutyCycle(0)
        for pin in [AIN1, AIN2, BIN1, BIN2]:
            G.output(pin, False)

    def _create_ui(self):
        thermal_frame = ttk.LabelFrame(self.root, text="MLX90640热成像监测")
        thermal_frame.place(x=20, y=20, width=550, height=400)
        self.thermal_fig, self.thermal_ax = plt.subplots(figsize=(6.5, 3.8), dpi=80)
        self._draw_thermal_image()
        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, thermal_frame)
        self.thermal_canvas.draw()
        self.thermal_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        radar_frame = ttk.LabelFrame(self.root, text="地图显示")
        radar_frame.place(x=20, y=440, width=550, height=300)
        self.radar_fig, self.radar_ax = plt.subplots(figsize=(6.5, 2.8), dpi=80)
        self._draw_radar_map(init=True)
        self.radar_canvas = FigureCanvasTkAgg(self.radar_fig, radar_frame)
        self.radar_canvas.draw()
        self.radar_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        map_ctrl_frame = ttk.Frame(radar_frame)
        map_ctrl_frame.pack(fill=tk.X, pady=5)
        ttk.Button(map_ctrl_frame, text="导入地图", command=self.load_map).pack(side=tk.LEFT, padx=5)
        ttk.Button(map_ctrl_frame, text="刷新地图", command=self.update_radar_map).pack(side=tk.LEFT, padx=5)

        gas_frame = ttk.LabelFrame(self.root, text="气体浓度预警（串口数据）")
        gas_frame.place(x=600, y=20, width=350, height=400)
        self.indicator_widgets = {}
        for idx, gas_name in enumerate(self.gas_data):
            ttk.Label(gas_frame, text=gas_name).grid(row=idx, column=0, padx=10, pady=15, sticky="w")
            value_label = ttk.Label(gas_frame, text="0.00")
            value_label.grid(row=idx, column=1, padx=10, pady=15)
            indicator = tk.Canvas(gas_frame, width=30, height=30, bg="white")
            indicator.grid(row=idx, column=2, padx=10, pady=15)
            self.indicator_widgets[gas_name] = (value_label, indicator)

        pos_frame = ttk.LabelFrame(self.root, text="机器人位置信息")
        pos_frame.place(x=600, y=440, width=350, height=120)
        ttk.Label(pos_frame, text="当前位置(X,Y):", font=("Arial", 12)).grid(row=0, column=0, padx=20, pady=20)
        self.pos_label = ttk.Label(pos_frame, text="(0.00, 0.00)", font=("Arial", 14))
        self.pos_label.grid(row=0, column=1, padx=10, pady=20)
        ttk.Button(pos_frame, text="刷新位置", command=self.update_robot_pos).grid(row=0, column=2, padx=10, pady=20)

        insp_frame = ttk.LabelFrame(self.root, text="巡检点管理")
        insp_frame.place(x=600, y=580, width=350, height=200)

        insp_list_frame = ttk.Frame(insp_frame)
        insp_list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.insp_listbox = tk.Listbox(insp_list_frame, height=5)
        self.insp_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        insp_scrollbar = ttk.Scrollbar(insp_list_frame, orient=tk.VERTICAL, command=self.insp_listbox.yview)
        insp_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.insp_listbox.config(yscrollcommand=insp_scrollbar.set)

        insp_btn_frame = ttk.Frame(insp_frame)
        insp_btn_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(insp_btn_frame, text="添加当前点", command=self.add_inspection_point).pack(side=tk.LEFT, padx=5)
        ttk.Button(insp_btn_frame, text="删除选中", command=self.remove_inspection_point).pack(side=tk.LEFT, padx=5)

        nav_frame = ttk.LabelFrame(self.root, text="导航控制")
        nav_frame.place(x=980, y=20, width=400, height=300)

        ttk.Button(nav_frame, text="1. 机器人定位", command=self.localize_robot).pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(nav_frame, text="2. 确认巡检点", command=self.confirm_inspection_points).pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(nav_frame, text="3. 开始导航", command=self.start_navigation).pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(nav_frame, text="停止导航", command=self.stop_navigation).pack(fill=tk.X, padx=10, pady=5)

        self.nav_status_label = ttk.Label(nav_frame, text="状态: 未定位", foreground="red")
        self.nav_status_label.pack(pady=10)

        move_frame = ttk.LabelFrame(self.root, text="机器人运动控制（手动优先）")
        move_frame.place(x=980, y=340, width=400, height=200)
        style = ttk.Style()
        style.configure("Move.TButton", font=("Arial", 12), padding=8)
        style.configure("Stop.TButton", font=("Arial", 12), padding=8, foreground="red")

        ttk.Button(move_frame, text="↑ 前进", style="Move.TButton", command=self.motor_up).grid(row=0, column=1, padx=15, pady=10)
        ttk.Button(move_frame, text="← 左转", style="Move.TButton", command=self.motor_left).grid(row=1, column=0, padx=15, pady=10)
        ttk.Button(move_frame, text="■ 停止", style="Stop.TButton", command=self.motor_stop).grid(row=1, column=1, padx=15, pady=10)
        ttk.Button(move_frame, text="右转 →", style="Move.TButton", command=self.motor_right).grid(row=1, column=2, padx=15, pady=10)
        ttk.Button(move_frame, text="↓ 后退", style="Move.TButton", command=self.motor_down).grid(row=2, column=1, padx=15, pady=10)

        for key, fn in [("<Up>", self.motor_up), ("<Down>", self.motor_down),
                        ("<Left>", self.motor_left), ("<Right>", self.motor_right),
                        ("<space>", self.motor_stop)]:
            self.root.bind(key, lambda _, f=fn: f())

    def load_map(self):
        if not self.ros_adapter.services_ready:
            messagebox.showwarning("警告", "ROS服务未就绪，请先启动建图节点:\nros2 launch robot_monitor_mapping mapping_launch.py")
            return
            
        map_file = filedialog.askopenfilename(
            title="选择地图文件",
            filetypes=[("YAML files", "*.yaml"), ("PGM files", "*.pgm"), ("All files", "*.*")]
        )
        if map_file:
            result = self.ros_adapter.call_load_map(map_file)
            if result and result.success:
                messagebox.showinfo("成功", f"地图加载成功: {result.message}")
                self.update_radar_map()
            else:
                messagebox.showerror("失败", f"地图加载失败: {result.message if result else '未知错误'}")

    def update_radar_map(self):
        if self.ros_adapter.map_data:
            self._draw_radar_map_with_map()
        else:
            self._draw_radar_map()
        self.radar_canvas.draw()

    def _draw_radar_map(self, init=False):
        ax = self.radar_ax
        ax.clear()
        if init:
            ax.text(0.5, 0.5, "导入地图后显示",
                    ha='center', va='center', transform=ax.transAxes, fontsize=10)
            ax.set_xlim(0, 10)
            ax.set_ylim(0, 10)
        else:
            ax.scatter(self.robot_pos['x'], self.robot_pos['y'], c='blue', s=50, marker='*', label='机器人位置')

        for point in self.inspection_points:
            color = 'green' if point.is_confirmed else 'yellow'
            ax.scatter(point.x, point.y, c=color, s=30, marker='o', label=point.point_name)

        ax.set_xlabel("X (m)", fontsize=9)
        ax.set_ylabel("Y (m)", fontsize=9)
        ax.set_title("地图显示", fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    def _draw_radar_map_with_map(self):
        ax = self.radar_ax
        ax.clear()

        map_msg = self.ros_adapter.map_data
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        map_array = np.array(map_msg.data).reshape((height, width))

        x_coords = np.linspace(origin_x, origin_x + width * resolution, width)
        y_coords = np.linspace(origin_y, origin_y + height * resolution, height)
        X, Y = np.meshgrid(x_coords, y_coords)

        ax.contourf(X, Y, map_array, levels=[-1, 0, 100], colors=['white', 'gray', 'black'], alpha=0.5)

        ax.scatter(self.robot_pos['x'], self.robot_pos['y'], c='blue', s=100, marker='*', label='机器人')

        for point in self.inspection_points:
            color = 'green' if point.is_confirmed else 'yellow'
            ax.scatter(point.x, point.y, c=color, s=50, marker='o')
            ax.annotate(point.point_name, (point.x, point.y), fontsize=8)

        ax.set_xlabel("X (m)", fontsize=9)
        ax.set_ylabel("Y (m)", fontsize=9)
        ax.set_title("地图显示", fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    def _update_indicator(self, gas_name, value):
        value_label, indicator = self.indicator_widgets[gas_name]
        value_label.config(text=f"{value:.2f}")

        t1 = WARNING_THRESHOLD[gas_name]["1级"]
        t3 = WARNING_THRESHOLD[gas_name]["3级"]
        color = "green" if value < t1 else ("yellow" if value < t3 else "red")

        indicator.delete("all")
        indicator.create_oval(5, 5, 25, 25, fill=color, outline="black")

    def _read_mlx90640_data(self):
        if not self.HARDWARE_AVAILABLE:
            return np.zeros((MLX90640_HEIGHT, MLX90640_WIDTH))

        try:
            self.mlx.getFrame(self.frame)
            matrix = np.reshape(self.frame, (MLX90640_HEIGHT, MLX90640_WIDTH))
            self.min_temp = round(float(np.min(matrix)), 1)
            self.max_temp = round(float(np.max(matrix)), 1)
            self.avg_temp = round(float(np.mean(matrix)), 1)

            now = time.time()
            if self.prev_avg_temp is None:
                self.prev_avg_temp = self.avg_temp
            if now - self.start_time >= 60:
                self.temp_change_per_min = self.avg_temp - self.prev_avg_temp
                self.prev_avg_temp = self.avg_temp
                self.has_enough_data = True
                self.start_time = now

            return matrix
        except RuntimeError as e:
            print(f"❌ 读取MLX90640硬件数据失败：{e}")
            return np.zeros((MLX90640_HEIGHT, MLX90640_WIDTH))

    def _draw_thermal_image(self):
        try:
            if self.cbar:
                try:
                    self.cbar.remove()
                except Exception:
                    pass
                self.cbar = None

            ax = self.thermal_ax
            ax.clear()

            if not self.HARDWARE_AVAILABLE:
                ax.text(0.5, 0.5, "MLX90640硬件未连接\n无法读取热成像数据",
                        ha='center', va='center', transform=ax.transAxes, fontsize=12, color='red')
                ax.set_xlim(0, 1)
                ax.set_ylim(0, 1)
                return

            thermal_im = ax.imshow(
                self.thermal_data,
                cmap='inferno',
                vmin=self.min_temp if self.min_temp > 0 else 20,
                vmax=self.max_temp if self.max_temp > 0 else 75
            )
            self.cbar = self.thermal_fig.colorbar(thermal_im, ax=ax, shrink=0.8)
            self.cbar.set_label('温度 (℃)', fontsize=10)

            info = [
                (0.95, f"Min: {self.min_temp}°C", 'cyan'),
                (0.90, f"Max: {self.max_temp}°C", 'red'),
                (0.85, f"Avg: {self.avg_temp}°C", 'lime'),
            ]
            for y, txt, color in info:
                ax.text(0.02, y, txt, transform=ax.transAxes, color=color, fontsize=9,
                        bbox=dict(facecolor='black', alpha=0.5))

            if self.has_enough_data:
                change = self.temp_change_per_min
                c = 'red' if change > 0 else ('cyan' if change < 0 else 'white')
                txt = f"Temp Change/Min: {change:.1f}°C"
            else:
                c, txt = 'white', "Temp Change/Min: 数据收集中..."
            ax.text(0.02, 0.80, txt, transform=ax.transAxes, color=c, fontsize=9,
                    bbox=dict(facecolor='black', alpha=0.5))

            ax.set_xlabel("像素列 (32)", fontsize=9)
            ax.set_ylabel("像素行 (24)", fontsize=9)
            ax.set_title("MLX90640红外热成像", fontsize=11)
            ax.set_xticks([])
            ax.set_yticks([])
        except Exception as e:
            print(f"❌ 绘制热成像图出错: {e}")
            self.cbar = None
            self.thermal_ax.clear()

    def update_data(self):
        try:
            self.thermal_data = self._read_mlx90640_data()
            self._draw_thermal_image()
            self.thermal_canvas.draw()

            self.gas_data = self.ros_adapter.gas_data.copy()
            serial_gas = self.read_gas_data_from_serial()
            self.gas_data.update(serial_gas)

            for gas_name, val in self.gas_data.items():
                self._update_indicator(gas_name, val)

            self.robot_pos = self.ros_adapter.robot_pos.copy()
            self.pos_label.config(text=f"({self.robot_pos['x']:.2f}, {self.robot_pos['y']:.2f})")

            if self.ros_adapter.is_localized:
                self.nav_status_label.config(text="状态: 已定位", foreground="green")
            elif not self.ros_adapter.services_ready:
                self.nav_status_label.config(text="状态: 服务未就绪", foreground="orange")
            else:
                self.nav_status_label.config(text="状态: 未定位", foreground="red")

        except Exception as e:
            print(f"❌ 数据更新出错: {e}")
        finally:
            self.root.after(UPDATE_INTERVAL_MS, self.update_data)

    def update_robot_pos(self):
        self.robot_pos = self.ros_adapter.robot_pos.copy()
        self.pos_label.config(text=f"({self.robot_pos['x']:.2f}, {self.robot_pos['y']:.2f})")

    def add_inspection_point(self):
        name = f"Point_{len(self.inspection_points) + 1}"
        point = InspectionPoint()
        point.point_name = name
        point.x = self.robot_pos['x']
        point.y = self.robot_pos['y']
        point.theta = 0.0
        point.is_confirmed = False

        self.inspection_points.append(point)
        self.insp_listbox.insert(tk.END, f"{name}: ({point.x:.2f}, {point.y:.2f})")
        self.update_radar_map()

    def remove_inspection_point(self):
        selection = self.insp_listbox.curselection()
        if selection:
            idx = selection[0]
            self.inspection_points.pop(idx)
            self.insp_listbox.delete(idx)
            self.update_radar_map()

    def localize_robot(self):
        if not self.ros_adapter.services_ready:
            messagebox.showwarning("警告", "ROS服务未就绪，请先启动导航节点:\nros2 launch robot_monitor_navigation navigation_launch.py")
            return
            
        self.nav_status_label.config(text="状态: 定位中...", foreground="orange")
        self.root.update()

        result = self.ros_adapter.call_localize()
        if result and result.success:
            self.nav_status_label.config(text="状态: 已定位", foreground="green")
            messagebox.showinfo("成功", f"定位成功\n位置: ({result.current_x:.2f}, {result.current_y:.2f})")
        else:
            self.nav_status_label.config(text="状态: 定位失败", foreground="red")
            messagebox.showerror("失败", f"定位失败: {result.message if result else '未知错误'}")

    def confirm_inspection_points(self):
        if not self.ros_adapter.services_ready:
            messagebox.showwarning("警告", "ROS服务未就绪，请先启动导航节点:\nros2 launch robot_monitor_navigation navigation_launch.py")
            return
            
        if not self.inspection_points:
            messagebox.showwarning("警告", "没有巡检点可确认")
            return

        result = self.ros_adapter.call_confirm_points(self.inspection_points)
        if result and result.success:
            for point in self.inspection_points:
                point.is_confirmed = True
            self.update_radar_map()
            messagebox.showinfo("成功", f"已确认 {len(self.inspection_points)} 个巡检点")
        else:
            messagebox.showerror("失败", f"确认失败: {result.message if result else '未知错误'}")

    def start_navigation(self):
        if not self.ros_adapter.services_ready:
            messagebox.showwarning("警告", "ROS服务未就绪，请先启动导航节点:\nros2 launch robot_monitor_navigation navigation_launch.py")
            return
            
        if not self.ros_adapter.is_localized:
            messagebox.showwarning("警告", "请先完成机器人定位")
            return

        confirmed_points = [p for p in self.inspection_points if p.is_confirmed]
        if not confirmed_points:
            messagebox.showwarning("警告", "没有已确认的巡检点")
            return

        result = self.ros_adapter.call_start_navigation(confirmed_points)
        if result and result.success:
            self.is_navigating = True
            self.nav_status_label.config(text="状态: 导航中...", foreground="blue")
            messagebox.showinfo("成功", "导航已启动")
        else:
            messagebox.showerror("失败", f"启动导航失败: {result.message if result else '未知错误'}")

    def stop_navigation(self):
        self.is_navigating = False
        self.ros_adapter.publish_manual_control(False)
        self.ros_adapter.publish_cmd_vel(0.0, 0.0)
        self.nav_status_label.config(text="状态: 已停止", foreground="red")
        self.motor_stop()

    def on_closing(self):
        try:
            self.motor_stop()
            if self.GPIO_AVAILABLE:
                self.GPIO.cleanup()
            if self.SERIAL_AVAILABLE and self.ser and self.ser.is_open:
                self.ser.close()
            self.ros_adapter.shutdown()
        except Exception as e:
            print(f"⚠️ 清理资源警告: {e}")
        finally:
            self.root.destroy()


def main(args=None):
    try:
        root = tk.Tk()
        app = DataMonitorPanel(root)
        root.mainloop()
    except Exception as e:
        print(f"❌ 程序启动异常：{e}")
        try:
            if 'app' in locals():
                app.on_closing()
        except Exception:
            pass


if __name__ == "__main__":
    main()

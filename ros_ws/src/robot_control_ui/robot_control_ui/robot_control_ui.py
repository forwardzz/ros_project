import math
import os
import queue
import shlex
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import matplotlib
import rclpy
from geometry_msgs.msg import Twist
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

from robot_mission_utils.inspection_planner import plan_mission_order, preview_current_order as preview_order_route
from robot_mission_utils.tsp_planner import TSPPlanner
from robot_monitor_interfaces.msg import GasData, InspectionPoint, RobotSafetyStatus
from robot_monitor_interfaces.srv import ConfirmInspectionPoints, Localize, StartNavigation

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


class RosUiAdapter:
    def __init__(self, args=None):
        rclpy.init(args=args)
        self.node = rclpy.create_node("robot_control_ui")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos
        )
        self.thermal_sub = self.node.create_subscription(
            Float32MultiArray, "/thermal_frame", self._thermal_cb, 10
        )
        self.gas_sub = self.node.create_subscription(
            GasData, "/gas_data", self._gas_cb, 10
        )
        self.safety_sub = self.node.create_subscription(
            RobotSafetyStatus, "/robot_safety_status", self._safety_cb, map_qos
        )
        self.localize_client = self.node.create_client(Localize, "/localize_robot")
        self.confirm_points_client = self.node.create_client(
            ConfirmInspectionPoints, "/confirm_inspection_points"
        )
        self.start_navigation_client = self.node.create_client(
            StartNavigation, "/start_navigation"
        )
        self.clear_rviz_points_client = self.node.create_client(
            Trigger, "/clear_rviz_points"
        )
        self.reset_safety_client = self.node.create_client(
            Trigger, "/reset_safety_monitor"
        )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_yaw_rate = 0.0
        self.scan_count = 0
        self.map_data = None
        self.last_scan_stamp = 0.0
        self.last_odom_stamp = 0.0
        self.last_map_stamp = 0.0
        self.last_thermal_stamp = 0.0
        self.last_gas_stamp = 0.0
        self.thermal_width = 32
        self.thermal_height = 24
        self.thermal_frame = []
        self.thermal_min = 0.0
        self.thermal_max = 0.0
        self.thermal_avg = 0.0
        self.thermal_change_per_min = 0.0
        self.thermal_change_ready = False
        self.thermal_baseline_avg = None
        self.thermal_baseline_time = 0.0
        self.gas_data = {
            "H2": 0.0,
            "CO": 0.0,
            "VOC": 0.0,
            "Smoke": 0.0,
        }
        self.safety_level = 'WAITING'
        self.safety_code = 'INIT'
        self.safety_message = 'No safety data'
        self.safety_mission_active = False
        self.last_safety_stamp = 0.0
        self.pending_safety_alert = None
        self.last_safety_alert_signature = None

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_yaw_rate = msg.twist.twist.angular.z
        self.last_odom_stamp = time.time()

    def _scan_cb(self, msg):
        self.scan_count = len(msg.ranges)
        self.last_scan_stamp = time.time()

    def _map_cb(self, msg):
        self.map_data = msg
        self.last_map_stamp = time.time()

    def _gas_cb(self, msg):
        self.gas_data["H2"] = float(msg.hydrogen_concentration)
        self.gas_data["CO"] = float(msg.co_concentration)
        self.gas_data["VOC"] = float(msg.voc_concentration)
        self.gas_data["Smoke"] = float(msg.smoke_concentration)
        self.last_gas_stamp = time.time()

    def _thermal_cb(self, msg):
        dims = msg.layout.dim
        if len(dims) >= 2 and dims[0].size > 0 and dims[1].size > 0:
            self.thermal_height = int(dims[0].size)
            self.thermal_width = int(dims[1].size)

        if not msg.data:
            return

        self.thermal_frame = list(msg.data)
        self.thermal_min = min(self.thermal_frame)
        self.thermal_max = max(self.thermal_frame)
        self.thermal_avg = sum(self.thermal_frame) / len(self.thermal_frame)
        now = time.time()
        if self.thermal_baseline_avg is None:
            self.thermal_baseline_avg = self.thermal_avg
            self.thermal_baseline_time = now
        elif now - self.thermal_baseline_time >= 60.0:
            self.thermal_change_per_min = self.thermal_avg - self.thermal_baseline_avg
            self.thermal_change_ready = True
            self.thermal_baseline_avg = self.thermal_avg
            self.thermal_baseline_time = now
        self.last_thermal_stamp = time.time()

    def _safety_cb(self, msg):
        self.safety_level = msg.level or 'WAITING'
        self.safety_code = msg.code or 'UNKNOWN'
        self.safety_message = msg.message or 'No details'
        self.safety_mission_active = bool(msg.mission_active)
        self.last_safety_stamp = time.time()
        signature = (self.safety_level, self.safety_code, self.safety_message)
        if self.safety_level in ('WARN', 'FAULT') and signature != self.last_safety_alert_signature:
            self.pending_safety_alert = signature
            self.last_safety_alert_signature = signature

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def call_service_async(self, client, request, done_callback, timeout_sec=6.0):
        def worker():
            if not client.wait_for_service(timeout_sec=timeout_sec):
                done_callback(None, f"Service {client.srv_name} is unavailable")
                return

            future = client.call_async(request)
            deadline = time.time() + timeout_sec
            while rclpy.ok() and not future.done() and time.time() < deadline:
                time.sleep(0.05)

            if not future.done():
                done_callback(None, f"Service {client.srv_name} timed out")
                return

            try:
                result = future.result()
            except Exception as exc:
                done_callback(None, str(exc))
                return
            done_callback(result, None)

        threading.Thread(target=worker, daemon=True).start()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()


class LaunchManager:
    def __init__(self, workspace_path, remote_user, remote_host, ros_setup_path, log_callback):
        self.workspace_path = workspace_path
        self.remote_user = remote_user
        self.remote_host = remote_host
        self.ros_setup_path = ros_setup_path
        self.log_callback = log_callback
        self.active_process = None
        self.thermal_process = None
        self.active_name = "idle"
        self.last_exit_code = None
        self.cyclone_uri = (
            "<CycloneDDS xmlns='https://cdds.io/config'>"
            "<Domain Id='any'>"
            "<Discovery>"
            "<ParticipantIndex>none</ParticipantIndex>"
            "<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>"
            "</Discovery>"
            "</Domain>"
            "</CycloneDDS>"
        )

    def _run_remote_cleanup(self, patterns):
        if not patterns:
            return
        cleanup_steps = [f"pkill -TERM -f {pattern} || true" for pattern in patterns]
        cleanup_steps.append("sleep 1")
        cleanup = " ; ".join(cleanup_steps)
        proc = subprocess.Popen(
            [
                "ssh",
                f"{self.remote_user}@{self.remote_host}",
                f"bash -lc {shlex.quote(cleanup)}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            stdout, _ = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, _ = proc.communicate()
            self.log_callback("[WARN] remote cleanup timed out")
        if stdout:
            self.log_callback(stdout.rstrip())

    def _stop_patterns_for(self, name):
        common = [
            "ros2",
            "mapping.launch.py",
            "navigation.launch.py",
            "sllidar_node",
            "rf2o_laser_odometry",
            "static_transform_publisher",
            "tracked_motor_driver",
            "gas_sensor_node",
        ]
        if name == "mapping":
            return [
                "ros2",
                "mapping.launch.py",
                "slam_toolbox",
                "thermal_camera_node",
                "gas_sensor_node",
                *common,
            ]
        if name == "navigation":
            return [
                "ros2",
                "navigation.launch.py",
                "nav2_amcl",
                "planner_server",
                "controller_server",
                "bt_navigator",
                "behavior_server",
                "smoother_server",
                "velocity_smoother",
                "map_server",
                "lifecycle_manager_navigation",
                "thermal_camera_node",
                "gas_sensor_node",
                *common,
            ]
        return [
            "ros2",
            "mapping.launch.py",
            "navigation.launch.py",
            "slam_toolbox",
            "nav2_amcl",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "behavior_server",
            "smoother_server",
            "velocity_smoother",
            "map_server",
            "lifecycle_manager_navigation",
            "sllidar_node",
            "rf2o_laser_odometry",
            "tracked_motor_driver",
            "static_transform_publisher",
            "thermal_camera_node",
            "gas_sensor_node",
        ]

    def _build_remote_command(self, command):
        setup_path = os.path.join(self.workspace_path, "install", "setup.bash")
        return (
            f"export CYCLONEDDS_URI={shlex.quote(self.cyclone_uri)} && "
            f"source {self.ros_setup_path} && "
            f"source {setup_path} && "
            f"cd {self.workspace_path} && "
            f"{command}"
        )

    def _build_ssh_invocation(self, command):
        remote_command = self._build_remote_command(command)
        return [
            "ssh",
            f"{self.remote_user}@{self.remote_host}",
            f"bash -lc {shlex.quote(remote_command)}",
        ]

    def start(self, name, command):
        if self.active_process and self.active_process.poll() is None:
            self.log_callback(f"[WARN] Stop current task before starting {name}.")
            return False

        self.active_process = subprocess.Popen(
            self._build_ssh_invocation(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        self.active_name = name
        self.log_callback(f"[RUN] {name}: {command}")
        threading.Thread(target=self._stream_output, args=(self.active_process, name), daemon=True).start()
        return True

    def run_once(self, name, command):
        def worker():
            self.log_callback(f"[RUN] {name}: {command}")
            proc = subprocess.Popen(
                self._build_ssh_invocation(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )
            self._stream_output(proc, name, track_active=False)

        threading.Thread(target=worker, daemon=True).start()

    def stop(self):
        target_name = self.active_name
        if not self.active_process or self.active_process.poll() is not None:
            patterns = self._stop_patterns_for(target_name)
            self._run_remote_cleanup(patterns)
            self.log_callback(f"[STOP] remote cleanup for {target_name or 'all'}")
            self.active_name = "idle"
            return

        try:
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)
            self.log_callback(f"[STOP] {self.active_name}")
        except ProcessLookupError:
            pass
        finally:
            self._run_remote_cleanup(self._stop_patterns_for(target_name))
            self.active_process = None
            self.active_name = "idle"

    def is_running(self):
        return bool(self.active_process and self.active_process.poll() is None)

    def start_thermal(self):
        if self.thermal_process and self.thermal_process.poll() is None:
            self.log_callback("[WARN] Thermal node is already running.")
            return False

        self._run_remote_cleanup(["thermal_camera_node", "gas_sensor_node", "sensor_monitor.launch.py"])
        command = "ros2 launch mapping_bringup sensor_monitor.launch.py"
        self.thermal_process = subprocess.Popen(
            self._build_ssh_invocation(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        self.log_callback(f"[RUN] thermal: {command}")
        threading.Thread(
            target=self._stream_output,
            args=(self.thermal_process, "thermal"),
            kwargs={"track_active": False, "track_thermal": True},
            daemon=True,
        ).start()
        return True

    def stop_thermal(self):
        try:
            if self.thermal_process and self.thermal_process.poll() is None:
                os.killpg(os.getpgid(self.thermal_process.pid), signal.SIGTERM)
                self.log_callback("[STOP] thermal")
        except ProcessLookupError:
            pass
        finally:
            self._run_remote_cleanup(["thermal_camera_node", "gas_sensor_node", "sensor_monitor.launch.py"])
            self.thermal_process = None

    def is_thermal_running(self):
        return bool(self.thermal_process and self.thermal_process.poll() is None)

    def _stream_output(self, proc, name, track_active=True, track_thermal=False):
        if proc.stdout is not None:
            for line in proc.stdout:
                self.log_callback(line.rstrip())
        code = proc.wait()
        self.last_exit_code = code
        self.log_callback(f"[EXIT] {name} -> {code}")
        if track_active and self.active_process is proc:
            self.active_process = None
            self.active_name = "idle"
        if track_thermal and self.thermal_process is proc:
            self.thermal_process = None


class RobotControlApp:
    def __init__(self, root, ros_adapter):
        self.root = root
        self.ros = ros_adapter
        self.node = ros_adapter.node

        self.workspace_path = self.node.declare_parameter(
            "workspace_path", "/home/yy/ros2_ws"
        ).value
        self.remote_user = self.node.declare_parameter(
            "remote_user", "yy"
        ).value
        self.remote_host = self.node.declare_parameter(
            "remote_host", "192.168.43.21"
        ).value
        self.ros_setup_path = self.node.declare_parameter(
            "ros_setup_path", "/opt/ros/jazzy/setup.bash"
        ).value
        self.default_map_path = self.node.declare_parameter(
            "map_path", "/home/yy/ros2_ws/map_name.yaml"
        ).value

        self.log_queue = queue.Queue()
        self.launch_manager = LaunchManager(
            self.workspace_path,
            self.remote_user,
            self.remote_host,
            self.ros_setup_path,
            self.log_queue.put,
        )
        self.manual_linear = 0.12
        self.manual_angular = 0.8
        self.pose_history = []
        self.route_preview = []
        self.mission_points = []
        self.point_counter = 1
        self.tsp_planner = TSPPlanner()
        self.last_map_render_key = None
        self.thermal_window = None
        self.thermal_popup_fig = None
        self.thermal_popup_ax = None
        self.thermal_popup_canvas = None
        self.thermal_popup_cbar = None

        self.root.title("Tracked Robot Control UI")
        self.root.geometry("1280x760")
        self.root.configure(bg="#ebe6dc")

        self.map_var = tk.StringVar(value=self.default_map_path)
        self.workspace_var = tk.StringVar(value=self.workspace_path)
        self.remote_user_var = tk.StringVar(value=self.remote_user)
        self.remote_host_var = tk.StringVar(value=self.remote_host)
        self.status_var = tk.StringVar(value="Idle")
        self.pose_var = tk.StringVar(value="x=0.00  y=0.00  yaw=0.0")
        self.scan_var = tk.StringVar(value="scan: no data")
        self.odom_var = tk.StringVar(value="odom: no data")
        self.map_var_status = tk.StringVar(value="map: no data")
        self.mission_var = tk.StringVar(value="mission: 0 points")
        self.safety_var = tk.StringVar(value="safety: waiting")
        self.thermal_var = tk.StringVar(value="thermal: no data")
        self.gas_var = tk.StringVar(value="gas: no data")
        self.gas_h2_var = tk.StringVar(value="H2: --")
        self.gas_co_var = tk.StringVar(value="CO: --")
        self.gas_voc_var = tk.StringVar(value="VOC: --")
        self.gas_smoke_var = tk.StringVar(value="Smoke: --")
        self.ssh_var = tk.StringVar(value=f"{self.remote_user}@{self.remote_host}")
        self.indicators = {}

        self._build_ui()
        self._bind_keys()
        self._schedule_update()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Card.TLabelframe", background="#ebe6dc")
        style.configure("Card.TLabelframe.Label", font=("Helvetica", 12, "bold"))
        style.configure("Primary.TButton", font=("Helvetica", 11, "bold"), padding=10)
        style.configure("Danger.TButton", font=("Helvetica", 11, "bold"), padding=10)
        style.configure("Drive.TButton", font=("Helvetica", 13, "bold"), padding=14)

        header = tk.Frame(self.root, bg="#153243", height=72)
        header.pack(fill=tk.X)
        tk.Label(
            header,
            text="Tracked Robot Control Center",
            bg="#153243",
            fg="#f4efe7",
            font=("Helvetica", 22, "bold"),
        ).pack(side=tk.LEFT, padx=24, pady=18)
        tk.Label(
            header,
            textvariable=self.status_var,
            bg="#153243",
            fg="#ffc857",
            font=("Helvetica", 14, "bold"),
        ).pack(side=tk.RIGHT, padx=24)

        body = tk.Frame(self.root, bg="#ebe6dc")
        body.pack(fill=tk.BOTH, expand=True, padx=18, pady=18)

        left = tk.Frame(body, bg="#ebe6dc", width=720)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        left.pack_propagate(False)
        right = tk.Frame(body, bg="#ebe6dc", width=500)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(18, 0))
        right.pack_propagate(False)

        self._build_launch_panel(left)
        self._build_mission_panel(left)
        self._build_log_panel(left)
        self._build_map_panel(right)
        self._build_thermal_panel(right)
        self._build_status_panel(right)
        self._build_drive_panel(right)

    def _build_launch_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Mission Control", style="Card.TLabelframe")
        frame.pack(fill=tk.X, pady=(0, 18))

        row1 = tk.Frame(frame, bg="#ebe6dc")
        row1.pack(fill=tk.X, padx=14, pady=(14, 8))
        tk.Label(row1, text="Workspace", bg="#ebe6dc", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT)
        tk.Entry(row1, textvariable=self.workspace_var, font=("Helvetica", 10)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(12, 0)
        )

        row2 = tk.Frame(frame, bg="#ebe6dc")
        row2.pack(fill=tk.X, padx=14, pady=8)
        tk.Label(row2, text="SSH", bg="#ebe6dc", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT)
        tk.Entry(row2, textvariable=self.remote_user_var, width=10, font=("Helvetica", 10)).pack(
            side=tk.LEFT, padx=(35, 8)
        )
        tk.Label(row2, text="@", bg="#ebe6dc", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT)
        tk.Entry(row2, textvariable=self.remote_host_var, font=("Helvetica", 10)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 0)
        )

        row3 = tk.Frame(frame, bg="#ebe6dc")
        row3.pack(fill=tk.X, padx=14, pady=8)
        tk.Label(row3, text="Map YAML", bg="#ebe6dc", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT)
        tk.Entry(row3, textvariable=self.map_var, font=("Helvetica", 10)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(18, 0)
        )

        buttons = tk.Frame(frame, bg="#ebe6dc")
        buttons.pack(fill=tk.X, padx=14, pady=(10, 16))
        ttk.Button(buttons, text="Start Mapping", style="Primary.TButton", command=self.start_mapping).pack(
            side=tk.LEFT, padx=(0, 10)
        )
        ttk.Button(buttons, text="Start Navigation", style="Primary.TButton", command=self.start_navigation).pack(
            side=tk.LEFT, padx=10
        )
        ttk.Button(buttons, text="Save Map", style="Primary.TButton", command=self.save_map).pack(
            side=tk.LEFT, padx=10
        )
        ttk.Button(buttons, text="Reset Safety", command=self.reset_safety).pack(
            side=tk.RIGHT, padx=(10, 0)
        )
        ttk.Button(buttons, text="Stop Launch", style="Danger.TButton", command=self.stop_launch).pack(
            side=tk.RIGHT
        )

    def _build_log_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Runtime Log", style="Card.TLabelframe")
        frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(
            frame,
            height=18,
            bg="#101820",
            fg="#d8f3dc",
            insertbackground="#d8f3dc",
            font=("Courier New", 10),
            wrap="word",
        )
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)

    def _build_mission_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Mission Points", style="Card.TLabelframe")
        frame.pack(fill=tk.X, pady=(0, 18))

        tk.Label(
            frame,
            text="Use current pose to create inspection points, then optimize visit order.\nRViz mission heading: use 2D Goal Pose (Mission Heading) near a picked point.",
            bg="#ebe6dc",
            font=("Helvetica", 10),
        ).pack(anchor="w", padx=12, pady=(12, 4))

        list_row = tk.Frame(frame, bg="#ebe6dc")
        list_row.pack(fill=tk.X, padx=12, pady=(6, 8))
        self.point_listbox = tk.Listbox(list_row, height=6, font=("Courier New", 10))
        self.point_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        scroll = ttk.Scrollbar(list_row, orient=tk.VERTICAL, command=self.point_listbox.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.point_listbox.configure(yscrollcommand=scroll.set)

        buttons = tk.Frame(frame, bg="#ebe6dc")
        buttons.pack(fill=tk.X, padx=12, pady=(0, 8))
        ttk.Button(buttons, text="Add Current Pose", command=self.add_current_pose_point).grid(
            row=0, column=0, padx=(0, 8), pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Delete Selected", command=self.delete_selected_point).grid(
            row=0, column=1, padx=8, pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Clear Points", command=self.clear_points).grid(
            row=0, column=2, padx=(8, 0), pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Optimize Order", command=self.optimize_points).grid(
            row=1, column=0, padx=(0, 8), pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Current Order", command=self.use_current_order).grid(
            row=1, column=1, padx=8, pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Sync Points", command=self.sync_points_to_robot).grid(
            row=1, column=2, padx=(8, 0), pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Check Localization", command=self.check_localization).grid(
            row=2, column=0, padx=(0, 8), pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Start Mission", command=self.start_mission).grid(
            row=2, column=1, padx=8, pady=4, sticky="ew"
        )
        ttk.Button(buttons, text="Clear RViz Points", command=self.clear_rviz_points).grid(
            row=2, column=2, padx=(8, 0), pady=4, sticky="ew"
        )
        for col in range(3):
            buttons.grid_columnconfigure(col, weight=1)

        tk.Label(
            frame,
            textvariable=self.mission_var,
            bg="#ebe6dc",
            font=("Helvetica", 10, "bold"),
        ).pack(anchor="w", padx=12, pady=(0, 12))

    def _build_map_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Map", style="Card.TLabelframe")
        frame.pack(fill=tk.X)
        self.map_canvas = tk.Canvas(
            frame,
            width=460,
            height=180,
            bg="#f7f3eb",
            highlightthickness=0,
        )
        self.map_canvas.pack(padx=12, pady=(12, 8))
        tk.Label(
            frame,
            text="OccupancyGrid, robot pose and recent trail",
            bg="#ebe6dc",
            font=("Helvetica", 10),
        ).pack(anchor="w", padx=12, pady=(0, 12))

    def _build_thermal_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Thermal Camera", style="Card.TLabelframe")
        frame.pack(fill=tk.X, pady=(18, 0))
        self.thermal_fig, self.thermal_ax = plt.subplots(figsize=(5.3, 2.5), dpi=90)
        self.thermal_cbar = None
        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, frame)
        self.thermal_canvas.draw()
        self.thermal_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=12, pady=(12, 8))
        buttons = tk.Frame(frame, bg="#ebe6dc")
        buttons.pack(fill=tk.X, padx=12, pady=(0, 8))
        ttk.Button(buttons, text="Start Thermal", command=self.start_thermal).pack(side=tk.LEFT)
        ttk.Button(buttons, text="Stop Thermal", command=self.stop_thermal).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(buttons, text="Open Thermal Window", command=self.open_thermal_window).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(buttons, text="Save Snapshot", command=self.save_thermal_snapshot).pack(side=tk.LEFT, padx=(10, 0))
        tk.Label(
            frame,
            textvariable=self.thermal_var,
            bg="#ebe6dc",
            font=("Helvetica", 10),
        ).pack(anchor="w", padx=12, pady=(0, 8))

        gas_frame = tk.Frame(frame, bg="#ebe6dc")
        gas_frame.pack(fill=tk.X, padx=12, pady=(0, 12))
        for idx, variable in enumerate([self.gas_h2_var, self.gas_co_var, self.gas_voc_var, self.gas_smoke_var]):
            row = idx // 2
            col = idx % 2
            tk.Label(
                gas_frame,
                textvariable=variable,
                bg="#ebe6dc",
                font=("Helvetica", 10, "bold"),
                anchor="w",
                width=18,
            ).grid(row=row, column=col, sticky="w", padx=(0, 16), pady=2)

    def _build_status_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Robot Status", style="Card.TLabelframe")
        frame.pack(fill=tk.X, pady=(18, 0))

        lights = tk.Frame(frame, bg="#ebe6dc")
        lights.pack(fill=tk.X, padx=12, pady=(12, 6))
        for name in ["SSH", "Laser", "Odom", "Map", "Safety", "Thermal", "Gas"]:
            lamp = tk.Frame(lights, bg="#ebe6dc")
            lamp.pack(fill=tk.X, pady=4)
            canvas = tk.Canvas(lamp, width=18, height=18, bg="#ebe6dc", highlightthickness=0)
            canvas.pack(side=tk.LEFT)
            canvas.create_oval(2, 2, 16, 16, fill="#8f8f8f", outline="")
            label = tk.Label(lamp, text=f"{name}: waiting", bg="#ebe6dc", font=("Helvetica", 10))
            label.pack(side=tk.LEFT, padx=8)
            self.indicators[name.lower()] = (canvas, label)

        cards = [
            ("Pose", self.pose_var, "#0f4c5c"),
            ("Laser", self.scan_var, "#437f97"),
            ("Odometry", self.odom_var, "#bc4b51"),
            ("Map", self.map_var_status, "#6d597a"),
            ("Mission", self.mission_var, "#9c6644"),
            ("Safety", self.safety_var, "#8b1e3f"),
            ("Thermal", self.thermal_var, "#6b705c"),
            ("Gas", self.gas_var, "#7c6a0a"),
        ]
        for title, variable, color in cards:
            card = tk.Frame(frame, bg=color, width=440, height=64)
            card.pack(fill=tk.X, padx=12, pady=10)
            card.pack_propagate(False)
            tk.Label(card, text=title, bg=color, fg="white", font=("Helvetica", 12, "bold")).pack(
                anchor="w", padx=12, pady=(10, 4)
            )
            tk.Label(card, textvariable=variable, bg=color, fg="#f7f7f7", font=("Helvetica", 11)).pack(
                anchor="w", padx=12
            )

    def _build_drive_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Manual Drive", style="Card.TLabelframe")
        frame.pack(fill=tk.X, pady=(18, 0))

        tk.Label(
            frame,
            text="Keyboard: W/A/S/D or Arrow Keys, Space to stop",
            bg="#ebe6dc",
            font=("Helvetica", 10),
        ).pack(anchor="w", padx=12, pady=(12, 4))

        pad = tk.Frame(frame, bg="#ebe6dc")
        pad.pack(padx=14, pady=14)

        ttk.Button(pad, text="↑", style="Drive.TButton", command=self.drive_forward).grid(row=0, column=1, padx=10, pady=10)
        ttk.Button(pad, text="←", style="Drive.TButton", command=self.turn_left).grid(row=1, column=0, padx=10, pady=10)
        ttk.Button(pad, text="STOP", style="Danger.TButton", command=self.stop_robot).grid(row=1, column=1, padx=10, pady=10)
        ttk.Button(pad, text="→", style="Drive.TButton", command=self.turn_right).grid(row=1, column=2, padx=10, pady=10)
        ttk.Button(pad, text="↓", style="Drive.TButton", command=self.drive_backward).grid(row=2, column=1, padx=10, pady=10)

        extra = tk.Frame(frame, bg="#ebe6dc")
        extra.pack(fill=tk.X, padx=12, pady=(0, 12))
        ttk.Button(extra, text="Forward Left", command=self.forward_left).grid(row=0, column=0, padx=(0, 8), pady=4, sticky="ew")
        ttk.Button(extra, text="Forward Right", command=self.forward_right).grid(row=0, column=1, padx=(8, 0), pady=4, sticky="ew")
        extra.grid_columnconfigure(0, weight=1)
        extra.grid_columnconfigure(1, weight=1)

    def _bind_keys(self):
        bindings = {
            "<Up>": self.drive_forward,
            "<Down>": self.drive_backward,
            "<Left>": self.turn_left,
            "<Right>": self.turn_right,
            "<space>": self.stop_robot,
            "w": self.drive_forward,
            "s": self.drive_backward,
            "a": self.turn_left,
            "d": self.turn_right,
        }
        for key, handler in bindings.items():
            self.root.bind(key, lambda _event, fn=handler: fn())

    def start_mapping(self):
        self._apply_workspace()
        self.log_queue.put("[INFO] Start mapping. Make sure navigation is not running at the same time.")
        self.launch_manager.start("mapping", "ros2 launch mapping_bringup mapping.launch.py")

    def start_navigation(self):
        self._apply_workspace()
        map_path = self.map_var.get().strip()
        if not map_path:
            messagebox.showerror("Missing Map", "Map YAML path is required.")
            return
        self.log_queue.put("[INFO] Start navigation. Stop mapping first to avoid TF conflicts between slam_toolbox and AMCL.")
        self.launch_manager.start(
            "navigation",
            f"ros2 launch mapping_bringup navigation.launch.py map:={map_path}",
        )

    def save_map(self):
        self._apply_workspace()
        prefix = os.path.splitext(self.map_var.get().strip())[0]
        if not prefix:
            messagebox.showerror("Missing Map", "Map path is required.")
            return
        self.launch_manager.run_once("save_map", f"ros2 run nav2_map_server map_saver_cli -f {prefix}")

    def stop_launch(self):
        self.launch_manager.stop()

    def drive_forward(self):
        self.ros.publish_cmd_vel(self.manual_linear, 0.0)

    def drive_backward(self):
        self.ros.publish_cmd_vel(-self.manual_linear, 0.0)

    def turn_left(self):
        self.ros.publish_cmd_vel(0.0, self.manual_angular)

    def turn_right(self):
        self.ros.publish_cmd_vel(0.0, -self.manual_angular)

    def forward_left(self):
        self.ros.publish_cmd_vel(self.manual_linear, self.manual_angular * 0.5)

    def forward_right(self):
        self.ros.publish_cmd_vel(self.manual_linear, -self.manual_angular * 0.5)

    def stop_robot(self):
        self.ros.publish_cmd_vel(0.0, 0.0)

    def start_thermal(self):
        self._apply_workspace()
        self.log_queue.put("[INFO] Start thermal and gas sensor nodes.")
        self.launch_manager.start_thermal()

    def stop_thermal(self):
        self.launch_manager.stop_thermal()

    def open_thermal_window(self):
        if self.thermal_window is not None and self.thermal_window.winfo_exists():
            self.thermal_window.lift()
            self._draw_thermal_view()
            return

        self.thermal_window = tk.Toplevel(self.root)
        self.thermal_window.title("Thermal Camera Detail")
        self.thermal_window.geometry("900x620")
        self.thermal_window.configure(bg="#ebe6dc")
        self.thermal_window.protocol("WM_DELETE_WINDOW", self._close_thermal_window)

        frame = ttk.LabelFrame(self.thermal_window, text="Thermal Detail", style="Card.TLabelframe")
        frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)
        self.thermal_popup_fig, self.thermal_popup_ax = plt.subplots(figsize=(8.4, 5.0), dpi=100)
        self.thermal_popup_canvas = FigureCanvasTkAgg(self.thermal_popup_fig, frame)
        self.thermal_popup_canvas.draw()
        self.thermal_popup_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=12, pady=12)
        self._draw_thermal_view()

    def _close_thermal_window(self):
        if self.thermal_popup_fig is not None:
            plt.close(self.thermal_popup_fig)
        self.thermal_window.destroy()
        self.thermal_window = None
        self.thermal_popup_fig = None
        self.thermal_popup_ax = None
        self.thermal_popup_canvas = None
        self.thermal_popup_cbar = None

    def save_thermal_snapshot(self):
        if not self.ros.thermal_frame:
            messagebox.showinfo("Thermal Snapshot", "No thermal data available yet.")
            return

        output_dir = os.path.join(os.path.expanduser("~"), "Pictures", "thermal_snapshots")
        os.makedirs(output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(output_dir, f"thermal_{timestamp}.png")
        self.thermal_fig.savefig(output_path, dpi=180, bbox_inches="tight")
        self.log_queue.put(f"[THERMAL] snapshot saved: {output_path}")
        messagebox.showinfo("Thermal Snapshot", f"Saved to:\n{output_path}")

    def add_current_pose_point(self):
        point = InspectionPoint()
        point.point_name = f"P{self.point_counter}"
        point.x = self.ros.robot_x
        point.y = self.ros.robot_y
        point.theta = self.ros.robot_yaw
        point.is_confirmed = True
        self.point_counter += 1
        self.mission_points.append(point)
        self._set_route_preview([])
        self._sync_point_listbox()
        self.log_queue.put(
            f"[MISSION] add {point.point_name} -> ({point.x:.2f}, {point.y:.2f}, yaw={math.degrees(point.theta):.1f}deg)"
        )

    def delete_selected_point(self):
        selection = self.point_listbox.curselection()
        if not selection:
            messagebox.showinfo("Mission Points", "Select a point to delete.")
            return
        index = selection[0]
        point = self.mission_points.pop(index)
        self._set_route_preview([])
        self._sync_point_listbox()
        self.log_queue.put(f"[MISSION] delete {point.point_name}")

    def clear_points(self):
        self.mission_points.clear()
        self._set_route_preview([])
        self._sync_point_listbox()
        self.log_queue.put("[MISSION] cleared all points")

    def optimize_points(self):
        if len(self.mission_points) < 2:
            self.log_queue.put("[MISSION] need at least 2 points to optimize order")
            return

        original_points = list(self.mission_points)
        advanced_plan = None
        if self.ros.map_data is not None:
            advanced_plan = plan_mission_order(
                self.ros.map_data,
                (self.ros.robot_x, self.ros.robot_y),
                original_points,
            )

        if advanced_plan:
            self.mission_points = [original_points[index] for index in advanced_plan.ordered_indices]
            self._set_route_preview(advanced_plan.preview_path)
            self._sync_point_listbox()
            names = " -> ".join(point.point_name for point in self.mission_points)
            self.log_queue.put(
                f"[MISSION] obstacle-aware order: {names} | cost={advanced_plan.final_cost:.1f}"
            )
            return

        anchor = self._make_anchor_point()
        points = [anchor, *original_points]
        self.tsp_planner.calculate_distance_matrix(points)
        order = self.tsp_planner.solve_tsp_dynamic_programming(points)
        if not order:
            self.log_queue.put("[WARN] mission order optimization returned no route")
            self._set_route_preview([])
            return
        ordered = [points[idx] for idx in order if idx != 0]
        self.mission_points = ordered
        preview = self._preview_current_order_plan()
        self._set_route_preview(preview.preview_path if preview else [])
        self._sync_point_listbox()
        names = " -> ".join(point.point_name for point in self.mission_points)
        self.log_queue.put(f"[MISSION] fallback order: {names}")

    def use_current_order(self):
        if not self.mission_points:
            self.log_queue.put("[MISSION] no points in current mission order")
            return
        preview = self._preview_current_order_plan()
        if preview:
            self._set_route_preview(preview.preview_path)
            self.log_queue.put(
                f"[MISSION] current order preview ready | cost={preview.final_cost:.1f}"
            )
        else:
            self._set_route_preview([])
            self.log_queue.put("[WARN] failed to preview current mission order on the map")
        names = " -> ".join(point.point_name for point in self.mission_points)
        self.log_queue.put(f"[MISSION] current order kept: {names}")

    def log_navigation_request(self):
        if not self.mission_points:
            self.log_queue.put("[MISSION] no points to export")
            return
        request_text = ", ".join(
            f"{point.point_name}({point.x:.2f},{point.y:.2f},{math.degrees(point.theta):.1f}deg)"
            for point in self.mission_points
        )
        self.log_queue.put(f"[MISSION] StartNavigation request preview: {request_text}")

    def clear_rviz_points(self):
        request = Trigger.Request()

        def done(result, error):
            self.root.after(0, lambda: self._handle_clear_rviz_points_result(result, error))

        self.log_queue.put("[MISSION] clearing RViz-picked mission points on robot")
        self.ros.call_service_async(self.ros.clear_rviz_points_client, request, done)

    def check_localization(self):
        request = Localize.Request()

        def done(result, error):
            self.root.after(0, lambda: self._handle_localize_result(result, error))

        self.log_queue.put("[MISSION] querying localization status")
        self.ros.call_service_async(self.ros.localize_client, request, done)

    def sync_points_to_robot(self):
        if not self.mission_points:
            messagebox.showinfo("Mission Points", "No mission points to sync.")
            return

        request = ConfirmInspectionPoints.Request()
        request.points = self.mission_points

        def done(result, error):
            self.root.after(0, lambda: self._handle_confirm_result(result, error))

        self.log_queue.put(f"[MISSION] syncing {len(self.mission_points)} points to robot")
        self.ros.call_service_async(self.ros.confirm_points_client, request, done)

    def start_mission(self):
        if self.ros.safety_level == "FAULT":
            messagebox.showerror(
                "Start Mission",
                f"Safety fault is latched: {self.ros.safety_message}\n\nReset Safety before starting a new mission.",
            )
            return

        request = StartNavigation.Request()
        if not self.mission_points:
            confirm = messagebox.askyesno(
                "Start Mission",
                "No local mission points in UI.\n\nUse the mission points picked in RViz and start execution?\n\nTip: Set point heading in RViz with 2D Goal Pose (Mission Heading).",
            )
            if not confirm:
                return
            self.log_queue.put("[MISSION] sending mission using RViz-picked points")
        else:
            request.waypoints = self.mission_points
            self.log_queue.put(f"[MISSION] sending mission with {len(self.mission_points)} UI points")

        def done(result, error):
            self.root.after(0, lambda: self._handle_start_mission_result(result, error))

        self.ros.call_service_async(self.ros.start_navigation_client, request, done, timeout_sec=10.0)

    def reset_safety(self):
        request = Trigger.Request()

        def done(result, error):
            self.root.after(0, lambda: self._handle_reset_safety_result(result, error))

        self.log_queue.put("[SAFETY] reset requested")
        self.ros.call_service_async(self.ros.reset_safety_client, request, done)

    def _apply_workspace(self):
        self.launch_manager.workspace_path = self.workspace_var.get().strip() or self.workspace_path
        self.launch_manager.remote_user = self.remote_user_var.get().strip() or self.remote_user
        self.launch_manager.remote_host = self.remote_host_var.get().strip() or self.remote_host

    def _handle_localize_result(self, result, error):
        if error is not None:
            self.log_queue.put(f"[ERROR] localization check failed: {error}")
            messagebox.showerror("Localization", error)
            return
        self.log_queue.put(f"[MISSION] {result.message}")
        if result.success:
            messagebox.showinfo("Localization", result.message)
        else:
            messagebox.showwarning("Localization", result.message)

    def _handle_confirm_result(self, result, error):
        if error is not None:
            self.log_queue.put(f"[ERROR] point sync failed: {error}")
            messagebox.showerror("Sync Points", error)
            return
        self.log_queue.put(f"[MISSION] {result.message}")
        if result.success:
            messagebox.showinfo("Sync Points", result.message)
        else:
            messagebox.showwarning("Sync Points", result.message)

    def _handle_clear_rviz_points_result(self, result, error):
        if error is not None:
            self.log_queue.put(f"[ERROR] clear RViz points failed: {error}")
            messagebox.showerror("Clear RViz Points", error)
            return
        self.log_queue.put(f"[MISSION] {result.message}")
        if result.success:
            messagebox.showinfo("Clear RViz Points", result.message)
        else:
            messagebox.showwarning("Clear RViz Points", result.message)

    def _handle_reset_safety_result(self, result, error):
        if error is not None:
            self.log_queue.put(f"[ERROR] reset safety failed: {error}")
            messagebox.showerror("Reset Safety", error)
            return
        self.log_queue.put(f"[SAFETY] {result.message}")
        if result.success:
            messagebox.showinfo("Reset Safety", result.message)
        else:
            messagebox.showwarning("Reset Safety", result.message)

    def _handle_start_mission_result(self, result, error):
        if error is not None:
            self.log_queue.put(f"[ERROR] mission start failed: {error}")
            messagebox.showerror("Start Mission", error)
            return
        self.log_queue.put(f"[MISSION] {result.message}")
        if result.success:
            messagebox.showinfo("Start Mission", result.message)
        else:
            messagebox.showwarning("Start Mission", result.message)

    def _schedule_update(self):
        self._flush_logs()
        self._handle_safety_alert()
        self._refresh_status()
        self.root.after(200, self._schedule_update)

    def _flush_logs(self):
        while not self.log_queue.empty():
            line = self.log_queue.get_nowait()
            self.log_text.insert(tk.END, line + "\n")
            self.log_text.see(tk.END)

    def _handle_safety_alert(self):
        if self.ros.pending_safety_alert is None:
            return
        level, code, message = self.ros.pending_safety_alert
        self.ros.pending_safety_alert = None
        self.log_queue.put(f"[SAFETY] {code}: {message}")
        if level == "FAULT":
            messagebox.showerror("Safety Fault", message)
        elif level == "WARN":
            messagebox.showwarning("Safety Warning", message)

    def _refresh_status(self):
        now = time.time()
        self.pose_var.set(
            f"x={self.ros.robot_x:.2f}  y={self.ros.robot_y:.2f}  yaw={math.degrees(self.ros.robot_yaw):.1f}deg"
        )
        self._record_pose()
        if now - self.ros.last_scan_stamp < 1.0:
            self.scan_var.set(f"scan alive, ranges={self.ros.scan_count}")
            self._set_indicator("laser", True, f"Laser: {self.ros.scan_count} ranges")
        else:
            self.scan_var.set("scan timeout")
            self._set_indicator("laser", False, "Laser: timeout")

        if now - self.ros.last_odom_stamp < 1.0:
            self.odom_var.set(f"odom alive, yaw_rate={self.ros.robot_yaw_rate:.2f}")
            self._set_indicator("odom", True, "Odom: online")
        else:
            self.odom_var.set("odom timeout")
            self._set_indicator("odom", False, "Odom: timeout")

        if now - self.ros.last_map_stamp < 2.0 and self.ros.map_data is not None:
            width = self.ros.map_data.info.width
            height = self.ros.map_data.info.height
            self.map_var_status.set(f"map alive, {width}x{height}")
            self._set_indicator("map", True, f"Map: {width}x{height}")
        else:
            self.map_var_status.set("map timeout")
            self._set_indicator("map", False, "Map: timeout")

        if now - self.ros.last_safety_stamp < 3.0:
            self.safety_var.set(f"{self.ros.safety_level}: {self.ros.safety_message}")
            if self.ros.safety_level == 'FAULT':
                self._set_indicator_state('safety', '#d00000', f"Safety: {self.ros.safety_code}")
            elif self.ros.safety_level == 'WARN':
                self._set_indicator_state('safety', '#f77f00', f"Safety: {self.ros.safety_code}")
            else:
                self._set_indicator_state('safety', '#2dc653', f"Safety: {self.ros.safety_code}")
        else:
            self.safety_var.set('safety: waiting')
            self._set_indicator_state('safety', '#8f8f8f', 'Safety: waiting')

        if now - self.ros.last_thermal_stamp < 2.0 and self.ros.thermal_frame:
            self.thermal_var.set(
                f"thermal alive, min={self.ros.thermal_min:.1f}C  max={self.ros.thermal_max:.1f}C  avg={self.ros.thermal_avg:.1f}C"
            )
            self._set_indicator("thermal", True, "Thermal: online")
        else:
            self.thermal_var.set("thermal timeout")
            self._set_indicator("thermal", False, "Thermal: timeout")

        if now - self.ros.last_gas_stamp < 2.0:
            self.gas_h2_var.set(f"H2: {self.ros.gas_data['H2']:.1f}")
            self.gas_co_var.set(f"CO: {self.ros.gas_data['CO']:.1f}")
            self.gas_voc_var.set(f"VOC: {self.ros.gas_data['VOC']:.1f}")
            self.gas_smoke_var.set(f"Smoke: {self.ros.gas_data['Smoke']:.1f}")
            self.gas_var.set(
                f"H2={self.ros.gas_data['H2']:.1f}  CO={self.ros.gas_data['CO']:.1f}  "
                f"VOC={self.ros.gas_data['VOC']:.1f}  Smoke={self.ros.gas_data['Smoke']:.1f}"
            )
            self._set_indicator("gas", True, "Gas: online")
        else:
            self.gas_h2_var.set("H2: --")
            self.gas_co_var.set("CO: --")
            self.gas_voc_var.set("VOC: --")
            self.gas_smoke_var.set("Smoke: --")
            self.gas_var.set("gas timeout")
            self._set_indicator("gas", False, "Gas: timeout")

        ssh_ok = self.launch_manager.last_exit_code in (None, 0) or self.launch_manager.is_running()
        self._set_indicator("ssh", ssh_ok, f"SSH: {self.ssh_var.get()}")

        if self.ros.safety_level == 'FAULT':
            self.status_var.set(f"FAULT: {self.ros.safety_code}")
        elif self.launch_manager.is_running():
            self.status_var.set(f"Running: {self.launch_manager.active_name}")
        else:
            self.status_var.set("Idle")

        self._refresh_mission_status()
        self._draw_map_view()
        self._draw_thermal_view()

    def _set_indicator(self, key, ok, text):
        color = '#2dc653' if ok else '#d00000'
        self._set_indicator_state(key, color, text)

    def _set_indicator_state(self, key, color, text):
        canvas, label = self.indicators[key]
        canvas.delete('all')
        canvas.create_oval(2, 2, 16, 16, fill=color, outline='')
        label.config(text=text)

    def _record_pose(self):
        point = (round(self.ros.robot_x, 2), round(self.ros.robot_y, 2))
        if not self.pose_history or self.pose_history[-1] != point:
            self.pose_history.append(point)
            if len(self.pose_history) > 80:
                self.pose_history.pop(0)

    def _sync_point_listbox(self):
        self.point_listbox.delete(0, tk.END)
        for index, point in enumerate(self.mission_points, start=1):
            self.point_listbox.insert(
                tk.END,
                f"{index:>2}. {point.point_name:<6} x={point.x:>6.2f}  y={point.y:>6.2f}",
            )
        self.last_map_render_key = None

    def _refresh_mission_status(self):
        if not self.mission_points:
            self.mission_var.set("mission: 0 points")
        else:
            names = " -> ".join(point.point_name for point in self.mission_points[:4])
            if len(self.mission_points) > 4:
                names += " ..."
            self.mission_var.set(f"mission: {len(self.mission_points)} points | {names}")

    def _make_anchor_point(self):
        point = InspectionPoint()
        point.point_name = "ROBOT"
        point.x = self.ros.robot_x
        point.y = self.ros.robot_y
        point.theta = self.ros.robot_yaw
        point.is_confirmed = True
        return point

    def _preview_current_order_plan(self):
        if self.ros.map_data is None or not self.mission_points:
            return None
        return preview_order_route(
            self.ros.map_data,
            (self.ros.robot_x, self.ros.robot_y),
            self.mission_points,
        )

    def _set_route_preview(self, preview_path):
        self.route_preview = list(preview_path)
        self.last_map_render_key = None

    def _draw_map_view(self):
        map_msg = self.ros.map_data
        key = (
            round(self.ros.robot_x, 2),
            round(self.ros.robot_y, 2),
            len(self.pose_history),
            getattr(map_msg.info, "width", 0) if map_msg else 0,
            getattr(map_msg.info, "height", 0) if map_msg else 0,
            int(self.ros.last_map_stamp),
            len(self.route_preview),
        )
        if key == self.last_map_render_key:
            return
        self.last_map_render_key = key

        canvas = self.map_canvas
        canvas.delete("all")
        width_px = int(canvas["width"])
        height_px = int(canvas["height"])
        canvas.create_rectangle(8, 8, width_px - 8, height_px - 8, fill="#fcfbf7", outline="#d5d5d5")

        if map_msg is None:
            canvas.create_text(
                width_px / 2,
                height_px / 2,
                text="No /map data yet",
                fill="#6c757d",
                font=("Helvetica", 14, "bold"),
            )
            return

        info = map_msg.info
        grid_w = info.width
        grid_h = info.height
        if grid_w == 0 or grid_h == 0:
            return

        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        inner_w = width_px - 16
        inner_h = height_px - 16
        step = max(1, max(grid_w, grid_h) // 80)
        cell_w = inner_w / max(1, grid_w / step)
        cell_h = inner_h / max(1, grid_h / step)

        data = map_msg.data
        for gy in range(0, grid_h, step):
            for gx in range(0, grid_w, step):
                value = data[gy * grid_w + gx]
                if value < 0:
                    color = "#d9d9d9"
                elif value > 50:
                    color = "#1f1f1f"
                else:
                    continue
                x0 = 8 + (gx / step) * cell_w
                y0 = 8 + inner_h - ((gy / step) + 1) * cell_h
                x1 = x0 + cell_w + 1
                y1 = y0 + cell_h + 1
                canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline=color)

        trail = []
        for x, y in self.pose_history:
            px, py = self._world_to_canvas(x, y, origin_x, origin_y, resolution, grid_w, grid_h, inner_w, inner_h)
            trail.extend([px, py])
        if len(trail) >= 4:
            canvas.create_line(*trail, fill="#3a86ff", width=2, smooth=True)

        preview = []
        for x, y in self.route_preview:
            px, py = self._world_to_canvas(x, y, origin_x, origin_y, resolution, grid_w, grid_h, inner_w, inner_h)
            preview.extend([px, py])
        if len(preview) >= 4:
            canvas.create_line(*preview, fill="#2a9d8f", width=3, smooth=True)

        for index, point in enumerate(self.mission_points, start=1):
            point_px, point_py = self._world_to_canvas(
                point.x,
                point.y,
                origin_x,
                origin_y,
                resolution,
                grid_w,
                grid_h,
                inner_w,
                inner_h,
            )
            canvas.create_oval(
                point_px - 4, point_py - 4, point_px + 4, point_py + 4, fill="#ff9f1c", outline=""
            )
            canvas.create_text(
                point_px + 10,
                point_py - 8,
                text=f"{index}:{point.point_name}",
                fill="#7f5539",
                anchor="w",
                font=("Helvetica", 9, "bold"),
            )

        robot_px, robot_py = self._world_to_canvas(
            self.ros.robot_x,
            self.ros.robot_y,
            origin_x,
            origin_y,
            resolution,
            grid_w,
            grid_h,
            inner_w,
            inner_h,
        )
        canvas.create_oval(robot_px - 5, robot_py - 5, robot_px + 5, robot_py + 5, fill="#e63946", outline="")
        if self.route_preview:
            canvas.create_text(18, 36, anchor="nw", text=f"Route preview points: {len(self.route_preview)}", fill="#2a9d8f", font=("Helvetica", 10, "bold"))
        canvas.create_text(18, 18, anchor="nw", text=f"Pose ({self.ros.robot_x:.2f}, {self.ros.robot_y:.2f})", fill="#153243", font=("Helvetica", 10, "bold"))

    def _world_to_canvas(self, x, y, origin_x, origin_y, resolution, grid_w, grid_h, inner_w, inner_h):
        map_w = grid_w * resolution
        map_h = grid_h * resolution
        rel_x = 0.0 if map_w == 0 else (x - origin_x) / map_w
        rel_y = 0.0 if map_h == 0 else (y - origin_y) / map_h
        px = 8 + max(0.0, min(1.0, rel_x)) * inner_w
        py = 8 + inner_h - max(0.0, min(1.0, rel_y)) * inner_h
        return px, py

    def _draw_thermal_view(self):
        self.thermal_cbar = self._render_thermal_axes(
            self.thermal_fig,
            self.thermal_ax,
            self.thermal_cbar,
            title="MLX90640 Thermal View",
        )
        self.thermal_canvas.draw_idle()

        if self.thermal_popup_fig is not None and self.thermal_popup_ax is not None and self.thermal_popup_canvas is not None:
            self.thermal_popup_cbar = self._render_thermal_axes(
                self.thermal_popup_fig,
                self.thermal_popup_ax,
                self.thermal_popup_cbar,
                title="MLX90640 Thermal Detail",
            )
            self.thermal_popup_canvas.draw_idle()

    def _render_thermal_axes(self, figure, ax, current_cbar, title):
        ax.clear()

        if current_cbar is not None:
            try:
                current_cbar.remove()
            except Exception:
                pass
            current_cbar = None

        if not self.ros.thermal_frame:
            ax.text(
                0.5,
                0.5,
                "No /thermal_frame data yet",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=12,
                color="red",
            )
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.set_xticks([])
            ax.set_yticks([])
            return current_cbar

        cols = max(1, self.ros.thermal_width)
        rows = max(1, self.ros.thermal_height)
        frame = self.ros.thermal_frame[: rows * cols]
        matrix = [frame[row * cols:(row + 1) * cols] for row in range(rows)]
        vmin = self.ros.thermal_min if self.ros.thermal_min > 0 else 20.0
        vmax = self.ros.thermal_max if self.ros.thermal_max > 0 else 75.0

        plot = ax.imshow(matrix, cmap="inferno", vmin=vmin, vmax=vmax)
        current_cbar = figure.colorbar(plot, ax=ax, shrink=0.82)
        current_cbar.set_label("Temp (C)", fontsize=9)

        ax.text(
            0.02,
            0.95,
            f"Min: {self.ros.thermal_min:.1f}C",
            transform=ax.transAxes,
            color="cyan",
            fontsize=9,
            bbox=dict(facecolor="black", alpha=0.5),
        )
        ax.text(
            0.02,
            0.90,
            f"Max: {self.ros.thermal_max:.1f}C",
            transform=ax.transAxes,
            color="red",
            fontsize=9,
            bbox=dict(facecolor="black", alpha=0.5),
        )
        ax.text(
            0.02,
            0.85,
            f"Avg: {self.ros.thermal_avg:.1f}C",
            transform=ax.transAxes,
            color="lime",
            fontsize=9,
            bbox=dict(facecolor="black", alpha=0.5),
        )

        if self.ros.thermal_change_ready:
            delta_text = f"Delta/min: {self.ros.thermal_change_per_min:.1f}C"
            delta_color = "red" if self.ros.thermal_change_per_min > 0 else "cyan"
        else:
            delta_text = "Delta/min: collecting..."
            delta_color = "white"
        ax.text(
            0.02,
            0.80,
            delta_text,
            transform=ax.transAxes,
            color=delta_color,
            fontsize=9,
            bbox=dict(facecolor="black", alpha=0.5),
        )

        ax.set_xlabel("Pixel X (32)", fontsize=9)
        ax.set_ylabel("Pixel Y (24)", fontsize=9)
        ax.set_title(title, fontsize=11)
        ax.set_xticks([])
        ax.set_yticks([])
        return current_cbar

    def on_close(self):
        try:
            self.stop_robot()
            self.stop_thermal()
            self.launch_manager.stop()
            self.ros.shutdown()
        finally:
            if self.thermal_window is not None and self.thermal_window.winfo_exists():
                self._close_thermal_window()
            self.root.destroy()


def main(args=None):
    root = tk.Tk()
    ros_adapter = RosUiAdapter(args=args)
    app = RobotControlApp(root, ros_adapter)
    try:
        root.mainloop()
    finally:
        if root.winfo_exists():
            app.on_close()

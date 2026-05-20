import os
import queue
import shlex
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan


class RosUiAdapter:
    def __init__(self, args=None):
        rclpy.init(args=args)
        self.node = rclpy.create_node("robot_control_ui")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.scan_sub = self.node.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.map_sub = self.node.create_subscription(OccupancyGrid, "/map", self._map_cb, 1)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw_rate = 0.0
        self.scan_count = 0
        self.map_data = None
        self.last_scan_stamp = 0.0
        self.last_odom_stamp = 0.0
        self.last_map_stamp = 0.0

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw_rate = msg.twist.twist.angular.z
        self.last_odom_stamp = time.time()

    def _scan_cb(self, msg):
        self.scan_count = len(msg.ranges)
        self.last_scan_stamp = time.time()

    def _map_cb(self, msg):
        self.map_data = msg
        self.last_map_stamp = time.time()

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

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
        self.active_name = "idle"
        self.last_exit_code = None

    def _build_remote_command(self, command):
        setup_path = os.path.join(self.workspace_path, "install", "setup.bash")
        return (
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
        if not self.active_process or self.active_process.poll() is not None:
            self.log_callback("[INFO] No active launch process.")
            self.active_name = "idle"
            return

        try:
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)
            self.log_callback(f"[STOP] {self.active_name}")
        except ProcessLookupError:
            pass
        finally:
            self.active_process = None
            self.active_name = "idle"

    def is_running(self):
        return bool(self.active_process and self.active_process.poll() is None)

    def _stream_output(self, proc, name, track_active=True):
        if proc.stdout is not None:
            for line in proc.stdout:
                self.log_callback(line.rstrip())
        code = proc.wait()
        self.last_exit_code = code
        self.log_callback(f"[EXIT] {name} -> {code}")
        if track_active and self.active_process is proc:
            self.active_process = None
            self.active_name = "idle"


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
            "remote_host", "192.168.43.16"
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
        self.last_map_render_key = None

        self.root.title("Tracked Robot Control UI")
        self.root.geometry("1180x760")
        self.root.configure(bg="#ebe6dc")

        self.map_var = tk.StringVar(value=self.default_map_path)
        self.workspace_var = tk.StringVar(value=self.workspace_path)
        self.remote_user_var = tk.StringVar(value=self.remote_user)
        self.remote_host_var = tk.StringVar(value=self.remote_host)
        self.status_var = tk.StringVar(value="Idle")
        self.pose_var = tk.StringVar(value="x=0.00  y=0.00")
        self.scan_var = tk.StringVar(value="scan: no data")
        self.odom_var = tk.StringVar(value="odom: no data")
        self.map_var_status = tk.StringVar(value="map: no data")
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

        left = tk.Frame(body, bg="#ebe6dc")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right = tk.Frame(body, bg="#ebe6dc")
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(18, 0))

        self._build_launch_panel(left)
        self._build_log_panel(left)
        self._build_map_panel(right)
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
        ttk.Button(buttons, text="Stop Launch", style="Danger.TButton", command=self.stop_launch).pack(
            side=tk.RIGHT
        )

    def _build_log_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Runtime Log", style="Card.TLabelframe")
        frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(
            frame,
            height=24,
            bg="#101820",
            fg="#d8f3dc",
            insertbackground="#d8f3dc",
            font=("Courier New", 10),
            wrap="word",
        )
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)

    def _build_map_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Map", style="Card.TLabelframe")
        frame.pack(fill=tk.X)
        self.map_canvas = tk.Canvas(
            frame,
            width=320,
            height=320,
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

    def _build_status_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Robot Status", style="Card.TLabelframe")
        frame.pack(fill=tk.X, pady=(18, 0))

        lights = tk.Frame(frame, bg="#ebe6dc")
        lights.pack(fill=tk.X, padx=12, pady=(12, 6))
        for name in ["SSH", "Laser", "Odom", "Map"]:
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
        ]
        for title, variable, color in cards:
            card = tk.Frame(frame, bg=color, width=280, height=92)
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
        ttk.Button(extra, text="Forward Left", command=self.forward_left).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(extra, text="Forward Right", command=self.forward_right).pack(side=tk.LEFT, padx=8)

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
        self.launch_manager.start("mapping", "ros2 launch mapping_bringup mapping.launch.py")

    def start_navigation(self):
        self._apply_workspace()
        map_path = self.map_var.get().strip()
        if not map_path:
            messagebox.showerror("Missing Map", "Map YAML path is required.")
            return
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

    def _apply_workspace(self):
        self.launch_manager.workspace_path = self.workspace_var.get().strip() or self.workspace_path
        self.launch_manager.remote_user = self.remote_user_var.get().strip() or self.remote_user
        self.launch_manager.remote_host = self.remote_host_var.get().strip() or self.remote_host

    def _schedule_update(self):
        self._flush_logs()
        self._refresh_status()
        self.root.after(200, self._schedule_update)

    def _flush_logs(self):
        while not self.log_queue.empty():
            line = self.log_queue.get_nowait()
            self.log_text.insert(tk.END, line + "\n")
            self.log_text.see(tk.END)

    def _refresh_status(self):
        now = time.time()
        self.pose_var.set(f"x={self.ros.robot_x:.2f}  y={self.ros.robot_y:.2f}")
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

        ssh_ok = self.launch_manager.last_exit_code in (None, 0) or self.launch_manager.is_running()
        self._set_indicator("ssh", ssh_ok, f"SSH: {self.ssh_var.get()}")

        if self.launch_manager.is_running():
            self.status_var.set(f"Running: {self.launch_manager.active_name}")
        else:
            self.status_var.set("Idle")

        self._draw_map_view()

    def _set_indicator(self, key, ok, text):
        canvas, label = self.indicators[key]
        color = "#2dc653" if ok else "#d00000"
        canvas.delete("all")
        canvas.create_oval(2, 2, 16, 16, fill=color, outline="")
        label.config(text=text)

    def _record_pose(self):
        point = (round(self.ros.robot_x, 2), round(self.ros.robot_y, 2))
        if not self.pose_history or self.pose_history[-1] != point:
            self.pose_history.append(point)
            if len(self.pose_history) > 80:
                self.pose_history.pop(0)

    def _draw_map_view(self):
        map_msg = self.ros.map_data
        key = (
            round(self.ros.robot_x, 2),
            round(self.ros.robot_y, 2),
            len(self.pose_history),
            getattr(map_msg.info, "width", 0) if map_msg else 0,
            getattr(map_msg.info, "height", 0) if map_msg else 0,
            int(self.ros.last_map_stamp),
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
        canvas.create_text(18, 18, anchor="nw", text=f"Pose ({self.ros.robot_x:.2f}, {self.ros.robot_y:.2f})", fill="#153243", font=("Helvetica", 10, "bold"))

    def _world_to_canvas(self, x, y, origin_x, origin_y, resolution, grid_w, grid_h, inner_w, inner_h):
        map_w = grid_w * resolution
        map_h = grid_h * resolution
        rel_x = 0.0 if map_w == 0 else (x - origin_x) / map_w
        rel_y = 0.0 if map_h == 0 else (y - origin_y) / map_h
        px = 8 + max(0.0, min(1.0, rel_x)) * inner_w
        py = 8 + inner_h - max(0.0, min(1.0, rel_y)) * inner_h
        return px, py

    def on_close(self):
        try:
            self.stop_robot()
            self.launch_manager.stop()
            self.ros.shutdown()
        finally:
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

import os
import shlex
import signal
import subprocess

from PyQt5.QtCore import QObject, QProcess, pyqtSignal

from .config import DEFAULT_ROS_SETUP_PATH, DEFAULT_WORKSPACE_PATH


class LaunchManager(QObject):
    """Run migrated-robot launches on the Raspberry Pi over SSH."""

    log_line = pyqtSignal(str)
    state_changed = pyqtSignal(str)

    def __init__(
        self,
        workspace_path=DEFAULT_WORKSPACE_PATH,
        ros_setup_path=DEFAULT_ROS_SETUP_PATH,
        remote_user="yy",
        remote_host="192.168.43.21",
    ):
        super().__init__()
        self.workspace_path = workspace_path
        self.ros_setup_path = ros_setup_path
        self.remote_user = remote_user
        self.remote_host = remote_host
        self.active_process = None
        self.thermal_process = None
        self.active_name = "idle"
        self.oneshot_processes = []

    def update_paths(self, workspace_path, ros_setup_path, remote_user=None, remote_host=None):
        self.workspace_path = workspace_path
        self.ros_setup_path = ros_setup_path
        if remote_user:
            self.remote_user = remote_user
        if remote_host:
            self.remote_host = remote_host

    def start_sim(self, *args, **kwargs):
        self.log_line.emit("[INFO] 仿真启动入口已移除；迁移项目只启动实车链路。")
        return False

    def start_mapping(self, *args, **kwargs):
        return self.start("mapping", "ros2 launch inspection_robot_bringup mapping.launch.py")

    def start_navigation(self, map_path, use_rviz=False, *args, **kwargs):
        command = (
            "ros2 launch inspection_robot_bringup navigation.launch.py "
            f"map:={shlex.quote(map_path)} use_rviz:={str(bool(use_rviz)).lower()}"
        )
        return self.start("navigation", command)

    def save_map(self, map_path):
        prefix = os.path.splitext(map_path)[0]
        command = f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(prefix)} -t /map"
        self.run_once("save_map", command)

    def start_thermal(self):
        if self.thermal_process and self.thermal_process.state() != QProcess.NotRunning:
            self.log_line.emit("[WARN] 热成像/气体节点已经运行。")
            return False
        self._remote_cleanup(["sensor_monitor.launch.py", "thermal_camera_node", "gas_sensor_node"])
        proc = self._make_process("thermal", track_active=False)
        self.thermal_process = proc
        proc.start("setsid", self._ssh_args("ros2 launch inspection_robot_hardware sensor_monitor.launch.py"))
        self.log_line.emit("[RUN] thermal: inspection_robot_hardware sensor_monitor.launch.py")
        return True

    def stop_thermal(self):
        self._remote_cleanup(["sensor_monitor.launch.py", "thermal_camera_node", "gas_sensor_node"])
        if self.thermal_process and self.thermal_process.state() != QProcess.NotRunning:
            self._terminate_process(self.thermal_process)
        self.thermal_process = None

    def start(self, name, command):
        if self.active_process is not None and self.active_process.state() != QProcess.NotRunning:
            self.log_line.emit(f"[WARN] 请先停止 {self.active_name}。")
            return False
        proc = self._make_process(name, track_active=True)
        self.active_process = proc
        self.active_name = name
        self.state_changed.emit(name)
        self.log_line.emit(f"[RUN] {name}: {command}")
        proc.start("setsid", self._ssh_args(command))
        return True

    def run_once(self, name, command):
        proc = self._make_process(name, track_active=False)
        self.oneshot_processes.append(proc)
        self.log_line.emit(f"[RUN] {name}: {command}")
        proc.start("setsid", self._ssh_args(command))

    def stop(self):
        if self.active_process is not None and self.active_process.state() != QProcess.NotRunning:
            self._terminate_process(self.active_process)
        self._remote_cleanup([
            "mapping.launch.py", "navigation.launch.py", "sllidar_node",
            "rf2o_laser_odometry", "tracked_motor_driver", "velocity_safety_gate",
            "safety_monitor", "controller_server", "slam_toolbox",
        ])
        self.active_process = None
        self.active_name = "idle"
        self.state_changed.emit("idle")

    def is_running(self):
        return self.active_process is not None and self.active_process.state() != QProcess.NotRunning

    def _remote_command(self, command):
        setup = os.path.join(self.workspace_path, "install", "setup.bash")
        return (
            f"source {shlex.quote(self.ros_setup_path)} && "
            f"source {shlex.quote(setup)} && "
            f"cd {shlex.quote(self.workspace_path)} && {command}"
        )

    def _ssh_args(self, command):
        return [
            "ssh",
            f"{self.remote_user}@{self.remote_host}",
            "bash",
            "-lc",
            self._remote_command(command),
        ]

    def _remote_cleanup(self, patterns):
        if not patterns:
            return
        command = " ; ".join(f"pkill -TERM -f {shlex.quote(pattern)} || true" for pattern in patterns)
        try:
            result = subprocess.run(
                self._ssh_args(command), capture_output=True, text=True, timeout=8, check=False
            )
            if result.stdout.strip():
                self.log_line.emit(result.stdout.strip())
            if result.stderr.strip():
                self.log_line.emit(result.stderr.strip())
        except Exception as exc:
            self.log_line.emit(f"[WARN] 远程清理失败: {exc}")

    @staticmethod
    def _terminate_process(proc):
        try:
            pid = int(proc.processId())
            os.killpg(pid, signal.SIGTERM)
        except (ProcessLookupError, TypeError, ValueError):
            pass
        proc.waitForFinished(2500)

    def _make_process(self, name, track_active):
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(lambda p=proc: self._read_output(p))
        proc.finished.connect(
            lambda code, status, p=proc, n=name, t=track_active: self._finished(p, n, t, code)
        )
        return proc

    def _read_output(self, proc):
        data = bytes(proc.readAllStandardOutput()).decode(errors="replace")
        for line in data.splitlines():
            self.log_line.emit(line)

    def _finished(self, proc, name, track_active, code):
        self.log_line.emit(f"[EXIT] {name} -> {code}")
        if track_active and proc is self.active_process:
            self.active_process = None
            self.active_name = "idle"
            self.state_changed.emit("idle")
        if proc is self.thermal_process:
            self.thermal_process = None
        if proc in self.oneshot_processes:
            self.oneshot_processes.remove(proc)

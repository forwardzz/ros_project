import os
import re
import shlex
import signal
import subprocess

from PyQt5.QtCore import QObject, QProcess, pyqtSignal

from .config import DEFAULT_ROS_SETUP_PATH, DEFAULT_WORKSPACE_PATH


ROBOT_PROCESS_PATTERNS = [
    "mapping.launch.py",
    "navigation.launch.py",
    "sllidar_node",
    "static_transform_publisher",
    "ybimu_driver",
    "rf2o_laser_odometry",
    "ekf_node",
    "tracked_motor_driver",
    "mission_manager",
    "actual_path_recorder",
    "velocity_safety_gate",
    "safety_monitor",
    "map_server",
    "amcl",
    "planner_server",
    "controller_server",
    "bt_navigator",
    "behavior_server",
    "smoother_server",
    "velocity_smoother",
    "lifecycle_manager",
    "slam_toolbox",
    "map_saver_cli",
]


class LaunchManager(QObject):
    """Run migrated-robot launches on the Raspberry Pi over SSH."""

    log_line = pyqtSignal(str)
    state_changed = pyqtSignal(str)
    map_preflight_finished = pyqtSignal(str, bool, str)
    map_save_finished = pyqtSignal(str, bool, str)

    def __init__(
        self,
        workspace_path=DEFAULT_WORKSPACE_PATH,
        ros_setup_path=DEFAULT_ROS_SETUP_PATH,
        remote_user="yy",
        remote_host="192.168.43.24",
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
        self.map_preflight_process = None
        self.map_save_process = None
        self.map_preflight_path = ""
        self.map_save_path = ""

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

    def start_mapping(
        self,
        motor_pair="disabled",
        left_inverted=False,
        right_inverted=False,
        actuation_enabled=False,
        *args,
        **kwargs,
    ):
        command = "ros2 launch inspection_robot_bringup mapping.launch.py " + self._motor_args(
            motor_pair, left_inverted, right_inverted, actuation_enabled
        )
        return self.start("mapping", command)

    def start_navigation(
        self,
        map_path,
        motor_pair="disabled",
        left_inverted=False,
        right_inverted=False,
        actuation_enabled=False,
        *args,
        regions_path=None,
        mission_home_x=0.0,
        mission_home_y=0.0,
        mission_home_yaw=0.0,
        **kwargs,
    ):
        if regions_path is None:
            regions_path = os.path.join(
                self.workspace_path, "maps", "inspection_regions.yaml"
            )
        command = (
            "ros2 launch inspection_robot_bringup navigation.launch.py "
            f"map:={shlex.quote(map_path)} "
            f"regions:={shlex.quote(regions_path)} "
            f"mission_home_x:={float(mission_home_x):.9g} "
            f"mission_home_y:={float(mission_home_y):.9g} "
            f"mission_home_yaw:={float(mission_home_yaw):.9g} "
            + self._motor_args(
                motor_pair, left_inverted, right_inverted, actuation_enabled
            )
        )
        return self.start("navigation", command)

    @staticmethod
    def _motor_args(motor_pair, left_inverted, right_inverted, actuation_enabled):
        return " ".join([
            f"motor_pair:={shlex.quote(str(motor_pair))}",
            f"left_inverted:={str(bool(left_inverted)).lower()}",
            f"right_inverted:={str(bool(right_inverted)).lower()}",
            f"actuation_enabled:={str(bool(actuation_enabled)).lower()}",
        ])

    def map_operation_in_progress(self):
        return any(
            proc is not None and proc.state() != QProcess.NotRunning
            for proc in (self.map_preflight_process, self.map_save_process)
        )

    def check_map_exists(self, map_path):
        """Asynchronously check for either artifact belonging to a remote map."""
        if self.map_operation_in_progress():
            self.log_line.emit("[WARN] 地图检查或保存操作已经在进行。")
            return False
        prefix = os.path.splitext(map_path)[0]
        yaml_path = shlex.quote(prefix + ".yaml")
        pgm_path = shlex.quote(prefix + ".pgm")
        command = (
            f"if [ -e {yaml_path} ] || [ -e {pgm_path} ]; "
            "then exit 20; else exit 21; fi"
        )
        proc = self._make_process("map_preflight", track_active=False)
        self.oneshot_processes.append(proc)
        self.map_preflight_process = proc
        self.map_preflight_path = map_path
        self.log_line.emit(f"[MAP] 检查远端目标: {map_path}")
        proc.start("setsid", self._ssh_args(command))
        return True

    @staticmethod
    def _map_save_command(map_path):
        """Build a staged remote save that restores existing artifacts on failure."""
        prefix = os.path.splitext(map_path)[0]
        parent = os.path.dirname(prefix)
        base = os.path.basename(prefix)
        yaml_path = prefix + ".yaml"
        pgm_path = prefix + ".pgm"
        yaml_name = base + ".yaml"
        pgm_name = base + ".pgm"
        temp_template = os.path.join(parent, f".{base}.save.XXXXXX")

        parent_q = shlex.quote(parent)
        template_q = shlex.quote(temp_template)
        yaml_q = shlex.quote(yaml_path)
        pgm_q = shlex.quote(pgm_path)
        temp_prefix = f'"$tmpdir"/{shlex.quote(base)}'
        temp_yaml = f'"$tmpdir"/{shlex.quote(yaml_name)}'
        temp_pgm = f'"$tmpdir"/{shlex.quote(pgm_name)}'

        return " ".join([
            "set -eu;",
            f"mkdir -p {parent_q};",
            f"tmpdir=$(mktemp -d {template_q});",
            "commit_started=false; committed=false; had_yaml=false; had_pgm=false;",
            "cleanup() { rc=$?; trap - EXIT HUP INT TERM;",
            'if [ "$commit_started" = true ] && [ "$committed" != true ]; then',
            f'if [ "$had_pgm" = true ]; then cp -p -- "$tmpdir/.old.pgm" {pgm_q}; else rm -f -- {pgm_q}; fi;',
            f'if [ "$had_yaml" = true ]; then cp -p -- "$tmpdir/.old.yaml" {yaml_q}; else rm -f -- {yaml_q}; fi;',
            "fi;",
            'rm -rf -- "$tmpdir"; return "$rc"; };',
            "trap cleanup EXIT; trap 'exit 129' HUP; trap 'exit 130' INT; trap 'exit 143' TERM;",
            f'if [ -e {yaml_q} ]; then cp -p -- {yaml_q} "$tmpdir/.old.yaml"; had_yaml=true; fi;',
            f'if [ -e {pgm_q} ]; then cp -p -- {pgm_q} "$tmpdir/.old.pgm"; had_pgm=true; fi;',
            "ros2 run nav2_map_server map_saver_cli",
            f"-t /map -f {temp_prefix} --fmt pgm --mode trinary",
            "--ros-args -p save_map_timeout:=10.0;",
            f"test -s {temp_yaml}; test -s {temp_pgm};",
            f"image=$(sed -n 's/^image:[[:space:]]*//p' {temp_yaml} | head -n 1);",
            f"test \"$image\" = {shlex.quote(pgm_name)};",
            "commit_started=true;",
            f"mv -f -- {temp_pgm} {pgm_q};",
            f"mv -f -- {temp_yaml} {yaml_q};",
            "committed=true;",
        ])

    def save_map(self, map_path):
        if self.map_operation_in_progress():
            self.log_line.emit("[WARN] 地图检查或保存操作已经在进行。")
            return False
        command = self._map_save_command(map_path)
        proc = self._make_process("save_map", track_active=False)
        self.oneshot_processes.append(proc)
        self.map_save_process = proc
        self.map_save_path = map_path
        self.log_line.emit(f"[RUN] save_map: {map_path}")
        proc.start("setsid", self._ssh_args(command))
        return True

    def list_maps(self):
        """Return map YAML paths from the configured remote workspace."""
        maps_dir = os.path.join(self.workspace_path, "maps")
        command = (
            f"find {shlex.quote(maps_dir)} -maxdepth 1 -type f "
            "\\( -name '*.yaml' -o -name '*.yml' \\) -print"
        )
        try:
            result = subprocess.run(
                self._ssh_args(command),
                capture_output=True,
                text=True,
                timeout=8,
                check=False,
            )
        except Exception as exc:
            self.log_line.emit(f"[WARN] 远端地图查询失败: {exc}")
            return []
        if result.returncode != 0:
            message = result.stderr.strip() or f"SSH exit {result.returncode}"
            self.log_line.emit(f"[WARN] 远端地图查询失败: {message}")
            return []
        return sorted(
            line.strip()
            for line in result.stdout.splitlines()
            if line.strip()
        )

    def start_thermal(self):
        if self.thermal_process and self.thermal_process.state() != QProcess.NotRunning:
            self.log_line.emit("[WARN] 热成像/气体节点已经运行。")
            return False
        self._remote_cleanup([
            "sensor_monitor.launch.py", "thermal_camera_node", "gas_sensor_node"
        ])
        proc = self._make_process("thermal", track_active=False)
        self.thermal_process = proc
        proc.start(
            "setsid",
            self._ssh_args(
                "ros2 launch inspection_robot_hardware sensor_monitor.launch.py"
            ),
        )
        self.log_line.emit("[RUN] thermal: inspection_robot_hardware sensor_monitor.launch.py")
        return True

    def stop_thermal(self):
        self._remote_cleanup([
            "sensor_monitor.launch.py", "thermal_camera_node", "gas_sensor_node"
        ])
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
        return proc

    def stop(self):
        if self.active_process is not None and self.active_process.state() != QProcess.NotRunning:
            self._terminate_process(self.active_process)
        for proc in list(self.oneshot_processes):
            if proc.state() != QProcess.NotRunning:
                self._terminate_process(proc)
        self._remote_cleanup(ROBOT_PROCESS_PATTERNS)
        self.active_process = None
        self.active_name = "idle"
        self.map_preflight_process = None
        self.map_save_process = None
        self.map_preflight_path = ""
        self.map_save_path = ""
        self.oneshot_processes = []
        self.state_changed.emit("idle")

    def is_running(self):
        return (
            self.active_process is not None
            and self.active_process.state() != QProcess.NotRunning
        )

    def _remote_command(self, command):
        setup = os.path.join(self.workspace_path, "install", "setup.bash")
        # Services and actions were unreliable when the PC used Fast DDS while
        # the robot used Cyclone DDS, even though simple topics interoperated.
        # Propagate the GUI process' selected RMW so both hosts use one stack.
        rmw_implementation = (
            os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp").strip()
            or "rmw_fastrtps_cpp"
        )
        discovery_range = (
            os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET").strip()
            or "SUBNET"
        )
        return (
            f"source {shlex.quote(self.ros_setup_path)} && "
            f"source {shlex.quote(setup)} && "
            "unset ROS_LOCALHOST_ONLY && "
            "export ROS_DOMAIN_ID=0 "
            f"ROS_AUTOMATIC_DISCOVERY_RANGE={shlex.quote(discovery_range)} "
            f"RMW_IMPLEMENTATION={shlex.quote(rmw_implementation)} && "
            f"cd {shlex.quote(self.workspace_path)} && {command}"
        )

    def _ssh_args(self, command):
        remote_shell = "bash -lc %s" % shlex.quote(self._remote_command(command))
        return [
            "ssh",
            f"{self.remote_user}@{self.remote_host}",
            remote_shell,
        ]

    def _remote_cleanup(self, patterns):
        if not patterns:
            return
        command = " ; ".join(
            f"pkill -TERM -f {shlex.quote(self._self_safe_pkill_pattern(pattern))} || true"
            for pattern in patterns
        )
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
    def _self_safe_pkill_pattern(pattern):
        """Match a target process without matching the cleanup shell itself."""
        escaped = re.escape(str(pattern))
        if not escaped:
            raise ValueError("Process cleanup pattern must not be empty")
        return f"[{escaped[0]}]{escaped[1:]}"

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
        proc.setProperty("captured_output", "")
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(lambda p=proc: self._read_output(p))
        proc.finished.connect(
            lambda code, status, p=proc, n=name, t=track_active: self._finished(p, n, t, code)
        )
        return proc

    def _read_output(self, proc):
        data = bytes(proc.readAllStandardOutput()).decode(errors="replace")
        proc.setProperty(
            "captured_output",
            str(proc.property("captured_output") or "") + data,
        )
        for line in data.splitlines():
            self.log_line.emit(line)

    def _finished(self, proc, name, track_active, code):
        self._read_output(proc)
        output = str(proc.property("captured_output") or "").strip()
        if proc is self.map_preflight_process:
            path = self.map_preflight_path
            self.map_preflight_process = None
            self.map_preflight_path = ""
            if proc in self.oneshot_processes:
                self.oneshot_processes.remove(proc)
            if code == 20:
                self.map_preflight_finished.emit(path, True, "")
            elif code == 21:
                self.map_preflight_finished.emit(path, False, "")
            else:
                message = output.splitlines()[-1] if output else f"SSH exit {code}"
                self.log_line.emit(f"[ERROR] 地图目标检查失败: {message}")
                self.map_preflight_finished.emit(path, False, message)
            return

        if proc is self.map_save_process:
            path = self.map_save_path
            self.map_save_process = None
            self.map_save_path = ""
            if proc in self.oneshot_processes:
                self.oneshot_processes.remove(proc)
            if code == 0:
                self.log_line.emit(f"[MAP] 地图保存完成: {path}")
                self.map_save_finished.emit(path, True, "")
            else:
                message = output.splitlines()[-1] if output else f"SSH exit {code}"
                self.log_line.emit(f"[ERROR] 地图保存失败: {message}")
                self.map_save_finished.emit(path, False, message)
            return

        self.log_line.emit(f"[EXIT] {name} -> {code}")
        if track_active and proc is self.active_process:
            self.active_process = None
            self.active_name = "idle"
            self.state_changed.emit("idle")
        if proc is self.thermal_process:
            self.thermal_process = None
        if proc in self.oneshot_processes:
            self.oneshot_processes.remove(proc)

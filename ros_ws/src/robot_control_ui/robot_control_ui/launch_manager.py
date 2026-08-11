"""SSH launch manager: start/stop robot tasks and thermal monitoring.

Kept separate from the Qt interface so process control can be tested directly.
"""

import os
import shlex
import signal
import subprocess
import threading


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
        self.cyclone_uri = "file:///home/yy/cyclonedds_unicast.xml"

    def _run_remote_cleanup(self, patterns):
        if not patterns:
            return
        safe_patterns = [self._self_safe_pkill_pattern(pattern) for pattern in patterns]
        cleanup_steps = [
            f"pkill -TERM -f -- {shlex.quote(pattern)} || true"
            for pattern in safe_patterns
        ]
        cleanup_steps.append("sleep 1")
        cleanup_steps.extend(
            f"pkill -KILL -f -- {shlex.quote(pattern)} || true"
            for pattern in safe_patterns
        )
        cleanup = " ; ".join(cleanup_steps)
        proc = subprocess.Popen(
            [
                "ssh",
                "-x",
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

    @staticmethod
    def _self_safe_pkill_pattern(pattern):
        if not pattern:
            return pattern
        return f"[{pattern[0]}]{pattern[1:]}"

    def _stop_patterns_for(self, name):
        common = [
            "ros2 launch mapping_bringup mapping.launch.py",
            "ros2 launch mapping_bringup navigation.launch.py",
            "sllidar_node",
            "rf2o_laser_odometry_node",
            "__node:=base_to_laser",
            "__node:=base_to_imu",
            "ybimu_driver",
            "ekf_node",
            "tracked_motor_driver",
            "safety_monitor",
            "velocity_safety_gate",
        ]
        mapping = ["slam_toolbox"]
        navigation = [
            "mission_manager",
            "actual_path_recorder",
            "nav2_amcl",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "behavior_server",
            "smoother_server",
            "velocity_smoother",
            "map_server",
            "lifecycle_manager_navigation",
        ]
        if name == "mapping":
            return common + mapping
        if name == "navigation":
            return common + navigation
        return list(dict.fromkeys(common + mapping + navigation))

    def _build_remote_command(self, command):
        setup_path = os.path.join(self.workspace_path, "install", "setup.bash")
        return (
            "unset DISPLAY WAYLAND_DISPLAY && "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"export CYCLONEDDS_URI={shlex.quote(self.cyclone_uri)} && "
            "export ROS_DOMAIN_ID=0 && "
            "export ROS_LOCALHOST_ONLY=0 && "
            f"source {shlex.quote(self.ros_setup_path)} && "
            f"source {shlex.quote(setup_path)} && "
            f"cd {shlex.quote(self.workspace_path)} && "
            f"{command}"
        )

    def _build_ssh_invocation(self, command):
        remote_command = self._build_remote_command(command)
        return [
            "ssh",
            "-x",
            f"{self.remote_user}@{self.remote_host}",
            f"bash -lc {shlex.quote(remote_command)}",
        ]

    def start(self, name, command):
        if self.active_process and self.active_process.poll() is None:
            self.log_callback(f"[WARN] Stop current task before starting {name}.")
            return False

        self.log_callback(f"[PREP] cleaning stale robot runtime before {name}")
        self._run_remote_cleanup(self._stop_patterns_for("runtime"))

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

    def query_map_files(self, directory, callback, timeout=5):
        """List first-level YAML map files without blocking the GUI thread.

        ``callback`` receives ``(paths, error)``.  Paths are absolute remote
        paths; errors are short, user-facing Chinese messages.
        """
        directory = os.path.normpath(directory or self.workspace_path)

        def worker():
            # Pass the directory as a positional shell argument.  This keeps
            # whitespace and shell metacharacters out of the command itself.
            script = (
                'directory="$1"; '
                '[ -d "$directory" ] || { echo "__MAP_DIR_MISSING__"; exit 3; }; '
                'find "$directory" -mindepth 1 -maxdepth 1 -type f '
                '-name "*.yaml" -print'
            )
            remote_command = shlex.join(
                ["bash", "-c", script, "map-list", directory]
            )
            invocation = [
                "ssh", "-x", "-o", "BatchMode=yes", "-o",
                f"ConnectTimeout={max(1, int(timeout))}",
                f"{self.remote_user}@{self.remote_host}",
                remote_command,
            ]
            try:
                result = subprocess.run(
                    invocation, capture_output=True, text=True,
                    timeout=timeout, check=False,
                )
            except subprocess.TimeoutExpired:
                callback([], "刷新超时：机器人 SSH 无响应")
                return
            except OSError as exc:
                callback([], f"无法执行 SSH：{exc}")
                return

            output = result.stdout or ""
            if result.returncode != 0:
                if "__MAP_DIR_MISSING__" in output:
                    error = f"远端目录不存在：{directory}"
                else:
                    detail = (result.stderr or output).strip().splitlines()
                    error = "SSH 查询失败" + (f"：{detail[-1]}" if detail else "")
                callback([], error)
                return
            paths = sorted(
                line.strip() for line in output.splitlines() if line.strip()
            )
            callback(paths, "")

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

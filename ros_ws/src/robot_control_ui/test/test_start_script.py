import fcntl
import os
from pathlib import Path
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[4]
START_SCRIPT = REPO_ROOT / "start.sh"
START_SOURCE = START_SCRIPT.read_text(encoding="utf-8") if START_SCRIPT.exists() else ""
HOST_ONLY = pytest.mark.skipif(
    not START_SCRIPT.exists(),
    reason="start.sh is a host-console entry point and is not installed on the robot",
)


@HOST_ONLY
def test_start_script_is_valid_and_is_local_console_only():
    subprocess.run(["bash", "-n", str(START_SCRIPT)], check=True)
    assert 'cd "${WORKSPACE}"' in START_SOURCE
    assert "colcon build --packages-select" in START_SOURCE
    assert "flock -n 9" in START_SOURCE
    assert "trap cleanup EXIT INT TERM HUP" in START_SOURCE
    assert "ros2 launch robot_control_ui ui.launch.py" in START_SOURCE
    assert 'rviz2 -d "${RVIZ_CONFIG}"' in START_SOURCE
    assert "mapping.launch.py" not in START_SOURCE
    assert "navigation.launch.py" not in START_SOURCE
    assert "ssh " not in START_SOURCE


@HOST_ONLY
def test_start_script_rejects_a_second_instance_before_building():
    lock_path = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / (
        f"ros_project_control_{os.getuid()}.lock"
    )
    with lock_path.open("w", encoding="utf-8") as lock_file:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            # A real start.sh instance already provides the lock for this contract check.
            pass
        result = subprocess.run(
            [str(START_SCRIPT)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    assert result.returncode == 1
    assert "already running" in result.stdout

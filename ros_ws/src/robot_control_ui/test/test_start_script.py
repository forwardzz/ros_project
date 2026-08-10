import fcntl
import os
from pathlib import Path
import re
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
    # dynamic robot-IP detection & robot-side CycloneDDS sync
    assert "robot_ip_detect.py" in START_SOURCE
    assert 'REMOTE_HOST="${REMOTE_HOST:-__AUTO__}"' in START_SOURCE
    assert "ssh -o BatchMode=yes" in START_SOURCE
    assert "generate_cyclone_xml" in START_SOURCE
    # build-choice prompt: interactive default-build, SKIP_BUILD override
    assert 'read -r -p "[BUILD]' in START_SOURCE
    assert "SKIP_BUILD" in START_SOURCE
    assert 'BUILD_CHOICE="build"' in START_SOURCE


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


def _render_cyclone_xml(local_ip="192.168.43.20", peer_ip="192.168.43.30"):
    """Execute the generate_cyclone_xml function from start.sh and return its XML."""
    match = re.search(r"^generate_cyclone_xml\(\) \{(.*?)^\}$", START_SOURCE, re.M | re.S)
    assert match, "generate_cyclone_xml function not found in start.sh"
    func = match.group(0)
    script = func + "\ngenerate_cyclone_xml %s %s\n" % (local_ip, peer_ip)
    proc = subprocess.run(["bash", "-c", script], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


@HOST_ONLY
def test_generated_cyclone_xml_contains_both_peers():
    xml = _render_cyclone_xml()
    assert '<Peer address="192.168.43.20"/>' in xml
    assert '<Peer address="192.168.43.30"/>' in xml
    # exactly two peer entries
    assert xml.count("<Peer address=") == 2


@HOST_ONLY
def test_generated_cyclone_xml_participant_settings():
    xml = _render_cyclone_xml()
    assert "<ParticipantIndex>auto</ParticipantIndex>" in xml
    assert "<MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>" in xml
    assert "<AllowMulticast>false</AllowMulticast>" in xml
    assert "<NetworkInterface address=" in xml


@HOST_ONLY
def test_generated_cyclone_xml_ports_symmetric_local_and_remote():
    local = _render_cyclone_xml(local_ip="192.168.43.20", peer_ip="192.168.43.30")
    remote = _render_cyclone_xml(local_ip="192.168.43.30", peer_ip="192.168.43.20")
    for token in (
        "<Base>7460</Base>",
        "<DomainGain>250</DomainGain>",
        "<ParticipantGain>2</ParticipantGain>",
        "<UnicastMetaOffset>10</UnicastMetaOffset>",
        "<UnicastDataOffset>11</UnicastDataOffset>",
    ):
        assert token in local
        assert token in remote
    # remote config is the mirror image
    assert '<Peer address="192.168.43.30"/>' in remote
    assert '<Peer address="192.168.43.20"/>' in remote
    assert '<NetworkInterface address="192.168.43.30"/>' in remote

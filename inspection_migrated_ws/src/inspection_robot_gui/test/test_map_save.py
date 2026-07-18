import math
import re
import shlex
import subprocess

import pytest
from PyQt5.QtWidgets import QMessageBox

from inspection_robot_gui.launch_manager import LaunchManager
from inspection_robot_gui.main_window import (
    MAP_SAVE_MAX_AGE_SEC,
    MainWindow,
    map_save_block_reason,
    normalize_remote_map_path,
)


WORKSPACE = "/home/yy/inspection_migrated_ws"


def test_remote_map_path_is_confined_and_normalized():
    assert normalize_remote_map_path(WORKSPACE, "") == (
        f"{WORKSPACE}/maps/inspection_map.yaml"
    )
    assert normalize_remote_map_path(WORKSPACE, "floor one") == (
        f"{WORKSPACE}/maps/floor one.yaml"
    )
    assert normalize_remote_map_path(
        WORKSPACE, f"{WORKSPACE}/maps/floor_one.yml"
    ) == f"{WORKSPACE}/maps/floor_one.yaml"


@pytest.mark.parametrize(
    "path",
    (
        "/home/zjy/local_map.yaml",
        "/home/yy/inspection_migrated_ws/other/map.yaml",
        "../outside.yaml",
        "map.pgm",
        ".yaml",
    ),
)
def test_remote_map_path_rejects_unsafe_or_non_yaml_targets(path):
    with pytest.raises(ValueError):
        normalize_remote_map_path(WORKSPACE, path)


def test_map_save_requires_active_mapping_and_fresh_map():
    assert "启动建图" in map_save_block_reason(False, "idle", 0.1)
    assert "启动建图" in map_save_block_reason(True, "navigation", 0.1)
    assert "新鲜" in map_save_block_reason(True, "mapping", math.inf)
    assert "新鲜" in map_save_block_reason(
        True, "mapping", MAP_SAVE_MAX_AGE_SEC + 0.01
    )
    assert map_save_block_reason(True, "mapping", 0.1) == ""


def test_staged_save_command_quotes_paths_and_validates_artifacts():
    path = f"{WORKSPACE}/maps/floor one.yaml"
    command = LaunchManager._map_save_command(path)

    assert "mktemp -d" in command
    assert "save_map_timeout:=10.0" in command
    assert "--fmt pgm --mode trinary" in command
    assert "test -s" in command
    assert "commit_started=true" in command
    assert "committed=true" in command
    assert ".old.yaml" in command
    assert ".old.pgm" in command
    assert subprocess.run(
        ["bash", "-n", "-c", command], check=False
    ).returncode == 0


def _replace_map_saver_with_fixture(command, base):
    replacement = " ".join((
        f"printf 'image: {base}.pgm\\nmode: trinary\\n' > \"$tmpdir\"/{base}.yaml;",
        f"printf 'P5\\n1 1\\n255\\n0' > \"$tmpdir\"/{base}.pgm;",
    ))
    replaced, count = re.subn(
        r"ros2 run nav2_map_server map_saver_cli .*?"
        r"--ros-args -p save_map_timeout:=10\.0;",
        replacement,
        command,
    )
    assert count == 1
    return replaced


def test_staged_save_commits_validated_pair(tmp_path):
    target = tmp_path / "map.yaml"
    command = LaunchManager._map_save_command(str(target))
    command = _replace_map_saver_with_fixture(command, "map")

    result = subprocess.run(["bash", "-c", command], check=False)

    assert result.returncode == 0
    assert target.read_text().startswith("image: map.pgm")
    assert (tmp_path / "map.pgm").read_text().startswith("P5")
    assert list(tmp_path.glob(".map.save.*")) == []


def test_staged_save_restores_old_pair_when_commit_fails(tmp_path):
    target = tmp_path / "map.yaml"
    image = tmp_path / "map.pgm"
    target.write_text("image: map.pgm\nold yaml\n")
    image.write_text("old pgm")
    command = LaunchManager._map_save_command(str(target))
    command = _replace_map_saver_with_fixture(command, "map")
    final_yaml_move = (
        f'mv -f -- "$tmpdir"/map.yaml {shlex.quote(str(target))};'
    )
    assert final_yaml_move in command
    command = command.replace(final_yaml_move, "false;")

    result = subprocess.run(["bash", "-c", command], check=False)

    assert result.returncode != 0
    assert target.read_text() == "image: map.pgm\nold yaml\n"
    assert image.read_text() == "old pgm"
    assert list(tmp_path.glob(".map.save.*")) == []


def test_map_preflight_checks_both_yaml_and_pgm(monkeypatch):
    manager = LaunchManager(workspace_path=WORKSPACE)
    starts = []

    class Process:
        def start(self, program, arguments):
            starts.append((program, arguments))

        def state(self):
            return 0

    monkeypatch.setattr(manager, "_make_process", lambda *_args, **_kwargs: Process())

    path = f"{WORKSPACE}/maps/floor one.yaml"
    assert manager.check_map_exists(path)
    assert len(starts) == 1
    remote_shell = starts[0][1][-1]
    assert "floor one.yaml" in remote_shell
    assert "floor one.pgm" in remote_shell
    assert "exit 20" in remote_shell
    assert "exit 21" in remote_shell


def test_existing_map_requires_confirmation_before_save(monkeypatch):
    calls = []

    class Button:
        def setEnabled(self, enabled):
            calls.append(("enabled", enabled))

    class Manager:
        def save_map(self, path):
            calls.append(("save", path))
            return True

    class Harness:
        _map_preflight_finished = MainWindow._map_preflight_finished
        _clear_pending_map_save = MainWindow._clear_pending_map_save

        def append_log(self, line):
            calls.append(("log", line))

    monkeypatch.setattr(QMessageBox, "question", lambda *_args, **_kwargs: QMessageBox.No)
    window = Harness()
    window.launch_manager = Manager()
    window.save_map_button = Button()
    window._pending_map_save_path = f"{WORKSPACE}/maps/existing.yaml"

    window._map_preflight_finished(window._pending_map_save_path, True, "")

    assert not any(call[0] == "save" for call in calls)
    assert window._pending_map_save_path == ""
    assert ("enabled", True) in calls


def test_new_map_starts_save_without_overwrite_prompt(monkeypatch):
    calls = []

    class Button:
        def setEnabled(self, enabled):
            calls.append(("enabled", enabled))

    class Manager:
        def save_map(self, path):
            calls.append(("save", path))
            return True

    class Harness:
        _map_preflight_finished = MainWindow._map_preflight_finished
        _clear_pending_map_save = MainWindow._clear_pending_map_save

    monkeypatch.setattr(
        QMessageBox,
        "question",
        lambda *_args, **_kwargs: pytest.fail("new targets must not prompt"),
    )
    window = Harness()
    window.launch_manager = Manager()
    window.save_map_button = Button()
    window._pending_map_save_path = f"{WORKSPACE}/maps/new.yaml"

    window._map_preflight_finished(window._pending_map_save_path, False, "")

    assert calls == [("save", f"{WORKSPACE}/maps/new.yaml")]
    assert window._pending_map_save_path.endswith("/new.yaml")

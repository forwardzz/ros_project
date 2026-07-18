from types import SimpleNamespace

from inspection_robot_gui.main_window import MainWindow


class FakeSettings:
    def __init__(self):
        self.values = {}

    def setValue(self, key, value):
        self.values[key] = value


class FakeRosAdapter:
    def __init__(self):
        self.start_navigation_client = object()
        self.calls = []

    def call_service_async(self, client, request, callback, timeout_sec):
        self.calls.append((client, request, callback, timeout_sec))


def test_start_mission_propagates_and_persists_return_option():
    ros = FakeRosAdapter()
    settings = FakeSettings()
    window = SimpleNamespace(
        waypoint_pause_spin=SimpleNamespace(value=lambda: 1.5),
        return_to_start_check=SimpleNamespace(isChecked=lambda: True),
        settings=settings,
        ros=ros,
        append_log=lambda _message: None,
        _emit_service_result=lambda *_args: None,
    )

    MainWindow.start_mission(window)

    assert len(ros.calls) == 1
    client, request, _callback, timeout_sec = ros.calls[0]
    assert client is ros.start_navigation_client
    assert request.waypoint_pause_sec == 1.5
    assert request.return_to_start is True
    assert timeout_sec == 10.0
    assert settings.values == {
        "waypoint_pause_sec": 1.5,
        "return_to_start": "true",
    }

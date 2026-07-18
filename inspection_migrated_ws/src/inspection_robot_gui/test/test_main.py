import signal

from inspection_robot_gui.main import _install_shutdown_handlers


def test_terminal_signal_runs_cleanup_once_and_quits():
    calls = []
    handlers = {}

    class LaunchManager:
        def stop_thermal(self):
            calls.append("stop_thermal")

        def stop(self):
            calls.append("stop")

    class Window:
        launch_manager = LaunchManager()

    class Ros:
        def shutdown(self):
            calls.append("ros_shutdown")

    class App:
        def quit(self):
            calls.append("app_quit")

    def register_signal(signum, handler):
        handlers[signum] = handler

    def schedule(delay_ms, callback):
        assert delay_ms == 0
        callback()

    _install_shutdown_handlers(
        App(),
        Window(),
        Ros(),
        register_signal=register_signal,
        schedule=schedule,
    )

    assert signal.SIGINT in handlers
    assert signal.SIGTERM in handlers
    handlers[signal.SIGINT](signal.SIGINT, None)
    handlers[signal.SIGTERM](signal.SIGTERM, None)

    assert calls == ["stop_thermal", "stop", "ros_shutdown", "app_quit"]

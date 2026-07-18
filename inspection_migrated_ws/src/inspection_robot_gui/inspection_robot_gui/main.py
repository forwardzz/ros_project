import signal
import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication

from .main_window import GuiSignals, MainWindow
from .ros_adapter import RosAdapter


def _configure_high_dpi():
    """Enable Qt 5 logical-pixel scaling before QApplication is created."""
    if QApplication.instance() is not None:
        return
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)


def _install_shutdown_handlers(
    app,
    window,
    ros,
    register_signal=signal.signal,
    schedule=QTimer.singleShot,
):
    """Turn terminal signals into one orderly GUI/robot cleanup request."""
    state = {"started": False}

    def shutdown():
        if state["started"]:
            return
        state["started"] = True
        cleanup_steps = (
            window.launch_manager.stop_thermal,
            window.launch_manager.stop,
            ros.shutdown,
        )
        for cleanup in cleanup_steps:
            try:
                cleanup()
            except Exception as exc:
                print(f"GUI shutdown warning: {exc}", file=sys.stderr)
        app.quit()

    def request_shutdown(_signum, _frame):
        # Python invokes the signal handler on the main thread. Queue cleanup
        # so it does not interrupt a Qt timer callback midway through an update.
        schedule(0, shutdown)

    register_signal(signal.SIGINT, request_shutdown)
    register_signal(signal.SIGTERM, request_shutdown)
    return shutdown


def main():
    _configure_high_dpi()
    app = QApplication(sys.argv)
    signals = GuiSignals()
    ros = RosAdapter(
        status_callback=signals.log,
        feedback_callback=signals.nav_feedback,
        result_callback=signals.nav_result,
        mission_status_callback=signals.mission_status,
    )
    window = MainWindow(ros, signals)
    window.show()
    _install_shutdown_handlers(app, window, ros)
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

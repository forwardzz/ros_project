import sys

from PyQt5.QtWidgets import QApplication

from .main_window import GuiSignals, MainWindow
from .ros_adapter import RosAdapter


def main():
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
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

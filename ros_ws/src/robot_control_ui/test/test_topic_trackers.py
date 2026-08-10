"""Runtime tests for RosUiAdapter topic-tracker wiring.

Catches the "tests passed but runtime AttributeError" gap: callbacks must
find ``topic_trackers`` and update the same object the UI reads.
"""

# Uses the installed robot_control_ui package (colcon install/setup.bash).


def test_ros_adapter_creates_trackers_before_callbacks():
    from robot_control_ui.robot_control_ui import RosUiAdapter

    adapter = RosUiAdapter(args=[])
    try:
        expected = {
            "/scan", "/odom", "/map", "/amcl_pose",
            "/robot_safety_status", "/mission_status_typed",
        }
        assert expected.issubset(set(adapter.topic_trackers))
    finally:
        adapter.shutdown()


def test_scan_callback_updates_shared_tracker():
    from sensor_msgs.msg import LaserScan

    from robot_control_ui.robot_control_ui import RosUiAdapter

    adapter = RosUiAdapter(args=[])
    try:
        msg = LaserScan()
        msg.ranges = [0.1, 0.2, 0.3]
        adapter._scan_cb(msg)  # must not raise AttributeError
        assert adapter.last_scan_stamp > 0
        tracker = adapter.topic_trackers["/scan"]
        assert tracker.received is True
        assert tracker.last_stamp == adapter.last_scan_stamp
        assert tracker.publishers >= 0
    finally:
        adapter.shutdown()


def test_odom_callback_updates_tracker():
    from nav_msgs.msg import Odometry

    from robot_control_ui.robot_control_ui import RosUiAdapter

    adapter = RosUiAdapter(args=[])
    try:
        adapter._odom_cb(Odometry())
        assert adapter.topic_trackers["/odom"].received is True
        assert adapter.last_odom_stamp > 0
    finally:
        adapter.shutdown()

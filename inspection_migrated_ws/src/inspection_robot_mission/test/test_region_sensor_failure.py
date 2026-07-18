from inspection_robot_mission.mission_manager import MissionManager


class FakeMissionManager:
    def __init__(self, failure_count=0):
        self.tf_consecutive_failures = failure_count
        self.stop_count = 0
        self.skipped_reasons = []

    def _publish_zero_cmd(self):
        self.stop_count += 1

    def _skip_current_region(self, reason):
        self.skipped_reasons.append(reason)


def test_region_sensor_failure_stops_immediately():
    manager = FakeMissionManager()

    skipped = MissionManager._handle_region_sensor_failure(manager, "laser timeout")

    assert manager.stop_count == 1
    assert manager.tf_consecutive_failures == 1
    assert manager.skipped_reasons == []
    assert skipped is False


def test_region_sensor_failure_skips_after_five_failures():
    manager = FakeMissionManager(failure_count=4)

    skipped = MissionManager._handle_region_sensor_failure(
        manager, "map to base_link transform unavailable"
    )

    assert manager.stop_count == 1
    assert manager.tf_consecutive_failures == 5
    assert manager.skipped_reasons == ["map to base_link transform unavailable"]
    assert skipped is True

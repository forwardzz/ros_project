import math

from mapping_bringup.velocity_safety_gate import VelocitySafetyState


def healthy_state(now=10.0):
    state = VelocitySafetyState()
    state.safety_stop = False
    state.heartbeat_time = now
    state.scan_time = now
    return state


def test_gate_is_zero_without_scan_heartbeat_or_safety_state():
    state = VelocitySafetyState()
    state.set_command("teleop", 0.1, 0.2, 10.0)
    assert state.output(10.0) == (0.0, 0.0)


def test_teleop_has_priority_and_values_are_limited():
    state = healthy_state()
    state.set_command("auto", 0.08, 0.2, 10.0)
    state.set_command("teleop", 0.5, -1.0, 10.0)
    assert state.output(10.0) == (0.18, -0.55)


def test_auto_forwards_after_teleop_timeout_then_stops_on_scan_timeout():
    state = healthy_state()
    state.set_command("teleop", 0.12, 0.0, 9.0)
    state.set_command("auto", 0.08, 0.2, 10.0)
    assert state.output(10.0) == (0.08, 0.2)
    assert state.output(10.51) == (0.0, 0.0)


def test_fault_estop_invalid_number_and_duplicate_gate_are_zero():
    state = healthy_state()
    state.set_command("auto", math.nan, 0.1, 10.0)
    assert state.output(10.0) == (0.0, 0.0)
    state.set_command("auto", 0.1, 0.1, 10.0)
    state.safety_stop = True
    assert state.output(10.0) == (0.0, 0.0)
    state.safety_stop = False
    state.software_estop = True
    assert state.output(10.0) == (0.0, 0.0)
    state.software_estop = False
    assert state.output(10.0, duplicate_gate=True) == (0.0, 0.0)

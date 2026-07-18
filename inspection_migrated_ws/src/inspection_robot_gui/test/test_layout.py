from inspection_robot_gui.main_window import recommended_window_size


def test_window_size_fits_this_laptops_logical_resolution():
    assert recommended_window_size(1440, 900) == (1200, 720)


def test_window_size_preserves_margins_on_common_small_screen():
    assert recommended_window_size(1366, 768) == (1200, 691)


def test_window_size_never_exceeds_available_geometry():
    assert recommended_window_size(800, 600) == (800, 600)

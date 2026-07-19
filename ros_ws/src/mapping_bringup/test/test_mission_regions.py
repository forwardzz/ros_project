from mapping_bringup.mission_regions import (
    InspectionRegion,
    generate_chassis_region_paths,
    regions_from_yaml,
    regions_to_yaml_data,
)
from mapping_bringup.mission_manager import MissionManager
from nav_msgs.msg import OccupancyGrid
from types import SimpleNamespace


def test_region_paths_preserve_region_creation_order_and_boustrophedon_order():
    regions = [
        InspectionRegion("FIRST", 0.0, 0.0, 1.0, 0.8),
        InspectionRegion("SECOND", 2.0, 0.0, 3.0, 0.8),
    ]
    paths = generate_chassis_region_paths(regions, 0.10, 0.23)
    assert len(paths) == 2
    assert paths[0] and paths[1]
    assert all(point.point_name.startswith("FIRST_") for point in paths[0])
    assert all(point.point_name.startswith("SECOND_") for point in paths[1])
    assert paths[0][0].x < paths[0][1].x
    assert paths[0][2].x > paths[0][3].x


def test_version_2_round_trip_and_version_1_compatibility():
    regions = [InspectionRegion("AREA_A", -1.0, -2.0, 3.0, 4.0)]
    data = regions_to_yaml_data(regions, 0.10, 0.23)
    assert data["version"] == 2
    assert regions_from_yaml(data) == regions

    legacy = dict(data)
    legacy["version"] = 1
    assert regions_from_yaml(legacy) == regions


def test_too_small_region_generates_no_chassis_path():
    region = InspectionRegion("SMALL", 0.0, 0.0, 0.40, 0.40)
    assert generate_chassis_region_paths([region], 0.10, 0.23) == [[]]


def make_map(fill=0):
    msg = OccupancyGrid()
    msg.info.width = 100
    msg.info.height = 100
    msg.info.resolution = 0.05
    msg.info.origin.position.x = -2.0
    msg.info.origin.position.y = -2.0
    msg.data = [fill] * (msg.info.width * msg.info.height)
    return msg


def test_region_staging_is_outside_region_and_blocked_region_is_skipped():
    region = InspectionRegion("AREA", 0.0, 0.0, 1.0, 0.8)
    route = generate_chassis_region_paths([region], 0.10, 0.23)[0]
    manager = SimpleNamespace(
        map_msg=make_map(),
        region_margin=0.23,
        region_staging_distance=0.20,
        current_map_pose={"x": -1.0, "y": 0.0},
    )
    selected_route, staging = MissionManager._select_region_staging(
        manager, region, route
    )
    assert selected_route
    assert not (
        region.min_x <= staging.x <= region.max_x
        and region.min_y <= staging.y <= region.max_y
    )

    manager.map_msg = make_map(fill=100)
    assert MissionManager._select_region_staging(manager, region, route) is None

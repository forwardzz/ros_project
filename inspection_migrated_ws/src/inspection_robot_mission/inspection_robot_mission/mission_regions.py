from dataclasses import dataclass
import math

from .ros_utils import make_inspection_point


@dataclass(frozen=True)
class InspectionRegion:
    name: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float


@dataclass(frozen=True)
class RegionGenerationResult:
    points: list
    first_error: str | None
    warnings: list


DEFAULT_STRAIGHT_RESOLUTION = 0.05
DEFAULT_ARC_RESOLUTION = 0.02


def generate_region_points(
    regions,
    sweep_spacing,
    region_margin,
    straight_resolution=DEFAULT_STRAIGHT_RESOLUTION,
    arc_resolution=DEFAULT_ARC_RESOLUTION,
):
    generated = []
    warnings = []
    first_error = None
    for region in regions:
        region_generated = generate_coverage_path_for_region(
            region,
            sweep_spacing,
            region_margin,
            straight_resolution,
            arc_resolution,
        )
        if region_generated:
            generated.extend(region_generated)
        if not region_generated:
            message = (
                f"{region.name} is too small for spacing={sweep_spacing:.2f}m "
                f"and margin={region_margin:.2f}m"
            )
            first_error = first_error or message
            warnings.append(message)

    assign_path_headings(generated)
    return RegionGenerationResult(generated, first_error, warnings)


def generate_region_paths(
    regions,
    sweep_spacing,
    region_margin,
    straight_resolution=DEFAULT_STRAIGHT_RESOLUTION,
    arc_resolution=DEFAULT_ARC_RESOLUTION,
):
    return [
        generate_coverage_path_for_region(
            region,
            sweep_spacing,
            region_margin,
            straight_resolution,
            arc_resolution,
        )
        for region in regions
    ]


def generate_chassis_path_for_region(region, sweep_spacing, region_margin):
    points = [
        make_inspection_point(f"{region.name}_P{index}", x, y, 0.0)
        for index, (x, y) in enumerate(
            generate_points_for_region(region, sweep_spacing, region_margin), start=1
        )
    ]
    assign_path_headings(points)
    return points


def generate_chassis_region_paths(regions, sweep_spacing, region_margin):
    return [
        generate_chassis_path_for_region(region, sweep_spacing, region_margin)
        for region in regions
    ]


def _append_line(samples, start, end, resolution, kind):
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(distance / max(resolution, 0.01))))
    for index in range(count + 1):
        if samples and index == 0:
            continue
        ratio = index / count
        samples.append(
            (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
                kind,
            )
        )


def _append_arc(samples, center, radius, start_angle, end_angle, resolution):
    arc_length = abs(end_angle - start_angle) * radius
    count = max(3, int(math.ceil(arc_length / max(resolution, 0.005))))
    for index in range(1, count + 1):
        ratio = index / count
        angle = start_angle + (end_angle - start_angle) * ratio
        samples.append(
            (
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
                "turn",
            )
        )


def generate_coverage_path_for_region(
    region,
    sweep_spacing,
    region_margin,
    straight_resolution=DEFAULT_STRAIGHT_RESOLUTION,
    arc_resolution=DEFAULT_ARC_RESOLUTION,
):
    """Generate dense long-axis sweeps joined by tangent semicircular U-turns."""
    min_x = region.min_x + region_margin
    min_y = region.min_y + region_margin
    max_x = region.max_x - region_margin
    max_y = region.max_y - region_margin
    if min_x >= max_x or min_y >= max_y:
        return []

    spacing = max(float(sweep_spacing), 0.05)
    samples = []
    if (max_x - min_x) >= (max_y - min_y):
        lanes = sweep_positions(min_y, max_y, spacing)
        if not lanes:
            return []
        max_radius = max((lanes[i + 1] - lanes[i]) * 0.5 for i in range(len(lanes) - 1)) if len(lanes) > 1 else 0.0
        if max_x - min_x <= max_radius * 2.0 + 0.02:
            return []
        left = min_x + max_radius
        right = max_x - max_radius
        for lane_index, y in enumerate(lanes):
            start, end = ((left, y), (right, y)) if lane_index % 2 == 0 else ((right, y), (left, y))
            _append_line(samples, start, end, straight_resolution, "straight")
            if lane_index + 1 >= len(lanes):
                continue
            next_y = lanes[lane_index + 1]
            radius = (next_y - y) * 0.5
            center_x = right if lane_index % 2 == 0 else left
            center_y = y + radius
            if lane_index % 2 == 0:
                _append_arc(samples, (center_x, center_y), radius, -math.pi / 2.0, math.pi / 2.0, arc_resolution)
            else:
                _append_arc(samples, (center_x, center_y), radius, -math.pi / 2.0, -3.0 * math.pi / 2.0, arc_resolution)
    else:
        lanes = sweep_positions(min_x, max_x, spacing)
        if not lanes:
            return []
        max_radius = max((lanes[i + 1] - lanes[i]) * 0.5 for i in range(len(lanes) - 1)) if len(lanes) > 1 else 0.0
        if max_y - min_y <= max_radius * 2.0 + 0.02:
            return []
        bottom = min_y + max_radius
        top = max_y - max_radius
        for lane_index, x in enumerate(lanes):
            start, end = ((x, bottom), (x, top)) if lane_index % 2 == 0 else ((x, top), (x, bottom))
            _append_line(samples, start, end, straight_resolution, "straight")
            if lane_index + 1 >= len(lanes):
                continue
            next_x = lanes[lane_index + 1]
            radius = (next_x - x) * 0.5
            center_x = x + radius
            center_y = top if lane_index % 2 == 0 else bottom
            if lane_index % 2 == 0:
                _append_arc(samples, (center_x, center_y), radius, math.pi, 0.0, arc_resolution)
            else:
                _append_arc(samples, (center_x, center_y), radius, math.pi, 2.0 * math.pi, arc_resolution)

    points = [
        make_inspection_point(f"{region.name}_P{index}", x, y, 0.0)
        for index, (x, y, _kind) in enumerate(samples, start=1)
    ]
    assign_path_headings(points)
    return points


def generate_points_for_region(region, sweep_spacing, region_margin):
    min_x = region.min_x + region_margin
    min_y = region.min_y + region_margin
    max_x = region.max_x - region_margin
    max_y = region.max_y - region_margin
    if min_x > max_x or min_y > max_y:
        return []

    width = max_x - min_x
    height = max_y - min_y
    spacing = max(sweep_spacing, 0.05)

    points = []
    if width >= height:
        rows = sweep_positions(min_y, max_y, spacing)
        for row_index, y in enumerate(rows):
            if row_index % 2 == 0:
                points.append((min_x, y))
                if width > 0.02:
                    points.append((max_x, y))
            else:
                points.append((max_x, y))
                if width > 0.02:
                    points.append((min_x, y))
    else:
        columns = sweep_positions(min_x, max_x, spacing)
        for col_index, x in enumerate(columns):
            if col_index % 2 == 0:
                points.append((x, min_y))
                if height > 0.02:
                    points.append((x, max_y))
            else:
                points.append((x, max_y))
                if height > 0.02:
                    points.append((x, min_y))
    return points


def sweep_positions(start, end, spacing):
    if start > end:
        return []
    positions = []
    value = start
    while value <= end + 1e-9:
        positions.append(value)
        value += spacing
    if not positions or end - positions[-1] > min(spacing * 0.5, 0.10):
        positions.append(end)
    return positions


def assign_path_headings(points):
    for index, point in enumerate(points):
        target = None
        source = point
        if index + 1 < len(points):
            target = points[index + 1]
        elif index > 0:
            target = point
            source = points[index - 1]
        if target is None:
            continue
        dx = target.x - source.x
        dy = target.y - source.y
        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            points[index].theta = math.atan2(dy, dx)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def transition_offset_for_margin(region_margin):
    return min(0.15, max(0.0, region_margin * 0.8))


def transition_point_is_useful(x, y, p, q, nxt, effective_offset, requested_offset):
    min_effective_offset = min(0.01, requested_offset * 0.5)
    if effective_offset < min_effective_offset:
        return False

    for point in (p, q, nxt):
        if math.hypot(x - point.x, y - point.y) < 0.02:
            return False
    return True


def smooth_region_sweep(points, region, region_margin):
    if len(points) < 3:
        return points
    safe_min_x = region.min_x + region_margin
    safe_min_y = region.min_y + region_margin
    safe_max_x = region.max_x - region_margin
    safe_max_y = region.max_y - region_margin
    if safe_min_x > safe_max_x or safe_min_y > safe_max_y:
        return points

    transition_offset = transition_offset_for_margin(region_margin)
    if transition_offset <= 1e-6:
        return points

    result = []
    for i in range(len(points)):
        p = points[i]
        result.append(p)
        if i + 2 >= len(points):
            continue
        q = points[i + 1]
        nxt = points[i + 2]
        dx = q.x - p.x
        dy = q.y - p.y
        if abs(dx) < 1e-3 and abs(dy) > 0.02:
            nrow_dir = 1.0 if nxt.x > q.x else -1.0
            out_dir = -nrow_dir
            x = clamp(q.x + out_dir * transition_offset, safe_min_x, safe_max_x)
            y = clamp(p.y + dy * 0.5, safe_min_y, safe_max_y)
            effective_offset = abs(x - q.x)
            if not transition_point_is_useful(
                x, y, p, q, nxt, effective_offset, transition_offset
            ):
                continue
            mid = make_inspection_point(
                "TR_{}".format(p.point_name),
                x,
                y,
                0.0,
            )
            result.append(mid)
        elif abs(dy) < 1e-3 and abs(dx) > 0.02:
            nrow_dir = 1.0 if nxt.y > q.y else -1.0
            out_dir = -nrow_dir
            x = clamp(p.x + dx * 0.5, safe_min_x, safe_max_x)
            y = clamp(q.y + out_dir * transition_offset, safe_min_y, safe_max_y)
            effective_offset = abs(y - q.y)
            if not transition_point_is_useful(
                x, y, p, q, nxt, effective_offset, transition_offset
            ):
                continue
            mid = make_inspection_point(
                "TR_{}".format(p.point_name),
                x,
                y,
                0.0,
            )
            result.append(mid)
    return result


def regions_to_yaml_data(regions, sweep_spacing, region_margin):
    return {
        "version": 2,
        "map_frame": "map",
        "sweep_spacing": sweep_spacing,
        "region_margin": region_margin,
        "regions": [
            {
                "name": region.name,
                "min_x": region.min_x,
                "min_y": region.min_y,
                "max_x": region.max_x,
                "max_y": region.max_y,
            }
            for region in regions
        ],
    }


def regions_from_yaml(data):
    if int(data.get("version", 1)) not in (1, 2):
        raise ValueError("unsupported inspection region file version")
    if data.get("map_frame", "map") != "map":
        raise ValueError("inspection region file must use map_frame=map")

    regions = []
    for index, item in enumerate(data.get("regions", []), start=1):
        min_x = float(item["min_x"])
        min_y = float(item["min_y"])
        max_x = float(item["max_x"])
        max_y = float(item["max_y"])
        regions.append(
            InspectionRegion(
                name=str(item.get("name") or f"REGION_{index}"),
                min_x=min(min_x, max_x),
                min_y=min(min_y, max_y),
                max_x=max(min_x, max_x),
                max_y=max(min_y, max_y),
            )
        )
    return regions

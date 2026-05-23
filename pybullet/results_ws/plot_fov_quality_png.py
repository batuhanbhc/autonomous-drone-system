"""
Render the current FOV-quality reward geometry to a PNG using only Python's
standard library.

This avoids runtime dependencies on numpy/matplotlib in minimal shells.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import zlib
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.camera_geometry import (
    footprint_corners_world,
    footprint_forward_extents,
    lateral_half_width_at_forward_distance,
    principal_point_world,
    ray_polygon_intersection_distance,
    world_to_drone_local,
)


# Keep this script stdlib-only. These defaults mirror the current shared config.
DEFAULT_DRONE_Z = 6.0
DEFAULT_TILT_DEG = 55.0
DEFAULT_HORIZONTAL_FOV_DEG = 55.8
DEFAULT_VERTICAL_FOV_DEG = 43.3
DEFAULT_REWARD_QUALITY_MODE = "principal_top_corner_linear"
DEFAULT_REWARD_QUALITY_GAMMA = 1.5
DEFAULT_COVERAGE_EDGE_QUALITY = 0.0


FONT_3X5 = {
    "0": ["111", "101", "101", "101", "111"],
    "1": ["010", "110", "010", "010", "111"],
    "2": ["111", "001", "111", "100", "111"],
    "3": ["111", "001", "111", "001", "111"],
    "4": ["101", "101", "111", "001", "001"],
    "5": ["111", "100", "111", "001", "111"],
    "6": ["111", "100", "111", "101", "111"],
    "7": ["111", "001", "001", "001", "001"],
    "8": ["111", "101", "111", "101", "111"],
    "9": ["111", "101", "111", "001", "111"],
    ".": ["000", "000", "000", "000", "010"],
    "=": ["000", "111", "000", "111", "000"],
    "L": ["100", "100", "100", "100", "111"],
    "R": ["110", "101", "110", "101", "101"],
    "q": ["000", "111", "101", "111", "001"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default=str(
            Path(__file__).resolve().parent
            / "fov_quality_plots"
            / "fov_quality_reward.png"
        ),
    )
    parser.add_argument("--width", type=int, default=1200)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--drone_x", type=float, default=0.0)
    parser.add_argument("--drone_y", type=float, default=0.0)
    parser.add_argument("--drone_z", type=float, default=DEFAULT_DRONE_Z)
    parser.add_argument("--yaw_deg", type=float, default=0.0)
    parser.add_argument("--tilt_deg", type=float, default=DEFAULT_TILT_DEG)
    parser.add_argument(
        "--horizontal_fov_deg",
        type=float,
        default=DEFAULT_HORIZONTAL_FOV_DEG,
    )
    parser.add_argument(
        "--vertical_fov_deg",
        type=float,
        default=DEFAULT_VERTICAL_FOV_DEG,
    )
    parser.add_argument(
        "--reward_quality_mode",
        choices=(
            "legacy",
            "principal_linear",
            "principal_squared",
            "principal_power",
            "principal_ray_linear",
            "principal_top_corner_linear",
            "trapezoid_inset_linear",
        ),
        default=DEFAULT_REWARD_QUALITY_MODE,
    )
    parser.add_argument(
        "--reward_quality_gamma",
        type=float,
        default=DEFAULT_REWARD_QUALITY_GAMMA,
    )
    parser.add_argument(
        "--coverage_edge_quality",
        type=float,
        default=DEFAULT_COVERAGE_EDGE_QUALITY,
    )
    return parser.parse_args()


def principal_radial_norm_for_point(
    *,
    drone_x: float,
    drone_y: float,
    drone_z: float,
    yaw: float,
    tilt_deg: float,
    horizontal_fov_deg: float,
    point_x: float,
    point_y: float,
    eps: float = 1e-6,
) -> float:
    principal_x, principal_y = principal_point_world(
        x=drone_x,
        y=drone_y,
        z=drone_z,
        yaw=yaw,
        camera_tilt_deg=tilt_deg,
    )
    radial_distance = math.hypot(point_x - principal_x, point_y - principal_y)
    if radial_distance <= eps:
        return 0.0
    principal_forward, _ = world_to_drone_local(
        principal_x,
        principal_y,
        drone_x,
        drone_y,
        yaw,
    )
    radius = lateral_half_width_at_forward_distance(
        forward=principal_forward,
        z=drone_z,
        horizontal_fov_deg=horizontal_fov_deg,
    )
    if radius <= eps:
        return 1.0
    return min(max(radial_distance / radius, 0.0), 1.0)


def legacy_quality_for_point(
    *,
    drone_x: float,
    drone_y: float,
    drone_z: float,
    yaw: float,
    tilt_deg: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    point_x: float,
    point_y: float,
    coverage_edge_quality: float,
    eps: float = 1e-6,
) -> float:
    forward, lateral = world_to_drone_local(point_x, point_y, drone_x, drone_y, yaw)
    min_forward, max_forward = footprint_forward_extents(
        z=drone_z,
        camera_tilt_deg=tilt_deg,
        vertical_fov_deg=vertical_fov_deg,
    )
    forward_span = max(max_forward - min_forward, eps)
    forward_norm = min(max((forward - min_forward) / forward_span, 0.0), 1.0)
    half_width = lateral_half_width_at_forward_distance(
        forward=forward,
        z=drone_z,
        horizontal_fov_deg=horizontal_fov_deg,
    )
    lateral_norm = 1.0 if half_width <= eps else min(abs(lateral) / half_width, 1.0)
    q_forward = 1.0 - forward_norm
    q_lateral = 1.0 - lateral_norm ** 2.0
    base_quality = q_forward * q_lateral
    return coverage_edge_quality + (1.0 - coverage_edge_quality) * base_quality


def coverage_quality_for_point(
    *,
    reward_quality_mode: str,
    reward_quality_gamma: float,
    coverage_edge_quality: float,
    drone_x: float,
    drone_y: float,
    drone_z: float,
    yaw: float,
    tilt_deg: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    point_x: float,
    point_y: float,
) -> float:
    if reward_quality_mode == "trapezoid_inset_linear":
        polygon = footprint_corners_world(
            x=drone_x,
            y=drone_y,
            z=drone_z,
            yaw=yaw,
            camera_tilt_deg=tilt_deg,
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
        )
        inset_distance, max_inset_distance = trapezoid_inset_distances(
            polygon=polygon,
            point=(point_x, point_y),
        )
        if max_inset_distance <= 1e-9:
            base_quality = 0.0
        else:
            base_quality = min(max(inset_distance / max_inset_distance, 0.0), 1.0)
        return coverage_edge_quality + (1.0 - coverage_edge_quality) * base_quality

    if reward_quality_mode == "principal_top_corner_linear":
        principal_x, principal_y = principal_point_world(
            x=drone_x,
            y=drone_y,
            z=drone_z,
            yaw=yaw,
            camera_tilt_deg=tilt_deg,
        )
        polygon = footprint_corners_world(
            x=drone_x,
            y=drone_y,
            z=drone_z,
            yaw=yaw,
            camera_tilt_deg=tilt_deg,
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
        )
        top_corners = sorted(polygon, key=lambda corner: corner[1], reverse=True)[:2]
        max_radius = max(
            math.hypot(corner_x - principal_x, corner_y - principal_y)
            for corner_x, corner_y in top_corners
        )
        if max_radius <= 1e-9:
            base_quality = 1.0
        else:
            radial_distance = math.hypot(point_x - principal_x, point_y - principal_y)
            radial_norm = min(max(radial_distance / max_radius, 0.0), 1.0)
            base_quality = (1.0 - radial_norm) ** reward_quality_gamma
        return coverage_edge_quality + (1.0 - coverage_edge_quality) * base_quality

    if reward_quality_mode == "principal_ray_linear":
        principal_x, principal_y = principal_point_world(
            x=drone_x,
            y=drone_y,
            z=drone_z,
            yaw=yaw,
            camera_tilt_deg=tilt_deg,
        )
        dir_x = point_x - principal_x
        dir_y = point_y - principal_y
        point_distance = math.hypot(dir_x, dir_y)
        if point_distance <= 1e-9:
            base_quality = 1.0
        else:
            polygon = footprint_corners_world(
                x=drone_x,
                y=drone_y,
                z=drone_z,
                yaw=yaw,
                camera_tilt_deg=tilt_deg,
                horizontal_fov_deg=horizontal_fov_deg,
                vertical_fov_deg=vertical_fov_deg,
            )
            boundary_distance = ray_polygon_intersection_distance(
                origin=(principal_x, principal_y),
                direction=(dir_x, dir_y),
                polygon=polygon,
            )
            if boundary_distance is None or boundary_distance <= 1e-9:
                base_quality = 0.0
            else:
                radial_norm = min(max(point_distance / boundary_distance, 0.0), 1.0)
                base_quality = 1.0 - radial_norm
        return coverage_edge_quality + (1.0 - coverage_edge_quality) * base_quality

    if reward_quality_mode == "legacy":
        return legacy_quality_for_point(
            drone_x=drone_x,
            drone_y=drone_y,
            drone_z=drone_z,
            yaw=yaw,
            tilt_deg=tilt_deg,
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
            point_x=point_x,
            point_y=point_y,
            coverage_edge_quality=coverage_edge_quality,
        )

    radial_norm = principal_radial_norm_for_point(
        drone_x=drone_x,
        drone_y=drone_y,
        drone_z=drone_z,
        yaw=yaw,
        tilt_deg=tilt_deg,
        horizontal_fov_deg=horizontal_fov_deg,
        point_x=point_x,
        point_y=point_y,
    )
    if reward_quality_mode == "principal_squared":
        base_quality = 1.0 - radial_norm ** 2
    elif reward_quality_mode == "principal_power":
        base_quality = (1.0 - radial_norm) ** reward_quality_gamma
    else:
        base_quality = 1.0 - radial_norm
    return coverage_edge_quality + (1.0 - coverage_edge_quality) * base_quality


def inward_halfplanes(
    polygon: list[tuple[float, float]],
) -> list[tuple[float, float, float]]:
    centroid_x = sum(x for x, _ in polygon) / len(polygon)
    centroid_y = sum(y for _, y in polygon) / len(polygon)
    constraints: list[tuple[float, float, float]] = []
    for idx in range(len(polygon)):
        ax, ay = polygon[idx]
        bx, by = polygon[(idx + 1) % len(polygon)]
        nx = by - ay
        ny = -(bx - ax)
        norm = math.hypot(nx, ny)
        if norm <= 1e-12:
            continue
        nx /= norm
        ny /= norm
        c = -(nx * ax + ny * ay)
        if nx * centroid_x + ny * centroid_y + c < 0.0:
            nx = -nx
            ny = -ny
            c = -c
        constraints.append((nx, ny, c))
    return constraints


def solve_two_lines(
    a1: float,
    b1: float,
    rhs1: float,
    a2: float,
    b2: float,
    rhs2: float,
) -> tuple[float, float] | None:
    det = a1 * b2 - a2 * b1
    if abs(det) <= 1e-12:
        return None
    x = (rhs1 * b2 - rhs2 * b1) / det
    y = (a1 * rhs2 - a2 * rhs1) / det
    return x, y


def feasible_inset_distance(
    constraints: list[tuple[float, float, float]],
    inset_distance: float,
) -> bool:
    candidates: list[tuple[float, float]] = []
    for i in range(len(constraints)):
        a1, b1, c1 = constraints[i]
        for j in range(i + 1, len(constraints)):
            a2, b2, c2 = constraints[j]
            candidate = solve_two_lines(
                a1,
                b1,
                inset_distance - c1,
                a2,
                b2,
                inset_distance - c2,
            )
            if candidate is not None:
                candidates.append(candidate)
    if not candidates:
        return False
    for x, y in candidates:
        if all(a * x + b * y + c >= inset_distance - 1e-9 for a, b, c in constraints):
            return True
    return False


def max_trapezoid_inset_distance(
    constraints: list[tuple[float, float, float]],
) -> float:
    hi = max(c for _, _, c in constraints) + 100.0
    lo = 0.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if feasible_inset_distance(constraints, mid):
            lo = mid
        else:
            hi = mid
    return lo


def trapezoid_inset_distances(
    polygon: list[tuple[float, float]],
    point: tuple[float, float],
) -> tuple[float, float]:
    constraints = inward_halfplanes(polygon)
    px, py = point
    inset_distance = min(a * px + b * py + c for a, b, c in constraints)
    max_inset_distance = max_trapezoid_inset_distance(constraints)
    return inset_distance, max_inset_distance


def distance_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    seg_len_sq = vx * vx + vy * vy
    if seg_len_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len_sq))
    proj_x = ax + t * vx
    proj_y = ay + t * vy
    return math.hypot(px - proj_x, py - proj_y)


def polygon_edge_distance(px: float, py: float, polygon: list[tuple[float, float]]) -> float:
    best = float("inf")
    for idx in range(len(polygon)):
        ax, ay = polygon[idx]
        bx, by = polygon[(idx + 1) % len(polygon)]
        best = min(best, distance_to_segment(px, py, ax, ay, bx, by))
    return best


def world_bounds(
    drone_x: float,
    drone_y: float,
    polygon: list[tuple[float, float]],
    principal_x: float,
    principal_y: float,
) -> tuple[float, float, float, float]:
    xs = [drone_x, principal_x] + [x for x, _ in polygon]
    ys = [drone_y, principal_y] + [y for _, y in polygon]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    pad_x = max(1.0, 0.15 * (max_x - min_x))
    pad_y = max(1.0, 0.15 * (max_y - min_y))
    return min_x - pad_x, max_x + pad_x, min_y - pad_y, max_y + pad_y


def world_to_pixel(
    x: float,
    y: float,
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    width: int,
    height: int,
) -> tuple[int, int]:
    px = int(round((x - min_x) / max(max_x - min_x, 1e-9) * (width - 1)))
    py = int(round((max_y - y) / max(max_y - min_y, 1e-9) * (height - 1)))
    return px, py


def point_in_polygon(point_x: float, point_y: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    count = len(polygon)
    j = count - 1
    for i in range(count):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > point_y) != (yj > point_y)) and (
            point_x < (xj - xi) * (point_y - yi) / max(yj - yi, 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def set_pixel(image: bytearray, width: int, x: int, y: int, rgb: tuple[int, int, int]) -> None:
    if x < 0 or x >= width or y < 0:
        return
    idx = (y * width + x) * 3
    if idx < 0 or idx + 2 >= len(image):
        return
    image[idx:idx + 3] = bytes(rgb)


def draw_disc(image: bytearray, width: int, height: int, cx: int, cy: int, radius: int, rgb: tuple[int, int, int]) -> None:
    r2 = radius * radius
    for y in range(max(0, cy - radius), min(height, cy + radius + 1)):
        for x in range(max(0, cx - radius), min(width, cx + radius + 1)):
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2:
                set_pixel(image, width, x, y, rgb)


def draw_line(
    image: bytearray,
    width: int,
    height: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    rgb: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    dx = x1 - x0
    dy = y1 - y0
    steps = max(abs(dx), abs(dy), 1)
    for step in range(steps + 1):
        t = step / steps
        px = int(round(x0 + t * dx))
        py = int(round(y0 + t * dy))
        draw_disc(image, width, height, px, py, max(thickness - 1, 0), rgb)


def draw_text(
    image: bytearray,
    width: int,
    height: int,
    x: int,
    y: int,
    text: str,
    rgb: tuple[int, int, int],
    scale: int = 2,
) -> None:
    cursor_x = x
    for char in text:
        glyph = FONT_3X5.get(char)
        if glyph is None:
            cursor_x += 4 * scale
            continue
        for row_idx, row in enumerate(glyph):
            for col_idx, value in enumerate(row):
                if value != "1":
                    continue
                for sy in range(scale):
                    for sx in range(scale):
                        px = cursor_x + col_idx * scale + sx
                        py = y + row_idx * scale + sy
                        if 0 <= px < width and 0 <= py < height:
                            set_pixel(image, width, px, py, rgb)
        cursor_x += (len(glyph[0]) + 1) * scale


def crosses_level(value_a: float | None, value_b: float | None, level: float, eps: float = 1e-9) -> bool:
    if value_a is None or value_b is None:
        return False
    if level <= eps:
        return (value_a <= eps and value_b > eps) or (value_b <= eps and value_a > eps)
    da = value_a - level
    db = value_b - level
    if abs(da) <= eps or abs(db) <= eps:
        return True
    return (da < 0.0 and db > 0.0) or (da > 0.0 and db < 0.0)


def write_png(path: str, width: int, height: int, rgb_bytes: bytearray) -> None:
    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        return (
            struct.pack("!I", len(data))
            + chunk_type
            + data
            + struct.pack("!I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
        )

    raw = bytearray()
    row_stride = width * 3
    for y in range(height):
        raw.append(0)
        start = y * row_stride
        raw.extend(rgb_bytes[start:start + row_stride])

    png = bytearray(b"\x89PNG\r\n\x1a\n")
    png.extend(chunk(b"IHDR", struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0)))
    png.extend(chunk(b"IDAT", zlib.compress(bytes(raw), level=9)))
    png.extend(chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def main() -> None:
    args = parse_args()
    width = args.width
    height = args.height
    yaw = math.radians(args.yaw_deg)

    polygon = footprint_corners_world(
        x=args.drone_x,
        y=args.drone_y,
        z=args.drone_z,
        yaw=yaw,
        camera_tilt_deg=args.tilt_deg,
        horizontal_fov_deg=args.horizontal_fov_deg,
        vertical_fov_deg=args.vertical_fov_deg,
    )
    principal_x, principal_y = principal_point_world(
        x=args.drone_x,
        y=args.drone_y,
        z=args.drone_z,
        yaw=yaw,
        camera_tilt_deg=args.tilt_deg,
    )
    min_x, max_x, min_y, max_y = world_bounds(
        args.drone_x,
        args.drone_y,
        polygon,
        principal_x,
        principal_y,
    )
    polygon_px = [
        world_to_pixel(
            wx,
            wy,
            min_x=min_x,
            max_x=max_x,
            min_y=min_y,
            max_y=max_y,
            width=width,
            height=height,
        )
        for wx, wy in polygon
    ]

    contour_levels = [0.0, 0.2, 0.4, 0.6, 0.8]
    contour_colors = {
        0.0: (148, 0, 211),
        0.2: (240, 177, 52),
        0.4: (84, 181, 232),
        0.6: (66, 201, 110),
        0.8: (222, 79, 79),
    }
    contour_tolerances = {
        0.0: 0.006,
        0.2: 0.012,
        0.4: 0.012,
        0.6: 0.012,
        0.8: 0.012,
    }
    image = bytearray([255] * width * height * 3)
    quality_map: list[float | None] = [None] * (width * height)
    inside_map: list[bool] = [False] * (width * height)
    trapezoid_constraints = None
    trapezoid_max_inset = None
    if args.reward_quality_mode == "trapezoid_inset_linear":
        trapezoid_constraints = inward_halfplanes(polygon)
        trapezoid_max_inset = max_trapezoid_inset_distance(trapezoid_constraints)
    world_per_pixel = max(
        (max_x - min_x) / max(width - 1, 1),
        (max_y - min_y) / max(height - 1, 1),
    )
    boundary_tol = 1.25 * world_per_pixel
    label_points: dict[float, tuple[int, int]] = {}

    for py in range(height):
        world_y = max_y - (py / max(height - 1, 1)) * (max_y - min_y)
        for px in range(width):
            world_x = min_x + (px / max(width - 1, 1)) * (max_x - min_x)
            inside = point_in_polygon(px + 0.5, py + 0.5, polygon_px)
            inside_map[py * width + px] = inside
            idx = (py * width + px) * 3
            if inside:
                if args.reward_quality_mode == "trapezoid_inset_linear":
                    inset_distance = min(
                        a * world_x + b * world_y + c
                        for a, b, c in trapezoid_constraints
                    )
                    if trapezoid_max_inset <= 1e-9:
                        base_quality = 0.0
                    else:
                        base_quality = min(
                            max(inset_distance / trapezoid_max_inset, 0.0),
                            1.0,
                        )
                    quality = args.coverage_edge_quality + (
                        1.0 - args.coverage_edge_quality
                    ) * base_quality
                else:
                    quality = coverage_quality_for_point(
                        reward_quality_mode=args.reward_quality_mode,
                        reward_quality_gamma=args.reward_quality_gamma,
                        coverage_edge_quality=args.coverage_edge_quality,
                        drone_x=args.drone_x,
                        drone_y=args.drone_y,
                        drone_z=args.drone_z,
                        yaw=yaw,
                        tilt_deg=args.tilt_deg,
                        horizontal_fov_deg=args.horizontal_fov_deg,
                        vertical_fov_deg=args.vertical_fov_deg,
                        point_x=world_x,
                        point_y=world_y,
                    )
                quality_map[py * width + px] = quality
                # Keep q=0 inside the FOV visually distinct from the outside
                # white background so zero-reward wedges do not look like an
                # overlay/stitching artifact.
                base = int(round(242.0 - 112.0 * quality))
                image[idx:idx + 3] = bytes((base, base, 252))
            if inside and polygon_edge_distance(world_x, world_y, polygon) <= boundary_tol:
                image[idx:idx + 3] = bytes((0, 0, 0))

    for py in range(height):
        for px in range(width):
            idx_flat = py * width + px
            center_quality = quality_map[idx_flat]
            if center_quality is None:
                continue
            right_quality = quality_map[idx_flat + 1] if px + 1 < width else None
            down_quality = quality_map[idx_flat + width] if py + 1 < height else None
            for level in contour_levels:
                if crosses_level(center_quality, right_quality, level) or crosses_level(center_quality, down_quality, level):
                    idx = idx_flat * 3
                    image[idx:idx + 3] = bytes(contour_colors[level])
                    current = label_points.get(level)
                    if current is None or px > current[0]:
                        label_points[level] = (px, py)
                    break

    # Extend contours cleanly to the clipped footprint boundary.
    for py in range(height):
        world_y = max_y - (py / max(height - 1, 1)) * (max_y - min_y)
        for px in range(width):
            idx_flat = py * width + px
            center_quality = quality_map[idx_flat]
            if center_quality is None:
                continue
            world_x = min_x + (px / max(width - 1, 1)) * (max_x - min_x)
            if polygon_edge_distance(world_x, world_y, polygon) > 2.5 * boundary_tol:
                continue
            has_outside_neighbor = False
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nx = px + dx
                ny = py + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                if not inside_map[ny * width + nx]:
                    has_outside_neighbor = True
                    break
            if not has_outside_neighbor:
                continue
            for level in contour_levels:
                if abs(center_quality - level) <= contour_tolerances[level]:
                    idx = idx_flat * 3
                    image[idx:idx + 3] = bytes(contour_colors[level])
                    current = label_points.get(level)
                    if current is None or px > current[0]:
                        label_points[level] = (px, py)
                    break

    drone_px, drone_py = world_to_pixel(
        args.drone_x,
        args.drone_y,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
    )
    principal_px, principal_py = world_to_pixel(
        principal_x,
        principal_y,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
    )
    left_near_px, left_near_py = world_to_pixel(
        polygon[0][0],
        polygon[0][1],
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
    )
    left_far_px, left_far_py = world_to_pixel(
        polygon[1][0],
        polygon[1][1],
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
    )
    right_far_px, right_far_py = world_to_pixel(
        polygon[2][0],
        polygon[2][1],
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
    )
    right_near_px, right_near_py = world_to_pixel(
        polygon[3][0],
        polygon[3][1],
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
    )
    draw_line(image, width, height, left_near_px, left_near_py, left_far_px, left_far_py, (255, 140, 0), thickness=2)
    draw_line(image, width, height, right_near_px, right_near_py, right_far_px, right_far_py, (0, 160, 0), thickness=2)
    left_mid_x = (left_near_px + left_far_px) // 2
    left_mid_y = (left_near_py + left_far_py) // 2
    right_mid_x = (right_near_px + right_far_px) // 2
    right_mid_y = (right_near_py + right_far_py) // 2
    draw_disc(image, width, height, drone_px, drone_py, 8, (215, 38, 56))
    draw_disc(image, width, height, principal_px, principal_py, 7, (18, 76, 178))
    draw_line(image, width, height, drone_px, drone_py, principal_px, principal_py, (18, 76, 178), thickness=1)
    draw_text(image, width, height, left_mid_x + 8, left_mid_y - 6, "L", (255, 140, 0), scale=3)
    draw_text(image, width, height, right_mid_x + 8, right_mid_y - 6, "R", (0, 160, 0), scale=3)
    for level in contour_levels:
        label_point = label_points.get(level)
        if label_point is None:
            continue
        label_x = min(width - 52, label_point[0] + 10)
        label_y = max(4, min(height - 14, label_point[1] - 5))
        draw_text(
            image,
            width,
            height,
            label_x,
            label_y,
            f"q={level:.1f}",
            contour_colors[level],
            scale=2,
        )

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    write_png(args.output, width, height, image)
    print(f"[plot_fov_quality_png] wrote {args.output}")


if __name__ == "__main__":
    main()

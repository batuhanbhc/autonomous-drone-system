import math
from typing import List, Tuple


_HORIZON_EPS = 1e-3


def _clamp_pitch_from_down(angle_rad: float) -> float:
    return max(-0.5 * math.pi + _HORIZON_EPS, min(0.5 * math.pi - _HORIZON_EPS, angle_rad))


def footprint_forward_extents(
    z: float,
    camera_tilt_deg: float,
    vertical_fov_deg: float,
) -> Tuple[float, float]:
    """
    Return signed forward distances on the ground in the drone-local frame.

    Convention:
      - camera_tilt_deg = 0   -> camera optical axis points toward the horizon
      - camera_tilt_deg = 90  -> camera optical axis points straight down

    The returned distances can be negative when the visible footprint extends
    behind the drone in the local forward axis.
    """
    tilt = _clamp_pitch_from_down(0.5 * math.pi - math.radians(camera_tilt_deg))
    half_v = 0.5 * math.radians(vertical_fov_deg)
    lower_angle = _clamp_pitch_from_down(tilt - half_v)
    upper_angle = _clamp_pitch_from_down(tilt + half_v)
    lower_forward = z * math.tan(lower_angle)
    upper_forward = z * math.tan(upper_angle)
    return min(lower_forward, upper_forward), max(lower_forward, upper_forward)


def lateral_half_width_at_forward_distance(
    forward: float,
    z: float,
    horizontal_fov_deg: float,
) -> float:
    """Half-width of the ground footprint at a given forward distance."""
    half_h = 0.5 * math.radians(horizontal_fov_deg)
    slant = math.hypot(z, forward)
    return slant * math.tan(half_h)


def footprint_center(
    x: float,
    y: float,
    z: float,
    yaw: float,
    camera_tilt_deg: float,
    vertical_fov_deg: float,
) -> Tuple[float, float]:
    min_forward, max_forward = footprint_forward_extents(z, camera_tilt_deg, vertical_fov_deg)
    center_shift = 0.5 * (min_forward + max_forward)
    cx = x + center_shift * math.cos(yaw)
    cy = y + center_shift * math.sin(yaw)
    return cx, cy


def principal_point_world(
    x: float,
    y: float,
    z: float,
    yaw: float,
    camera_tilt_deg: float,
) -> Tuple[float, float]:
    """
    Ground intersection of the camera principal ray.

    Convention:
      - camera_tilt_deg = 0   -> camera optical axis points toward the horizon
      - camera_tilt_deg = 90  -> camera optical axis points straight down
    """
    tilt = _clamp_pitch_from_down(0.5 * math.pi - math.radians(camera_tilt_deg))
    forward = z * math.tan(tilt)
    px = x + forward * math.cos(yaw)
    py = y + forward * math.sin(yaw)
    return px, py


def world_to_drone_local(
    px: float,
    py: float,
    drone_x: float,
    drone_y: float,
    yaw: float,
) -> Tuple[float, float]:
    dx = px - drone_x
    dy = py - drone_y
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return forward, lateral


def point_in_footprint_local(
    forward: float,
    lateral: float,
    z: float,
    camera_tilt_deg: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
) -> bool:
    min_forward, max_forward = footprint_forward_extents(z, camera_tilt_deg, vertical_fov_deg)
    if forward < min_forward or forward > max_forward:
        return False
    tilt = _clamp_pitch_from_down(0.5 * math.pi - math.radians(camera_tilt_deg))
    half_v = 0.5 * math.radians(vertical_fov_deg)
    pitch_from_down = math.atan2(forward, z)
    if abs(pitch_from_down - tilt) > half_v + 1e-9:
        return False
    half_width = lateral_half_width_at_forward_distance(forward, z, horizontal_fov_deg)
    return abs(lateral) <= half_width


def point_in_footprint_world(
    px: float,
    py: float,
    drone_x: float,
    drone_y: float,
    z: float,
    yaw: float,
    camera_tilt_deg: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
) -> bool:
    forward, lateral = world_to_drone_local(px, py, drone_x, drone_y, yaw)
    return point_in_footprint_local(
        forward=forward,
        lateral=lateral,
        z=z,
        camera_tilt_deg=camera_tilt_deg,
        horizontal_fov_deg=horizontal_fov_deg,
        vertical_fov_deg=vertical_fov_deg,
    )


def footprint_corners_world(
    x: float,
    y: float,
    z: float,
    yaw: float,
    camera_tilt_deg: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
) -> List[Tuple[float, float]]:
    min_forward, max_forward = footprint_forward_extents(z, camera_tilt_deg, vertical_fov_deg)
    min_half_width = lateral_half_width_at_forward_distance(min_forward, z, horizontal_fov_deg)
    max_half_width = lateral_half_width_at_forward_distance(max_forward, z, horizontal_fov_deg)

    local_corners = [
        (min_forward, min_half_width),
        (max_forward, max_half_width),
        (max_forward, -max_half_width),
        (min_forward, -min_half_width),
    ]

    corners = []
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    for forward, lateral in local_corners:
        wx = x + cos_yaw * forward - sin_yaw * lateral
        wy = y + sin_yaw * forward + cos_yaw * lateral
        corners.append((wx, wy))
    return corners


def ray_polygon_intersection_distance(
    origin: Tuple[float, float],
    direction: Tuple[float, float],
    polygon: List[Tuple[float, float]],
    eps: float = 1e-9,
) -> float | None:
    """
    Return the distance from `origin` to the first intersection between the
    ray `origin + t * direction` (t >= 0) and the polygon boundary.

    `direction` does not need to be unit length. The returned distance is in
    world units along the ray.
    """
    ox, oy = origin
    dx, dy = direction
    dir_norm = math.hypot(dx, dy)
    if dir_norm <= eps or len(polygon) < 2:
        return None

    best_t = None
    num_vertices = len(polygon)
    for idx in range(num_vertices):
        ax, ay = polygon[idx]
        bx, by = polygon[(idx + 1) % num_vertices]
        sx = bx - ax
        sy = by - ay

        denom = dx * sy - dy * sx
        if abs(denom) <= eps:
            continue

        qpx = ax - ox
        qpy = ay - oy
        t = (qpx * sy - qpy * sx) / denom
        u = (qpx * dy - qpy * dx) / denom
        if t < -eps or u < -eps or u > 1.0 + eps:
            continue

        t = max(t, 0.0)
        if best_t is None or t < best_t:
            best_t = t

    if best_t is None:
        return None
    return best_t * dir_norm

import math

import pybullet as p

from sim.camera_geometry import footprint_corners_world, principal_point_world


class DebugDrawer:
    def __init__(
        self,
        tilt_deg=45.0,
        horizontal_fov_deg=60.0,
        vertical_fov_deg=45.0,
        show_reward_contours: bool = False,
        coverage_edge_quality: float = 0.2,
        client_id: int | None = None,
    ):
        self.tilt_deg = tilt_deg
        self.horizontal_fov_deg = horizontal_fov_deg
        self.vertical_fov_deg = vertical_fov_deg
        self.show_reward_contours = bool(show_reward_contours)
        self.coverage_edge_quality = float(coverage_edge_quality)
        self.client_id = client_id

    def _pb(self) -> dict:
        return {"physicsClientId": self.client_id} if self.client_id is not None else {}

    def clear(self):
        if p.isConnected(self.client_id):
            p.removeAllUserDebugItems(**self._pb())

    def clear_inactive_drones(self, active_drone_ids):
        # With the simple clear-and-redraw path, inactive cleanup is handled by clear().
        return

    def _compute_footprint_corners(self, drone_state):
        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]

        cx, cy = principal_point_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.tilt_deg,
        )
        world_corners = [
            [wx, wy, 0.02]
            for wx, wy in footprint_corners_world(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                camera_tilt_deg=self.tilt_deg,
                horizontal_fov_deg=self.horizontal_fov_deg,
                vertical_fov_deg=self.vertical_fov_deg,
            )
        ]
        return world_corners, [cx, cy, 0.02]

    def _draw_reward_quality_contours(self, drone_state, reward_calc=None):
        if not self.show_reward_contours:
            return

        x, y, z = drone_state["position"]
        yaw = drone_state["yaw"]
        cx, cy = principal_point_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.tilt_deg,
        )
        corners = footprint_corners_world(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            camera_tilt_deg=self.tilt_deg,
            horizontal_fov_deg=self.horizontal_fov_deg,
            vertical_fov_deg=self.vertical_fov_deg,
        )
        if reward_calc is None:
            return

        contour_levels = [0.95, 0.85, 0.70, 0.55]
        contour_colors = [
            [0.2, 1.0, 1.0],
            [0.2, 0.9, 0.2],
            [1.0, 0.85, 0.2],
            [1.0, 0.45, 0.1],
        ]
        z_draw = 0.03
        xs = [corner[0] for corner in corners]
        ys = [corner[1] for corner in corners]
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        samples_per_axis = 20
        tol = 0.025
        marker = 0.08

        for quality, color in zip(contour_levels, contour_colors):
            if quality <= reward_calc.coverage_edge_quality:
                continue
            label_point = None
            for ix in range(samples_per_axis):
                wx = x0 + (x1 - x0) * ix / max(samples_per_axis - 1, 1)
                for iy in range(samples_per_axis):
                    wy = y0 + (y1 - y0) * iy / max(samples_per_axis - 1, 1)
                    if not reward_calc.point_in_footprint_world_for_reward(
                        drone_state=drone_state,
                        point=(wx, wy),
                    ):
                        continue
                    q = reward_calc.coverage_quality_for_point(
                        drone_state=drone_state,
                        point=(wx, wy),
                    )
                    if abs(q - quality) > tol:
                        continue
                    point = [wx, wy, z_draw]
                    p.addUserDebugLine(
                        [wx - marker, wy, z_draw],
                        [wx + marker, wy, z_draw],
                        color,
                        lineWidth=1,
                        lifeTime=0,
                        **self._pb(),
                    )
                    p.addUserDebugLine(
                        [wx, wy - marker, z_draw],
                        [wx, wy + marker, z_draw],
                        color,
                        lineWidth=1,
                        lifeTime=0,
                        **self._pb(),
                    )
                    if label_point is None:
                        label_point = point

            if label_point is None:
                continue
            p.addUserDebugText(
                text=f"q={quality:.2f}",
                textPosition=label_point,
                textColorRGB=color,
                textSize=0.9,
                lifeTime=0,
                **self._pb(),
            )

    def draw_camera_footprint(self, drone_state, drone_id=None, reward_calc=None):
        if not p.isConnected(self.client_id):
            return

        x, y, z = drone_state["position"]
        corners, center = self._compute_footprint_corners(drone_state)

        for i in range(4):
            p.addUserDebugLine(
                corners[i],
                corners[(i + 1) % 4],
                [0, 1, 0],
                lineWidth=2,
                lifeTime=0,
                **self._pb(),
            )

        p.addUserDebugLine(
            [x, y, z],
            center,
            [0, 0, 1],
            lineWidth=2,
            lifeTime=0,
            **self._pb(),
        )
        self._draw_reward_quality_contours(drone_state, reward_calc=reward_calc)

        if drone_id is not None:
            p.addUserDebugText(
                text=f"D{drone_id}",
                textPosition=[x, y, z + 0.25],
                textColorRGB=[1, 1, 1],
                textSize=1.2,
                lifeTime=0,
                **self._pb(),
            )

    def draw_detections(self, people, visible_ids):
        if not p.isConnected(self.client_id):
            return

        visible_set = set(visible_ids)

        for idx, person in enumerate(people):
            if person is None or not person.is_valid():
                continue

            if idx in visible_set:
                color = [0.0, 1.0, 0.0, 1.0]
            else:
                color = [1.0, 0.0, 0.0, 1.0]

            p.changeVisualShape(person.body_id, -1, rgbaColor=color, **self._pb())

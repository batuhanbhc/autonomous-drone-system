import math
import random

import pybullet as p


class Person:
    def __init__(
        self,
        start_pos=(2.0, 2.0, 0.35),
        half_extents=(0.10, 0.10, 0.35),
        color=(1.0, 0.2, 0.2, 1.0),
    ):
        self.start_pos = start_pos
        self.half_extents = half_extents
        self.color = color

        self.body_id = None

        self.speed = random.uniform(0.08, 0.22)
        self.heading = random.uniform(-math.pi, math.pi)

        self.state = "moving"
        self.state_timer = 0.0
        self._set_new_state_duration()

        # Group-related attributes
        self.is_grouped = False
        self.group_id = None
        self.group_center = None   # (x, y)
        self.group_radius = 1.0

        # Personal-space / separation settings
        self.personal_space = 0.28
        self.separation_gain = 0.65
        self.max_separation_push = 0.18

    def _set_new_state_duration(self):
        if self.state == "moving":
            self.state_timer = random.uniform(2.0, 6.0)
        else:
            self.state_timer = random.uniform(1.0, 4.0)

    def is_valid(self):
        if not p.isConnected():
            return False
        if self.body_id is None:
            return False
        try:
            p.getBasePositionAndOrientation(self.body_id)
            return True
        except Exception:
            return False

    def spawn(self):
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=self.half_extents
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=self.half_extents,
            rgbaColor=self.color
        )
        self.body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.start_pos
        )

    def assign_group(self, group_id, group_center, group_radius=1.0):
        self.is_grouped = True
        self.group_id = group_id
        self.group_center = group_center
        self.group_radius = group_radius

    def clear_group(self):
        self.is_grouped = False
        self.group_id = None
        self.group_center = None
        self.group_radius = 1.0

    def reset(
        self,
        pos=(2.0, 2.0, 0.35),
        grouped=False,
        group_id=None,
        group_center=None,
        group_radius=1.0
    ):
        if not self.is_valid():
            return

        quat = p.getQuaternionFromEuler((0.0, 0.0, 0.0))
        p.resetBasePositionAndOrientation(self.body_id, pos, quat)

        self.speed = random.uniform(0.08, 0.22)
        self.heading = random.uniform(-math.pi, math.pi)
        self.state = random.choice(["moving", "stopped"])
        self._set_new_state_duration()

        if grouped and group_center is not None:
            self.assign_group(
                group_id=group_id,
                group_center=group_center,
                group_radius=group_radius
            )
        else:
            self.clear_group()

    def _update_motion_state(self, dt: float):
        self.state_timer -= dt
        if self.state_timer <= 0.0:
            if self.state == "moving":
                self.state = "stopped"
            else:
                self.state = "moving"
                self.heading = random.uniform(-math.pi, math.pi)
                self.speed = random.uniform(0.08, 0.22)
            self._set_new_state_duration()

    def _compute_separation_offset(self, x, y, other_people_positions, min_dist=None):
        if other_people_positions is None:
            return 0.0, 0.0

        if min_dist is None:
            min_dist = self.personal_space

        push_x = 0.0
        push_y = 0.0

        for ox, oy, _ in other_people_positions:
            dx = x - ox
            dy = y - oy
            dist = math.hypot(dx, dy)

            if dist < 1e-8:
                continue

            if dist < min_dist:
                strength = self.separation_gain * (min_dist - dist) / min_dist
                push_x += (dx / dist) * strength
                push_y += (dy / dist) * strength

        push_mag = math.hypot(push_x, push_y)
        if push_mag > self.max_separation_push and push_mag > 1e-8:
            scale = self.max_separation_push / push_mag
            push_x *= scale
            push_y *= scale

        return push_x, push_y

    def _enforce_min_distance(self, new_x, new_y, other_people_positions, min_dist=None):
        if other_people_positions is None:
            return new_x, new_y

        if min_dist is None:
            min_dist = self.personal_space

        corrected_x = new_x
        corrected_y = new_y

        for ox, oy, _ in other_people_positions:
            dx = corrected_x - ox
            dy = corrected_y - oy
            dist = math.hypot(dx, dy)

            if dist < 1e-8:
                angle = random.uniform(-math.pi, math.pi)
                corrected_x += min_dist * math.cos(angle)
                corrected_y += min_dist * math.sin(angle)
                continue

            if dist < min_dist:
                overlap = min_dist - dist
                corrected_x += (dx / dist) * overlap
                corrected_y += (dy / dist) * overlap

        return corrected_x, corrected_y

    def step_random_walk(
        self,
        dt: float,
        x_min=-5,
        x_max=5,
        y_min=-5,
        y_max=5,
        other_people_positions=None,
    ):
        if not self.is_valid():
            return

        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        x, y, z = pos

        self._update_motion_state(dt)

        if self.state == "stopped":
            p.resetBasePositionAndOrientation(
                self.body_id,
                (x, y, z),
                p.getQuaternionFromEuler((0.0, 0.0, 0.0))
            )
            return

        if random.random() < 0.01:
            self.heading += random.uniform(-0.4, 0.4)

        vx = self.speed * math.cos(self.heading)
        vy = self.speed * math.sin(self.heading)

        sep_x, sep_y = self._compute_separation_offset(
            x, y, other_people_positions
        )
        vx += sep_x
        vy += sep_y

        next_x = x + vx * dt
        next_y = y + vy * dt

        if next_x < x_min or next_x > x_max:
            self.heading = math.pi - self.heading
        if next_y < y_min or next_y > y_max:
            self.heading = -self.heading

        vx = self.speed * math.cos(self.heading) + sep_x
        vy = self.speed * math.sin(self.heading) + sep_y

        new_x = x + vx * dt
        new_y = y + vy * dt

        new_x, new_y = self._enforce_min_distance(
            new_x, new_y, other_people_positions
        )

        new_x = max(x_min, min(x_max, new_x))
        new_y = max(y_min, min(y_max, new_y))

        p.resetBasePositionAndOrientation(
            self.body_id,
            (new_x, new_y, z),
            p.getQuaternionFromEuler((0.0, 0.0, 0.0))
        )

    def step_grouped_walk(
        self,
        dt: float,
        x_min=-5,
        x_max=5,
        y_min=-5,
        y_max=5,
        other_people_positions=None,
    ):
        if not self.is_valid():
            return

        if not self.is_grouped or self.group_center is None:
            self.step_random_walk(
                dt,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                other_people_positions=other_people_positions,
            )
            return

        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        x, y, z = pos

        self._update_motion_state(dt)

        if self.state == "stopped":
            p.resetBasePositionAndOrientation(
                self.body_id,
                (x, y, z),
                p.getQuaternionFromEuler((0.0, 0.0, 0.0))
            )
            return

        gcx, gcy = self.group_center
        dx = gcx - x
        dy = gcy - y
        dist_to_center = math.hypot(dx, dy)

        if random.random() < 0.03:
            self.heading += random.uniform(-0.5, 0.5)

        if dist_to_center > self.group_radius:
            desired_heading = math.atan2(dy, dx)
            heading_mix = random.uniform(0.6, 0.9)
            self.heading = heading_mix * desired_heading + (1.0 - heading_mix) * self.heading
        else:
            if dist_to_center > 1e-6 and random.random() < 0.1:
                desired_heading = math.atan2(dy, dx)
                heading_mix = random.uniform(0.1, 0.25)
                self.heading = heading_mix * desired_heading + (1.0 - heading_mix) * self.heading

        move_speed = min(self.speed, 0.18)

        vx = move_speed * math.cos(self.heading)
        vy = move_speed * math.sin(self.heading)

        sep_x, sep_y = self._compute_separation_offset(
            x, y, other_people_positions
        )
        vx += sep_x
        vy += sep_y

        next_x = x + vx * dt
        next_y = y + vy * dt

        if next_x < x_min or next_x > x_max:
            self.heading = math.pi - self.heading
        if next_y < y_min or next_y > y_max:
            self.heading = -self.heading

        vx = move_speed * math.cos(self.heading) + sep_x
        vy = move_speed * math.sin(self.heading) + sep_y

        new_x = x + vx * dt
        new_y = y + vy * dt

        new_x, new_y = self._enforce_min_distance(
            new_x, new_y, other_people_positions
        )

        new_x = max(x_min, min(x_max, new_x))
        new_y = max(y_min, min(y_max, new_y))

        p.resetBasePositionAndOrientation(
            self.body_id,
            (new_x, new_y, z),
            p.getQuaternionFromEuler((0.0, 0.0, 0.0))
        )

    def step(
        self,
        dt: float,
        x_min=-5,
        x_max=5,
        y_min=-5,
        y_max=5,
        other_people_positions=None,
    ):
        if self.is_grouped:
            self.step_grouped_walk(
                dt,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                other_people_positions=other_people_positions,
            )
        else:
            self.step_random_walk(
                dt,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                other_people_positions=other_people_positions,
            )

    def get_position(self):
        if not self.is_valid():
            return self.start_pos
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        return pos
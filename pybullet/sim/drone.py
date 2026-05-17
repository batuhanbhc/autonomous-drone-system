import math
from typing import Tuple
import pybullet as p


class Drone:
    def __init__(
        self,
        start_pos=(0.0, 0.0, 1.0),
        size=(0.08, 0.08, 0.03),
        color=(0.2, 0.2, 1.0, 1.0),
        vel_tau_s: float = 0.0,
        yaw_rate_tau_s: float = 0.0,
        client_id: int | None = None,
    ):
        self.start_pos     = start_pos
        self.size          = size
        self.color         = color
        self.body_id       = None
        self.vel_tau_s     = float(vel_tau_s)
        self.yaw_rate_tau_s = float(yaw_rate_tau_s)
        self.client_id     = client_id
        self._actual_vx       = 0.0
        self._actual_vy       = 0.0
        self._actual_yaw_rate = 0.0

    def _pb(self) -> dict:
        return {"physicsClientId": self.client_id} if self.client_id is not None else {}

    def spawn(self):
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=self.size, **self._pb()
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX, halfExtents=self.size, rgbaColor=self.color, **self._pb()
        )
        self.body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.start_pos,
            **self._pb(),
        )

    def reset(self, pos=(0.0, 0.0, 1.0), yaw=0.0):
        quat = p.getQuaternionFromEuler((0.0, 0.0, yaw), **self._pb())
        p.resetBasePositionAndOrientation(self.body_id, pos, quat, **self._pb())
        self._actual_vx       = 0.0
        self._actual_vy       = 0.0
        self._actual_yaw_rate = 0.0

    def apply_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
        dt: float,
        x_min: float = -10.0,
        x_max: float = 10.0,
        y_min: float = -10.0,
        y_max: float = 10.0,
        z_min: float = 0.3,
        z_max: float = 8.0,
    ):
        """
        Apply a velocity command for one control interval.

        The drone is assumed to achieve the commanded velocity instantly.
        For one decision interval dt:
          - x += vx * dt
          - y += vy * dt
          - z += vz * dt
          - yaw += yaw_rate * dt

        There is no momentum model between decisions. Final position is
        clamped to world bounds and yaw is wrapped to (-pi, pi].
        """
        if not p.isConnected(self.client_id) or self.body_id is None:
            return

        pos, quat = p.getBasePositionAndOrientation(self.body_id, **self._pb())
        x, y, z   = pos
        _, _, yaw = p.getEulerFromQuaternion(quat, **self._pb())

        if self.vel_tau_s > 0.0:
            alpha = min(1.0, dt / self.vel_tau_s)
            self._actual_vx += (vx - self._actual_vx) * alpha
            self._actual_vy += (vy - self._actual_vy) * alpha
        else:
            self._actual_vx = vx
            self._actual_vy = vy

        if self.yaw_rate_tau_s > 0.0:
            alpha = min(1.0, dt / self.yaw_rate_tau_s)
            self._actual_yaw_rate += (yaw_rate - self._actual_yaw_rate) * alpha
        else:
            self._actual_yaw_rate = yaw_rate

        new_x = x + self._actual_vx * dt
        new_y = y + self._actual_vy * dt
        new_z = z + vz * dt
        new_yaw = yaw + self._actual_yaw_rate * dt
        new_yaw = (new_yaw + math.pi) % (2.0 * math.pi) - math.pi

        new_x = max(x_min, min(x_max, new_x))
        new_y = max(y_min, min(y_max, new_y))
        new_z = max(z_min, min(z_max, new_z))

        new_quat = p.getQuaternionFromEuler((0.0, 0.0, new_yaw), **self._pb())
        p.resetBasePositionAndOrientation(
            self.body_id, (new_x, new_y, new_z), new_quat, **self._pb()
        )

    def get_pose(self) -> Tuple[Tuple[float, float, float], float]:
        pos, quat = p.getBasePositionAndOrientation(self.body_id, **self._pb())
        _, _, yaw = p.getEulerFromQuaternion(quat, **self._pb())
        return pos, yaw

    def get_state_dict(self):
        pos, yaw = self.get_pose()
        return {
            "position": pos,
            "yaw":      yaw,
        }

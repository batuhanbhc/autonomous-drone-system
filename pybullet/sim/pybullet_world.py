import pybullet as p
import pybullet_data


class PyBulletWorld:
    def __init__(self, gui: bool = True, time_step: float = 1.0 / 60.0):
        self.gui = gui
        self.time_step = time_step
        self.client_id = None
        self.plane_id = None
        self.wall_ids = []

    def connect(self):
        """Connect to PyBullet."""
        if self.client_id is not None:
            return

        connection_mode = p.GUI if self.gui else p.DIRECT
        self.client_id = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(self.time_step)
        p.setGravity(0, 0, -9.81)

    def reset(self):
        """Reset the simulation world."""
        if self.client_id is None:
            self.connect()

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.plane_id = p.loadURDF("plane.urdf")
        self.wall_ids = []

    def add_boundary_walls(self, x_min=-5, x_max=5, y_min=-5, y_max=5, height=1.0, thickness=0.1):
        """Create simple box walls around the environment."""
        wall_specs = [
            # center_x, center_y, half_x, half_y, half_z
            ((x_min - thickness, (y_min + y_max) / 2, height / 2), (thickness, (y_max - y_min) / 2, height / 2)),
            ((x_max + thickness, (y_min + y_max) / 2, height / 2), (thickness, (y_max - y_min) / 2, height / 2)),
            (((x_min + x_max) / 2, y_min - thickness, height / 2), ((x_max - x_min) / 2, thickness, height / 2)),
            (((x_min + x_max) / 2, y_max + thickness, height / 2), ((x_max - x_min) / 2, thickness, height / 2)),
        ]

        for pos, half_extents in wall_specs:
            collision_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=half_extents
            )
            visual_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                rgbaColor=[0.6, 0.6, 0.6, 1.0]
            )
            wall_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=pos
            )
            self.wall_ids.append(wall_id)

    def step(self, num_steps: int = 1):
        """Advance the simulation."""
        for _ in range(num_steps):
            p.stepSimulation()

    def disconnect(self):
        """Disconnect from PyBullet."""
        if self.client_id is not None:
            p.disconnect()
            self.client_id = None
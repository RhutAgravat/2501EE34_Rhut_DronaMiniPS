"""Real PyBullet physics backend; ground truth is evaluation-only."""

import numpy as np
import pybullet as p
import pybullet_data

from config import load_config


class SwarmSim:
    """PyBullet backend for the DronaSwarm proxy quadrotors."""

    def __init__(self, n=1, gui=False, config=None):
        self.cfg = config or load_config()

        self.hz = self.cfg["physics"]["hz"]
        self.dt = 1.0 / self.hz
        self.t = 0.0

        self.rng = np.random.default_rng(self.cfg["seed"])
        self.rc = {drone_id: [1000, 1000, 1000, 1000] for drone_id in range(n)}
        self.ids = []

        self.batt = {drone_id: 1.0 for drone_id in range(n)}
        self.drift = self.rng.uniform(-0.15, 0.15, (n, 3))

        self.attitude = [
            {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "spin_phase": 0.0,
            }
            for _ in range(n)
        ]

        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setRealTimeSimulation(
            0,
            physicsClientId=self.client,
        )

        p.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=1,
            physicsClientId=self.client,
        )

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath(),
            physicsClientId=self.client,
        )
        p.setGravity(
            *self.cfg["physics"]["gravity"],
            physicsClientId=self.client,
        )
        p.setTimeStep(
            self.dt,
            physicsClientId=self.client,
        )
        p.loadURDF(
            "plane.urdf",
            physicsClientId=self.client,
        )

        collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.08, 0.05, 0.025],
            physicsClientId=self.client,
        )

        colors = [
            [0.1, 0.5, 1.0, 1.0],   # Blue
            [1.0, 0.3, 0.2, 1.0],   # Red
            [0.2, 0.9, 0.4, 1.0],   # Green
            [1.0, 0.8, 0.1, 1.0],   # Yellow
        ]

        spacing = float(
            self.cfg["formation"].get("line_spacing", 0.4)
        )

        for drone_id in range(n):
            visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.08, 0.05, 0.025],
                rgbaColor=colors[drone_id % len(colors)],
                physicsClientId=self.client,
            )

            body_id = p.createMultiBody(
                baseMass=self.cfg["physics"]["mass"],
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=[
                    (drone_id - (n - 1) / 2) * spacing,
                    0.0,
                    1.0,
                ],
                physicsClientId=self.client,
            )

            self.ids.append(body_id)

    def set_rc(self, drone_id, rc):
        """Store an already-clamped RC command for the next physics step."""
        self.rc[drone_id] = rc

    def step(self, steps=1):
        """Advance real PyBullet physics by the requested number of steps."""

        physics = self.cfg["physics"]
        control = self.cfg["control"]

        mass = physics["mass"]
        max_thrust_to_weight = physics.get("max_thrust_to_weight", 1.8)
        horizontal_drag = physics["linear_drag_n_s_per_m"]
        vertical_drag = physics.get(
            "vertical_drag_n_s_per_m",
            horizontal_drag,
        )
        horizontal_force = float(
            physics.get("horizontal_force_n", 0.25)
        )

        for _ in range(steps):
            for drone_id, body_id in enumerate(self.ids):
                roll_rc, pitch_rc, yaw_rc, throttle_rc = self.rc[drone_id]

                thrust_fraction = max(
                    0.0,
                    (throttle_rc - 1000) / 1000,
                )

                thrust = (
                    thrust_fraction
                    * mass
                    * 9.81
                    * max_thrust_to_weight
                    * self.batt[drone_id]
                )

                velocity = np.asarray(
                    p.getBaseVelocity(
                        body_id,
                        physicsClientId=self.client,
                    )[0]
                )

                drag = np.array([
                    horizontal_drag * velocity[0],
                    horizontal_drag * velocity[1],
                    vertical_drag * velocity[2],
                ])

                force = (
                    np.array([
                        (pitch_rc - 1500) / 500 * horizontal_force,
                        (roll_rc - 1500) / 500 * horizontal_force,
                        thrust,
                    ])
                    + self.drift[drone_id] * mass
                    - drag
                )

                # Apply the force at the drone's actual centre of mass.
                # This avoids introducing an unintended moment by applying
                # the force at the world origin.
                position = p.getBasePositionAndOrientation(
                    body_id,
                    physicsClientId=self.client,
                )[0]

                position, orientation = p.getBasePositionAndOrientation(
                    body_id,
                    physicsClientId=self.client,
                )

                linear_velocity, angular_velocity = p.getBaseVelocity(
                    body_id,
                    physicsClientId=self.client,
                )

                p.applyExternalForce(
                    body_id,
                    -1,
                    force.tolist(),
                    position,
                    p.WORLD_FRAME,
                    physicsClientId=self.client,
                )

                self.batt[drone_id] = max(
                    0.93,
                    1.0
                    - 0.07
                    * self.t
                    / self.cfg["disturbance"]["battery_sag_duration"],
                )

                # Display-only proxy attitude. This does not feed controller logic.
                attitude = self.attitude[drone_id]

                attitude["roll"] = float(
                    np.clip((roll_rc - 1500) / 500, -1.0, 1.0)
                    * np.deg2rad(
                        control.get("max_tilt_deg", 15)
                    )
                )

                attitude["pitch"] = float(
                    np.clip((pitch_rc - 1500) / 500, -1.0, 1.0)
                    * np.deg2rad(
                        control.get("max_tilt_deg", 15)
                    )
                )

                attitude["yaw"] += float(
                    np.clip((yaw_rc - 1500) / 500, -1.0, 1.0)
                    * 2.5
                    * self.dt
                )

                attitude["spin_phase"] = (
                    attitude["spin_phase"]
                    + (
                        8.0
                        + max(0, throttle_rc - 1000)
                        / 1000
                        * 35.0
                    )
                    * self.dt
                ) % (2 * np.pi)

                orientation = p.getQuaternionFromEuler([
                    attitude["roll"],
                    attitude["pitch"],
                    attitude["yaw"],
                ])

                # Update display-only attitude without losing physical velocity.
                position, _ = p.getBasePositionAndOrientation(
                    body_id,
                    physicsClientId=self.client,
                )

                p.resetBasePositionAndOrientation(
                    body_id,
                    position,
                    orientation,
                    physicsClientId=self.client,
                )

                p.resetBaseVelocity(
                    body_id,
                    linearVelocity=linear_velocity,
                    angularVelocity=angular_velocity,
                    physicsClientId=self.client,
                )

            p.stepSimulation(physicsClientId=self.client)
            self.t += self.dt

    def ground_truth(self):
        """Return PyBullet state for logging/metrics/camera rendering only."""

        return np.array([
            p.getBasePositionAndOrientation(
                body_id,
                physicsClientId=self.client,
            )[0]
            for body_id in self.ids
        ], dtype=float)

    def battery(self, drone_id):
        """Return normalized simulated battery thrust capability."""
        return self.batt[drone_id]

    def visual_attitude(self, drone_id):
        """Return visualization-only roll/pitch/yaw/spin state."""
        return dict(self.attitude[drone_id])

    def close(self):
        """Disconnect the PyBullet client."""
        if p.isConnected(self.client):
            p.disconnect(self.client)
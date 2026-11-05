
"""Cascaded estimate-only PID controller."""

import numpy as np


class CascadedPID:

    def __init__(self, cfg):
        self.cfg = cfg
        self.integral = {}
        self.prev = {}
        self.filtered_derivative = {}
        self.prev_estimate = {}
        self.estimated_velocity = {}

    def command(
        self,
        i,
        target,
        estimate,
        dt=None,
        target_velocity=None,
        target_acceleration=None,
    ):
        if estimate is None:
            return [
                1500, 1500, 1500,
                self.cfg["control"].get("hover_rc", 1333)
            ]

        estimate = np.asarray(estimate, dtype=float)

        if not np.isfinite(estimate).all():
            return [
                1500, 1500, 1500,
                self.cfg["control"].get("hover_rc", 1333)
            ]

        c = self.cfg["control"]

        dt = float(
            dt if dt is not None else 1.0 / c["hz"]
        )

        dt = max(dt, 1e-6)

        kp = np.asarray(c["kp"], dtype=float)
        kv = np.asarray(c["kv"], dtype=float)

        target = np.asarray(target, dtype=float)

        target_velocity = (
            np.zeros(3)
            if target_velocity is None
            else np.asarray(target_velocity, dtype=float)
        )

        target_acceleration = (
            np.zeros(3)
            if target_acceleration is None
            else np.asarray(target_acceleration, dtype=float)
        )

        # Position error uses camera-localization estimates only.
        error = target - estimate

        # Estimate velocity from successive localized positions.
        if i not in self.prev_estimate:
            measured_velocity = np.zeros(3)
        else:
            measured_velocity = (
                estimate - self.prev_estimate[i]
            ) / dt

        self.prev_estimate[i] = estimate.copy()

        # Low-pass filtering of estimated velocity.
        alpha = float(c.get("derivative_alpha", 0.35))

        previous_velocity = self.estimated_velocity.get(
            i, np.zeros(3)
        )

        measured_velocity = (
            alpha * measured_velocity
            + (1.0 - alpha) * previous_velocity
        )

        self.estimated_velocity[i] = measured_velocity.copy()

        # Position loop: desired velocity.
        desired_velocity = (
            target_velocity + kp * error
        )

        desired_velocity = np.clip(
            desired_velocity, -1.0, 1.0
        )

        # Velocity loop: desired acceleration.
        velocity_error = (
            desired_velocity - measured_velocity
        )

        old_integral = self.integral.get(
            i, np.zeros(3)
        )

        candidate_integral = np.clip(
            old_integral + error * dt,
            -float(c.get("vertical_integral_limit", 0.4)),
            float(c.get("vertical_integral_limit", 0.4)),
        )

        acceleration = (
            target_acceleration
            + kv * velocity_error
            + candidate_integral
        )

        # Horizontal acceleration remains tilt-limited.
        # Horizontal acceleration limit in m/s^2.
        tilt = np.deg2rad(float(c.get("max_tilt_deg", 15)))
        horizontal_limit = 9.81 * np.tan(tilt)

        acceleration[:2] = np.clip(
            acceleration[:2],
            -horizontal_limit,
            horizontal_limit,
        )

        # Vertical acceleration has its own limit.
        vertical_limit = float(c.get("vertical_accel_limit", 1.2))

        acceleration[2] = np.clip(
            acceleration[2],
            -vertical_limit,
            vertical_limit,
        )

        acceleration[2] = np.clip(
            acceleration[2],
            -vertical_limit,
            vertical_limit,
        )

        # RC mapping is linear around calibrated hover.
        hover = float(c.get("hover_rc", 1333))

        vertical_scale = float(
            c.get("vertical_rc_per_accel", 33.98)
        )

        throttle_unclamped = (
            hover + acceleration[2] * vertical_scale
        )

        throttle = int(np.clip(
            throttle_unclamped, 1000, 2000
        ))

        # Conditional anti-windup:
        # do not accumulate error when saturation would
        # drive the command farther into saturation.
        saturated_high = (
            throttle >= 2000 and error[2] > 0
        )

        saturated_low = (
            throttle <= 1000 and error[2] < 0
        )

        if not (saturated_high or saturated_low):
            self.integral[i] = candidate_integral
        else:
            self.integral[i] = old_integral

        mass = float(self.cfg["physics"]["mass"])
        horizontal_force = float(
            self.cfg["physics"]["horizontal_force_n"]
        )

        roll = int(np.clip(
            1500 + acceleration[1] * mass / horizontal_force * 500,
            1000, 2000
        ))

        pitch = int(np.clip(
            1500 + acceleration[0] * mass / horizontal_force * 500,
            1000, 2000
        ))

        return [
            roll,
            pitch,
            1500,
            throttle,
        ]

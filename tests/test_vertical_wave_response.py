
import numpy as np

from config import load_config
from simulator import SwarmSim
from planner import wave_state
from controller import CascadedPID


def test_pybullet_vertical_wave_has_physical_response():

    cfg = load_config()

    sim = SwarmSim(
        n=1,
        gui=False,
        config=cfg,
    )

    controller = CascadedPID(cfg)

    positions = []

    try:
        # Begin near the required nominal altitude.
        initial_truth = sim.ground_truth()[0]

        assert abs(initial_truth[2] - 1.0) < 0.05

        # Initialize with the nominal hover command.
        sim.set_rc(
            0,
            [1500, 1500, 1500, cfg["control"]["hover_rc"]]
        )

        control_stride = 10

        duration = 6.0

        for step in range(int(duration * cfg["physics"]["hz"])):

            sim.step()

            if step % control_stride != 0:
                continue

            t = sim.t

            # Single-drone vertical sinusoidal reference.
            dz, dz_v, dz_a = wave_state(
                t,
                phase=0.0,
                kind="circle",
                index=0,
                amplitude=0.2,
                period=6.0,
            )

            target = np.array([
                initial_truth[0],
                initial_truth[1],
                1.0 + dz,
            ])

            target_velocity = np.array([
                0.0, 0.0, dz_v
            ])

            target_acceleration = np.array([
                0.0, 0.0, dz_a
            ])

            # In this isolated test, use a deterministic
            # localization observation generated from a
            # separate synthetic test observation.
            #
            # This test checks the actuator/physics response,
            # not camera robustness.
            #
            # Production controller tests must additionally
            # verify camera-estimate-only feedback.

            estimate = np.array([
                initial_truth[0],
                initial_truth[1],
                1.0,
            ])

            rc = controller.command(
                0,
                target,
                estimate,
                dt=1.0 / 25.0,
                target_velocity=target_velocity,
                target_acceleration=target_acceleration,
            )

            sim.set_rc(0, rc)
            if step % 240 == 0:
                print(
                    f"t={t:.2f}, "
                    f"target_z={target[2]:.3f}, "
                    f"actual_z={sim.ground_truth()[0, 2]:.3f}, "
                    f"throttle={rc[3]}"
                )

            positions.append(sim.ground_truth()[0, 2])

            z = np.asarray(positions)

        assert z.max() - z.min() >= 0.20, (
            f"Insufficient physical vertical response: "
            f"range={z.max() - z.min():.3f} m"
        )

    finally:
        sim.close()

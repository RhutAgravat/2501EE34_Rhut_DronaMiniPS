import numpy as np
from config import load_config
from simulator import SwarmSim


def test_constant_throttle_produces_vertical_motion():
    cfg = load_config()
    sim = SwarmSim(n=1, gui=False, config=cfg)

    try:
        z0 = sim.ground_truth()[0, 2]

        # Apply a fixed throttle above hover for one second.
        sim.set_rc(0, [1500, 1500, 1500, 1400])

        for _ in range(cfg["physics"]["hz"]):
            sim.step()

        z1 = sim.ground_truth()[0, 2]

        print(f"\nInitial altitude: {z0:.4f} m")
        print(f"Final altitude:   {z1:.4f} m")
        print(f"Altitude change:  {z1-z0:.4f} m")

        assert z1 > z0 + 0.05, (
            "Constant throttle did not produce expected upward motion"
        )

    finally:
        sim.close()
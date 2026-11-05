from pathlib import Path


def test_horizontal_rc_has_control_authority_in_pybullet_proxy():
    from config import load_config
    from simulator import SwarmSim

    sim = SwarmSim(1, gui=False, config=load_config(Path(__file__).parents[1] / "config.yaml"))
    try:
        sim.set_rc(0, [1500, 2000, 1500, 1870])
        sim.step(240)
        assert sim.ground_truth()[0, 0] > 0.10
    finally:
        sim.close()

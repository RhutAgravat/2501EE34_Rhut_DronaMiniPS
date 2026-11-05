from pathlib import Path
import numpy as np


def test_config_loads_all_sections():
    from config import load_config
    cfg = load_config(Path(__file__).parents[1] / "config.yaml")
    assert cfg["physics"]["hz"] >= 240
    assert cfg["control"]["hz"] == 25
    assert cfg["arena"]["min_separation"] == 0.35


def test_required_modules_import():
    import simulator, pluto_interface, planner, validator, camera
    import localization, controller, safety, metrics, logger
    assert all(m is not None for m in (simulator, pluto_interface, planner, validator,
                                       camera, localization, controller, safety, metrics, logger))


def test_line_and_circle_shapes():
    from planner import line_slots, circle_slots
    line = line_slots(4, 0.4, 1.0)
    circle = circle_slots(4, 0.3, 1.0)
    assert line.shape == (4, 3) and circle.shape == (4, 3)
    assert np.allclose(line[:, 2], 1.0) and np.allclose(circle[:, 2], 1.0)


def test_wave_derivatives_match_finite_difference():
    from planner import wave_state
    t, dt = 1.7, 1e-5
    p, v, a = wave_state(t, 0.3, "line", 2)
    p0 = wave_state(t-dt, 0.3, "line", 2)[0]
    p1 = wave_state(t+dt, 0.3, "line", 2)[0]
    assert abs(v - (p1-p0)/(2*dt)) < 1e-5
    assert abs(a - (p1-2*p+p0)/(dt*dt)) < 2e-3


def test_validator_reports_invalid_plan():
    from planner import line_slots
    from validator import validate_slots
    result = validate_slots(line_slots(10, 0.4, 1.0))
    assert not result.ok
    assert result.failures and result.failures[0].drone_ids


def test_simulator_interface_and_rc_clamp():
    from simulator import SwarmSim
    from pluto_interface import PlutoInterface
    sim = SwarmSim(1, gui=False)
    try:
        p = PlutoInterface(sim)
        assert p.arm(0)
        out = p.send_rc(0, [500, 2500, 1500, 1500])
        assert out == [1000, 2000, 1500, 1500]
        sim.step(8)
        assert sim.battery(0) < 1.0
    finally:
        sim.close()

def test_simulator_body_mass_matches_configured_mass():
    import pybullet as p
    from simulator import SwarmSim
    sim = SwarmSim(1, gui=False)
    try:
        assert p.getDynamicsInfo(sim.ids[0], -1, physicsClientId=sim.client)[0] == sim.cfg['physics']['mass']
    finally:
        sim.close()

def test_visual_proxy_attitude_changes_deterministically_with_rc():
    import pybullet as p
    from simulator import SwarmSim
    from pluto_interface import PlutoInterface
    sim = SwarmSim(1, gui=False)
    try:
        radio = PlutoInterface(sim)
        radio.arm(0)
        radio.send_rc(0, [1700, 1500, 1600, 1900])
        sim.step(24)
        first = np.asarray(p.getBasePositionAndOrientation(sim.ids[0], physicsClientId=sim.client)[1])
        sim.step(24)
        second = np.asarray(p.getBasePositionAndOrientation(sim.ids[0], physicsClientId=sim.client)[1])
        assert not np.allclose(first, [0, 0, 0, 1])
        assert not np.allclose(first, second)
        assert sim.visual_attitude(0)["spin_phase"] > 0
    finally:
        sim.close()

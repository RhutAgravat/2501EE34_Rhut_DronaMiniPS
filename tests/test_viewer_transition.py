import numpy as np


def test_circle_viewer_transition_keeps_all_markers_and_avoids_safety_landings():
    from config import load_config
    from simulator import SwarmSim
    from pluto_interface import PlutoInterface
    from planner import line_slots, circle_slots, circle_transition_targets, mission_targets
    from camera import OverheadCamera
    from localization import Localizer
    from controller import CascadedPID
    from safety import SafetySupervisor

    cfg = load_config()
    n = 4
    sim = SwarmSim(n, False, cfg)
    radio = PlutoInterface(sim)
    camera, localizer = OverheadCamera(cfg), Localizer(cfg, n)
    controller, safety = CascadedPID(cfg), SafetySupervisor(cfg)
    initial = line_slots(n, cfg['formation']['line_spacing'], cfg['formation']['height'])
    circle = circle_slots(n, cfg['formation']['circle_radius'], cfg['formation']['height'])
    mission_circle = circle[[2, 1, 3, 0]]
    stride = round(cfg['physics']['hz'] / cfg['control']['hz'])
    for i in range(n):
        radio.arm(i)
        radio.send_rc(i, [1500, 1500, 1500, cfg['control']['hover_rc']])
    try:
        for step in range(int(15 * cfg['physics']['hz'])):
            sim.step()
            if step % stride:
                continue
            now = sim.t
            target = circle_transition_targets(now / 5.0, initial, circle)
            if now >= 5.0:
                target, _, _ = mission_targets(now - 5.0, mission_circle, 'circle')
            estimates = localizer.update(camera.capture(sim.ground_truth(), now), now)
            ready = all(estimates[i]['position'] is not None and np.isfinite(estimates[i]['position']).all() for i in range(n))
            actions = safety.check(estimates, now) if ready else {}
            assert 'land' not in actions.values()
            for i in range(n):
                rc = [1500, 1500, 1500, cfg['control']['hover_rc']] if not ready else controller.command(i, target[i], estimates[i]['position'])
                radio.send_rc(i, rc)
        assert not any(event[1] == 'lost' for event in safety.events)
    finally:
        sim.close()

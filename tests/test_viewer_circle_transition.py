import numpy as np


def test_nominal_circle_transition_keeps_all_markers_valid_without_landing():
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
    initial = line_slots(n, cfg['formation']['line_spacing'], 1.0)
    circle = circle_slots(n, cfg['formation']['circle_radius'], 1.0)
    mission_circle = circle[[2, 1, 3, 0]]
    stride = round(cfg['physics']['hz'] / cfg['control']['hz'])
    saw_ready = False
    try:
        for i in range(n):
            radio.arm(i)
            radio.send_rc(i, [1500, 1500, 1500, cfg['control']['hover_rc']])
        for step in range(int(15 * cfg['physics']['hz'])):
            sim.step()
            if step % stride:
                continue
            t = sim.t
            target = circle_transition_targets(t / 5.0, initial, circle)
            if t >= 5.0:
                target, _, _ = mission_targets(t - 5.0, mission_circle, 'circle')
            estimates = localizer.update(camera.capture(sim.ground_truth(), t), t)
            ready = all(e['position'] is not None and np.isfinite(e['position']).all() for e in estimates.values())
            actions = safety.check(estimates, t) if ready else {}
            if ready:
                saw_ready = True
                assert not any(action == 'land' for action in actions.values())
            for i in range(n):
                rc = [1500, 1500, 1500, cfg['control']['hover_rc']] if not ready else controller.command(i, target[i], estimates[i]['position'])
                radio.send_rc(i, rc)
        assert saw_ready
        assert not any(kind in {'lost', 'separation_breach', 'geofence_breach'} for _, kind, _ in safety.events)
    finally:
        sim.close()

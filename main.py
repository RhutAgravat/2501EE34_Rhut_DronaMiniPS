
"""Headless SwarmSim-Lite mission runner with estimate-only control."""
import argparse
import json
from pathlib import Path

import numpy as np

from config import load_config
from simulator import SwarmSim
from pluto_interface import PlutoInterface
from planner import (
    line_slots,
    circle_slots,
    smoothstep5,
    mission_targets,
    circle_transition_targets,
)
from camera import OverheadCamera
from localization import Localizer
from controller import CascadedPID
from safety import SafetySupervisor
from logger import MissionLogger
from metrics import calculate


def run(name='line', n=4, duration=2.0, root='.'):
    cfg = load_config(Path(root) / 'config.yaml')

    kind = 'circle' if 'circle' in name else 'line'

    line_spacing = float(cfg['formation'].get('line_spacing', .4))

    base = (
        line_slots(n, line_spacing, 1.)
        if kind == 'line'
        else circle_slots(n, .42, 1.)
    )

    mission_base = (
        base[[2, 1, 3, 0]]
        if kind == 'circle' and n == 4
        else base
    )

    initial = line_slots(n, line_spacing, 1.)

    sim = SwarmSim(n, False, cfg)
    radio = PlutoInterface(sim)

    cam = OverheadCamera(cfg)
    loc = Localizer(cfg, n)
    ctl = CascadedPID(cfg)

    safe = SafetySupervisor(cfg)
    log = MissionLogger(
        Path(root) / 'logs',
        Path(root) / 'plots'
    )

    # Arm drones and initialize at calibrated hover throttle.
    for i in range(n):
        radio.arm(i)
        radio.send_rc(
            i,
            [1500, 1500, 1500, cfg["control"]["hover_rc"]]
        )

    truth, est, targets = [], [], []

    status_counts = {
        'valid': 0,
        'stale': 0,
        'lost': 0
    }

    control_stride = max(
        1,
        round(cfg['physics']['hz'] / cfg['control']['hz'])
    )

    try:
        for step in range(int(duration * cfg['physics']['hz'])):

            sim.step()

            if step % control_stride:
                continue

            t = sim.t

            # Ground truth is used only for camera rendering and evaluation.
            gt = sim.ground_truth()

            frame = cam.capture(gt, t)
            estimates = loc.update(frame, t)

            actions = safe.check(estimates, t)

            # Formation transition.
            if kind == 'circle':
                formation_target = circle_transition_targets(
                    t / 5.0,
                    initial,
                    base
                )
            else:
                s, _, _ = smoothstep5(min(1.0, t / 5.0))
                formation_target = initial + s * (base - initial)

            target = formation_target

            # Initialize feedforward terms.
            target_velocity = np.zeros((n, 3))
            target_acceleration = np.zeros((n, 3))

            # Start wave only after formation transition.
            if 'wave' in name and t >= 5.0:
                (
                    target,
                    target_velocity,
                    target_acceleration,
                ) = mission_targets(
                    t - 5.0,
                    mission_base,
                    kind,
                )

            estimate_array = np.full((n, 3), np.nan)

            for i in range(n):

                position = estimates[i].get('position')

                status_counts[
                    estimates[i].get('status', 'lost')
                ] += 1

                if position is not None:
                    estimate_array[i] = position

                # Safety override: land.
                if actions.get(i) == "land":
                    radio.send_rc(
                        i,
                        [1500, 1500, 1500, 1000]
                    )

                # Safety override: hold at hover.
                elif actions.get(i) == "hold":
                    radio.send_rc(
                        i,
                        [
                            1500,
                            1500,
                            1500,
                            cfg["control"]["hover_rc"]
                        ]
                    )

                # Normal cascaded PID control.
                else:
                    radio.send_rc(
                        i,
                        ctl.command(
                            i,
                            target[i],
                            position,
                            dt=1.0 / cfg["control"]["hz"],
                            target_velocity=target_velocity[i],
                            target_acceleration=target_acceleration[i],
                        )
                    )

            targets.append(target.copy())
            truth.append(gt.copy())
            est.append(estimate_array)

            valid = np.isfinite(estimate_array).all(axis=1)

            error = (
                float(
                    np.mean(
                        np.linalg.norm(
                            target[valid] - estimate_array[valid],
                            axis=1
                        )
                    )
                )
                if np.any(valid)
                else float('nan')
            )

            log.add({
                'time': t,
                'error': error,
                'events': len(safe.events)
            })

        log.save(name)

        met = calculate(
            np.array(targets),
            np.array(est),
            np.array(truth),
            safe.events,
            status_counts,
            loc.rejected
        )

        met.update({
            'mission': name,
            'samples': len(targets),
            'rejected_outliers': loc.rejected,
            'stale_lost_events': len(safe.events),
            'detector_pipeline': 'aruco+PnP',
            'ground_truth_control_feedback': False
        })

        out = Path(root) / 'logs' / f'{name}_metrics.json'

        out.write_text(json.dumps(met, indent=2))

        return met

    finally:
        sim.close()


if __name__ == '__main__':

    ap = argparse.ArgumentParser()

    ap.add_argument('--mission', default='line')
    ap.add_argument('--duration', type=float, default=2.)
    ap.add_argument('--root', default='.')

    a = ap.parse_args()

    print(
        json.dumps(
            run(a.mission, 4, a.duration, a.root),
            indent=2
        )
    )

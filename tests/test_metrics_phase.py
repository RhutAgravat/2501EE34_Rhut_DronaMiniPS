import numpy as np


def test_phase_metric_is_zero_for_identical_phased_wave_estimates():
    from metrics import calculate
    samples, drones = 240, 4
    t = np.arange(samples) * 0.025
    target = np.zeros((samples, drones, 3), dtype=float)
    target[:, :, 0] = np.arange(drones)[None, :] * 0.4
    for drone in range(drones):
        target[:, drone, 2] = 1.0 + 0.2 * np.sin(2 * np.pi * t / 6.0 - drone * np.pi / 2)
    result = calculate(target, target.copy())
    assert result['phase_synchronization_error_deg'] is not None
    assert result['phase_synchronization_error_deg'] < 1e-6


def test_phase_metric_detects_quarter_period_estimate_lag():
    from metrics import calculate
    samples, drones = 240, 4
    t = np.arange(samples) * 0.025
    target = np.zeros((samples, drones, 3), dtype=float)
    target[:, :, 0] = np.arange(drones)[None, :] * 0.4
    for drone in range(drones):
        target[:, drone, 2] = 1.0 + 0.2 * np.sin(2 * np.pi * t / 6.0 - drone * np.pi / 2)
    estimate = target.copy()
    estimate[:, :, 2] = 1.0 + 0.2 * np.sin(2 * np.pi * t[:, None] / 6.0 - np.arange(drones)[None, :] * np.pi / 2 - np.pi / 2)
    result = calculate(target, estimate)
    assert 70.0 < result['phase_synchronization_error_deg'] < 110.0

import numpy as np


def test_circle_wave_entry_path_preserves_required_separation():
    from planner import circle_transition_targets, circle_slots, line_slots

    initial = line_slots(4, 0.4, 1.0)
    circle = circle_slots(4, 0.42, 1.0)
    samples = np.array([circle_transition_targets(u, initial, circle) for u in np.linspace(0.0, 1.0, 401)])
    distances = np.linalg.norm(samples[:, :, None, :] - samples[:, None, :, :], axis=-1)
    pairwise = distances[:, np.triu_indices(4, 1)[0], np.triu_indices(4, 1)[1]]
    assert float(np.min(pairwise)) >= 0.35

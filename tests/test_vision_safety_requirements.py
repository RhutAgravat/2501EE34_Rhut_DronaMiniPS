import numpy as np


def test_rendered_aruco_frame_is_detected_and_pose_estimated():
    from camera import OverheadCamera
    from localization import Localizer
    from config import load_config

    cfg = load_config()
    cfg["camera"].update(drop_probability=0.0, pixel_noise_sigma=0.0, latency_s=0.08)
    camera = OverheadCamera(cfg, seed=1)
    localizer = Localizer(cfg, 1)
    pose = np.array([[0.20, -0.10, 1.0]])
    assert camera.capture(pose, 0.00) is None
    frame = camera.capture(pose, 0.08)
    estimates = localizer.update(frame, 0.08)
    assert estimates[0]["status"] == "valid"
    assert np.linalg.norm(estimates[0]["position"] - pose[0]) < 0.03
    assert abs(estimates[0]["yaw"]) < 0.05
    assert frame.capture_timestamp == 0.0
    assert frame.delivered_timestamp == 0.08


def test_camera_output_contains_no_ground_truth_position():
    from camera import OverheadCamera
    from config import load_config

    cfg = load_config()
    cfg["camera"].update(drop_probability=0.0, pixel_noise_sigma=0.0)
    frame = OverheadCamera(cfg, seed=2).capture(np.array([[0.0, 0.0, 1.0]]), 0.0)
    assert frame is None


def test_safety_overrides_separation_and_geofence():
    from config import load_config
    from safety import SafetySupervisor

    safety = SafetySupervisor(load_config())
    estimates = {
        0: {"position": np.array([0.0, 0.0, 1.0]), "status": "valid"},
        1: {"position": np.array([0.1, 0.0, 1.0]), "status": "valid"},
        2: {"position": np.array([0.0, 0.0, 1.0]), "status": "valid"},
        3: {"position": np.array([2.0, 0.0, 1.0]), "status": "valid"},
    }
    actions = safety.check(estimates, 1.0)
    assert actions[0] == actions[1] == actions[2] == "hold"
    assert actions[3] == "land"
    assert any(event[1] == "separation_breach" for event in safety.events)
    assert any(event[1] == "geofence_breach" for event in safety.events)

# Compliance Matrix

| Requirement | Implementation file(s) | Test/evidence | Status |
|---|---|---|---|
| Real PyBullet physics, 240 Hz | `simulator.py` | `pytest tests -q` (21 passed); missions | implemented |
| Exclusive Pluto RC interface | `pluto_interface.py` | RC/arm tests | implemented |
| Configurable parameters | `config.yaml`, `config.py` | config tests | implemented |
| Generic line/circle/Hungarian planner | `planner.py` | formation tests; circle transition safety regression | implemented |
| Quintic transitions and waves | `planner.py` | derivative tests | implemented |
| 0.02 s preflight validator/N=10 | `validator.py` | validator tests | implemented |
| 8 cm DICT_4X4_50 rendered markers, warpPerspective, ArUco detect, PnP | `camera.py`, `localization.py` | `test_vision_safety_requirements.py`; 80 ms frame metadata | implemented |
| Delayed/noisy/dropout/outlier behavior | `camera.py`, `localization.py` | 80 ms frame metadata, F1/F3 | implemented |
| Estimated-pose-only controller feedback | `main.py`, `controller.py` | mission output `ground_truth_control_feedback: false`; source path audit | implemented |
| Cascaded estimated-pose PID, smoothing, anti-windup, tilt/RC limits, yaw hold | `controller.py` | `tests -q`; line/circle/wave missions | implemented |
| Separation/geofence safety overrides | `safety.py`, `main.py` | safety test; mission/fault checks | implemented |
| Logs, plots, RMS/shape/phase/separation/localization/fault metrics | `logger.py`, `metrics.py` | `logs/*_metrics.json`; 21 tests; circle-wave RMS 0.0826000338 m and separation 0.3938353615 m | implemented |
| README/report/final status | documentation files | final verification commands below | updated |

Ground truth is evaluation-only: it is used to synthesize camera pixels and calculate
metrics, but the controller receives only `Localizer` output (or `None`).


# Report notes

## Status labels
- **Implemented:** modular PyBullet backend, exclusive RC interface, generic formations, Hungarian assignment, quintic helper, analytical waves, preflight checks, rendered 8 cm `DICT_4X4_50` marker images via pinhole projection and `cv2.warpPerspective`, OpenCV ArUco detection, PnP pose/yaw estimation, noise/drop/80 ms delayed frames, smoothing, status/jump rejection, estimated-pose cascaded PID, deterministic visual attitude/spin proxy, separation/geofence safety overrides, mission logs/plots and expanded metrics.
- **Measured:** `python -m pytest tests -q` = 21 passed. Line, circle, line-wave and circle-wave missions completed; targeted simulator-authority and transition-safety tests passed.
- **Theoretically derived:** wave velocity/acceleration use derivatives of `A sin(ωt-φ)`; minimum separation is pairwise Euclidean distance; pinhole projection is `u=fx*x/z+cx`, `v=cy-fy*y/z`.
- **Methodology-only:** full quadrotor underactuation, Kalman delay compensation, yaw drift identification, downwash, thrust/tilt feasibility analysis, scaling study, F2 (1.2 s hide), and F4 (firmware failsafe).

## Latest measured mission metrics
- 2 s line: RMS 0.0061668810 m; formation shape RMS 0.0040440876 m; localization RMS 0.0084113052 m; minimum separation 0.4432461294 m.
- 2 s circle: RMS 0.0340043436 m; formation shape RMS 0.0309795135 m; localization RMS 0.0099665264 m; minimum separation 0.4054022029 m.
- 6 s line wave: RMS 0.0581039660 m; shape RMS 0.0217944030 m; localization RMS 0.0092812861 m; minimum separation 0.4432461294 m.
- 6 s circle wave: RMS 0.0826000338 m; shape RMS 0.0614634793 m; localization RMS 0.0150712256 m; minimum separation 0.3938353615 m.
- All metrics include valid sample counts, stale/lost counts, rejected outlier count, safety event count and `ground_truth_control_feedback: false`.

## Limitations
The low-level vehicle remains a force-actuated PyBullet proxy rather than a full quadrotor motor/firmware model. The camera is a synthetic planar-marker render, not a physical camera. Circle-wave meets the stated 6 s RMS and minimum-separation targets in the verified estimate-only run. F2/F4 are not completed as executable firmware tests. ROS 2, Docker, CUDA and MuJoCo are intentionally not used.


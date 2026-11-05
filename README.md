# DronaSwarm SwarmSim-Lite

A modular PyBullet swarm simulation. `simulator.py` uses real PyBullet dynamics and a visible, deterministic RC-driven attitude/spin proxy. Ground truth is used only to synthesize camera pixels and calculate evaluation metrics; the controller receives only `Localizer` estimates.

## Run
```text
.venv\\Scripts\\activate
.venv\\Scripts\\python.exe -m pytest tests -q
.venv\\Scripts\\python.exe main.py --mission line --duration 2
.venv\\Scripts\\python.exe main.py --mission circle --duration 2
.venv\\Scripts\\python.exe main.py --mission line_wave --duration 6
.venv\\Scripts\\python.exe main.py --mission circle_wave --duration 6
.venv\\Scripts\\python.exe fault_drills.py
```

Artifacts are written to `logs/` and `plots/`. Configuration is in `config.yaml`. Invalid N=10 validator demonstrations are stored as `logs/invalid_*_validation.json`.

## Implemented camera/control path
- 8 cm `DICT_4X4_50` markers rendered into overhead pinhole images with `warpPerspective`.
- Pixel noise, probabilistic frame drops, explicit 80 ms capture/delivery timestamps, ArUco detection, PnP pose, smoothing, stale/lost status, and jump rejection.
- Cascaded position-to-velocity / velocity-to-RC control with derivative smoothing, integral clamping, calibrated hover throttle, yaw hold, tilt and RC saturation.
- Safety overrides for stale/hidden/lost drones, geofence breaches, and separation hold-then-land.

## Limitations
The low-level vehicle remains a force-actuated PyBullet proxy rather than a full quadrotor motor/firmware model. The camera is a synthetic planar-marker render. Kalman delay compensation, downwash, yaw-drift identification, firmware failsafe validation, F2 (1.2 s hide) and F4 remain methodology-only. Mission metrics are reported honestly; wave runs can exceed the 10 cm tracking target.

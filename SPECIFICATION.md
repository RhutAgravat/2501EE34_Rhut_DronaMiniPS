# SwarmSim-Lite Specification

## Fixed constants
- Python 3.10+; real physics simulator: PyBullet (preferred), no hand-coded physics model.
- Generic `N`; demonstrations use four nano-drones.
- Arena: `x=[-1.0,1.0] m`, `y=[-0.6,0.6] m`, `z=[0.6,1.4] m`.
- Minimum separation: `0.35 m`; control loop `25 Hz`; physics loop at least `240 Hz`.
- Line height `1.0 m`; wave amplitude `0.2 m`; period `6 s`; three wave cycles.
- Camera: overhead `(0,0,3)`, downward; `1280x720`; `fx=fy=900`; principal point `(640,360)`; 8 cm ArUco markers, `DICT_4X4_50`, marker ID equals drone ID.
- Imperfections: fixed per-axis drift up to `±0.15 m/s²`; thrust maximum linearly sags 100%→93% over 120 s; camera latency 80 ms; drop probability 5%; Gaussian pixel noise sigma 3 px.
- RC values `1000..2000`: roll/pitch ±15°, yaw ±90°/s, throttle 1000 zero and 2000 maximum; 1500 is not assumed hover.

## Required modules and architecture
- `config.yaml` holds every tunable value: simulation frequencies, arena, formations/wave, camera/noise, disturbances, PID, RC, safety.
- PyBullet simulation exposes physical state but only evaluation/logging may read ground truth outside simulator.
- `PlutoInterface` is the exclusive command route and implements arm/disarm, RC clamp, RC-to-actuation mapping; low-level attitude logic is confined and documented there.
- Planner supplies generic line/circle slots, Hungarian slot assignment, shared clock, quintic transitions, mission sequence: takeoff, formation, 5 s hold, 3 waves, 3 s hold, land.
- Vision produces delayed/noisy marker images through pinhole projection and perspective warping, detects ArUco, estimates x/y/z/yaw, rejects impossible jumps and exposes valid/stale/lost statuses.
- Controller is cascaded PID: position→velocity→RC, yaw hold, filtering, saturation and anti-windup; its feedback interface only accepts localization estimates.
- Safety supervisor owns stale/lost/separation/geofence overrides and timestamped events.
- Monitoring/metrics generate CSV logs, PNG plots, live/optional dashboard, target/estimated/ground-truth trails and geofence.

## Required trajectories
- Line slots use configured spacing and height 1 m. Circle slots use configured radius and `theta_i=2πi/N`.
- Line wave: `z_i=1+0.2 sin(ωt-iπ/2)`.
- Circle wave: `z_i=1+0.2 sin(ωt-theta_i)`.
- Minimum-jerk transitions and analytical wave velocity/acceleration are required.

## Preflight validation
Sample complete plans every 0.02 s; reject nonfinite values, geofence violations and separation below 0.35 m. Test N=10 line and circle and report exact time, drone IDs, measured/required values and suggested changes. Keep interfaces for future velocity/acceleration/thrust/tilt checks.

## Fault drills
- F1: hide drone 2 marker for 0.4 s.
- F2 documentation: hide drone 3 marker for 1.2 s.
- F3: inject one false 1 m position jump for drone 1.
- F4 documentation: drone 0 gets no command for 1 s and firmware failsafe activates.

## Required metrics and acceptance targets
RMS tracking error (target ≤0.10 m), formation shape error, phase synchronization error (target ≤10°), minimum separation (≥0.35 m), localization RMS, max overshoot, stale/lost/rejected-outlier counts and safety event counts.

## Deliverables
`requirements.txt`, `config.yaml`, modular source, `tests/`, CSV logs, PNG plots, README with setup/run/fault commands and limitations, `report_notes.md`, `FINAL_STATUS.md`, phase checkpoints and autonomous state. ROS 2 is optional bonus only; Docker/CUDA/MuJoCo are not required.

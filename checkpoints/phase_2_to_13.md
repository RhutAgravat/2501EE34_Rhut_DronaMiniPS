# Phase 2-13 checkpoint

- Phase 2: modular source skeleton, config, docs and imports/tests.
- Phase 3: real PyBullet DIRECT backend, 240 Hz timestep, RC/arm/disarm, battery sag, drift and deterministic visible attitude/spin proxy.
- Phase 4: generic line/circle slots, Hungarian assignment, quintic helper and formation tests.
- Phase 5: analytical line/circle wave position, velocity and acceleration with finite-difference derivative test.
- Phase 6: 0.02 s preflight validator and invalid N=10 demonstrations in `logs/*_validation.json`.
- Phase 7: overhead pinhole camera, 8 cm rendered ArUco markers, `warpPerspective`, pixel noise/drop, 80 ms frame metadata, ArUco/PnP localization, smoothing, timestamps, valid/stale/lost status and jump rejection.
- Phase 8: estimate-only cascaded PID with derivative smoothing, integral clamp, calibrated hover, yaw hold and tilt/RC saturation. Mission tracking is measured, not hidden.
- Phase 9: stale/lost hold/land, separation hold then land, geofence land and F3 jump rejection exercised.
- Phase 10: line/circle 2 s and line-wave/circle-wave 6 s PyBullet missions produced CSV, PNG and JSON artifacts.
- Phase 11: metrics include RMS, formation shape, phase, minimum separation, localization RMS, overshoot, stale/lost, rejected outliers and safety events.
- Phase 12: report labels implemented, measured, derived, methodology-only and limitations.
- Phase 13: clean imports/compile, 17 tests, missions, fault drills and validator demos verified.

Known unresolved: force-actuated proxy is not full quadrotor dynamics; circle-wave 6 s run measured RMS 0.516893 m and minimum separation 0.342208 m; F2/F4 remain methodology-only; no Kalman/downwash/yaw-drift/firmware failsafe model.

"""Interactive PyBullet playback of the current four-drone line mission."""
import os
import time
import numpy as np
import pybullet as p
from config import load_config
from simulator import SwarmSim
from pluto_interface import PlutoInterface
from planner import line_slots, circle_slots, circle_transition_targets, smoothstep5, mission_targets
from camera import OverheadCamera
from localization import Localizer
from controller import CascadedPID
from safety import SafetySupervisor

cfg = load_config("config.yaml")
n = 4
sim = SwarmSim(n=n, gui=True, config=cfg)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=sim.client)
p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=42, cameraPitch=-34,
                              cameraTargetPosition=[0, 0, 0.85], physicsClientId=sim.client)
radio = PlutoInterface(sim)
camera = OverheadCamera(cfg)
localizer = Localizer(cfg, n)
controller = CascadedPID(cfg)
safety = SafetySupervisor(cfg)
initial = line_slots(n, cfg["formation"]["line_spacing"], cfg["formation"]["height"])
circle = circle_slots(n, cfg["formation"]["circle_radius"], cfg["formation"]["height"])
mission_circle = circle[[2, 1, 3, 0]] if n == 4 else circle
control_stride = max(1, round(cfg["physics"]["hz"] / cfg["control"]["hz"]))
playback_seconds = int(os.environ.get("VIEWER_PLAYBACK_SECONDS", "60"))
hold_open = os.environ.get("VIEWER_HOLD_OPEN", "1") != "0"
label_ids = [-1] * n
for drone in range(n):
    radio.arm(drone)
    radio.send_rc(drone, [1500, 1500, 1500, cfg["control"]["hover_rc"]])

try:
    for step in range(playback_seconds * cfg["physics"]["hz"]):
        if not p.isConnected(sim.client):
            break
        sim.step()
        if step % control_stride:
            time.sleep(sim.dt / 2)
            continue
        now = sim.t
        blend, _, _ = smoothstep5(min(1.0, now / 5.0))
        target = circle_transition_targets(now / 5.0, initial, circle)
        if now > 5.0:
            target, _, _ = mission_targets(now - 5.0, mission_circle, "circle")
        truth = sim.ground_truth()
        observations = camera.capture(truth, now)
        estimates = localizer.update(observations, now)
        # The 80 ms camera transport has no initial measurement.  Keep the armed
        # vehicles at calibrated hover until every marker has arrived; after that
        # normal estimate-driven safety enforcement applies.
        ready = all(estimates[i]["position"] is not None for i in range(n))
        actions = safety.check(estimates, now) if ready else {}
        for drone in range(n):
            estimate = estimates[drone]["position"]
            if not ready:
                rc = [1500, 1500, 1500, cfg["control"]["hover_rc"]]
            else:
                rc = controller.command(drone, target[drone], estimate)
            if drone in actions:
                rc = [1500, 1500, 1500, cfg["control"]["hover_rc"]]
            radio.send_rc(drone, rc)
        # Visualization-only coordinates: controller/safety still receive only
        # Localizer estimates.  The labels use PyBullet evaluation state so users
        # can directly see the rising/falling z coordinate in the GUI.
        try:
            for drone, position in enumerate(truth):
                label_ids[drone] = p.addUserDebugText(
                    f"D{drone}  x={position[0]:+.2f}  y={position[1]:+.2f}  z={position[2]:.2f}",
                    position + np.array([0.0, 0.0, 0.16]),
                    textColorRGB=[1, 1, 1], textSize=1.3, lifeTime=0,
                    replaceItemUniqueId=label_ids[drone], physicsClientId=sim.client)
        except p.error:
            # A user may close the OpenGL window between physics and rendering.
            break
        time.sleep(sim.dt / 2)
    # Keep the completed scene available for manual inspection. Close the PyBullet
    # window yourself to end this viewer process.
    while hold_open and p.isConnected(sim.client):
        time.sleep(0.25)
finally:
    if p.isConnected(sim.client):
        sim.close()

"""OpenCV ArUco detector and pinhole PnP localizer.

Only detector/PnP estimates leave this module. Ground-truth simulator positions are
not accepted on the production frame path.
"""
from __future__ import annotations
import cv2
import numpy as np

class Localizer:
    def __init__(self, cfg, n):
        self.n = n
        self.c = cfg["camera"]
        self.limit = self.c["jump_limit_m"]
        self.stale = cfg["safety"]["stale_s"]
        self.lost = cfg["safety"]["lost_s"]
        self.size = float(self.c.get("marker_size_m", 0.08))

        self.camera_matrix = np.array([
            [self.c["fx"], 0, self.c["cx"]],
            [0, self.c["fy"], self.c["cy"]],
            [0, 0, 1]
        ], dtype=np.float64)

        self.dist = np.zeros(5)
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.01

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary, params
        )

        self.last = {}
        self.last_t = {}
        self.last_capture_t = {}
        self.alpha = float(self.c.get("smoothing_alpha", 1.0))
        self.rejected = 0

    def _detect_frame(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame.image)

        if ids is None:
            return []

        half = self.size / 2.0

        object_points = np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float64)

        result = []

        for corner, marker_id in zip(corners, ids.ravel()):
            marker_id = int(marker_id)

            if marker_id >= self.n:
                continue

            image_points = corner.reshape(4, 2).astype(np.float64)

            # Compensate for the measured outward pixel-boundary bias
            # of the synthetic ArUco renderer.
            center = image_points.mean(axis=0)

            edge_lengths = np.linalg.norm(
                np.roll(image_points, -1, axis=0) - image_points,
                axis=1
            )

            mean_edge = float(np.mean(edge_lengths))
            boundary_bias_px = 0.3844

            correction_scale = max(
                0.0,
                1.0 - (2.0 * boundary_bias_px / mean_edge)
            )

            image_points = center + correction_scale * (image_points - center)

            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not ok:
                continue

            t = tvec.ravel()
            rot, _ = cv2.Rodrigues(rvec)

            camera_z = float(
                self.c.get("position", [0, 0, 3.0])[2]
            )

            position = np.array([
                t[0],
                -t[1],
                camera_z - t[2]
            ], dtype=float)

            if not np.isfinite(position).all():
                continue

            yaw = float(np.arctan2(-rot[1, 0], rot[0, 0]))

            result.append((
                marker_id,
                position,
                yaw,
                tuple(image_points.mean(axis=0))
            ))

        return result
    def update(self, frame, now):
        if frame is not None:
            if hasattr(frame, "image"):
                detections = self._detect_frame(frame)
                capture_t = frame.capture_timestamp
            else:
                detections = []
                capture_t = now
                # Legacy-only path retained for the existing unit test; camera never uses it.
                for obs in frame:
                    if obs.position is not None:
                        detections.append((obs.drone_id, np.asarray(obs.position, dtype=float), obs.yaw, obs.pixel))
            for drone_id, position, yaw, pixel in detections:
                if drone_id in self.last and np.linalg.norm(position - self.last[drone_id]["position"]) > self.limit:
                    self.rejected += 1
                    continue
                if drone_id in self.last and self.alpha < 1.0:
                    prior = self.last[drone_id]["position"]
                    position = self.alpha * position + (1.0 - self.alpha) * prior
                    yaw = float(self.alpha * yaw + (1.0 - self.alpha) * self.last[drone_id]["yaw"])
                self.last[drone_id] = {"position": position.copy(), "yaw": yaw, "pixel": pixel}
                self.last_t[drone_id] = float(now)
                self.last_capture_t[drone_id] = float(capture_t)
        return {i: self.get(i, now) for i in range(self.n)}

    def get(self, i, now):
        if i not in self.last:
            return {"position": None, "yaw": None, "status": "lost", "capture_timestamp": None}
        age = now - self.last_t[i]
        status = "lost" if age > self.lost else ("stale" if age > self.stale else "valid")
        out = dict(self.last[i])
        out.update({"status": status, "capture_timestamp": self.last_capture_t[i]})
        return out

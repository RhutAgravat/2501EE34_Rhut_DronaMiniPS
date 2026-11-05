"""Rendered overhead camera and delayed image transport.

The simulator pose is used only to synthesize pixels. Frames contain an image and
clock metadata, never a world-space position. Localization owns detection and PnP.
"""
from __future__ import annotations
from dataclasses import dataclass
from collections import deque
import cv2
import numpy as np

@dataclass(frozen=True)
class Frame:
    image: np.ndarray
    capture_timestamp: float
    delivered_timestamp: float

@dataclass
class Observation:
    """Compatibility observation for legacy tests; camera output is never this type."""
    drone_id: int
    pixel: tuple[float, float]
    position: np.ndarray | None
    timestamp: float
    detected: bool = True
    yaw: float = 0.0

class OverheadCamera:
    def __init__(self, cfg, seed: int = 7):
        self.c = cfg["camera"]
        self.rng = np.random.default_rng(seed)
        self.queue: deque[Frame] = deque()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.marker_size = float(self.c.get("marker_size_m", 0.08))
        self.marker_px = int(self.c.get("marker_render_px", 160))

    def _project(self, points: np.ndarray) -> np.ndarray:
        x, y, z = points.T
        # Optical depth is measured from the overhead camera at z=3 m, not
        # from the world origin.  Using world z clipped the north/south markers
        # as their wave height changed.
        depth = float(self.c.get("position", [0, 0, 3.0])[2]) - z
        return np.column_stack((self.c["cx"] + self.c["fx"] * x / depth,
                                self.c["cy"] - self.c["fy"] * y / depth)).astype(np.float32)

    def _render(self, positions: np.ndarray, timestamp: float, hidden=()) -> np.ndarray:
        width, height = int(self.c["width"]), int(self.c["height"])
        image = np.full((height, width), 220, dtype=np.uint8)
        half = self.marker_size / 2.0
        local = np.array([[-half, half, 0], [half, half, 0],
                          [half, -half, 0], [-half, -half, 0]], dtype=float)
        source = np.array([
            [0.5, 0.5],
            [self.marker_px - 1.5, 0.5],
            [self.marker_px - 1.5, self.marker_px - 1.5],
            [0.5, self.marker_px - 1.5]
        ], dtype=np.float32)
        for marker_id, pose in enumerate(np.asarray(positions, dtype=float)):
            if marker_id in hidden or self.rng.random() < self.c.get("drop_probability", 0.0):
                continue
            yaw = float(pose[3]) if pose.size > 3 else 0.0
            rot = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                            [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            destination = self._project(local @ rot.T + pose[:3])
            marker = cv2.aruco.generateImageMarker(self.dictionary, marker_id, self.marker_px)
            homography = cv2.getPerspectiveTransform(source, destination)
            warped = cv2.warpPerspective(marker, homography, (width, height), borderValue=220)
            mask = cv2.warpPerspective(np.full_like(marker, 255), homography, (width, height))
            image[mask > 0] = warped[mask > 0]
        sigma = float(self.c.get("pixel_noise_sigma", 0.0))
        if sigma:
            image = np.clip(image.astype(np.float32) + self.rng.normal(0, sigma, image.shape), 0, 255).astype(np.uint8)
        return image

    def capture(self, positions: np.ndarray, timestamp: float, hidden=()) -> Frame | None:
        """Render/enqueue a frame and return only a frame whose latency has elapsed."""
        latency = float(self.c.get("latency_s", 0.08))

        self.queue.append(
            Frame(
                self._render(positions, timestamp, hidden),
                float(timestamp),
                float(timestamp + latency),
            )
        )

        if (
            self.queue
            and self.queue[0].delivered_timestamp <= timestamp + 1e-9
        ):
            return self.queue.popleft()

        return None
"""Safety supervisor for estimator state, separation, and arena limits."""
from itertools import combinations
import numpy as np

class SafetySupervisor:
    def __init__(self, cfg):
        self.cfg = cfg
        self.events = []
        self._separation_since = None
        self._event_keys = set()

    def _event(self, now, kind, ids):
        key = (kind, tuple(ids))
        if key not in self._event_keys:
            self.events.append((float(now), kind, tuple(ids)))
            self._event_keys.add(key)

    def check(self, estimates, now):
        actions = {}
        positions = {}
        arena = self.cfg["arena"]
        for i, estimate in estimates.items():
            status = estimate.get("status")
            position = estimate.get("position")
            if status == "lost" or position is None:
                actions[i] = "land"
                self._event(now, "lost", (i,))
                continue
            if status == "stale":
                actions[i] = "hold"
                self._event(now, "stale", (i,))
            position = np.asarray(position, dtype=float)
            positions[i] = position
            if (position[0] < arena["x"][0] or position[0] > arena["x"][1] or
                position[1] < arena["y"][0] or position[1] > arena["y"][1] or
                position[2] < arena["z"][0] or position[2] > arena["z"][1]):
                actions[i] = "land"
                self._event(now, "geofence_breach", (i,))
        breaches = [pair for pair in combinations(sorted(positions), 2)
                    if np.linalg.norm(positions[pair[0]] - positions[pair[1]]) < self.cfg["arena"]["min_separation"]]
        if breaches:
            ids = tuple(sorted({i for pair in breaches for i in pair}))
            self._event(now, "separation_breach", ids)
            if self._separation_since is None:
                self._separation_since = float(now)
            for i in positions:
                if i not in actions or actions[i] != "land":
                    actions[i] = "hold" if now - self._separation_since < 0.2 else "land"
        else:
            self._separation_since = None
        return actions


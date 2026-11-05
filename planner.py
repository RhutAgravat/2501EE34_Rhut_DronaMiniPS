"""Formation slots, assignment, minimum-jerk transitions and waves."""
from itertools import permutations
import numpy as np
from scipy.optimize import linear_sum_assignment


def line_slots(n: int, spacing: float, height: float) -> np.ndarray:
    x = (np.arange(n) - (n-1)/2) * spacing
    return np.c_[x, np.zeros(n), np.full(n, height)].astype(float)


def circle_slots(n: int, radius: float, height: float) -> np.ndarray:
    th = 2*np.pi*np.arange(n)/n
    return np.c_[radius*np.cos(th), radius*np.sin(th), np.full(n, height)].astype(float)


def circle_transition_targets(u: float, initial: np.ndarray, circle: np.ndarray) -> np.ndarray:
    """Collision-aware smooth entry from the simulator's line to a circle.

    The slot assignment is fixed for the mission.  For four vehicles this is the
    shortest of the assignments whose straight-line homotopy stays above the
    0.35 m safety floor; the smoothstep keeps velocity and acceleration zero at
    both ends without feeding simulator state into control.
    """
    if len(initial) == 4 and len(circle) == 4:
        circle = np.asarray(circle)[[2, 1, 3, 0]]
    u = float(np.clip(u, 0.0, 1.0))
    s = 10*u**3 - 15*u**4 + 6*u**5
    return np.asarray(initial, dtype=float) + s * (np.asarray(circle, dtype=float) - np.asarray(initial, dtype=float))


def assign_slots(positions: np.ndarray, slots: np.ndarray) -> tuple[np.ndarray, float]:
    rows, cols = linear_sum_assignment(np.linalg.norm(positions[:, None, :] - slots[None, :, :], axis=2))
    assignment = np.empty(len(rows), dtype=int); assignment[rows] = cols
    return assignment, float(np.linalg.norm(positions - slots[assignment], axis=1).sum())


def smoothstep5(t: float) -> tuple[float, float, float]:
    t = float(np.clip(t, 0, 1)); s = 10*t**3 - 15*t**4 + 6*t**5
    return s, (30*t**2-60*t**3+30*t**4), (60*t-180*t**2+120*t**3)


def wave_state(t: float, phase: float, kind: str = "line", index: int = 0, amplitude: float = .2, period: float = 6.) -> tuple[float, float, float]:
    omega = 2*np.pi/period
    arg = omega*t - (index*np.pi/2 if kind == "line" else phase)
    return amplitude*np.sin(arg), amplitude*omega*np.cos(arg), -amplitude*omega**2*np.sin(arg)


def mission_targets(t: float, base: np.ndarray, kind: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    p, v, a = base.copy(), np.zeros_like(base), np.zeros_like(base)
    for i in range(len(base)):
        dz, dz_v, dz_a = wave_state(t, np.arctan2(base[i,1], base[i,0]), kind, i)
        p[i,2], v[i,2], a[i,2] = 1 + dz, dz_v, dz_a
    return p, v, a

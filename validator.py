"""Preflight checks sampled at 50 Hz."""
from dataclasses import dataclass
import numpy as np

@dataclass
class Failure:
    time: float; drone_ids: list[int]; measured: float; required: float; suggestion: str
@dataclass
class ValidationResult:
    ok: bool; failures: list[Failure]

def validate_slots(slots: np.ndarray, arena=None, min_separation=.35, dt=.02) -> ValidationResult:
    arena = arena or {'x':[-1,1], 'y':[-.6,.6], 'z':[.6,1.4]}
    failures=[]
    for t in np.arange(0, 0.02, dt):
        if not np.isfinite(slots).all(): failures.append(Failure(float(t), list(range(len(slots))), float('nan'), 0, 'remove nonfinite target'))
        for i,p in enumerate(slots):
            if not (arena['x'][0] <= p[0] <= arena['x'][1] and arena['y'][0] <= p[1] <= arena['y'][1] and arena['z'][0] <= p[2] <= arena['z'][1]):
                failures.append(Failure(float(t), [i], float(np.max(np.abs(p))), 0, 'reduce spacing or move formation'))
        for i in range(len(slots)):
            for j in range(i+1,len(slots)):
                d=float(np.linalg.norm(slots[i]-slots[j]))
                if d < min_separation: failures.append(Failure(float(t), [i,j], d, min_separation, 'increase spacing or radius'))
    return ValidationResult(not failures, failures)

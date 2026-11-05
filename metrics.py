"""Mission metrics calculated from logged estimates and evaluation truth."""
import numpy as np

def calculate(target, estimate, truth=None, events=(), statuses=None, rejected_outliers=0):
    target = np.asarray(target, dtype=float); estimate = np.asarray(estimate, dtype=float)
    valid = np.isfinite(estimate).all(axis=-1)
    err = np.linalg.norm(target - estimate, axis=-1)
    values = err[valid]
    out = {
        'rms_tracking_error_m': float(np.sqrt(np.mean(values ** 2))) if values.size else None,
        'formation_shape_error_m': None,
        'phase_synchronization_error_deg': None,
        'minimum_separation_m': None,
        'max_overshoot_m': float(np.max(values)) if values.size else None,
        'valid_estimate_samples': int(values.size),
        'safety_events': len(list(events)),
        'rejected_outlier_count': int(rejected_outliers),
    }
    if target.ndim == 3 and target.shape[1] > 1:
        target_d = np.linalg.norm(target[:, :, None, :] - target[:, None, :, :], axis=-1)
        est_d = np.linalg.norm(estimate[:, :, None, :] - estimate[:, None, :, :], axis=-1)
        pair = np.triu(np.ones(target_d.shape[1:]), 1).astype(bool)
        shape = (est_d-target_d)[:, pair]
        out['formation_shape_error_m'] = float(np.sqrt(np.nanmean(shape**2))) if shape.size else None
        sep = est_d[:, pair]
        out['minimum_separation_m'] = float(np.nanmin(sep)) if sep.size else None
        # Compare the dominant vertical-wave Fourier phase in estimate versus target.
        # Static formations have no meaningful phase metric.
        tz = target[:, :, 2] - np.nanmean(target[:, :, 2], axis=0, keepdims=True)
        ez = estimate[:, :, 2] - np.nanmean(estimate[:, :, 2], axis=0, keepdims=True)
        # Restrict phase analysis to rows where the planned formation is actually
        # waving.  Including the 5 s static formation transition corrupts the
        # Fourier bin for the subsequent exactly-three-cycle wave interval.
        planned_wave = (np.nanmax(target[:, :, 2], axis=1) - np.nanmin(target[:, :, 2], axis=1)) > 1e-3
        phase_rows = planned_wave & np.isfinite(tz).all(axis=1) & np.isfinite(ez).all(axis=1)
        if phase_rows.sum() >= 8:
            target_fft = np.fft.rfft(tz[phase_rows], axis=0)
            estimate_fft = np.fft.rfft(ez[phase_rows], axis=0)
            spectrum = np.nanmean(np.abs(target_fft), axis=1)
            if spectrum.size > 1 and np.nanmax(spectrum[1:]) > 1e-6:
                harmonic = int(np.nanargmax(spectrum[1:]) + 1)
                delta = np.angle(np.exp(1j * (np.angle(estimate_fft[harmonic]) - np.angle(target_fft[harmonic]))))
                out['phase_synchronization_error_deg'] = float(np.nanmean(np.abs(np.rad2deg(delta))))
    if truth is not None:
        truth = np.asarray(truth, dtype=float)
        loc_err = np.linalg.norm(truth - estimate, axis=-1)[valid]
        out['localization_rms_m'] = float(np.sqrt(np.mean(loc_err ** 2))) if loc_err.size else None
    if statuses:
        out['stale_count'] = int(statuses.get('stale', 0)); out['lost_count'] = int(statuses.get('lost', 0))
    return out


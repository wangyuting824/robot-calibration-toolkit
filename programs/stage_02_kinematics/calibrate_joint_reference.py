"""Fit q_reference = scale * raw + offset to independently measured SI positions."""
import numpy as np
from scipy.optimize import least_squares
from core.io import InputError, dataset, number, result
from core.metrics import summary, rank_diagnostics


def run(args):
    rows, meta = dataset(args.data, args.manifest, ("joint", "raw", "reference"), "calibration")
    if not meta.get("reference_instrument"):
        raise InputError("An independent reference instrument/source must be declared")
    if meta.get("reference_unit") not in ("rad", "m") or not meta.get("raw_unit") or number(args.f_scale) <= 0:
        raise InputError("Declare raw_unit, SI reference_unit (rad/m), and a positive loss scale")
    output = {}
    for name in sorted({row["joint"] for row in rows}):
        samples = [row for row in rows if row["joint"] == name]
        raw = np.array([number(r["raw"]) for r in samples])
        observed = np.array([number(r["reference"]) for r in samples])
        if len(raw) < 3 or np.ptp(raw) == 0 or np.ptp(observed) == 0:
            raise InputError(f"{name}: at least three distinct reference samples are required")
        center = np.mean(raw)
        design = np.c_[raw - center, np.ones(len(raw))]
        initial = np.linalg.lstsq(design, observed, rcond=None)[0]
        fitted = least_squares(lambda p: design @ p - observed, initial, loss="soft_l1", f_scale=number(args.f_scale))
        scale, offset = float(fitted.x[0]), float(fitted.x[1] - fitted.x[0] * center)
        if abs(scale) < np.finfo(float).eps:
            raise InputError("Zero feedback conversion scale")
        output[name] = {"scale": scale, "offset": offset, "raw_zero": -offset / scale,
                        "convention": "q_model = scale * raw + offset",
                        "raw_unit": meta.get("raw_unit"), "reference_unit": meta.get("reference_unit"),
                        "residual": summary(scale * raw + offset - observed),
                        "diagnostics": rank_diagnostics(fitted.jac)}
    return result("calibrate-joint-reference", {"joints": output, "dataset": meta,
                  "qualification": "Numerical fit; validate independently before changing hardware mapping"},
                  [args.data, args.manifest])

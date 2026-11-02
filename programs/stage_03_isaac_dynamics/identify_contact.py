"""Explicit contact-model regression, not an automatic vendor measurement procedure."""
import numpy as np
from scipy.optimize import lsq_linear
from core.io import InputError, dataset, load, number, result
from core.metrics import summary, rank_diagnostics


def run(args):
    config = load(args.config)
    mode = config.get("model")
    columns = {"sliding_coulomb_viscous": ("normal_force", "slip_velocity", "tangential_effort"),
               "normal_linear_compliance": ("penetration", "closing_velocity", "normal_force"),
               "restitution": ("approach_speed", "separation_speed")}
    if mode not in columns:
        raise InputError("Select an explicit supported contact model")
    rows, meta = dataset(args.data, args.manifest, columns[mode], "calibration")
    if not meta.get("procedure_reference") or not meta.get("force_reference"):
        raise InputError("Contact regression requires a documented experiment and calibrated measurement source")
    values = np.array([[number(r[k]) for k in columns[mode]] for r in rows])
    if mode == "sliding_coulomb_viscous":
        scale = number(config.get("velocity_scale"))
        if scale <= 0 or np.any(values[:, 0] <= 0) or np.any(values[:, 1] == 0):
            raise InputError("Sliding data require positive normal force, nonzero slip, positive velocity_scale")
        design = np.c_[values[:, 0] * np.tanh(values[:, 1] / scale), values[:, 1]]
        observed, names, bounds = values[:, 2], ("dynamic_friction", "viscous_contact"), (0, np.inf)
    elif mode == "normal_linear_compliance":
        if np.any(values[:, 0] < 0) or np.any(values[:, 2] < 0):
            raise InputError("Penetration and normal force must be nonnegative")
        design, observed = values[:, :2], values[:, 2]
        names, bounds = ("normal_stiffness", "normal_damping"), (0, np.inf)
    else:
        if np.any(values[:, 0] <= 0) or np.any(values[:, 1] < 0):
            raise InputError("Impact speed magnitudes must be positive/nonnegative")
        design, observed = values[:, 0, None], values[:, 1]
        names, bounds = ("restitution",), (0, 1)
    diagnostics = rank_diagnostics(design)
    if len(rows) <= len(names) or not diagnostics["identifiable"]:
        raise InputError("Contact parameters are not identifiable from these samples")
    fitted = lsq_linear(design, observed, bounds=bounds)
    return result("identify-contact", {"model": mode, "parameters": dict(zip(names, fitted.x.tolist())),
                  "diagnostics": diagnostics, "residual": summary(design @ fitted.x - observed), "dataset": meta,
                  "qualification": "Project model fit; map only to the matching simulator contact model and validate on holdout trials",
                  "static_friction_identified": False, "automatically_applied": False},
                  [args.data, args.manifest, args.config])

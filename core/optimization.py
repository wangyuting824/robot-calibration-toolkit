"""Project pose-residual calibration, using SciPy bounded least squares.

Basis: ROS robot_calibration free joint/frame parameters and independent
observations. This implementation is not the ROS package or a vendor procedure.
Gauge freedoms are detected numerically; rank-deficient results are rejected.
"""
import numpy as np
from scipy.optimize import least_squares

from core.io import InputError, number, vector
from core.metrics import rank_diagnostics
from core.units_and_frames import transform, pose_error, pose_from_row

POSE_COLUMNS = ("x", "y", "z", "qx", "qy", "qz", "qw")
COMPONENTS = ("x", "y", "z", "roll", "pitch", "yaw")


def decode(config, specs, values):
    base = vector(config.get("base_xyz_rpy", [0] * 6), 6, "base_xyz_rpy").copy()
    tool = vector(config.get("tool_xyz_rpy", [0] * 6), 6, "tool_xyz_rpy").copy()
    offsets, deltas = {}, {}
    for spec, value in zip(specs, values):
        kind = spec["kind"]
        if kind == "joint_offset":
            offsets[spec["joint"]] = float(value)
        elif kind in ("base", "tool"):
            (base if kind == "base" else tool)[COMPONENTS.index(spec["component"])] += value
        elif kind == "joint_origin":
            delta = deltas.setdefault(spec["joint"], np.zeros(6))
            delta[COMPONENTS.index(spec["component"])] = value
        else:
            raise InputError(f"Unsupported parameter kind: {kind}")
    return base, tool, offsets, deltas


def predict(model, row, config, specs=(), values=()):
    base, tool, offsets, deltas = decode(config, specs, values)
    tip = config.get("tip_link")
    if tip not in model.links:
        raise InputError(f"Unknown tip_link: {tip}")
    q = {name: number(row[f"q:{name}"]) for name in model.independent}
    return transform(base[:3], base[3:]) @ model.transforms(q, offsets, deltas)[tip] @ transform(tool[:3], tool[3:])


def errors(model, rows, config, specs=(), values=()):
    return np.array([pose_error(predict(model, row, config, specs, values), pose_from_row(row)) for row in rows])


def fit_kinematics(model, rows, config):
    specs = config.get("free_parameters", [])
    if not specs:
        raise InputError("Select explicit free_parameters; no automatic release of all parameters")
    keys = []
    for spec in specs:
        key = (spec.get("kind"), spec.get("joint"), spec.get("component"))
        if key in keys:
            raise InputError("Duplicate free parameter")
        keys.append(key)
        if spec["kind"] == "joint_offset" and spec.get("joint") not in model.independent:
            raise InputError("Only independent movable joint offsets are supported")
        if spec["kind"] == "joint_origin" and spec.get("joint") not in model.joints:
            raise InputError("Unknown joint for origin calibration")
        if spec["kind"] in ("base", "tool", "joint_origin") and spec.get("component") not in COMPONENTS:
            raise InputError("Invalid free-frame component")
    lower = [number(s.get("lower")) for s in specs]
    upper = [number(s.get("upper")) for s in specs]
    if any(a >= b or not a <= 0 <= b for a, b in zip(lower, upper)):
        raise InputError("Bounds must be ordered and contain initial correction zero")
    position_sigma = number(config.get("position_sigma_m"))
    angle_sigma = number(config.get("angle_sigma_rad"))
    if min(position_sigma, angle_sigma) <= 0:
        raise InputError("Positive measurement standard deviations are required")
    scale = np.array([position_sigma] * 3 + [angle_sigma] * 3)
    if len(rows) * 6 <= len(specs):
        raise InputError("Insufficient independent observations")
    fn = lambda x: (errors(model, rows, config, specs, x) / scale).ravel()
    fitted = least_squares(fn, np.zeros(len(specs)), bounds=(lower, upper),
                           loss=config.get("loss", "soft_l1"), max_nfev=1000, x_scale="jac")
    diagnostics = rank_diagnostics(fitted.jac)
    if not fitted.success or not diagnostics["identifiable"]:
        raise InputError(f"Calibration did not converge or is rank-deficient: {diagnostics}")
    at_bound = [i for i, v in enumerate(fitted.x) if min(v - lower[i], upper[i] - v) < 1e-6 * (upper[i] - lower[i])]
    return {"config": config, "specifications": specs, "values": fitted.x.tolist(),
            "diagnostics": diagnostics, "parameters_at_bound": at_bound,
            "fit_is_acceptance": False}

"""Fit joint armature, viscous/Coulomb friction and bias after fixed-base RNEA.

Rigid link masses/COM/inertias must already have traceable sources. This is a
project identification model, not an NVIDIA or robot-vendor identification tool.
"""
import numpy as np
from scipy.optimize import lsq_linear
from core.dynamics import inverse_dynamics
from core.io import InputError, dataset, load, number, result, vector
from core.metrics import summary, rank_diagnostics
from core.model import Model


def run(args):
    model = Model(args.urdf)
    required = [f"{field}:{name}" for name in model.independent for field in ("q", "dq", "ddq", "tau")]
    rows, meta = dataset(args.data, args.manifest, required, "calibration")
    config = load(args.config)
    if config.get("fixed_base") is not True or not config.get("inertia_source"):
        raise InputError("Identify only fixed-base trees with traceable rigid-body inertia parameters")
    if meta.get("torque_semantics") != "joint_actuator_effort" or not meta.get("effort_reference"):
        raise InputError("Declare a qualified joint-actuator-effort measurement; motor current alone is not torque")
    speed_scale = number(config.get("friction_velocity_scale"))
    if speed_scale <= 0:
        raise InputError("Positive friction_velocity_scale required")
    gravity = vector(config.get("gravity"), 3, "gravity")
    rigid = []
    for row in rows:
        state = [{n: number(row[f"{key}:{n}"]) for n in model.independent} for key in ("q", "dq", "ddq")]
        rigid.append(inverse_dynamics(model, *state, gravity=gravity))
    output = {}
    for name in config.get("joints", model.independent):
        if name not in model.independent:
            raise InputError(f"Unknown independent joint {name}")
        dq = np.array([number(r["dq:" + name]) for r in rows])
        ddq = np.array([number(r["ddq:" + name]) for r in rows])
        torque = np.array([number(r["tau:" + name]) for r in rows])
        baseline = np.array([r[name] for r in rigid])
        design = np.c_[ddq, dq, np.tanh(dq / speed_scale), np.ones(len(dq))]
        diagnostics = rank_diagnostics(design)
        if len(rows) <= 4 or not diagnostics["identifiable"]:
            raise InputError(f"{name}: excitation does not identify armature/friction/bias: {diagnostics}")
        fit = lsq_linear(design, torque - baseline, bounds=([0, 0, 0, -np.inf], [np.inf] * 4))
        if not fit.success:
            raise InputError(f"{name}: dynamics fit did not converge")
        output[name] = {"armature": float(fit.x[0]), "viscous_friction": float(fit.x[1]),
                        "coulomb_friction": float(fit.x[2]), "effort_bias": float(fit.x[3]),
                        "residual_effort": summary(baseline + design @ fit.x - torque), "diagnostics": diagnostics}
    return result("identify-dynamics", {"joints": output, "dataset": meta, "config": config,
                  "model": "tau = RNEA(q,dq,ddq) + armature*ddq + b*dq + fc*tanh(dq/v_scale) + bias",
                  "link_inertias_estimated": False, "fit_is_acceptance": False},
                  [args.urdf, args.data, args.manifest, args.config])

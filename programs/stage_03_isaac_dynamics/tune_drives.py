"""Identify force-PD gains only from the isolated PD effort component.

The PD form is documented by NVIDIA; fitting and acceptance are project code.
Total actuator effort with unknown gravity/feedforward is explicitly rejected.
"""
import numpy as np
from scipy.optimize import lsq_linear
from core.io import InputError, dataset, number, result
from core.metrics import summary, rank_diagnostics


def run(args):
    rows, meta = dataset(args.data, args.manifest,
                         ("joint", "q", "dq", "q_command", "dq_command", "pd_effort", "saturated"), "calibration")
    if meta.get("torque_semantics") != "isolated_pd_effort" or meta.get("drive_type") != "force":
        raise InputError("Gain fitting requires isolated PD effort and force drive; arbitrary vendor PID gains cannot be transferred")
    output = {}
    for name in sorted({r["joint"] for r in rows}):
        samples = [r for r in rows if r["joint"] == name and r["saturated"].lower() in ("false", "0")]
        design = np.array([[number(r["q_command"]) - number(r["q"]), number(r["dq_command"]) - number(r["dq"])] for r in samples])
        if len(samples) <= 2:
            raise InputError(f"{name}: insufficient unsaturated samples")
        diagnostics = rank_diagnostics(design)
        if not diagnostics["identifiable"]:
            raise InputError(f"{name}: position and velocity excitation are not independent")
        observed = np.array([number(r["pd_effort"]) for r in samples])
        fitted = lsq_linear(design, observed, bounds=(0, np.inf))
        output[name] = {"stiffness": float(fitted.x[0]), "damping": float(fitted.x[1]),
                        "drive_type": "force", "diagnostics": diagnostics,
                        "residual_effort": summary(design @ fitted.x - observed)}
    return result("tune-drives", {"joints": output, "dataset": meta,
                  "qualification": "Candidate force-PD gains; run Isaac tests and compare against real response before acceptance"},
                  [args.data, args.manifest])

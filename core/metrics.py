import numpy as np
from core.io import InputError


def summary(values):
    values = np.asarray(values, dtype=float)
    if not values.size or not np.isfinite(values).all():
        raise InputError("Metrics require nonempty finite values")
    return {"n": int(values.size), "rmse": float(np.sqrt(np.mean(values ** 2))),
            "mae": float(np.mean(np.abs(values))),
            "mean": float(np.mean(values)), "max_abs": float(np.max(np.abs(values))),
            "p95_abs": float(np.percentile(np.abs(values), 95))}


def rank_diagnostics(jacobian):
    singular = np.linalg.svd(jacobian, compute_uv=False)
    threshold = max(jacobian.shape) * np.finfo(float).eps * singular[0] if singular.size else 0
    rank = int(np.sum(singular > max(threshold, singular[0] * 1e-7))) if singular.size else 0
    return {"rank": rank, "parameters": int(jacobian.shape[1]),
            "singular_values": singular.tolist(),
            "identifiable": rank == jacobian.shape[1],
            "condition_number": float(singular[0] / singular[-1]) if singular.size and singular[-1] > threshold else None}

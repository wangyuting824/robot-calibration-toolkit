"""Compare matched experiments on a declared test-relative clock, no time warping."""
import numpy as np
from core.io import InputError, dataset, load, number, result, write_csv
from core.metrics import summary


def run(args):
    cfg = load(args.acceptance).get("dynamics", {})
    metrics_required = cfg.get("required_metrics", ["q", "dq"])
    allowed = {"q", "dq", "tau", "normal_force", "slip_velocity"}
    if not isinstance(metrics_required, list) or not metrics_required or len(set(metrics_required)) != len(metrics_required) or set(metrics_required) - allowed:
        raise InputError("Declare a nonempty list of unique supported metrics")
    columns = ("joint", "t", *metrics_required)
    real, real_meta = dataset(args.real_data, args.real_manifest, columns, "validation")
    sim, sim_meta = dataset(args.sim_data, args.sim_manifest, columns, "validation")
    for field in ("test_case_id", "input_sha256", "initial_state", "payload_id", "time_basis"):
        if real_meta.get(field) is None or real_meta[field] != sim_meta.get(field):
            raise InputError(f"Real/sim experiment metadata do not match: {field}")
    if real_meta["time_basis"] != "test_relative":
        raise InputError("Normalize both experiments to their documented test-relative time origin")
    if real_meta["data_kind"] not in ("real", "synthetic") or sim_meta["data_kind"] not in ("simulation", "synthetic"):
        raise InputError("Declare actual real/simulation sources, or mark both synthetic for algorithm tests")
    real_names, sim_names = {r["joint"] for r in real}, {r["joint"] for r in sim}
    if real_names != sim_names:
        raise InputError("Real/simulation joint or contact entity names differ")
    stats, curves, passed, configured = {}, [], True, True
    for name in sorted(real_names):
        reference = sorted((r for r in real if r["joint"] == name), key=lambda r: number(r["t"]))
        predicted = sorted((r for r in sim if r["joint"] == name), key=lambda r: number(r["t"]))
        tr, ts = np.array([number(r["t"]) for r in reference]), np.array([number(r["t"]) for r in predicted])
        if len(ts) < 2 or np.any(np.diff(ts) <= 0) or np.any(np.diff(tr) <= 0) or tr[0] < ts[0] or tr[-1] > ts[-1]:
            raise InputError("Timestamps must be unique/increasing with complete simulation overlap; no extrapolation")
        stats[name] = {}
        for field in metrics_required:
            if any(r.get(field) in (None, "") for r in reference + predicted):
                raise InputError(f"Missing required measured metric {name}/{field}")
            if field == "tau" and (not real_meta.get("effort_reference") or not sim_meta.get("effort_reference")):
                raise InputError("Torque comparison requires declared measurement semantics/reference on both datasets")
            observed = np.array([number(r[field]) for r in reference])
            interpolated = np.interp(tr, ts, [number(r[field]) for r in predicted])
            difference = interpolated - observed
            stats[name][field] = summary(difference)
            limit = cfg.get("max_abs", {}).get(field)
            configured &= limit is not None
            passed &= limit is not None and max(abs(difference)) <= number(limit)
            curves.extend({"joint": name, "t": float(t), "metric": field, "real": float(r), "simulation": float(s), "error": float(e)}
                          for t, r, s, e in zip(tr, observed, interpolated, difference))
    if args.errors_csv:
        write_csv(args.errors_csv, curves)
    status = "unverified" if not configured else "passed" if passed else "failed"
    if status == "passed" and (real_meta["data_kind"] != "real" or sim_meta["data_kind"] != "simulation"):
        status = "synthetic_pass"
    return result("validate-dynamics", {"metrics": stats, "required_metrics": metrics_required,
                  "errors_csv": str(args.errors_csv) if args.errors_csv else None,
                  "real_dataset": real_meta, "simulation_dataset": sim_meta, "acceptance": cfg,
                  "alignment": "Declared common test-relative clock; piecewise-linear interpolation; no offset fitting"},
                  [args.real_data, args.real_manifest, args.sim_data, args.sim_manifest, args.acceptance], status)

"""Verify recorded limits against an explicit specification. Never searches for stops."""
import numpy as np
from core.io import InputError, dataset, load, number, result


def run(args):
    rows, meta = dataset(args.data, args.manifest, ("joint", "boundary", "angle"), "validation")
    specification = load(args.specification)
    if meta.get("event_kind") not in ("software_boundary", "switch_latch", "manufacturer_stop_record"):
        raise InputError("Declare event_kind; final stopped positions are not switch-latch measurements")
    if not meta.get("procedure_reference"):
        raise InputError("Supply the actual manufacturer's procedure reference; no generic probing procedure is assumed")
    known = specification.get("joints", {})
    if set(r["joint"] for r in rows) - set(known):
        raise InputError("Measurements contain a joint absent from the limit specification")
    output, all_pass, all_configured = {}, True, True
    for name, spec in known.items():
        lower, upper = number(spec.get("lower")), number(spec.get("upper"))
        if lower >= upper or not spec.get("source"):
            raise InputError(f"{name}: ordered limits and source required")
        kind = spec.get("kind")
        if kind != meta["event_kind"]:
            raise InputError(f"{name}: event/specification kind mismatch")
        output[name] = {}
        for boundary, expected in (("lower", lower), ("upper", upper)):
            samples = [number(r["angle"]) for r in rows if r["joint"] == name and r["boundary"] == boundary]
            if not samples:
                raise InputError(f"Missing {name}/{boundary} samples")
            tolerance = spec.get("tolerance")
            repeat_limit = spec.get("repeatability_peak_to_peak")
            count = spec.get("minimum_repeats")
            configured = all(v is not None for v in (tolerance, repeat_limit, count))
            all_configured &= configured
            spread = float(np.ptp(samples))
            max_error = max(abs(v - expected) for v in samples)
            passed = configured and max_error <= number(tolerance) and spread <= number(repeat_limit) and len(samples) >= int(count)
            all_pass &= bool(passed)
            output[name][boundary] = {"expected": expected, "n": len(samples), "min": min(samples),
                "max": max(samples), "mean": float(np.mean(samples)), "peak_to_peak": spread,
                "max_abs_error": max_error, "passed": bool(passed) if configured else None}
    if any(r["boundary"] not in ("lower", "upper") for r in rows):
        raise InputError("boundary must be lower or upper")
    status = "unverified" if not all_configured else "passed" if all_pass else "failed"
    if meta["data_kind"] != "real" and status == "passed":
        status = "synthetic_pass" if meta["data_kind"] == "synthetic" else "simulation_pass"
    return result("verify-joint-limits", {"joints": output, "dataset": meta,
                  "writes_hardware_limits": False, "mechanical_stops_are_not_operating_limits": True},
                  [args.data, args.manifest, args.specification], status)

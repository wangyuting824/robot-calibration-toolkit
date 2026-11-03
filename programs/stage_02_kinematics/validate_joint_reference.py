"""Validate fitted feedback coordinates on separate reference observations.

Only a passing, explicitly configured holdout check can export a validated
profile. Synthetic observations remain restricted to simulation use.
"""
from pathlib import Path
import os

import numpy as np

from core.io import InputError, csv_rows, dataset, load, number, result, save, sha256
from core.joint_mapping import checked_evidence
from core.metrics import summary
from core.model import Model


def run(args):
    if Path(args.mapping_output).exists():
        raise InputError("Mapping output already exists; choose a new version")
    if Path(args.mapping_output).resolve() == Path(args.output).resolve():
        raise InputError("Mapping and validation result must have different paths")
    fit = load(args.calibration)
    if fit.get("command") != "calibrate-joint-reference" or fit.get("status") != "computed":
        raise InputError("Supply a successful calibrate-joint-reference result")
    original = [checked_evidence(e, Path.cwd()) for e in fit.get("inputs", [])]
    train_csv = [p for p in original if p.suffix.lower() == ".csv"]
    if len(train_csv) != 1:
        raise InputError("Cannot identify the original training data")
    train = csv_rows(train_csv[0], ("sample_id", "joint", "raw", "reference"))
    rows, meta = dataset(args.data, args.manifest, ("sample_id", "joint", "raw", "reference"), "validation")
    training = fit["dataset"]
    for key in ("reference_kind", "reference_definition", "reference_instrument", "raw_unit", "reference_unit", "robot_id", "feedback_coordinates_id", "urdf_sha256"):
        if not meta.get(key) or meta[key] != training.get(key):
            raise InputError(f"Training/validation must declare consistent {key}")
    if meta["urdf_sha256"] != sha256(args.urdf):
        raise InputError("Reference records belong to a different URDF")
    if meta["reference_unit"] not in ("rad", "m") or meta["raw_unit"] not in ("rad", "m"):
        raise InputError("Normalize reference and source joint coordinates to SI before validation")
    real = training.get("data_kind") == "real" and meta["data_kind"] == "real"
    if real and meta["reference_kind"] not in ("independent_measurement", "manufacturer_calibration"):
        raise InputError("Real coordinate validation needs an independent reference or manufacturer calibration")
    if not real and training.get("data_kind") != meta["data_kind"]:
        raise InputError("Do not mix real, synthetic and simulation reference datasets")
    if training["dataset_id"] == meta["dataset_id"] or sha256(args.data) == sha256(train_csv[0]):
        raise InputError("Holdout data must differ from training")
    if not meta.get("sha256") or not training.get("sha256"):
        raise InputError("Training and holdout data must have SHA-256 manifests")
    if {r["sample_id"] for r in rows} & {r["sample_id"] for r in train}:
        raise InputError("Holdout sample IDs overlap with training")
    for records in (rows, train):
        keys = [(r["joint"], r["sample_id"]) for r in records]
        if len(set(keys)) != len(keys) or any(not r["sample_id"].strip() for r in records):
            raise InputError("Each joint needs unique, nonempty sample IDs")
    configured = load(args.acceptance).get("joint_reference", {}).get("joints", {})
    if set(fit["joints"]) != set(configured) or {r["joint"] for r in rows} != set(configured):
        raise InputError("Fit, acceptance and holdout joint coverage must match")
    model = Model(args.urdf)
    outputs, all_passed = {}, True
    for name, fitted in fit["joints"].items():
        spec = configured[name]
        target = spec.get("target_joint", name)
        if target not in model.independent:
            raise InputError("Reference mapping targets must be independent URDF joints")
        unit = "m" if model.joints[target].get("type") == "prismatic" else "rad"
        if meta["reference_unit"] != unit:
            raise InputError("Reference unit differs from URDF joint type; opening width is not finger angle")
        tolerance = number(spec.get("max_abs_error"))
        count = spec.get("minimum_validation_samples")
        if tolerance <= 0 or isinstance(count, bool) or not isinstance(count, int) or count < 3:
            raise InputError("Declare positive max_abs_error and minimum_validation_samples >= 3")
        observed = [r for r in rows if r["joint"] == name]
        training_rows = [r for r in train if r["joint"] == name]
        domain = [min(number(r["raw"]) for r in training_rows), max(number(r["raw"]) for r in training_rows)]
        if len(observed) < count or len({number(r["raw"]) for r in observed}) < 3:
            raise InputError("Holdout requires enough distinct reference positions")
        if any(not domain[0] <= number(r["raw"]) <= domain[1] for r in observed):
            raise InputError("Holdout exceeds training domain; no extrapolation")
        for row in training_rows + observed:
            reference = number(row["reference"])
            if model.joints[target].get("type") != "continuous":
                limit = model.joints[target].find("limit")
                if not number(limit.get("lower")) <= reference <= number(limit.get("upper")):
                    raise InputError("Reference positions disagree with URDF operating range")
        scale, offset = number(fitted["scale"]), number(fitted["offset"])
        if fitted.get("diagnostics", {}).get("identifiable") is not True:
            raise InputError("Unidentifiable reference fit cannot be exported")
        error = [scale * number(r["raw"]) + offset - number(r["reference"]) for r in observed]
        passed = max(abs(v) for v in error) <= tolerance
        all_passed &= passed
        outputs[name] = {"source_joint": name, "target_joint": target, "scale": scale, "offset": offset,
            "raw_unit": meta["raw_unit"], "reference_unit": unit, "valid_raw_range": domain,
            "residual": summary(np.array(error)), "max_abs_error_allowed": tolerance, "passed": passed}
    status = "failed" if not all_passed else "passed" if real else "synthetic_pass" if meta["data_kind"] == "synthetic" else "simulation_pass"
    # Absolute evidence paths make the mapping independent of the caller's cwd.
    paths = [Path(p).resolve() for p in (args.data, args.manifest, args.calibration, args.acceptance, args.urdf)] + original
    report = result("validate-joint-reference", {"joints": outputs, "dataset": meta,
        "urdf_sha256": sha256(args.urdf),
        "training_data_kind": training["data_kind"], "robot_id": meta["robot_id"],
        "input_coordinates_id": meta["feedback_coordinates_id"], "is_real_robot_acceptance": False,
        "hardware_control_commands_sent": False}, paths, status)
    if all_passed:
        # The CLI finalizes source/run metadata, writes the report, then hashes
        # that exact file into the profile. There is no cyclic report hash.
        from core.provenance import basis
        report["methodology"] = basis("validate-joint-reference")
        profile_path = Path(args.mapping_output).resolve()
        report_path = Path(args.output).resolve()
        profile = {"schema_version": 1, "kind": "joint_feedback_mapping", "status": "validated_reference",
            "urdf_sha256": sha256(args.urdf), "robot_id": meta["robot_id"],
            "input_coordinates_id": meta["feedback_coordinates_id"], "unmapped_policy": "preserve_unverified",
            "source": meta["reference_instrument"] + "; " + meta["reference_definition"],
            "validation": {"path": os.path.relpath(report_path, profile_path.parent)},
            "joints": [{k: value[k] for k in ("source_joint", "target_joint", "scale", "offset", "raw_unit", "reference_unit", "valid_raw_range")} for value in outputs.values()]}
        # CLI finalization supplies run_id before computing this report hash.
        report["mapping_output"] = str(profile_path)
        report["_mapping_export"] = profile
    return report

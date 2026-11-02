import numpy as np
from core.io import InputError, dataset, load, number, result, sha256, write_csv
from core.metrics import summary
from core.model import Model
from core.optimization import POSE_COLUMNS, errors


def run(args):
    model = Model(args.urdf)
    rows, meta = dataset(args.data, args.manifest, ["sample_id", *POSE_COLUMNS, *["q:" + n for n in model.independent]], "validation")
    calibration = load(args.calibration)
    if any(i["sha256"] == sha256(args.data) for i in calibration.get("inputs", [])):
        raise InputError("Validation data must not reuse the calibration dataset")
    if meta["dataset_id"] == calibration["training_dataset"]["dataset_id"]:
        raise InputError("Validation dataset_id must differ from training")
    if set(r["sample_id"] for r in rows) & set(calibration["training_sample_ids"]):
        raise InputError("Validation sample IDs overlap with training")
    if meta.get("observed_frame") != calibration["config"].get("observed_frame") or not meta.get("pose_source"):
        raise InputError("Measurement frame/source mismatch")
    urdf_inputs = [i for i in calibration.get("inputs", []) if i["path"].endswith(".urdf")]
    if not urdf_inputs or sha256(args.urdf) != urdf_inputs[0]["sha256"]:
        raise InputError("Use the nominal URDF used in fitting; corrections must not be applied twice")
    residual = errors(model, rows, calibration["config"], calibration["specifications"], calibration["values"])
    position, angle = np.linalg.norm(residual[:, :3], axis=1), np.linalg.norm(residual[:, 3:], axis=1)
    stats = {"position_m": summary(position), "orientation_rad": summary(angle)}
    acceptance = load(args.acceptance).get("kinematics", {})
    thresholds = (acceptance.get("max_position_m"), acceptance.get("max_orientation_rad"))
    configured = all(v is not None for v in thresholds)
    passed = configured and max(position) <= number(thresholds[0]) and max(angle) <= number(thresholds[1])
    status = "unverified" if not configured else "passed" if passed else "failed"
    if status == "passed" and meta["data_kind"] != "real":
        status = "synthetic_pass" if meta["data_kind"] == "synthetic" else "simulation_pass"
    if args.errors_csv:
        write_csv(args.errors_csv, [{"sample_id": r["sample_id"], "position_error_m": float(p), "orientation_error_rad": float(a)}
                                  for r, p, a in zip(rows, position, angle)])
    return result("validate-kinematics", {"metrics": stats, "dataset": meta, "acceptance": acceptance,
                                         "errors_csv": str(args.errors_csv) if args.errors_csv else None,
                                         "training_data_kind": calibration["training_dataset"]["data_kind"],
                                         "calibration_sha256": sha256(args.calibration)},
                  [args.urdf, args.data, args.manifest, args.calibration, args.acceptance], status)

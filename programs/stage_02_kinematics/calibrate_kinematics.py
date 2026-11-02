from core.io import InputError, dataset, load, result
from core.model import Model
from core.optimization import POSE_COLUMNS, fit_kinematics


def run(args):
    model = Model(args.urdf)
    rows, meta = dataset(args.data, args.manifest, ["sample_id", *POSE_COLUMNS, *["q:" + n for n in model.independent]], "calibration")
    config = load(args.config)
    if not config.get("observed_frame") or meta.get("observed_frame") != config.get("observed_frame") or not meta.get("pose_source"):
        raise InputError("Declare consistent observed_frame and independent pose_source")
    fitted = fit_kinematics(model, rows, config)
    fitted["training_dataset"] = meta
    fitted["training_sample_ids"] = [r["sample_id"] for r in rows]
    return result("calibrate-kinematics", fitted, [args.urdf, args.data, args.manifest, args.config])

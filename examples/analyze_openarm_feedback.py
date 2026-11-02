"""Describe motor observations against a nominal profile; never change mapping."""
from pathlib import Path
import argparse
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.io import InputError, csv_rows, number, result, save
from core.model import Model


def analyze(data, urdf):
    rows = csv_rows(data, ("interface", "motor_id", "joint_label_nominal", "motor_position_rad", "raw_payload_hex"))
    model = Model(urdf)
    measurements, flagged = {}, []
    for name in sorted({r["joint_label_nominal"] for r in rows}):
        samples = [r for r in rows if r["joint_label_nominal"] == name]
        values = [number(r["motor_position_rad"]) for r in samples]
        motor_ids = {(r["interface"], r["motor_id"]) for r in samples}
        if len(motor_ids) != 1 or name not in model.joints:
            raise InputError("Ambiguous or unknown nominal joint label")
        item = {"interface": samples[0]["interface"], "motor_id": samples[0]["motor_id"], "samples": len(samples),
            "motor_min_rad": min(values), "motor_max_rad": max(values), "motor_mean_rad": float(np.mean(values)),
            "observed_spread_rad": max(values) - min(values), "physical_mapping_verified": False}
        if int(samples[0]["motor_id"]) == 8:
            item["range_comparison"] = "not_checked; gripper transmission/sign mapping requires separate confirmation"
        else:
            limit = model.joints[name].find("limit")
            lower, upper = number(limit.get("lower")), number(limit.get("upper"))
            inside = min(values) >= lower and max(values) <= upper
            item.update(nominal_lower_rad=lower, nominal_upper_rad=upper,
                assumed_identity_mapping_inside_nominal_range=inside,
                comparison_assumption="Configured arm driver publishes motor position unchanged; physical side/direction/zero unverified")
            if not inside:
                flagged.append(name)
        measurements[name] = item
    return result("analyze-openarm-feedback", {"measurements": measurements, "mapping_review_required": flagged,
        "identity_mapping_verified": False, "automatic_sign_offset_or_limit_changes": False,
        "scope": "Sequential raw motor telemetry and nominal range consistency under a stated mapping assumption",
        "is_real_robot_acceptance": False}, [data, urdf], "needs_mapping_review" if flagged else "observed_only")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", required=True)
    p.add_argument("--urdf", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    report = analyze(args.data, args.urdf)
    save(args.output, report)
    print(report["status"], report["mapping_review_required"])

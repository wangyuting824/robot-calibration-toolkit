"""Synthetic known-answer fit -> holdout -> profile -> read-only replay."""
import argparse
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.cli import main
from core.io import load, save, sha256, write_csv
from core.model import Model


def run(urdf, joint, destination):
    model = Model(urdf)
    element = model.joints[joint]
    lower, upper = [float(element.find("limit").get(k)) for k in ("lower", "upper")]
    unit = "m" if element.get("type") == "prismatic" else "rad"
    out = Path(destination).resolve()
    out.mkdir(parents=True, exist_ok=False)
    # Known synthetic relationship: q_model=-q_source. It is not a robot recommendation.
    scale, offset = -1., 0.
    base = {"data_kind": "synthetic", "units": {"length": "m", "angle": "rad", "time": "s", "effort": "N_or_Nm"},
        "clock": "synthetic sample index", "source": "known-answer synthetic observations; NOT physical measurements",
        "reference_kind": "synthetic_ground_truth", "reference_definition": "Known URDF coordinate from generated mathematics",
        "reference_instrument": "synthetic oracle", "raw_unit": unit, "reference_unit": unit,
        "robot_id": "synthetic-example", "feedback_coordinates_id": "synthetic-publisher", "urdf_sha256": sha256(urdf)}
    for role, fractions in (("calibration", np.linspace(.1, .9, 7)), ("validation", [.2, .4, .6, .8])):
        rows = []
        for i, fraction in enumerate(fractions):
            reference = lower + float(fraction) * (upper - lower)
            rows.append({"sample_id": f"synthetic-{role}-{i}", "joint": joint, "raw": (reference - offset) / scale, "reference": reference})
        path = out / (role + ".csv")
        write_csv(path, rows)
        save(path.with_suffix(".yaml"), {**base, "dataset_id": "synthetic-" + role, "role": role, "sha256": sha256(path)})
    save(out / "acceptance.yaml", {"joint_reference": {"joints": {joint: {
        "max_abs_error": 1e-8, "minimum_validation_samples": 3}}}, "scope": "Synthetic software tolerance only"})

    def execute(command, *options, expected=0):
        code = main([command, *map(str, options), "--output", str(out / (command + ".json"))])
        if code != expected:
            raise RuntimeError(f"{command}: expected {expected}, got {code}")
    execute("calibrate-joint-reference", "--data", out / "calibration.csv", "--manifest", out / "calibration.yaml", "--f-scale", .001)
    execute("validate-joint-reference", "--data", out / "validation.csv", "--manifest", out / "validation.yaml",
        "--calibration", out / "calibrate-joint-reference.json", "--acceptance", out / "acceptance.yaml", "--urdf", urdf,
        "--mapping-output", out / "mapping.yaml")
    raw = -(lower + upper) / 2
    write_csv(out / "feedback.csv", [{"sequence": 0, "joint": joint, "q": raw, "dq": "", "effort": ""}])
    meta = {**base, "role": "validation", "dataset_id": "synthetic-feedback", "coordinates_id": "synthetic-publisher",
        "joint_units": {joint: unit}, "sha256": sha256(out / "feedback.csv")}
    save(out / "feedback.yaml", meta)
    execute("map-joint-feedback", "--data", out / "feedback.csv", "--manifest", out / "feedback.yaml",
        "--mapping", out / "mapping.yaml", "--urdf", urdf, "--allow-candidate", "--output-data", out / "mapped.csv",
        "--output-manifest", out / "mapped.yaml")
    save(out / "replay.yaml", {"class": "adapters.mapped_joint_state:MappedJointStateAdapter", "options": {
        "mapping": str(out / "mapping.yaml"), "urdf": str(Path(urdf).resolve()), "allow_candidate": True,
        "source": {"class": "adapters.replay_adapter:ReplayAdapter", "options": {"path": str(out / "feedback.csv")}}}})
    execute("capture-data", "--interface", out / "replay.yaml", "--manifest", out / "feedback.yaml",
        "--output-data", out / "adapter_capture.csv", "--output-manifest", out / "adapter_capture.yaml")
    save(out / "summary.json", {"status": "synthetic_pass", "data_kind": "synthetic", "joint": joint,
        "fitted": load(out / "calibrate-joint-reference.json")["joints"][joint],
        "mapped_adapter_data_kind": load(out / "adapter_capture.yaml")["data_kind"],
        "hardware_control_commands_sent": False, "is_real_robot_acceptance": False})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "joint", "destination"):
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    run(args.urdf, args.joint, args.destination)

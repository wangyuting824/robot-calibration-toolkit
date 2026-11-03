"""Recover known offsets on each seven-joint OpenArm using MuJoCo-generated poses.

All generated observations are SYNTHETIC. This exercises the actual calibration
and holdout-validation CLI on a larger model, without an external instrument.
It cannot measure this physical robot's offsets or calibration accuracy.
"""
import argparse
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.io import InputError, load, save, sha256, write_csv
from core.model import Model
from core.mujoco_model import default_poses, set_pose, validate_static


def run(args):
    import mujoco
    nominal = Model(args.urdf)
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    validate_static(nominal, model)
    data = mujoco.MjData(model)
    destination = Path(args.destination).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    random = np.random.default_rng(20261103)
    middle = default_poses(nominal)[0]
    amplitude = {n: .2 if nominal.joints[n].get("type") == "continuous" else
        .15 * (float(nominal.joints[n].find("limit").get("upper")) - float(nominal.joints[n].find("limit").get("lower")))
        for n in nominal.independent}
    acceptance = destination / "acceptance.yaml"
    save(acceptance, {"kinematics": {"max_position_m": 1e-7, "max_orientation_rad": 1e-7},
        "scope": "Synthetic known-answer software test tolerances, not robot accuracy requirements"})
    results = {}
    for side in ("left", "right"):
        names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
        if set(names) - set(nominal.independent):
            raise InputError("This example requires the named OpenArm seven-joint arms")
        truth = dict(zip(names, [.01, -.008, .006, -.005, .004, -.003, .002]))
        config = {"observed_frame": "synthetic_model_world", "tip_link": f"openarm_{side}_ee_base_link",
            "position_sigma_m": 1e-4, "angle_sigma_rad": 1e-4, "loss": "linear",
            "free_parameters": [{"kind": "joint_offset", "joint": name, "lower": -.05, "upper": .05} for name in names]}
        save(destination / (side + "_config.yaml"), config)
        for role, count in (("calibration", 48), ("validation", 24)):
            rows = []
            for i in range(count):
                q = {n: v + amplitude[n] * random.uniform(-1, 1) for n, v in middle.items()}
                corrected = {n: v + truth.get(n, 0) for n, v in q.items()}
                set_pose(nominal, model, data, corrected)
                body = data.body(config["tip_link"])
                values = [*body.xpos, *np.roll(body.xquat, -1)]
                rows.append({"sample_id": f"synthetic-{side}-{role}-{i}",
                    **dict(zip(("x", "y", "z", "qx", "qy", "qz", "qw"), map(float, values))),
                    **{"q:" + n: float(v) for n, v in q.items()}})
            csv = destination / f"{side}_{role}.csv"
            write_csv(csv, rows)
            save(csv.with_suffix(".yaml"), {"dataset_id": f"synthetic-openarm-{side}-{role}", "role": role,
                "data_kind": "synthetic", "units": {"length": "m", "angle": "rad", "time": "s", "effort": "N_or_Nm"},
                "clock": "generated sample index; no hardware timestamps", "observed_frame": config["observed_frame"],
                "source": "MuJoCo FK with known injected offsets; NOT a physical measurement",
                "pose_source": "synthetic MuJoCo mj_forward", "sha256": sha256(csv)})
        fit_path = destination / (side + "_fit.json")
        validation_path = destination / (side + "_validation.json")
        for role, command, extra, output in (
            ("calibration", "calibrate-kinematics", ["--config", destination / (side + "_config.yaml")], fit_path),
            ("validation", "validate-kinematics", ["--calibration", fit_path, "--acceptance", acceptance,
                "--errors-csv", destination / (side + "_errors.csv")], validation_path)):
            csv = destination / f"{side}_{role}.csv"
            invocation = [sys.executable, str(ROOT / "run.py"), command, "--urdf", args.urdf,
                "--data", csv, "--manifest", csv.with_suffix(".yaml"), *extra, "--output", output]
            with (destination / (side + "_" + role + ".log")).open("w") as log:
                subprocess.run(list(map(str, invocation)), stdout=log, stderr=subprocess.STDOUT, check=True)
        fitted = load(fit_path)
        errors = {name: value - truth[name] for name, value in zip(names, fitted["values"])}
        maximum = max(abs(v) for v in errors.values())
        if maximum > 1e-6 or load(validation_path)["status"] != "synthetic_pass":
            raise InputError(f"Known-answer offset recovery failed: {side}")
        results[side] = {"injected_offsets_rad": truth, "recovery_errors_rad": errors,
            "max_offset_error_rad": maximum, "jacobian_rank": fitted["diagnostics"],
            "calibration_samples": 48, "holdout_samples": 24, "validation": load(validation_path)["metrics"]}
    save(destination / "summary.json", {"status": "synthetic_pass", "data_kind": "synthetic", "arms": results,
        "mujoco_version": mujoco.__version__, "urdf_sha256": sha256(args.urdf), "mjcf_sha256": sha256(args.mjcf),
        "is_real_robot_acceptance": False, "hardware_control_commands_sent": False})
    print("OpenArm known-offset recovery passed for both seven-joint arms; SYNTHETIC observations only.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "mjcf", "destination"):
        parser.add_argument("--" + name, required=True)
    run(parser.parse_args())

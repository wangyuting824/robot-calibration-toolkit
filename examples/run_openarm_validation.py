"""Repeat the OpenArm nominal-model test; optional diagnostics never actuate it."""
from pathlib import Path
import argparse
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.io import InputError, load, save, csv_rows
from core.model import Model
from examples.prepare_openarm import prepare
from examples.openarm_motor_profile import motor_profile
from examples.analyze_openarm_feedback import analyze


def run(args):
    output = Path(args.destination).resolve()
    prepare(args.urdf, args.description_root, output / "prepared")
    results = output / "results"
    results.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    for key in ("CONDA_PREFIX", "ISAAC_PATH", "EXP_PATH", "CARB_APP_PATH"):
        env.pop(key, None)
    env["PYTHONPATH"] = os.pathsep.join(str(p) for p in (ROOT, args.extra_pythonpath) if p)

    def execute(command, *options, sdk=False, allow_blocked=False, python=None, report_name=None):
        name = report_name or command
        target = results / (name + ".json")
        with (results / (name + ".log")).open("w") as log:
            process = subprocess.run([str(python or (args.launcher if sdk else sys.executable)), str(ROOT / "run.py"),
                command, *map(str, options), "--output", str(target)], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        report = load(target)
        print(command + ": " + report["status"], flush=True)
        if process.returncode and not (allow_blocked and report["status"] == "blocked"):
            raise InputError(f"OpenArm software test failed: inspect {target} and its log")
        return report

    hardware = execute("check-openarm", *(["--query-state"] if args.query_state else []),
                       *(["--probe-warning"] if args.probe_warning else []),
                       "--raw-frames", output / "raw_can_frames.json", allow_blocked=True) if args.check_can else None
    parameters, mechanisms = output / "prepared/parameters.xlsx", output / "prepared/mechanisms.yaml"
    execute("validate-structure", "--parameters", parameters, "--mechanisms", mechanisms, "--require-inertia")
    execute("build-urdf", "--parameters", parameters, "--mechanisms", mechanisms, "--require-inertia",
            "--baseline", output / "prepared/nominal_physical.urdf", "--destination", output / "model")
    built = execute("build-isaac-asset", "--urdf", output / "model/robot.urdf", "--config", output / "prepared/isaac.yaml",
                    "--destination", output / "isaac", sdk=True)
    execute("validate-isaac-asset", "--urdf", output / "model/robot.urdf", "--stage", built["scene"],
            "--acceptance", output / "prepared/acceptance.yaml", sdk=True)
    model = Model(output / "model/robot.urdf")
    bounds = {n: [float(model.joints[n].find("limit").get(k)) for k in ("lower", "upper")] for n in model.independent}
    middle = {n: sum(bounds[n]) / 2 for n in model.independent}
    poses = [middle]
    for name in model.independent:
        for direction in (-1, 1):
            q = dict(middle)
            q[name] += direction * min(0.05, (bounds[name][1] - bounds[name][0]) * 0.1)
            model.check_q(q)
            poses.append(q)
    if len(built["articulation_paths"]) != 1:
        raise InputError("Choose a single OpenArm articulation root for this test")
    save(output / "runtime_config.yaml", {"joint_positions": poses, "options": {
        "articulation_path": built["articulation_paths"][0], "physics_dt": 1 / 240,
        "headless": True, "device": "cpu", "expected_isaac_sim_version": "6.1.0"}})
    save(output / "runtime_acceptance.yaml", {"runtime_kinematics": {"max_position_m": 1e-4, "max_orientation_rad": 1e-3},
        "scope": "Numerical software comparison thresholds, not hardware accuracy requirements"})
    runtime = execute("validate-runtime-kinematics", "--urdf", output / "model/robot.urdf", "--stage", built["scene"],
        "--config", output / "runtime_config.yaml", "--acceptance", output / "runtime_acceptance.yaml",
        "--errors-csv", output / "runtime_errors.csv", sdk=True)
    feedback, replay = None, None
    if args.capture_motors:
        if hardware is None or hardware["status"] != "ready":
            raise InputError("Motor recording requires a successful current CAN diagnostic")
        save(output / "readonly_interface.yaml", motor_profile())
        save(output / "motor_manifest.yaml", {"dataset_id": output.name + "-motor-feedback", "role": "validation", "data_kind": "real",
            "units": {"length": "m", "angle": "rad", "time": "s", "effort": "N_or_Nm"},
            "clock": "local monotonic; sequential replies, not synchronized joint snapshots", "joint_mapping_verified": False,
            "source": "Installed OpenArm official codec; motor telemetry, not calibrated joint coordinates"})
        execute("capture-data", "--interface", output / "readonly_interface.yaml", "--manifest", output / "motor_manifest.yaml",
            "--output-data", output / "motor_feedback.csv", "--output-manifest", output / "motor_feedback_manifest.yaml",
            "--max-samples", "48", python=args.motor_python)
        feedback = analyze(output / "motor_feedback.csv", output / "model/robot.urdf")
        save(results / "feedback-analysis.json", feedback)
        if not feedback["mapping_review_required"]:
            telemetry = csv_rows(output / "motor_feedback.csv")
            measured_poses = []
            for cycle in sorted({int(r["cycle"]) for r in telemetry}):
                arm = {r["joint_label_nominal"]: float(r["motor_position_rad"]) for r in telemetry
                       if int(r["cycle"]) == cycle and int(r["motor_id"]) < 8}
                if len(arm) != 14:
                    raise InputError("Incomplete arm recording cycle")
                q = {n: 0.0 if "finger" in n else arm[n] for n in model.independent}
                model.check_q(q)
                measured_poses.append(q)
            config = load(output / "runtime_config.yaml")
            config.update(joint_positions=measured_poses,
                input_semantics="14 arm motor values using configured identity mapping; gripper reference=0 is a simulation setting, not a measurement",
                joint_mapping_verified=False, is_real_robot_acceptance=False)
            save(output / "feedback_runtime_config.yaml", config)
            replay = execute("validate-runtime-kinematics", "--urdf", output / "model/robot.urdf", "--stage", built["scene"],
                "--config", output / "feedback_runtime_config.yaml", "--acceptance", output / "runtime_acceptance.yaml",
                "--errors-csv", output / "feedback_runtime_errors.csv", sdk=True, report_name="feedback-runtime-kinematics")
    execute("generate-report", "--results", results, "--destination", output / "report")
    summary = {"status": "software_validation_pass", "data_kind": "nominal_model",
        "hardware_status": hardware["status"] if hardware else "not_checked", "is_real_robot_acceptance": False,
        "hardware_control_commands_sent": False, "runtime_info": runtime["runtime_info"],
        "poses": runtime["poses"], "links_per_pose": runtime["links_per_pose"], "metrics": runtime["metrics"],
        "scope": "Source-model reproduction, static USD consistency and FK vs PhysX tensor poses; no physical calibration"}
    if feedback:
        summary["real_motor_feedback"] = {"samples": sum(r["samples"] for r in feedback["measurements"].values()),
            "motors": len(feedback["measurements"]), "mapping_review_required": feedback["mapping_review_required"],
            "calibrated_joint_coordinates": False}
    if replay:
        summary["arm_feedback_in_simulation"] = {"poses": replay["poses"], "links_per_pose": replay["links_per_pose"],
            "status": replay["status"], "metrics": replay["metrics"], "gripper_physical_alignment_verified": False}
    save(output / "summary.json", summary)
    return summary


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "description-root", "launcher", "destination"):
        p.add_argument("--" + name, required=True)
    p.add_argument("--extra-pythonpath")
    p.add_argument("--check-can", action="store_true")
    p.add_argument("--query-state", action="store_true")
    p.add_argument("--probe-warning", action="store_true")
    p.add_argument("--capture-motors", action="store_true")
    p.add_argument("--motor-python", help="Interpreter with the official OpenArm codec installed")
    args = p.parse_args()
    if args.query_state and not args.check_can:
        p.error("--query-state requires --check-can")
    if args.probe_warning and not args.query_state:
        p.error("--probe-warning requires --query-state")
    if args.capture_motors and (not args.check_can or not args.motor_python):
        p.error("--capture-motors requires --check-can and --motor-python")
    run(args)

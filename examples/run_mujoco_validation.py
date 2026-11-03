"""Compile a resolved URDF and test it in MuJoCo; optional recorded ROS feedback.

No hardware interface is imported. Input joint-state CSV is produced separately
by capture-data / Ros2JointStateAdapter. This is not an Isaac delivery check.
"""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.io import InputError, csv_rows, dataset, number, result, save, write_csv, portable_inputs
from core.model import Model
from core.mujoco_model import SOURCES, compare_fk, default_poses, physics_smoke, prepare, validate_static


def recorded_poses(nominal, path, reference_joints):
    rows = csv_rows(path, ("sequence", "joint", "q"))
    if set(reference_joints) - set(nominal.independent):
        raise InputError("Unknown reference joint")
    grouped, ranges = {}, {}
    for row in rows:
        name, value = row["joint"], number(row["q"])
        frame = grouped.setdefault(int(row["sequence"]), {})
        if name in frame:
            raise InputError("Duplicate joint in a recorded message")
        frame[name] = value
        if name in nominal.independent:
            el = nominal.joints[name]
            bounds = None if el.get("type") == "continuous" else [float(el.find("limit").get(k)) for k in ("lower", "upper")]
            item = ranges.setdefault(name, {"joint": name, "samples": 0, "min_q": value, "max_q": value,
                "lower": bounds[0] if bounds else "", "upper": bounds[1] if bounds else "", "out_of_range_samples": 0})
            item.update(samples=item["samples"] + 1, min_q=min(item["min_q"], value), max_q=max(item["max_q"], value))
            item["out_of_range_samples"] += int(bounds is not None and not bounds[0] <= value <= bounds[1])
    poses, rejected = [], []
    for sequence, frame in sorted(grouped.items()):
        missing = set(nominal.independent) - set(frame) - set(reference_joints)
        if missing:
            raise InputError(f"Missing recorded joints: {missing}")
        # Explicit reference joints are never presented as measured values.
        q = {name: 0.0 if name in reference_joints else frame[name] for name in nominal.independent}
        try:
            nominal.check_q(q)
            poses.append(q)
        except InputError as exc:
            rejected.append({"sequence": sequence, "reason": str(exc)})
    return poses, list(ranges.values()), rejected


def run(args):
    if args.joint_states and not args.joint_manifest:
        raise InputError("Recorded joint states require --joint-manifest with units, source, clock and SHA-256")
    feedback_meta = None
    if args.joint_states:
        _, feedback_meta = dataset(args.joint_states, args.joint_manifest, ("sequence", "joint", "q"), "validation")
        if not feedback_meta.get("sha256"):
            raise InputError("Recorded joint-state manifest requires SHA-256")
    import mujoco
    nominal = Model(args.urdf)
    destination = Path(args.destination).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    mjcf = prepare(args.urdf, destination / "model")
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    poses = default_poses(nominal)
    save(destination / "test_poses.json", {"poses": poses, "mode": "nominal_test_poses", "hardware_control_commands_sent": False})
    static = validate_static(nominal, model)
    numerical = compare_fk(nominal, model, poses, destination / "fk_errors.csv")
    feedback = None
    if args.joint_states:
        recorded, ranges, rejected = recorded_poses(nominal, args.joint_states, args.reference_joints)
        write_csv(destination / "feedback_ranges.csv", ranges)
        feedback = {"source": str(Path(args.joint_states).resolve()), "accepted_messages": len(recorded),
            "dataset": feedback_meta,
            "rejected_messages": rejected, "range_checks": ranges, "joint_mapping_verified": False,
            "reference_joints_set_to_zero": args.reference_joints, "is_real_robot_acceptance": False}
        if recorded:
            feedback["fk"] = compare_fk(nominal, model, recorded, destination / "feedback_fk_errors.csv")
            save(destination / "recorded_poses.json", {"poses": recorded, "mode": "recorded_arm_values_with_explicit_reference_joints", "joint_mapping_verified": False})
            # Start at the recorded arm pose, then allow visible software-only pose exploration.
            poses = recorded[:1] + poses
    save(destination / "view_poses.json", {"poses": poses, "mode": "recorded_arm_values_with_explicit_reference_joints" if feedback and feedback["accepted_messages"] else "nominal_test_poses",
        "pose_semantics": "If feedback was accepted: first pose=recorded arm values with reference joints; remaining poses=nominal software test poses",
        "joint_mapping_verified": False, "hardware_control_commands_sent": False})
    smoke = physics_smoke(nominal, model)
    failed = numerical["status"] == "failed" or bool(feedback and (
        not feedback["accepted_messages"] or feedback["rejected_messages"] or feedback.get("fk", {}).get("status") == "failed"))
    summary = result("mujoco-software-validation", {"backend": "MuJoCo", "version": mujoco.__version__,
        "static": static, "fk": numerical, "feedback": feedback, "physics_smoke": smoke, "sources": SOURCES,
        "model": str(mjcf), "is_real_robot_acceptance": False, "hardware_control_commands_sent": False,
        "scope": "Nominal model compiler/FK/engine checks. Recorded feedback coordinates remain uncalibrated; no physical measurement or Isaac acceptance."},
        [args.urdf, args.joint_states, args.joint_manifest], status="failed" if failed else "simulation_pass")
    save(destination / "summary.json", portable_inputs(summary, destination / "summary.json"))
    print(f"MuJoCo {mujoco.__version__}: {summary['status']}; {numerical['comparisons']} link comparisons", flush=True)
    if failed:
        raise InputError("MuJoCo numerical comparison failed; inspect summary.json")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--destination", required=True)
    parser.add_argument("--joint-states")
    parser.add_argument("--joint-manifest")
    parser.add_argument("--reference-joints", nargs="*", default=[], help="Explicitly replace these feedback values with simulation zero; retained in report")
    args = parser.parse_args()
    try:
        run(args)
    except Exception as exc:
        destination = Path(args.destination).resolve()
        destination.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        failure = destination / ("failure_" + stamp + ".json")
        save(failure, result("mujoco-software-validation", {"error": str(exc), "sources": SOURCES,
            "is_real_robot_acceptance": False, "hardware_control_commands_sent": False}, [args.urdf], status="blocked"))
        print(f"MuJoCo check blocked: {exc}\nResult: {failure}", file=sys.stderr)
        sys.exit(2)

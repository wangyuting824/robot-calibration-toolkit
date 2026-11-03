"""Audit saved physical width observations and current publisher coordinates.

Reads files only. Width observations are never relabelled joint-angle reference
data. Hypotheses are tested for consistency, never installed on a controller.
"""
import argparse
from collections import Counter
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.io import InputError, dataset, load, number, result, save, write_csv
from core.model import Model


def audit(urdf, feedback, historical, destination, feedback_manifest):
    nominal = Model(urdf)
    original = load(historical)
    if original.get("units") != {"q": "rad", "width": "mm"} or not original.get("measurement_definition") or not original.get("instrument"):
        raise InputError("Historical width records need units, measurement definition and instrument")
    samples = original.get("samples", [])
    name = original.get("joint")
    if name not in nominal.independent or not samples:
        raise InputError("Historical joint is missing from URDF or no samples were recorded")
    limit = nominal.joints[name].find("limit")
    lower, upper = [number(limit.get(k)) for k in ("lower", "upper")]
    rows = [{"sample_id": f"historical-{i}", "phase": s["phase"], "q_feedback_rad": number(s["q_rad"]),
        "width_measured_mm": number(s["width_mm"]), "feedback_span_rad": number(s["q_span_rad"])} for i, s in enumerate(samples)]
    opening = [r for r in rows if r["phase"] == "opening"]
    closing = [r for r in rows if r["phase"] == "closing"]
    validation = []
    # Reproduce the saved method, preserving the original holdout partition.
    for direction in ("opening", "closing"):
        training = [r for r in rows if r["phase"] == direction]
        q = np.array([r["q_feedback_rad"] for r in training])
        w = np.array([r["width_measured_mm"] for r in training])
        monotonic = len(q) >= 2 and (np.all(np.diff(q) > 0) or np.all(np.diff(q) < 0)) and (np.all(np.diff(w) > 0) or np.all(np.diff(w) < 0))
        for r in [r for r in rows if r["phase"] == "validate_" + direction]:
            if not monotonic:
                validation.append({"phase": direction, "status": "unverified", "reason": "Nonmonotonic training sequence; no relabelling or point deletion"})
            elif not min(q) <= r["q_feedback_rad"] <= max(q):
                validation.append({"phase": direction, "status": "unverified", "reason": "Holdout lies outside training domain; no extrapolation"})
            else:
                order = np.argsort(q)
                prediction = float(np.interp(r["q_feedback_rad"], q[order], w[order]))
                validation.append({"phase": direction, "status": "computed", "predicted_mm": prediction,
                    "measured_mm": r["width_measured_mm"], "error_mm": prediction - r["width_measured_mm"]})
    current_rows, current_meta = dataset(feedback, feedback_manifest, ("joint", "q"))
    if not current_meta.get("sha256") or current_meta.get("joint_units", {}).get(name) != "rad":
        raise InputError("Current feedback needs a SHA-256 manifest and explicit radian joint unit")
    current = [number(r["q"]) for r in current_rows if r["joint"] == name]
    if not current:
        raise InputError("Current feedback has no matching gripper")
    old = [r["q_feedback_rad"] for r in rows]
    hypotheses = [{"mapping": label, "current_inside_urdf": sum(lower <= scale * q <= upper for q in current),
        "current_total": len(current), "historical_inside_urdf": sum(lower <= scale * q <= upper for q in old),
        "historical_total": len(old), "cross_session_coordinates_verified": False}
        for label, scale in (("identity q_model=q_published", 1), ("sign_flip q_model=-q_published", -1))]
    target = Path(destination)
    target.mkdir(parents=True, exist_ok=False)
    write_csv(target / "historical_width_records.csv", rows)
    report = result("audit-openarm-gripper", {"joint": name, "instrument": original["instrument"],
        "measurement_definition": original["measurement_definition"], "historical_records": len(rows),
        "phases": dict(Counter(r["phase"] for r in rows)), "width_holdout": validation, "hypotheses": hypotheses,
        "urdf_range_rad": [lower, upper], "current_range_rad": [min(current), max(current)],
        "current_coordinates_id": current_meta.get("coordinates_id"), "robot_id": current_meta.get("robot_id"),
        "status_reason": "Historical physical measurements found, but they are width references with failed/incomplete validation; no unique current angle mapping is established",
        "reference_angle_available": False, "mapping_ready_for_physical_use": False,
        "hardware_control_commands_sent": False, "is_real_robot_acceptance": False,
        "sources": {"position_semantics": "https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg",
            "signed_transmission": "https://github.com/ros-controls/ros2_control/blob/jazzy/transmission_interface/include/transmission_interface/simple_transmission.hpp"}},
        [urdf, feedback, feedback_manifest, historical], status="unverified")
    save(target / "audit.json", report)
    print(f"Historical physical measurements found: {len(rows)}. Current coordinate correction remains unverified.")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "feedback", "feedback-manifest", "historical", "destination"):
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    audit(args.urdf, args.feedback, args.historical, args.destination, args.feedback_manifest)

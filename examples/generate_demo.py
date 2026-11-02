"""Generate small SYNTHETIC datasets with known answers, never real robot evidence."""
from pathlib import Path
import argparse
import numpy as np
from scipy.spatial.transform import Rotation

from core.dynamics import inverse_dynamics
from core.io import save, sha256, write_csv
from core.model import Model
from core.optimization import predict
from core.schemas import create_workbook

URDF = '''<?xml version="1.0"?>
<robot name="synthetic_two_joint_arm">
 <link name="base"><inertial><origin xyz="0 0 0"/><mass value="10"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
 <link name="arm1"><inertial><origin xyz="0.5 0 0"/><mass value="2"/><inertia ixx="0.02" iyy="0.18" izz="0.18" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0.5 0 0"/><geometry><box size="1 0.1 0.1"/></geometry></visual><collision><origin xyz="0.5 0 0"/><geometry><box size="1 0.1 0.1"/></geometry></collision></link>
 <joint name="j1" type="revolute"><parent link="base"/><child link="arm1"/><origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="100" velocity="2"/></joint>
 <link name="arm2"><inertial><origin xyz="0.35 0 0"/><mass value="1.5"/><inertia ixx="0.02" iyy="0.08" izz="0.08" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0.35 0 0"/><geometry><box size="0.7 0.1 0.1"/></geometry></visual><collision><origin xyz="0.35 0 0"/><geometry><box size="0.7 0.1 0.1"/></geometry></collision></link>
 <joint name="j2" type="revolute"><parent link="arm1"/><child link="arm2"/><origin xyz="1 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="100" velocity="2"/></joint>
</robot>'''


def manifest(name, role, **extra):
    return {"schema_version": 1, "dataset_id": "synthetic-" + name, "role": role,
            "data_kind": "synthetic", "units": {"length": "m", "angle": "rad", "time": "s", "effort": "N_or_Nm"},
            "clock": "synthetic_test_clock", "source": "generated mathematics; NOT a real measurement", **extra}


def generate(destination):
    root = Path(destination).resolve()
    root.mkdir(parents=True, exist_ok=True)
    urdf = root / "nominal.urdf"
    urdf.write_text(URDF, encoding="utf-8")
    model = Model(urdf)
    rng = np.random.default_rng(824)
    config = {"observed_frame": "measurement_world", "tip_link": "arm2",
              "base_xyz_rpy": [0.1, -0.2, 0.3, 0.02, -0.01, 0.1],
              "tool_xyz_rpy": [0.7, 0, 0, 0, 0, 0],
              "position_sigma_m": 0.0001, "angle_sigma_rad": 0.0001,
              "free_parameters": [{"kind": "joint_offset", "joint": n, "lower": -0.1, "upper": 0.1} for n in model.independent]}
    save(root / "calibration.yaml", config)
    truth = [0.025, -0.035]
    for role, count in (("calibration", 40), ("validation", 20)):
        samples = []
        for index in range(count):
            row = {"sample_id": f"{role}-{index}", **{"q:" + n: float(v) for n, v in zip(model.independent, rng.uniform(-1, 1, 2))}}
            pose = predict(model, row, config, config["free_parameters"], truth)
            row.update(dict(zip(("x", "y", "z"), pose[:3, 3])))
            row.update(dict(zip(("qx", "qy", "qz", "qw"), Rotation.from_matrix(pose[:3, :3]).as_quat())))
            samples.append(row)
        write_csv(root / f"kinematics_{role}.csv", samples)
        save(root / f"kinematics_{role}.yaml", manifest("kinematics-" + role, role,
             observed_frame="measurement_world", pose_source="synthetic ground truth", sha256=sha256(root / f"kinematics_{role}.csv")))
    records = {"robot_info": {"name": "synthetic_two_joint_arm", "units": "m rad kg s"}, "links": [], "joints": [], "geometry": []}
    for name, el in model.links.items():
        inertia = el.find("inertial")
        records["links"].append({"name": name, "mass": float(inertia.find("mass").get("value")),
          "com_xyz": inertia.find("origin").get("xyz"), "com_rpy": "0 0 0",
          "inertia": " ".join(inertia.find("inertia").get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
          "status": "confirmed", "source": "synthetic fixture only", "inertial_status": "confirmed", "inertial_source": "synthetic fixture only"})
        records["geometry"].append({"target_link": name, "source_link": name, "origin_xyz": "0 0 0", "origin_rpy": "0 0 0"})
    for name, el in model.joints.items():
        records["joints"].append({"name": name, "parent": el.find("parent").get("link"), "child": el.find("child").get("link"),
          "type": "revolute", "origin_xyz": el.find("origin").get("xyz"), "origin_rpy": "0 0 0", "axis": el.find("axis").get("xyz"),
          "lower": -2, "upper": 2, "velocity": 2, "effort": 100, "status": "confirmed", "source": "synthetic fixture only"})
    save(root / "parameters.yaml", records)
    save(root / "mechanisms.yaml", {"status": "confirmed", "source": "synthetic mechanism", "closed_loops": []})
    create_workbook(root / "parameters.xlsx", {**records, "robot_info": [{"key": k, "value": v} for k, v in records["robot_info"].items()]})
    reference = [{"joint": n, "raw": float(raw), "reference": float(0.00012 * raw - 0.3)}
                 for n in model.independent for raw in np.linspace(1000, 5000, 12)]
    write_csv(root / "joint_reference.csv", reference)
    save(root / "joint_reference.yaml", manifest("joint-reference", "calibration", reference_instrument="synthetic reference",
                                               raw_unit="effective_counts", reference_unit="rad"))
    limits = [{"joint": n, "boundary": boundary, "angle": float(value + rng.uniform(-1e-6, 1e-6))}
              for n in model.independent for boundary, value in (("lower", -2), ("upper", 2)) for _ in range(6)]
    write_csv(root / "limits.csv", limits)
    save(root / "limits_manifest.yaml", manifest("limits", "validation", event_kind="software_boundary",
                                                procedure_reference="SYNTHETIC; no physical procedure"))
    save(root / "limits_spec.yaml", {"joints": {n: {"lower": -2, "upper": 2, "kind": "software_boundary", "source": "synthetic configuration",
        "tolerance": 1e-4, "repeatability_peak_to_peak": 1e-4, "minimum_repeats": 5} for n in model.independent}})
    dyn_config = {"fixed_base": True, "inertia_source": "synthetic fixture", "gravity": [0, 0, -9.81], "friction_velocity_scale": 0.05}
    save(root / "dynamics_config.yaml", dyn_config)
    dynamic = []
    for _ in range(60):
        q, dq, ddq = [{n: float(v) for n, v in zip(model.independent, rng.uniform(-1, 1, 2))} for _ in range(3)]
        rigid = inverse_dynamics(model, q, dq, ddq)
        row = {f"{key}:{n}": state[n] for key, state in (("q", q), ("dq", dq), ("ddq", ddq)) for n in model.independent}
        row.update({"tau:" + n: rigid[n] + 0.12 * ddq[n] + 0.2 * dq[n] + 0.3 * np.tanh(dq[n] / 0.05) + 0.04 for n in model.independent})
        dynamic.append(row)
    write_csv(root / "dynamics_fit.csv", dynamic)
    save(root / "dynamics_fit.yaml", manifest("dynamics-fit", "calibration", torque_semantics="joint_actuator_effort", effort_reference="synthetic effort"))
    drives = [{"joint": n, "q": 0, "dq": 0, "q_command": float(e), "dq_command": float(ed), "pd_effort": float(100 * e + 5 * ed), "saturated": False}
              for n in model.independent for e, ed in rng.uniform(-0.02, 0.02, (30, 2))]
    write_csv(root / "drives.csv", drives)
    save(root / "drives.yaml", manifest("drives", "calibration", torque_semantics="isolated_pd_effort", drive_type="force"))
    contact = []
    for force, velocity in zip(rng.uniform(5, 30, 30), rng.uniform(0.1, 1, 30)):
        contact.append({"normal_force": float(force), "slip_velocity": float(velocity),
                        "tangential_effort": float(0.4 * force * np.tanh(velocity / 0.01) + 0.2 * velocity)})
    write_csv(root / "contact.csv", contact)
    save(root / "contact.yaml", manifest("contact", "calibration", procedure_reference="SYNTHETIC only", force_reference="synthetic effort"))
    save(root / "contact_config.yaml", {"model": "sliding_coulomb_viscous", "velocity_scale": 0.01})
    paired_meta = {"test_case_id": "synthetic_sine", "input_sha256": "synthetic-input", "initial_state": {"j1": 0, "j2": 0},
                   "payload_id": "synthetic-load", "time_basis": "test_relative", "effort_reference": "synthetic effort"}
    for name, perturbation in (("real", 0), ("sim", 1e-4)):
        paired = [{"joint": n, "t": float(t), "q": float(np.sin(t) + perturbation),
                   "dq": float(np.cos(t) + perturbation), "tau": float(2 * np.sin(t) + perturbation)}
                  for n in model.independent for t in np.linspace(0, 1, 30)]
        write_csv(root / f"response_{name}.csv", paired)
        save(root / f"response_{name}.yaml", manifest("response-" + name, "validation", **paired_meta))
    save(root / "acceptance.yaml", {"kinematics": {"max_position_m": 1e-4, "max_orientation_rad": 1e-4},
         "asset": {"limit_tolerance_rad_or_m": 1e-6}, "dynamics": {"required_metrics": ["q", "dq", "tau"], "max_abs": {"q": 0.001, "dq": 0.001, "tau": 0.001}}})
    replay_rows = [{"t": float(t), "q:j1": float(t / 10), "q:j2": float(-t / 10)} for t in np.linspace(0, 1, 10)]
    write_csv(root / "replay.csv", replay_rows)
    save(root / "replay_interface.yaml", {"class": "adapters.replay_adapter:ReplayAdapter", "options": {"path": str(root / "replay.csv")}})
    save(root / "replay_manifest.yaml", manifest("replay", "validation"))
    save(root / "D03_readiness.yaml", {"status": "not_run", "reason": "Synthetic offline demo has no Isaac runtime validation"})
    return root


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", required=True)
    print(generate(parser.parse_args().destination))

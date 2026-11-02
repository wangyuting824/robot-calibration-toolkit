from types import SimpleNamespace as Args
import numpy as np
import pytest
from core.io import InputError, load, save, dataset, csv_rows, write_csv
from core.model import Model
from core.optimization import fit_kinematics
from programs.stage_02_kinematics import calibrate_joint_reference, calibrate_kinematics, validate_kinematics, verify_joint_limits
from programs.stage_03_isaac_dynamics import identify_dynamics, tune_drives, identify_contact, validate_dynamics


def fit(demo):
    return calibrate_kinematics.run(Args(urdf=demo / "nominal.urdf", data=demo / "kinematics_calibration.csv",
        manifest=demo / "kinematics_calibration.yaml", config=demo / "calibration.yaml"))


def test_joint_reference_recovery(demo):
    output = calibrate_joint_reference.run(Args(data=demo / "joint_reference.csv", manifest=demo / "joint_reference.yaml", f_scale=0.001))
    assert output["joints"]["j1"]["scale"] == pytest.approx(0.00012, abs=1e-12)
    assert output["joints"]["j1"]["offset"] == pytest.approx(-0.3, abs=1e-10)


def test_kinematic_recovery_and_holdout(demo):
    output = fit(demo)
    np.testing.assert_allclose(output["values"], [0.025, -0.035], atol=1e-8)
    save(demo / "fit.json", output)
    checked = validate_kinematics.run(Args(urdf=demo / "nominal.urdf", data=demo / "kinematics_validation.csv",
        manifest=demo / "kinematics_validation.yaml", calibration=demo / "fit.json", acceptance=demo / "acceptance.yaml", errors_csv=None))
    assert checked["status"] == "synthetic_pass"
    assert checked["metrics"]["position_m"]["max_abs"] < 1e-8


def test_calibration_does_not_allow_training_reuse(demo):
    save(demo / "fit.json", fit(demo))
    meta = load(demo / "kinematics_calibration.yaml")
    meta["role"], meta["dataset_id"] = "validation", "pretend-new"
    save(demo / "bad_manifest.yaml", meta)
    with pytest.raises(InputError, match="reuse"):
        validate_kinematics.run(Args(urdf=demo / "nominal.urdf", data=demo / "kinematics_calibration.csv",
            manifest=demo / "bad_manifest.yaml", calibration=demo / "fit.json", acceptance=demo / "acceptance.yaml", errors_csv=None))


def test_unobservable_joint_offset_and_same_axis_origin_rejected(demo):
    config = load(demo / "calibration.yaml")
    config["free_parameters"].append({"kind": "joint_origin", "joint": "j1", "component": "yaw", "lower": -0.1, "upper": 0.1})
    rows = csv_rows(demo / "kinematics_calibration.csv")
    with pytest.raises(InputError, match="rank-deficient"):
        fit_kinematics(Model(demo / "nominal.urdf"), rows, config)


def test_missing_acceptance_threshold_is_not_pass(demo):
    save(demo / "fit.json", fit(demo))
    save(demo / "no_threshold.yaml", {"kinematics": {}})
    checked = validate_kinematics.run(Args(urdf=demo / "nominal.urdf", data=demo / "kinematics_validation.csv",
        manifest=demo / "kinematics_validation.yaml", calibration=demo / "fit.json", acceptance=demo / "no_threshold.yaml", errors_csv=None))
    assert checked["status"] == "unverified"


def test_limit_events_and_repeatability(demo):
    args = Args(data=demo / "limits.csv", manifest=demo / "limits_manifest.yaml", specification=demo / "limits_spec.yaml")
    assert verify_joint_limits.run(args)["status"] == "synthetic_pass"
    spec = load(args.specification)
    spec["joints"]["j1"]["upper"] = 1.9
    save(args.specification, spec)
    assert verify_joint_limits.run(args)["status"] == "failed"
    meta = load(args.manifest)
    meta["event_kind"] = "final_stopped_position"
    save(args.manifest, meta)
    with pytest.raises(InputError, match="event_kind"):
        verify_joint_limits.run(args)


def test_dynamics_parameters_recovered(demo):
    output = identify_dynamics.run(Args(urdf=demo / "nominal.urdf", data=demo / "dynamics_fit.csv", manifest=demo / "dynamics_fit.yaml", config=demo / "dynamics_config.yaml"))
    for row in output["joints"].values():
        np.testing.assert_allclose([row[k] for k in ("armature", "viscous_friction", "coulomb_friction", "effort_bias")], [0.12, 0.2, 0.3, 0.04], atol=1e-9)
    assert output["link_inertias_estimated"] is False


def test_motor_current_is_not_assumed_to_be_joint_torque(demo):
    meta = load(demo / "dynamics_fit.yaml")
    meta["torque_semantics"] = "motor_current"
    save(demo / "dynamics_fit.yaml", meta)
    with pytest.raises(InputError, match="joint-actuator-effort"):
        identify_dynamics.run(Args(urdf=demo / "nominal.urdf", data=demo / "dynamics_fit.csv", manifest=demo / "dynamics_fit.yaml", config=demo / "dynamics_config.yaml"))


def test_pd_and_contact_fits(demo):
    drives = tune_drives.run(Args(data=demo / "drives.csv", manifest=demo / "drives.yaml"))
    assert drives["joints"]["j1"]["stiffness"] == pytest.approx(100)
    assert drives["joints"]["j1"]["damping"] == pytest.approx(5)
    contact = identify_contact.run(Args(data=demo / "contact.csv", manifest=demo / "contact.yaml", config=demo / "contact_config.yaml"))
    assert contact["parameters"]["dynamic_friction"] == pytest.approx(0.4)
    assert contact["static_friction_identified"] is False


@pytest.mark.parametrize("field", ["test_case_id", "payload_id", "initial_state", "input_sha256", "time_basis"])
def test_mismatched_experiments_rejected(demo, field):
    meta = load(demo / "response_sim.yaml")
    meta[field] = "mismatched"
    save(demo / "response_sim.yaml", meta)
    with pytest.raises(InputError, match=field):
        validate_dynamics.run(Args(real_data=demo / "response_real.csv", real_manifest=demo / "response_real.yaml",
            sim_data=demo / "response_sim.csv", sim_manifest=demo / "response_sim.yaml", acceptance=demo / "acceptance.yaml", errors_csv=None))


def test_response_comparison_and_no_extrapolation(demo):
    args = Args(real_data=demo / "response_real.csv", real_manifest=demo / "response_real.yaml",
                sim_data=demo / "response_sim.csv", sim_manifest=demo / "response_sim.yaml", acceptance=demo / "acceptance.yaml", errors_csv=None)
    assert validate_dynamics.run(args)["status"] == "synthetic_pass"
    rows = csv_rows(args.sim_data)
    rows = [r for r in rows if float(r["t"]) > 0]
    write_csv(args.sim_data, rows)
    with pytest.raises(InputError, match="overlap"):
        validate_dynamics.run(args)


def test_unit_mismatch_is_rejected(demo):
    meta = load(demo / "joint_reference.yaml")
    meta["units"]["angle"] = "deg"
    save(demo / "joint_reference.yaml", meta)
    with pytest.raises(InputError, match="units"):
        dataset(demo / "joint_reference.csv", demo / "joint_reference.yaml")


def test_contact_only_comparison_does_not_require_fictional_angles(demo):
    rows = [{"joint": "pad_floor", "t": i * 0.1, "normal_force": 10 + i, "slip_velocity": 0.02} for i in range(3)]
    for kind in ("real", "sim"):
        write_csv(demo / f"contact_{kind}.csv", rows)
    save(demo / "contact_acceptance.yaml", {"dynamics": {"required_metrics": ["normal_force", "slip_velocity"],
        "max_abs": {"normal_force": 0.01, "slip_velocity": 0.001}}})
    report = validate_dynamics.run(Args(real_data=demo / "contact_real.csv", real_manifest=demo / "response_real.yaml",
        sim_data=demo / "contact_sim.csv", sim_manifest=demo / "response_sim.yaml", acceptance=demo / "contact_acceptance.yaml", errors_csv=None))
    assert report["status"] == "synthetic_pass"
    assert set(report["metrics"]["pad_floor"]) == {"normal_force", "slip_velocity"}

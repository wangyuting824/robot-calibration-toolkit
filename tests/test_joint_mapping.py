"""Known-answer fixtures and failure gates; no physical reference claims."""
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from adapters.mapped_joint_state import MappedJointStateAdapter
from adapters.ros2_joint_states import message_rows
from core.cli import main
from core.io import InputError, load, save, sha256, write_csv
from core.joint_mapping import JointMapping


@pytest.fixture
def reference_case(demo):
    meta = {"dataset_id": "train", "role": "calibration", "data_kind": "synthetic",
        "units": {"length": "m", "angle": "rad", "time": "s", "effort": "N_or_Nm"},
        "clock": "synthetic sample index", "source": "known-answer test fixture, not a measurement",
        "raw_unit": "rad", "reference_unit": "rad", "reference_kind": "synthetic_ground_truth",
        "reference_definition": "nominal j1 angle", "reference_instrument": "synthetic oracle",
        "robot_id": "synthetic-example", "feedback_coordinates_id": "synthetic-publisher-v1", "urdf_sha256": sha256(demo / "nominal.urdf")}
    for name, values in (("train", [-.8, -.5, -.1, .2, .6, .9]), ("holdout", [-.7, -.3, .1, .5, .8])):
        csv = demo / (name + ".csv")
        write_csv(csv, [{"sample_id": f"{name}-{i}", "joint": "j1", "raw": raw, "reference": -.7 * raw + .03} for i, raw in enumerate(values)])
        save(demo / (name + ".yaml"), {**meta, "dataset_id": name,
            "role": "calibration" if name == "train" else "validation", "sha256": sha256(csv)})
    save(demo / "reference_acceptance.yaml", {"joint_reference": {"joints": {"j1": {
        "max_abs_error": 1e-7, "minimum_validation_samples": 3}}}})
    assert main(["calibrate-joint-reference", "--data", str(demo / "train.csv"), "--manifest", str(demo / "train.yaml"),
        "--f-scale", ".001", "--output", str(demo / "fit.json")]) == 0
    return demo


def validate(path):
    return main(["validate-joint-reference", "--data", str(path / "holdout.csv"), "--manifest", str(path / "holdout.yaml"),
        "--calibration", str(path / "fit.json"), "--urdf", str(path / "nominal.urdf"),
        "--acceptance", str(path / "reference_acceptance.yaml"), "--mapping-output", str(path / "mapping.yaml"),
        "--output", str(path / "validation.json")])


def test_known_answer_holdout_exports_traceable_candidate(reference_case):
    p = reference_case
    assert validate(p) == 0
    report = load(p / "validation.json")
    assert report["status"] == "synthetic_pass"
    assert report["joints"]["j1"]["scale"] == pytest.approx(-.7)
    with pytest.raises(InputError, match="allow-candidate"):
        JointMapping(p / "mapping.yaml", p / "nominal.urdf")
    mapping = JointMapping(p / "mapping.yaml", p / "nominal.urdf", True)
    mapped = mapping.row({"sequence": 0, "joint": "j1", "q": .4, "dq": .2, "effort": 3})
    assert mapped["q"] == pytest.approx(-.25) and mapped["dq"] == pytest.approx(-.14)
    assert mapped["q_source"] == .4 and mapped["effort_source"] == 3 and mapped["effort"] == ""
    assert mapped["joint_mapping_verified"] is False
    with pytest.raises(InputError, match="already mapped"):
        mapping.row(mapped)
    with pytest.raises(InputError, match="extrapolation"):
        mapping.row({"joint": "j1", "q": 2})


@pytest.mark.parametrize("field,value", [("robot_id", "other-unit"), ("feedback_coordinates_id", "other-publisher"),
    ("reference_unit", "m"), ("reference_kind", "controller_target")])
def test_reference_definition_mismatch_blocks_export(reference_case, field, value):
    p = reference_case
    meta = load(p / "holdout.yaml")
    meta[field] = value
    save(p / "holdout.yaml", meta)
    assert validate(p) == 2
    assert not (p / "mapping.yaml").exists()


def test_changed_source_and_changed_coefficients_are_rejected(reference_case):
    p = reference_case
    assert validate(p) == 0
    profile = load(p / "mapping.yaml")
    profile["joints"][0]["offset"] += .1
    save(p / "modified.yaml", profile)
    with pytest.raises(InputError, match="differ from validated"):
        JointMapping(p / "modified.yaml", p / "nominal.urdf", True)
    (p / "train.csv").write_text((p / "train.csv").read_text() + "\n")
    with pytest.raises(InputError, match="changed evidence"):
        JointMapping(p / "mapping.yaml", p / "nominal.urdf", True)


def test_overlapping_ids_and_bad_holdout_do_not_export(reference_case):
    p = reference_case
    csv = p / "holdout.csv"
    write_csv(csv, [{"sample_id": "train-0", "joint": "j1", "raw": -.7, "reference": .52}])
    meta = load(p / "holdout.yaml")
    meta["sha256"] = sha256(csv)
    save(p / "holdout.yaml", meta)
    assert validate(p) == 2 and not (p / "mapping.yaml").exists()


def test_rebinding_profile_cannot_reuse_validation_for_another_model(reference_case):
    p = reference_case
    assert validate(p) == 0
    other = p / "other.urdf"
    other.write_bytes((p / "nominal.urdf").read_bytes() + b"\n")
    profile = load(p / "mapping.yaml")
    profile["urdf_sha256"] = sha256(other)
    save(p / "other_mapping.yaml", profile)
    with pytest.raises(InputError, match="does not bind"):
        JointMapping(p / "other_mapping.yaml", other, True)


def test_failed_accuracy_threshold_does_not_export(reference_case):
    p = reference_case
    write_csv(p / "holdout.csv", [{"sample_id": f"bad-{i}", "joint": "j1", "raw": raw,
        "reference": -.7 * raw + .04} for i, raw in enumerate([-.6, 0, .6])])
    meta = load(p / "holdout.yaml")
    meta["sha256"] = sha256(p / "holdout.csv")
    save(p / "holdout.yaml", meta)
    assert validate(p) == 2
    assert load(p / "validation.json")["status"] == "failed"
    assert not (p / "mapping.yaml").exists()


def test_readonly_wrapper_and_offline_cli_preserve_source(reference_case):
    p = reference_case
    assert validate(p) == 0
    write_csv(p / "states.csv", [{"sequence": 0, "joint": "j1", "q": .4, "dq": "", "effort": ""}])
    original = (p / "states.csv").read_bytes()
    meta = {**load(p / "holdout.yaml"), "dataset_id": "raw-record", "data_kind": "real",
        "coordinates_id": "synthetic-publisher-v1", "joint_units": {"j1": "rad"}, "sha256": sha256(p / "states.csv")}
    save(p / "states.yaml", meta)
    args = ["map-joint-feedback", "--data", str(p / "states.csv"), "--manifest", str(p / "states.yaml"),
        "--mapping", str(p / "mapping.yaml"), "--urdf", str(p / "nominal.urdf"), "--allow-candidate",
        "--output-data", str(p / "mapped.csv"), "--output-manifest", str(p / "mapped.yaml"), "--output", str(p / "mapped.json")]
    assert main(args) == 0
    assert (p / "states.csv").read_bytes() == original
    assert load(p / "mapped.yaml")["data_kind"] == "simulation"
    source = {"class": "adapters.replay_adapter:ReplayAdapter", "options": {"path": str(p / "states.csv")}}
    adapter = MappedJointStateAdapter({"source": source, "mapping": str(p / "mapping.yaml"),
        "urdf": str(p / "nominal.urdf"), "allow_candidate": True})
    assert adapter.export_metadata(meta)["transport"] == "replay"
    assert list(adapter.samples())[0]["q"] == pytest.approx(-.25)
    with pytest.raises(InputError, match="no motion"):
        adapter.command({})
    adapter.close()
    with pytest.raises(InputError, match="already mapped"):
        JointMapping(p / "mapping.yaml", p / "nominal.urdf", True).metadata(load(p / "mapped.yaml"))


def test_unimplemented_ros_fields_stay_empty_with_published_zero_preserved():
    msg = NS(name=["finger"], position=[.1], velocity=[0.], effort=[0.],
        header=NS(stamp=NS(sec=1, nanosec=1), frame_id=""))
    row = message_rows(msg, 0, .1, ["finger"], ["finger"], ["finger"])[0]
    assert row["dq"] == row["effort"] == ""
    assert row["dq_published"] == row["effort_published"] == 0


def test_declared_real_reference_branch_with_mock_metadata(reference_case):
    # This is a temporary unit-test mock of declared metadata, not real metrology.
    p = reference_case
    for name in ("train", "holdout"):
        meta = load(p / (name + ".yaml"))
        meta.update(data_kind="real", reference_kind="independent_measurement", source="UNIT TEST MOCK ONLY")
        save(p / (name + ".yaml"), meta)
    assert main(["calibrate-joint-reference", "--data", str(p / "train.csv"), "--manifest", str(p / "train.yaml"),
        "--f-scale", ".001", "--output", str(p / "fit.json")]) == 0
    assert validate(p) == 0
    mapping = JointMapping(p / "mapping.yaml", p / "nominal.urdf")
    assert mapping.qualified
    assert mapping.row({"joint": "j1", "q": 0})["joint_mapping_verified"]

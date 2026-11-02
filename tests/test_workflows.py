from pathlib import Path
from types import SimpleNamespace as Args
from zipfile import ZipFile
import json
import pytest

from core.cli import main, COMMANDS
from core.io import InputError, load, save, csv_rows
from core.provenance import basis
from core.schemas import parameters, validate_parameters, inertia_matrix
from programs.common import prepare_inputs, run_test, generate_report, package_delivery
from programs.stage_01_definition import build_urdf


def test_all_commands_have_explicit_method_ownership():
    for command in COMMANDS:
        item = basis(command)
        assert item["kind"]
        assert "implementation" in item
    assert basis("run-nvidia-sysid")["kind"] == "invokes_installed_official_NVIDIA_tool"
    assert "project" in basis("identify-contact")["kind"]


def test_packaged_reference_registry_matches_documentation():
    root = Path(__file__).resolve().parents[1]
    assert (root / "core/sources.yaml").read_bytes() == (root / "docs/sources.yaml").read_bytes()


def test_workbook_and_yaml_round_trip(demo):
    for filename in ("parameters.xlsx", "parameters.yaml"):
        assert validate_parameters(parameters(demo / filename), require_inertia=True)["ready"]


def test_separate_inertial_confirmation_required(demo):
    data = parameters(demo / "parameters.yaml")
    data["links"][1]["inertial_status"] = "unconfirmed"
    assert validate_parameters(data)["ready"]
    assert not validate_parameters(data, True)["ready"]


def test_unphysical_inertia_rejected():
    with pytest.raises(InputError):
        inertia_matrix([1, 1, 3, 0, 0, 0])


def test_archive_traversal_rejected(tmp_path):
    archive = tmp_path / "bad.zip"
    with ZipFile(archive, "w") as z:
        z.writestr("../escape.txt", "no")
    with pytest.raises(InputError, match="unsafe"):
        prepare_inputs.run(Args(archive=archive, destination=tmp_path / "extract", workbook=tmp_path / "parameters.xlsx"))
    assert not (tmp_path / "escape.txt").exists()


def test_baseline_preparation_is_unconfirmed_and_preserves_edits(demo):
    archive = demo / "baseline.zip"
    with ZipFile(archive, "w") as z:
        z.write(demo / "nominal.urdf", "robot/robot.urdf")
    args = Args(archive=archive, destination=demo / "extract", workbook=demo / "prepared.xlsx")
    prepare_inputs.run(args)
    data = parameters(args.workbook)
    assert not validate_parameters(data)["ready"]
    assert all(r["inertial_status"] == "unconfirmed" for r in data["links"])
    with pytest.raises(InputError, match="already exists"):
        prepare_inputs.run(args)


def test_build_model_and_draft_gate(demo):
    args = Args(parameters=demo / "parameters.xlsx", baseline=demo / "nominal.urdf", destination=demo / "model",
                calibration=None, mechanisms=demo / "mechanisms.yaml", require_inertia=True, draft=False)
    output = build_urdf.run(args)
    assert output["status"] == "built"
    assert load(demo / "model/model_manifest.yaml")["inertial_parameters_confirmed"]
    args.mechanisms = None
    with pytest.raises(InputError, match="mechanisms"):
        build_urdf.run(args)
    args.draft = True
    assert build_urdf.run(args)["status"] == "draft"


def test_cli_errors_produce_machine_readable_report(tmp_path):
    out = tmp_path / "blocked.json"
    code = main(["validate-structure", "--parameters", str(tmp_path / "missing.xlsx"), "--output", str(out)])
    assert code == 2
    report = load(out)
    assert report["status"] == "blocked"
    assert report["methodology"]["references"]


def test_replay_cannot_be_used_as_real_robot(demo):
    args = Args(target="real", interface=demo / "replay_interface.yaml", trajectory=demo / "replay.csv",
                urdf=demo / "nominal.urdf", execute=True, output_data=demo / "should_not_exist.csv")
    with pytest.raises(InputError):
        run_test.run(args)
    assert not args.output_data.exists()


def test_report_escapes_untrusted_text(tmp_path):
    save(tmp_path / "record.json", {"command": "example", "status": "computed", "source": "<script>alert(1)</script>"})
    generate_report.run(Args(results=tmp_path, destination=tmp_path / "report"))
    text = (tmp_path / "report/report.html").read_text()
    assert "<script>" not in text
    assert "&lt;script&gt;" in text


def test_synthetic_results_cannot_be_final_delivery(demo):
    config = {"stage": "kinematics", "reports": [], "artifacts": []}
    save(demo / "delivery.yaml", config)
    with pytest.raises(InputError, match="Final delivery"):
        package_delivery.run(Args(config=demo / "delivery.yaml", final=True, destination=demo / "final.zip"))
    assert not (demo / "final.zip").exists()


def test_isaac_commands_report_missing_capability(tmp_path):
    code = main(["run-nvidia-sysid", "--config", "configs/nvidia_sysid.yaml", "--output", str(tmp_path / "sysid.json")])
    assert code == 2
    assert load(tmp_path / "sysid.json")["status"] == "blocked"


def test_malformed_yaml_still_produces_a_blocked_report(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("joints: [unclosed")
    out = tmp_path / "blocked.json"
    assert main(["validate-structure", "--parameters", str(bad), "--output", str(out)]) == 2
    assert "Invalid JSON/YAML" in load(out)["error"]


@pytest.mark.parametrize("content", ["joint,joint\nj1,j2\n", "joint,t\nj1,0,extra\n", "joint,t\nj1\n"])
def test_ambiguous_csv_rejected(tmp_path, content):
    path = tmp_path / "bad.csv"
    path.write_text(content)
    with pytest.raises(InputError, match="CSV"):
        csv_rows(path)


def test_official_wrapper_preserves_plan_and_timeout_evidence(tmp_path, monkeypatch):
    import sys
    import subprocess
    from programs.stage_03_isaac_dynamics import run_nvidia_sysid
    sdk = tmp_path / "sdk"
    tool = sdk / "exts/isaacsim.robot_setup.sysid/tools/headless_sysid_solve.py"
    tool.parent.mkdir(parents=True)
    tool.write_text("# Test double; no NVIDIA code or optimizer executed\n")
    (sdk / "VERSION").write_text("6.1.0-rc.26+test")
    stage, telemetry = tmp_path / "robot.usda", tmp_path / "data.csv"
    stage.write_text("# fixture")
    telemetry.write_text("t,q\n0,0\n")
    cfg = tmp_path / "sysid.yaml"
    save(cfg, {"python_launcher": sys.executable, "official_tool": str(tool), "stage": str(stage),
        "telemetry": str(telemetry), "recipe": "manipulator", "robot_prim": "/World/Robot"})
    args = Args(config=cfg, execute=False, log=tmp_path / "run.log")
    plan = run_nvidia_sysid.run(args)
    assert plan["status"] == "planned" and not plan["executed"]
    assert plan["official_tool_sha256"]

    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 1)
    monkeypatch.setattr(run_nvidia_sysid.subprocess, "run", timeout)
    args.execute = True
    report = run_nvidia_sysid.run(args)
    assert report["status"] == "failed" and report["timed_out"]

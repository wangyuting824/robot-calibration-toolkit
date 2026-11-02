"""Offline regression using small fixtures; never sends to a physical CAN bus."""
from pathlib import Path
from types import SimpleNamespace as Args
import struct
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from core.io import InputError, load
from core.model import Model
from examples.prepare_openarm import prepare
from programs.common.check_openarm import query_ready, state_request
from programs.stage_03_isaac_dynamics.validate_runtime_kinematics import run as runtime_check


@pytest.mark.parametrize("motor_id", [1, 8])
def test_only_documented_non_actuating_query_packet(motor_id):
    frame = state_request(motor_id)
    can_id, length, flags, _, _, data = struct.unpack("=IBBBB64s", frame)
    assert len(frame) == 72 and can_id == 0x7FF and length == 8 and flags == 1
    assert data[:8] == bytes((motor_id, 0, 0xCC, 0, 0, 0, 0, 0))
    assert not any(marker in data[:8] for marker in (0xFC, 0xFD, 0xFE))


@pytest.mark.parametrize("motor_id", [True, 0, 9, 1.5])
def test_query_rejects_unknown_motor_ids(motor_id):
    with pytest.raises(InputError):
        state_request(motor_id)


def test_query_blocks_unhealthy_interfaces():
    state = {"flags": ["UP"], "linkinfo": {"info_data": {"state": "ERROR-ACTIVE", "ctrlmode": ["FD"],
        "bittiming": {"bitrate": 1_000_000}, "data_bittiming": {"bitrate": 5_000_000}}}}
    assert query_ready(state)
    state["linkinfo"]["info_data"]["state"] = "ERROR-WARNING"
    assert not query_ready(state)
    assert query_ready(state, allow_warning=True)
    state["linkinfo"]["info_data"]["state"] = "ERROR-PASSIVE"
    assert not query_ready(state, allow_warning=True)


def test_prepare_preserves_fixed_frames_without_inventing_mass(demo):
    tree = ET.parse(demo / "nominal.urdf")
    root = tree.getroot()
    ET.SubElement(root, "link", name="world")
    joint = ET.SubElement(root, "joint", name="world_mount", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="base")
    ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(root, "link", name="tool_frame")
    joint = ET.SubElement(root, "joint", name="tool_mount", type="fixed")
    ET.SubElement(joint, "parent", link="arm2")
    ET.SubElement(joint, "child", link="tool_frame")
    ET.SubElement(joint, "origin", xyz="0.7 0 0", rpy="0 0 0")
    source = demo / "framed.urdf"
    tree.write(source)
    original = source.read_bytes()
    destination = demo / "prepared"
    info = prepare(source, demo, destination)
    assert source.read_bytes() == original
    assert {r["link"] for r in info["excluded_empty_fixed_frames"]} == {"world", "tool_frame"}
    assert info["is_real_robot_acceptance"] is False
    assert "tool_frame" not in Model(destination / "nominal_physical.urdf").links
    original_model, physical_model = Model(source), Model(destination / "nominal_physical.urdf")
    q = {n: 0.2 for n in physical_model.independent}
    for name, pose in physical_model.transforms(q).items():
        np.testing.assert_allclose(pose, original_model.transforms(q)[name], atol=1e-12)
    assert load(destination / "parameters.yaml")["robot_info"]["validation_scope"] == "nominal_model_software_test_only"


def test_runtime_checks_configuration_before_loading_sdk(demo):
    with pytest.raises(InputError, match="joint_positions"):
        runtime_check(Args(urdf=demo / "nominal.urdf", config=demo / "calibration.yaml"))


def test_unhealthy_can_diagnostic_never_sends(monkeypatch, tmp_path):
    from programs.common import check_openarm
    state = {"flags": ["UP"], "linkinfo": {"info_data": {"state": "ERROR-WARNING"}}}
    monkeypatch.setattr(check_openarm, "inspect_interface", lambda name: state)
    monkeypatch.setattr(check_openarm, "active_driver_present", lambda: False)
    class ReceiveOnly:
        def setsockopt(self, *args): pass
        def bind(self, *args): pass
        def setblocking(self, *args): pass
        def close(self): pass
        def send(self, *args): pytest.fail("Unhealthy interface must never send a frame")
    monkeypatch.setattr(check_openarm.socket, "socket", lambda *args: ReceiveOnly())
    monkeypatch.setattr(check_openarm.select, "select", lambda *args: ([], [], []))
    report = check_openarm.run(Args(interfaces=["can0"], listen_seconds=.001, query_state=True, raw_frames=tmp_path / "raw.json"))
    assert report["status"] == "blocked" and report["state_query_frames_sent"] == 0
    assert report["hardware_control_commands_sent"] is False


def test_backend_exception_preserves_machine_readable_failure(monkeypatch, tmp_path):
    import core.cli as cli
    from types import SimpleNamespace
    def fail(args):
        raise Exception("SDK tensor backend unavailable")
    monkeypatch.setattr(cli, "import_module", lambda name: SimpleNamespace(run=fail))
    out = tmp_path / "failure.json"
    assert cli.main(["preflight", "--output", str(out)]) == 2
    report = load(out)
    assert report["error"] == "SDK tensor backend unavailable"
    assert report["status"] == "blocked"


def test_readonly_capture_uses_only_codec_and_keeps_motor_semantics(monkeypatch):
    import sys
    from types import SimpleNamespace
    import adapters.openarm_readonly as module
    raw = bytes((1, 128, 0, 128, 0, 0, 25, 26))
    class Bus:
        def __init__(self): self.pending = []; self.sent = []
        def setsockopt(self, *args): pass
        def bind(self, *args): pass
        def setblocking(self, *args): pass
        def close(self): pass
        def send(self, frame):
            assert frame[8:16] == bytes((1, 0, 0xCC, 0, 0, 0, 0, 0))
            self.sent.append(frame)
            self.pending.append(struct.pack("=IBBBB64s", 0x11, 8, 1, 0, 0, raw))
            return 72
        def recv(self, size): return self.pending.pop(0)
    bus = Bus()
    sdk = SimpleNamespace(MotorType=SimpleNamespace(DM4310=1), Motor=lambda *args: object(),
        CanPacketEncoder=SimpleNamespace(create_refresh_command=lambda motor: SimpleNamespace(send_can_id=0x7FF, data=[1, 0, 0xCC, 0, 0, 0, 0, 0])),
        CanPacketDecoder=SimpleNamespace(parse_motor_state_data=lambda *args: SimpleNamespace(valid=True, position=.25, velocity=0, torque=0, t_mos=25, t_rotor=26)))
    monkeypatch.setitem(sys.modules, "openarm_can", sdk)
    monkeypatch.setattr(module, "active_driver_present", lambda: False)
    monkeypatch.setattr(module, "inspect_interface", lambda name: {})
    monkeypatch.setattr(module, "query_ready", lambda state: True)
    monkeypatch.setattr(module.socket, "socket", lambda *args: bus)
    monkeypatch.setattr(module.select, "select", lambda *args: ([bus] if bus.pending else [], [], []))
    adapter = module.OpenArmReadOnlyAdapter({"cycles": 1, "mapping_reference": "test profile", "motors": [
        {"interface": "can0", "motor_id": 1, "motor_type": "DM4310", "joint_label": "nominal_j1"}]})
    rows = list(adapter.samples())
    adapter.close()
    assert len(bus.sent) == 1 and rows[0]["motor_position_rad"] == .25
    assert rows[0]["joint_mapping_verified"] is False
    assert not any(k.startswith("q:") for k in rows[0])
    with pytest.raises(InputError, match="no motion interface"):
        adapter.command({"nominal_j1": 0})


def test_nominal_range_analysis_does_not_infer_calibrated_joint_or_gripper(demo):
    from core.io import write_csv
    from examples.analyze_openarm_feedback import analyze
    source = demo / "motors.csv"
    write_csv(source, [{"interface": "can0", "motor_id": 1, "joint_label_nominal": "j1", "motor_position_rad": 3,
                        "raw_payload_hex": "test"}])
    report = analyze(source, demo / "nominal.urdf")
    assert report["status"] == "needs_mapping_review" and report["mapping_review_required"] == ["j1"]
    assert report["identity_mapping_verified"] is False
    assert report["automatic_sign_offset_or_limit_changes"] is False

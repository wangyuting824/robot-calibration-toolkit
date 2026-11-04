from copy import deepcopy
import io
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import pytest
import yaml

from core.joint_motion import (MotionError, check_start, check_tracking, model_joints,
    motion_lock, owned_goal_uuid, plan_motion, quintic, validate_profile)
from core.motion_cli import create_demo, main
from core.motion_ros2 import RosMotion


@pytest.fixture
def motion_profile(tmp_path):
    directory = tmp_path/"demo"
    create_demo(directory)
    config = yaml.safe_load((directory/"mujoco.yaml").read_text())
    return directory, config


def make_plan(profile, **changes):
    directory, config = profile
    kwargs = dict(joint="joint1", value=5, unit="deg", relative=True, duration=2)
    kwargs.update(changes)
    return plan_motion(config, model_joints(directory/"robot.urdf"),
                       config["simulation"]["initial_positions"], **kwargs)


def test_relative_move_uses_actual_nonzero_start_and_holds_full_group(motion_profile):
    plan = make_plan(motion_profile)
    assert plan["start"]["joint1"] == .1
    assert plan["target"]["joint1"] == pytest.approx(.1+math.radians(5))
    assert plan["target"]["joint2"] == plan["start"]["joint2"] == -.1
    assert plan["target"]["slider"] == plan["start"]["slider"] == .01
    assert plan["points"][0]["positions"] == [.1, -.1, .01]
    assert plan["points"][0]["velocities"] == plan["points"][-1]["velocities"] == [0, 0, 0]
    assert all(len(p["positions"]) == 3 for p in plan["points"])


def test_absolute_angle_and_linear_mm_have_distinct_meanings(motion_profile):
    assert make_plan(motion_profile, relative=False)["target"]["joint1"] == pytest.approx(math.radians(5))
    assert make_plan(motion_profile, joint="slider", unit="mm")["target"]["slider"] == pytest.approx(.015)
    with pytest.raises(MotionError, match="degrees"):
        make_plan(motion_profile, joint="slider")


@pytest.mark.parametrize("changes,match", [({"value": 1000}, "outside"), ({"duration": .01}, "velocity/acceleration"),
    ({"joint": "unknown"}, "controller"), ({"value": float("nan")}, "finite"), ({"duration": -1}, "positive")])
def test_invalid_motions_are_rejected_before_any_backend(motion_profile, changes, match):
    with pytest.raises(MotionError, match=match):
        make_plan(motion_profile, **changes)


def test_start_and_nonselected_axis_drift_require_new_plan(motion_profile):
    _, config = motion_profile
    plan = make_plan(motion_profile)
    changed = dict(plan["start"], joint2=-.09)
    with pytest.raises(MotionError, match="start changed"):
        check_start(config, plan, changed)
    with pytest.raises(MotionError, match="nonselected"):
        check_tracking(config, plan, changed, 0)


def test_quintic_continuous_velocity_and_acceleration_limits(motion_profile):
    plan = make_plan(motion_profile)
    values = [quintic(.1, plan["target"]["joint1"], 2, i*.002) for i in range(1001)]
    assert max(abs(v[1]) for v in values) <= plan["peak_velocity"]+1e-12
    assert max(abs(v[2]) for v in values) <= plan["peak_acceleration"]+1e-12
    assert values[-1][1:] == (0, 0)


@pytest.mark.parametrize("mutation,match", [(lambda c: c["limits"]["joint1"].update(upper=2), "expands"),
    (lambda c: c["limits"]["joint1"].update(max_velocity=100), "velocity"),
    (lambda c: c["controller_joints"].append("joint1"), "unique")])
def test_profiles_cannot_expand_urdf_or_duplicate_joints(motion_profile, mutation, match):
    directory, config = motion_profile
    mutation(config)
    with pytest.raises(MotionError, match=match):
        validate_profile(config, directory/"robot.urdf")


def test_mimic_and_fixed_joints_cannot_be_independent_targets(motion_profile):
    directory, config = motion_profile
    tree = ET.parse(directory/"robot.urdf")
    joint = tree.getroot().find("joint[@name='joint2']")
    ET.SubElement(joint, "mimic", joint="joint1", multiplier="1", offset="0")
    tree.write(directory/"mimic.urdf")
    with pytest.raises(MotionError, match="independent"):
        validate_profile(config, directory/"mimic.urdf")
    joint.set("type", "fixed")
    tree.write(directory/"fixed.urdf")
    with pytest.raises(MotionError, match="independent"):
        validate_profile(config, directory/"fixed.urdf")


def test_unconfirmed_real_execution_never_opens_ros(motion_profile, tmp_path, monkeypatch):
    directory, config = motion_profile
    config.update(backend="ros2", target="real", joint_state_topic="/joint_states", action_name="/arm/follow_joint_trajectory")
    path = directory/"real.yaml"; path.write_text(yaml.safe_dump(config))
    monkeypatch.setattr("core.motion_cli.run_ros", lambda *a: pytest.fail("Must not open ROS for an unconfirmed real execution"))
    output = tmp_path/"blocked"
    assert main(["move", "--config", str(path), "--joint", "joint1", "--delta-deg", "5", "--duration", "2", "--execute", "--run-dir", str(output)]) == 2
    result = json.loads((output/"result.json").read_text())
    assert "confirmed.model_and_joint_mapping" in result["error"]


def test_test_target_requires_isolated_domain_and_namespace(motion_profile, monkeypatch):
    directory, config = motion_profile
    config.update(backend="ros2", target="protocol_test", test_domain_id=221, action_name="/real_arm/follow_joint_trajectory")
    monkeypatch.setenv("ROS_DOMAIN_ID", "221"); monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    with pytest.raises(MotionError, match="isolated"):
        validate_profile(config, directory/"robot.urdf", True)


def test_old_run_result_is_preserved(motion_profile, tmp_path):
    directory, _ = motion_profile
    output = tmp_path/"old";output.mkdir();(output/"result.json").write_text('{"original":true}')
    assert main(["move", "--config", str(directory/"mujoco.yaml"), "--joint", "joint1", "--delta-deg", "5", "--duration", "2", "--run-dir", str(output)]) == 2
    assert (output/"result.json").read_text() == '{"original":true}'


def test_classified_delivery_runs_motion_launcher_after_extraction(demo, tmp_path):
    import os
    import subprocess
    import sys
    from zipfile import ZipFile
    from core.fieldwork import init_project, export
    from core.io import save
    root = Path(__file__).resolve().parents[1]
    archive = tmp_path/"model.zip"
    with ZipFile(archive, "w") as z:
        z.write(demo/"nominal.urdf", "model/robot.urdf")
    project = tmp_path/"field"
    init_project(NS(project=project, archive=archive, docx=None, step=None), root)
    (project/"models/READINESS.txt").write_text("SYNTHETIC stage delivery only")
    save(project/"results/R01/readiness.json", {"status": "pending", "data_kind": "synthetic"})
    destination = tmp_path/"delivery.zip"
    export(project, "R01", root, destination, False)
    moved = tmp_path/"解压后 更换目录"
    with ZipFile(destination) as z:
        for member in z.namelist():
            if member.startswith("D04/"):
                z.extract(member, moved)
    completed = subprocess.run(["bash", str(moved/"D04/控制关节.sh"), "demo", "--destination", str(tmp_path/"new demo")],
        env=dict(os.environ, ROBOTCAL_PYTHON=sys.executable), cwd=tmp_path, capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    assert (tmp_path/"new demo/mujoco.yaml").is_file()


def test_same_controller_lock_blocks_another_execution(tmp_path):
    with motion_lock(tmp_path/"controller.lock"):
        with pytest.raises(MotionError, match="Another"):
            with motion_lock(tmp_path/"controller.lock"):
                pytest.fail("Must not acquire another controller lock")


@pytest.mark.parametrize("value", ["0"*32, "", "zz", "01"*15, None])
def test_cancel_all_and_invalid_goal_ids_are_rejected(value):
    with pytest.raises(MotionError, match="UUID"):
        owned_goal_uuid(value)


def fake_subscription(config, publishers=1):
    session = RosMotion.__new__(RosMotion)
    session.cfg = dict(config, state_publisher_node="/driver")
    session.state = None; session.state_error = None; session.sequence = 0
    session.telemetry = io.StringIO()
    endpoint = NS(node_name="driver", node_namespace="/", endpoint_gid=[1, 2])
    now = time.time()
    session.node = NS(get_publishers_info_by_topic=lambda topic: [endpoint]*publishers,
                      get_clock=lambda: NS(now=lambda: NS(nanoseconds=round(now*1e9))))
    stamp = NS(sec=int(now), nanosec=round((now-int(now))*1e9))
    message = NS(name=["joint1", "joint2", "slider"], position=[.1, -.1, .01], velocity=[], header=NS(stamp=stamp))
    return session, message


def test_timestamp_and_publisher_checks_work_without_ros(motion_profile):
    _, config = motion_profile
    config["joint_state_topic"] = "/joint_states"
    session, message = fake_subscription(config)
    session.on_state(message, {})  # Installed Jazzy metadata can omit the publisher GID.
    assert session.state["publisher"] == "/driver"
    session.on_state(message, {})
    assert "not increasing" in str(session.state_error)
    session, message = fake_subscription(config, publishers=2)
    session.on_state(message, {})
    assert "exactly one publisher" in str(session.state_error)
    session, message = fake_subscription(config)
    message.header.stamp.sec -= 20
    session.on_state(message, {})
    assert "stale" in str(session.state_error)


@pytest.mark.parametrize("joint,flag,value", [("joint1", "--delta-deg", "5"), ("joint1", "--target-deg", "-5"), ("slider", "--delta-mm", "5")])
def test_mujoco_motion_runs_dynamics_and_keeps_other_axes(motion_profile, tmp_path, joint, flag, value):
    pytest.importorskip("mujoco")
    directory, config = motion_profile
    run = tmp_path/"run"
    assert main(["move", "--config", str(directory/"mujoco.yaml"), "--joint", joint, flag, value,
                 "--duration", "2", "--execute", "--run-dir", str(run)]) == 0
    result = json.loads((run/"result.json").read_text())
    assert result["status"] == "completed" and result["hardware_control_commands_sent"] is False
    for name in config["controller_joints"]:
        assert abs(result["errors"][name]) < config["limits"][name]["goal_tolerance"]
    assert len((run/"feedback.csv").read_text().splitlines()) > 100

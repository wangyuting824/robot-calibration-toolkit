from types import SimpleNamespace as NS
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from adapters.ros2_joint_states import Ros2JointStateAdapter, message_rows
from core.io import InputError


def message():
    return NS(name=["a", "b"], position=[.1, -.2], velocity=[], effort=[],
        header=NS(stamp=NS(sec=123, nanosec=456), frame_id="base"))


def test_ros_state_preserves_timestamp_and_absent_measurements():
    rows = message_rows(message(), 2, .3, ["a", "b"])
    assert rows[1]["q"] == -.2 and rows[0]["stamp_nanosec"] == 456
    assert rows[0]["dq"] == "" and rows[0]["effort"] == ""
    assert rows[0]["joint_mapping_verified"] is False
    assert Ros2JointStateAdapter.capabilities["command"] is False
    with pytest.raises(InputError, match="no command"):
        Ros2JointStateAdapter.command(None, {})


@pytest.mark.parametrize("field,value", [("name", ["a", "a"]), ("position", [.1]),
    ("velocity", [1]), ("position", [float("nan"), 0])])
def test_ros_state_rejects_malformed_messages(field, value):
    msg = message()
    setattr(msg, field, value)
    with pytest.raises(InputError):
        message_rows(msg, 0, .1, ["a", "b"])


def test_ros_state_rejects_unrelated_publisher():
    with pytest.raises(InputError, match="missing"):
        message_rows(message(), 0, .1, ["robot_joint"])


def test_ros_adapter_owns_context_executor_and_never_publishes(monkeypatch):
    state = {}
    class Context:
        active = True
        def ok(self): return self.active
        def shutdown(self): self.active = False
    class Node:
        def create_subscription(self, kind, topic, callback, qos): state["callback"] = callback
        def destroy_node(self): state["node_closed"] = True
    class Executor:
        def __init__(self, *, context): state["context"] = context
        def add_node(self, node): state["node"] = node
        def spin_once(self, *, timeout_sec): state["callback"](message())
        def shutdown(self): state["executor_closed"] = True
    monkeypatch.setitem(sys.modules, "rclpy", NS(init=lambda **kw: None, create_node=lambda *a, **kw: Node()))
    monkeypatch.setitem(sys.modules, "rclpy.context", NS(Context=Context))
    monkeypatch.setitem(sys.modules, "rclpy.executors", NS(SingleThreadedExecutor=Executor))
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, "sensor_msgs", NS())
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", NS(JointState=object))
    adapter = Ros2JointStateAdapter({"messages": 1, "expected_joints": ["a", "b"]})
    assert len(list(adapter.samples())) == 2
    adapter.close()
    assert state["node_closed"] and state["executor_closed"] and not state["context"].ok()


def test_mujoco_compile_fk_and_physics(demo):
    mujoco = pytest.importorskip("mujoco")
    from core.model import Model
    from core.mujoco_model import prepare, validate_static, compare_fk, default_poses, physics_smoke
    source = demo / "nominal.urdf"
    unchanged = source.read_bytes()
    nominal = Model(source)
    mjcf = prepare(source, demo / "mujoco")
    compiled = mujoco.MjModel.from_xml_path(str(mjcf))
    assert source.read_bytes() == unchanged
    assert validate_static(nominal, compiled)["independent_joints"] == 2
    assert compare_fk(nominal, compiled, default_poses(nominal), demo / "errors.csv")["status"] == "simulation_pass"
    assert physics_smoke(nominal, compiled)["status"] == "simulation_pass"
    compiled.body("arm1").mass[0] *= 2
    with pytest.raises(InputError, match="inertia mismatch"):
        validate_static(nominal, compiled)


def test_mujoco_mimic_ratio_offset_and_constraint(demo):
    mujoco = pytest.importorskip("mujoco")
    from core.model import Model
    from core.mujoco_model import prepare, validate_static, compare_fk
    tree = ET.parse(demo / "nominal.urdf")
    ET.SubElement(tree.getroot().find("joint[@name='j2']"), "mimic", joint="j1", multiplier="-.5", offset=".1")
    path = demo / "mimic.urdf"
    tree.write(path)
    nominal = Model(path)
    model = mujoco.MjModel.from_xml_path(str(prepare(path, demo / "mujoco")))
    assert validate_static(nominal, model)["mimic_equalities"] == 1
    assert compare_fk(nominal, model, [{"j1": .4}], demo / "errors.csv")["status"] == "simulation_pass"
    model.eq_data[0, 1] = 1
    with pytest.raises(InputError, match="mimic equality"):
        validate_static(nominal, model)


def test_feedback_range_problem_is_not_clipped_or_hidden(demo):
    from core.io import write_csv
    from core.model import Model
    from examples.run_mujoco_validation import recorded_poses
    path = demo / "feedback.csv"
    write_csv(path, [{"sequence": 0, "joint": "j1", "q": 0}, {"sequence": 0, "joint": "j2", "q": 4}])
    nominal = Model(demo / "nominal.urdf")
    poses, ranges, rejected = recorded_poses(nominal, path, [])
    assert not poses and rejected and ranges[1]["out_of_range_samples"] == 1
    poses, ranges, rejected = recorded_poses(nominal, path, ["j2"])
    assert poses[0]["j2"] == 0 and ranges[1]["max_q"] == 4


def test_mujoco_rejects_untraceable_feedback_before_loading_engine():
    from examples.run_mujoco_validation import run
    with pytest.raises(InputError, match="joint-manifest"):
        run(NS(joint_states="untrusted.csv", joint_manifest=None))

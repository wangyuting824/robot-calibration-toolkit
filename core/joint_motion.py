"""Shared motion planning and checks. Standard library only, including in ROS Python.

The quintic interpolator is project code. ROS transport uses the documented
FollowJointTrajectory action; this module never opens a hardware connection.
"""
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

SOURCES = {
    "trajectory_action": "https://github.com/ros-controls/control_msgs/blob/jazzy/control_msgs/action/FollowJointTrajectory.action",
    "controller": "https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html",
    "cancel": "https://github.com/ros2/rcl_interfaces/blob/jazzy/action_msgs/srv/CancelGoal.srv",
    "state": "https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg",
    "mujoco": "https://mujoco.readthedocs.io/en/3.17.0/APIreference/APItypes.html",
}


class MotionError(ValueError):
    pass


def number(value, name="value", positive=False):
    if isinstance(value, bool):
        raise MotionError(f"{name}: boolean is not a number")
    try:
        value = float(value)
    except (ValueError, TypeError) as exc:
        raise MotionError(f"{name}: a finite number is required") from exc
    if not math.isfinite(value) or (positive and value <= 0):
        raise MotionError(f"{name}: a finite {'positive ' if positive else ''}number is required")
    return value


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1048576), b""):
            h.update(block)
    return h.hexdigest()


def owned_goal_uuid(value):
    try:
        token = bytes.fromhex(value)
    except (ValueError, TypeError) as exc:
        raise MotionError("Invalid owned goal UUID") from exc
    if len(token) != 16 or not any(token):
        raise MotionError("A nonzero owned goal UUID is required; cancel-all is never allowed")
    return token


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + f".{os.getpid()}.tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def model_joints(path):
    robot = ET.parse(path).getroot()
    if robot.tag != "robot":
        raise MotionError("Select a URDF robot model")
    output = {}
    for joint in robot.findall("joint"):
        name, kind = joint.get("name"), joint.get("type")
        if not name or name in output:
            raise MotionError("Duplicate/missing URDF joint name")
        item = {"type": kind, "dependent": joint.find("mimic") is not None}
        if kind in ("revolute", "prismatic", "continuous"):
            lim = joint.find("limit")
            if lim is None:
                raise MotionError(f"{name}: URDF limit/velocity/effort required")
            item.update(velocity=number(lim.get("velocity"), name+" velocity", True),
                        effort=number(lim.get("effort"), name+" effort", True))
            if kind != "continuous":
                item.update(lower=number(lim.get("lower")), upper=number(lim.get("upper")))
                if item["lower"] >= item["upper"]:
                    raise MotionError(f"{name}: invalid URDF range")
        output[name] = item
    return output


def validate_profile(config, urdf, execute=False):
    if config.get("backend") not in ("ros2", "mujoco"):
        raise MotionError("backend must be ros2 or mujoco")
    target = config.get("target")
    if target not in ("real", "simulation", "protocol_test"):
        raise MotionError("target must be real, simulation or protocol_test")
    if config["backend"] == "mujoco" and target != "simulation":
        raise MotionError("MuJoCo cannot be a real robot target")
    if config["backend"] == "ros2" and target == "simulation":
        raise MotionError("ROS protocol fixtures use the isolated protocol_test target")
    if target == "protocol_test":
        domain = config.get("test_domain_id")
        if (not isinstance(domain, int) or isinstance(domain, bool) or not 200 <= domain <= 232
                or os.environ.get("ROS_DOMAIN_ID") != str(domain) or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
                or not str(config.get("action_name", "")).startswith("/robotcal_test/")):
            raise MotionError("protocol_test requires an isolated /robotcal_test/ action, domain 200..232 and ROS_LOCALHOST_ONLY=1")
    names = config.get("controller_joints")
    if not isinstance(names, list) or not names or len(set(names)) != len(names) or any(not isinstance(n, str) or not n for n in names):
        raise MotionError("controller_joints must be the full, ordered, unique controller joint list")
    joints = model_joints(urdf)
    limits = config.get("limits", {})
    for name in names:
        if name not in joints or joints[name]["type"] not in ("revolute", "prismatic", "continuous") or joints[name]["dependent"]:
            raise MotionError(f"{name}: controller targets must be independent movable URDF joints")
        spec = limits.get(name, {})
        lower, upper = number(spec.get("lower"), name+" lower"), number(spec.get("upper"), name+" upper")
        if lower >= upper:
            raise MotionError(f"{name}: ordered operating bounds required, including for multi-turn joints")
        if joints[name]["type"] != "continuous" and (lower < joints[name]["lower"]-1e-12 or upper > joints[name]["upper"]+1e-12):
            raise MotionError(f"{name}: configured range expands the URDF limits")
        if number(spec.get("max_velocity"), name+" max_velocity", True) > joints[name]["velocity"]+1e-12:
            raise MotionError(f"{name}: velocity exceeds the URDF limit")
        for key in ("max_acceleration", "path_tolerance", "goal_tolerance", "hold_tolerance", "start_tolerance", "stopped_velocity_tolerance"):
            number(spec.get(key), name+" "+key, True)
    for key in ("state_timeout_s", "max_state_age_s", "max_timestamp_skew_s", "server_timeout_s", "goal_time_tolerance_s", "cancel_timeout_s", "settle_s", "point_interval_s", "lead_time_s"):
        number(config.get(key), key, True)
    if config["backend"] == "ros2":
        for key in ("joint_state_topic", "action_name"):
            if not isinstance(config.get(key), str) or not config[key].startswith("/"):
                raise MotionError(f"Configure an absolute {key}")
    if execute and target == "real":
        confirmations = config.get("confirmed", {})
        for key in ("model_and_joint_mapping", "operating_limits", "control_and_stop"):
            if confirmations.get(key) is not True:
                raise MotionError(f"Real motion blocked: confirmed.{key} is not true")
        for key in ("robot_id", "coordinates_id", "operating_procedure", "commissioning_reference", "motion_scope_reference", "cancel_behavior_reference", "exclusive_control_reference", "state_publisher_node", "action_server_node"):
            if not isinstance(config.get(key), str) or not config[key].strip():
                raise MotionError(f"Real motion requires {key}")
        if any(not limits[n].get("source") for n in names):
            raise MotionError("Every operating range needs a source")
    return joints


def check_state(config, positions):
    for name in config["controller_joints"]:
        q = number(positions.get(name), name+" actual position")
        lim = config["limits"][name]
        if not number(lim["lower"])-1e-12 <= q <= number(lim["upper"])+1e-12:
            raise MotionError(f"{name}: actual position outside configured operating range")


def quintic(start, end, duration, elapsed):
    u = min(1.0, max(0.0, elapsed/duration))
    delta = end-start
    return (start+delta*(10*u**3-15*u**4+6*u**5),
            delta*(30*u**2-60*u**3+30*u**4)/duration,
            delta*(60*u-180*u**2+120*u**3)/duration**2)


def plan_motion(config, joints, positions, joint, value, unit, relative, duration):
    check_state(config, positions)
    if joint not in config["controller_joints"]:
        raise MotionError("Selected joint is not in this controller group")
    if (joints[joint]["type"] == "prismatic") != (unit == "mm"):
        raise MotionError("Use degrees for rotary joints and mm for prismatic joints")
    duration = number(duration, "duration", True)
    amount = number(value, "target/delta")*(math.pi/180 if unit == "deg" else .001)
    start = {n: number(positions[n]) for n in config["controller_joints"]}
    end = dict(start)
    end[joint] = start[joint]+amount if relative else amount
    check_state(config, end)
    delta = abs(end[joint]-start[joint])
    lim = config["limits"][joint]
    minimum = max(1.875*delta/number(lim["max_velocity"]),
                  math.sqrt((10*math.sqrt(3)/3)*delta/number(lim["max_acceleration"])))
    if duration+1e-12 < minimum:
        raise MotionError(f"Requested duration violates velocity/acceleration limits; use at least {minimum:.6g} s")
    count = max(2, math.ceil(duration/number(config["point_interval_s"])))
    if count > 10000:
        raise MotionError("Trajectory exceeds 10000 intervals; increase point_interval_s")
    points = []
    for i in range(count+1):
        t = duration*i/count
        values = [quintic(start[n], end[n], duration, t) for n in config["controller_joints"]]
        points.append({"t": t, "positions": [v[0] for v in values],
                       "velocities": [v[1] for v in values], "accelerations": [v[2] for v in values]})
    return {"joint": joint, "joint_names": list(config["controller_joints"]), "start": start,
            "target": end, "duration_s": duration, "relative": relative, "input_unit": unit,
            "input_value": value, "points": points, "interpolation": "quintic; endpoint velocity and acceleration zero",
            "peak_velocity": 1.875*delta/duration, "peak_acceleration": (10*math.sqrt(3)/3)*delta/duration**2}


def check_start(config, plan, positions):
    check_state(config, positions)
    for n in config["controller_joints"]:
        if abs(positions[n]-plan["start"][n]) > number(config["limits"][n]["start_tolerance"]):
            raise MotionError(f"{n}: start changed after planning; no automatic repositioning")


def check_tracking(config, plan, positions, elapsed):
    check_state(config, positions)
    for n in config["controller_joints"]:
        desired = quintic(plan["start"][n], plan["target"][n], plan["duration_s"], elapsed)[0]
        key = "path_tolerance" if n == plan["joint"] else "hold_tolerance"
        if abs(positions[n]-desired) > number(config["limits"][n][key]):
            raise MotionError(f"{n}: {'tracking' if n == plan['joint'] else 'nonselected joint hold'} tolerance exceeded")


@contextmanager
def motion_lock(path):
    import fcntl
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise MotionError("Another tool motion owns this controller; use its run receipt to stop") from exc
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def wait_until(predicate, spin, timeout, message):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise MotionError(message)
        spin(min(.02, max(0, deadline-time.monotonic())))

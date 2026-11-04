"""ROS 2 FollowJointTrajectory client. No drive enable, homing or controller switching.

Run in the installed ROS Python, not necessarily the offline toolkit Python.
Only the UUID created by this run may be canceled. Cancellation is verified
separately from its acknowledgement; it is not a hardware emergency stop.
"""
import json
import os
from pathlib import Path
import signal
import time
import uuid

from core.joint_motion import (MotionError, SOURCES, check_start, check_state,
    check_tracking, digest, motion_lock, number, owned_goal_uuid, plan_motion, validate_profile,
    wait_until, write_json)


def full_node(name, namespace):
    return namespace.rstrip("/")+"/"+name


def duration_message(seconds):
    from builtin_interfaces.msg import Duration
    ns = round(seconds*1e9)
    return Duration(sec=ns//1000000000, nanosec=ns%1000000000)


class RosMotion:
    def __init__(self, request, directory):
        import rclpy
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import JointState
        from action_msgs.msg import GoalStatusArray
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        self.rclpy, self.cfg, self.request, self.directory = rclpy, request["config"], request, directory
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("robotcal_motion_"+uuid.uuid4().hex[:10], context=self.context,
                                      start_parameter_services=False, enable_rosout=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.state, self.state_error, self.sequence, self.statuses = None, None, 0, []
        self.client = None
        self.owned_uuid = None
        self.terminal = False
        self.telemetry = (directory/"feedback.jsonl").open("a", encoding="utf-8")
        if self.cfg.get("joint_state_topic"):
            self.node.create_subscription(JointState, self.cfg["joint_state_topic"], self.on_state, qos_profile_sensor_data)
        if self.cfg.get("action_name"):
            self.client = ActionClient(self.node, FollowJointTrajectory, self.cfg["action_name"])
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.node.create_subscription(GoalStatusArray, self.cfg["action_name"]+"/_action/status", self.on_status, qos)

    def spin(self, seconds=.02):
        self.executor.spin_once(timeout_sec=seconds)

    def on_status(self, message):
        self.statuses = [(bytes(s.goal_info.goal_id.uuid).hex(), s.status) for s in message.status_list]

    def on_state(self, message, info):
        try:
            endpoints = self.node.get_publishers_info_by_topic(self.cfg["joint_state_topic"])
            if not endpoints:
                return  # Discovery may lag the first data sample.
            publisher_gid = info.get("publisher_gid") if isinstance(info, dict) else getattr(info, "publisher_gid", None)
            if publisher_gid is None:
                # Jazzy's installed binding exposes timestamps but not a GID.
                # In that case accept only a single, explicitly identified publisher.
                if len(endpoints) != 1:
                    raise MotionError("This ROS binding has no per-message publisher GID; state topic must have exactly one publisher")
                source = full_node(endpoints[0].node_name, endpoints[0].node_namespace)
            else:
                source = next((full_node(p.node_name, p.node_namespace) for p in endpoints
                               if bytes(p.endpoint_gid) == bytes(publisher_gid)), None)
            expected = self.cfg.get("state_publisher_node")
            if expected and source != expected:
                return
            if not expected and len(endpoints) != 1:
                raise MotionError("State topic has multiple publishers; configure state_publisher_node")
            names = list(message.name)
            if not names or len(set(names)) != len(names) or len(message.position) != len(names):
                raise MotionError("Invalid JointState names/position array")
            if len(message.velocity) not in (0, len(names)):
                raise MotionError("Invalid JointState velocity array")
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            now = self.node.get_clock().now().nanoseconds*1e-9
            if stamp <= 0 or not 0 <= message.header.stamp.nanosec < 1000000000:
                raise MotionError("A valid nonzero measurement timestamp is required")
            if self.state and stamp <= self.state["stamp"]:
                raise MotionError("JointState timestamps are not increasing")
            if abs(now-stamp) > number(self.cfg.get("max_timestamp_skew_s", 1), "max_timestamp_skew_s", True):
                raise MotionError("JointState measurement timestamp is stale or uses a different clock")
            positions = {n: number(v, n) for n, v in zip(names, message.position)}
            velocities = {n: number(v, n+" velocity") for n, v in zip(names, message.velocity)}
            self.sequence += 1
            self.state = {"positions": positions, "velocities": velocities, "stamp": stamp,
                          "received": time.monotonic(), "sequence": self.sequence, "publisher": source}
            self.telemetry.write(json.dumps({"measurement_time_s": stamp, "received_monotonic_s": self.state["received"],
                "publisher": source, "positions": positions, "velocities": velocities}, allow_nan=False)+"\n")
            self.telemetry.flush()
        except Exception as exc:
            self.state_error = exc

    def fresh_state(self):
        if self.state_error:
            raise MotionError(str(self.state_error))
        if not self.state or time.monotonic()-self.state["received"] > number(self.cfg["max_state_age_s"]):
            raise MotionError("Actual-position feedback is missing or stale")
        return self.state

    def settle(self, timeout, targets=None):
        deadline, stable_since, previous = time.monotonic()+timeout, None, None
        while time.monotonic() < deadline:
            self.spin()
            if self.state_error:
                raise MotionError(str(self.state_error))
            if not self.state:
                continue
            state = self.fresh_state()
            if previous and state["sequence"] == previous["sequence"]:
                continue
            check_state(self.cfg, state["positions"])
            quiet = previous is not None
            for n in self.cfg["controller_joints"]:
                if n in state["velocities"]:
                    velocity = state["velocities"][n]
                elif previous:
                    velocity = (state["positions"][n]-previous["positions"][n])/(state["stamp"]-previous["stamp"])
                else:
                    quiet = False
                    continue
                spec = self.cfg["limits"][n]
                quiet &= abs(velocity) <= number(spec["stopped_velocity_tolerance"])
                if targets:
                    tolerance = number(spec["goal_tolerance"])
                    if n != self.request.get("motion", {}).get("joint"):
                        tolerance = min(tolerance, number(spec["hold_tolerance"]))
                    quiet &= abs(state["positions"][n]-targets[n]) <= tolerance
            stable_since = (stable_since or time.monotonic()) if quiet else None
            previous = state
            if stable_since and time.monotonic()-stable_since >= number(self.cfg["settle_s"]):
                return dict(state["positions"])
        raise MotionError("Feedback did not confirm a settled position within the configured tolerance/time")

    def server_nodes(self):
        from rclpy.action import get_action_server_names_and_types_by_node
        found = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            entries = get_action_server_names_and_types_by_node(self.node, name, namespace)
            if any(action == self.cfg.get("action_name") and "control_msgs/action/FollowJointTrajectory" in types for action, types in entries):
                found.append(full_node(name, namespace))
        return found

    def require_server(self):
        if self.client is None:
            raise MotionError("Configure action_name for the existing trajectory controller")
        wait_until(lambda: self.client.server_is_ready() and len(self.server_nodes()) > 0,
                   self.spin, number(self.cfg["server_timeout_s"]), "No FollowJointTrajectory server found; no controller is started automatically")
        nodes = self.server_nodes()
        if len(nodes) != 1 or (self.cfg.get("action_server_node") and nodes != [self.cfg["action_server_node"]]):
            raise MotionError(f"Unexpected or ambiguous action server: {nodes}")
        return nodes[0]

    def inspect(self):
        deadline = time.monotonic()+number(self.cfg.get("server_timeout_s", 2), positive=True)
        while time.monotonic() < deadline:
            self.spin()
        from rclpy.action import get_action_names_and_types
        publishers = self.node.get_publishers_info_by_topic(self.cfg.get("joint_state_topic", "/joint_states"))
        return {"status": "inspected", "nodes": [full_node(n, ns) for n, ns in self.node.get_node_names_and_namespaces()],
                "topics": self.node.get_topic_names_and_types(), "actions": get_action_names_and_types(self.node),
                "state_publishers": [full_node(p.node_name, p.node_namespace) for p in publishers],
                "configured_action_servers": self.server_nodes(), "state": self.state,
                "state_error": str(self.state_error) if self.state_error else None,
                "qualification": "Graph discovery and received values; physical feedback origin, joint mapping and control commissioning still require verification"}

    def cancel_owned(self, goal_hex):
        goal_bytes = owned_goal_uuid(goal_hex)
        from action_msgs.srv import CancelGoal
        from unique_identifier_msgs.msg import UUID
        from control_msgs.action import FollowJointTrajectory
        client = self.node.create_client(CancelGoal, self.cfg["action_name"]+"/_action/cancel_goal")
        reply = {"goal_uuid": goal_hex, "cancel_requested": False, "stop_verified": False}
        try:
            timeout = number(self.cfg["cancel_timeout_s"])
            if not client.wait_for_service(timeout_sec=timeout):
                raise MotionError("Cancellation service unavailable")
            request = CancelGoal.Request()
            request.goal_info.goal_id = UUID(uuid=list(goal_bytes))
            future = client.call_async(request)
            reply["cancel_requested"] = True
            wait_until(future.done, self.spin, timeout, "No cancellation acknowledgement")
            result = future.result()
            reply["cancel_return_code"] = result.return_code
            # Acknowledgement alone does not prove a stop. Query only this UUID.
            result_client = self.node.create_client(FollowJointTrajectory.Impl.GetResultService,
                                                    self.cfg["action_name"]+"/_action/get_result")
            if not result_client.wait_for_service(timeout_sec=timeout):
                raise MotionError("Action result service unavailable")
            query = FollowJointTrajectory.Impl.GetResultService.Request()
            query.goal_id = UUID(uuid=list(goal_bytes))
            done = result_client.call_async(query)
            wait_until(done.done, self.spin, timeout, "Cancellation not followed by a terminal action result")
            reply["terminal_status"] = done.result().status
            if done.result().status not in (4, 5, 6):
                raise MotionError("The controller did not confirm a terminal status for the owned goal")
            self.settle(timeout)
            reply["stop_verified"] = True
        except Exception as exc:
            reply["error"] = str(exc)
        return reply

    def move(self, urdf):
        cfg, request = self.cfg, self.request
        joints = validate_profile(cfg, urdf, request["execute"])
        self.require_server()
        start = self.settle(number(cfg["state_timeout_s"]))
        motion = request["motion"]
        plan = plan_motion(cfg, joints, start, **motion)
        write_json(self.directory/"plan.json", plan)
        print(f"{plan['joint']}: {plan['start'][plan['joint']]:.9g} → {plan['target'][plan['joint']]:.9g} SI, {plan['duration_s']:g}s", flush=True)
        if not request["execute"]:
            return {"status": "planned", "plan": plan, "motion_command_sent": False}
        if any(status in (1, 2, 3) for _, status in self.statuses):
            raise MotionError("An existing action goal is active; this program will not preempt it")
        next_sequence = self.sequence
        wait_until(lambda: self.sequence > next_sequence or self.state_error is not None, self.spin,
                   number(cfg["max_state_age_s"]), "No new feedback before sending")
        check_start(cfg, plan, self.fresh_state()["positions"])
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        from unique_identifier_msgs.msg import UUID
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = plan["joint_names"]
        start_ns = self.node.get_clock().now().nanoseconds+round(number(cfg["lead_time_s"])*1e9)
        goal.trajectory.header.stamp.sec = start_ns//1000000000
        goal.trajectory.header.stamp.nanosec = start_ns%1000000000
        for point in plan["points"]:
            goal.trajectory.points.append(JointTrajectoryPoint(positions=point["positions"], velocities=point["velocities"],
                accelerations=point["accelerations"], time_from_start=duration_message(point["t"])))
        for name in plan["joint_names"]:
            spec = cfg["limits"][name]
            goal.path_tolerance.append(JointTolerance(name=name, position=number(spec["path_tolerance"] if name == plan["joint"] else spec["hold_tolerance"])))
            goal.goal_tolerance.append(JointTolerance(name=name, position=number(spec["goal_tolerance"])))
        goal.goal_time_tolerance = duration_message(number(cfg["goal_time_tolerance_s"]))
        self.owned_uuid = uuid.uuid4().hex
        receipt = {"goal_uuid": self.owned_uuid, "action_name": cfg["action_name"], "robot_id": cfg.get("robot_id"),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"), "target": cfg["target"],
            "request": "request.json", "request_sha256": digest(self.directory/"request.json"),
            "state": "submission_pending", "pid": os.getpid()}
        write_json(self.directory/"goal.json", receipt)
        try:
            future = self.client.send_goal_async(goal, goal_uuid=UUID(uuid=list(bytes.fromhex(self.owned_uuid))))
            wait_until(future.done, self.spin, number(cfg["server_timeout_s"]), "Goal acceptance timed out")
            handle = future.result()
            if not handle.accepted:
                self.terminal = True
                receipt["state"] = "rejected"
                write_json(self.directory/"goal.json", receipt)
                return {"status": "failed", "error": "Controller rejected the goal", "goal_accepted": False,
                        "motion_command_sent": True, "goal_uuid": self.owned_uuid}
            receipt["state"] = "accepted"
            write_json(self.directory/"goal.json", receipt)
            result_future = handle.get_result_async()
            deadline = time.monotonic()+number(cfg["lead_time_s"])+plan["duration_s"]+number(cfg["goal_time_tolerance_s"])
            while not result_future.done():
                self.spin()
                state = self.fresh_state()
                elapsed = state["stamp"]-start_ns*1e-9
                check_tracking(cfg, plan, state["positions"], elapsed)
                if time.monotonic() > deadline:
                    raise MotionError("Motion completion timed out")
            result = result_future.result()
            self.terminal = result.status in (4, 5, 6)
            receipt.update(state="terminal", terminal_status=result.status, error_code=result.result.error_code)
            write_json(self.directory/"goal.json", receipt)
            if result.status == 5:
                final = self.settle(number(cfg["cancel_timeout_s"]))
                return {"status": "stopped", "motion_command_sent": True, "goal_uuid": self.owned_uuid,
                        "stop_verified": True, "final_positions": final}
            if result.status != 4 or result.result.error_code != 0:
                raise MotionError(f"Controller result status={result.status}, error={result.result.error_code}: {result.result.error_string}")
            final = self.settle(number(cfg["goal_time_tolerance_s"]), plan["target"])
            return {"status": "completed", "plan": plan, "motion_command_sent": True, "goal_uuid": self.owned_uuid,
                    "final_positions": final, "errors": {n: final[n]-plan["target"][n] for n in plan["joint_names"]}}
        except (Exception, KeyboardInterrupt) as exc:
            cancellation = self.cancel_owned(self.owned_uuid)
            write_json(self.directory/"cancel.json", cancellation)
            return {"status": "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed", "error": str(exc) or "Interrupted by operator",
                    "motion_command_sent": True, "goal_uuid": self.owned_uuid, "cancellation": cancellation}

    def close(self):
        self.telemetry.close()
        self.executor.remove_node(self.node)
        if self.client:
            self.client.destroy()
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()


def main(request_path):
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    request_path = Path(request_path).resolve()
    request = json.loads(request_path.read_text())
    directory = request_path.parent
    session = None
    try:
        if os.environ.get("ROBOTCAL_DISABLE_NETWORK") == "1":
            raise MotionError("ROS transport is disabled by ROBOTCAL_DISABLE_NETWORK=1")
        if request["operation"] != "stop" and digest(directory/request["config_path"]) != request["config_sha256"]:
            raise MotionError("Configuration changed after request creation")
        cfg = request["config"]
        if request["operation"] == "move":
            urdf = (directory/cfg["urdf"]).resolve()
            if digest(urdf) != request["urdf_sha256"]:
                raise MotionError("URDF changed after request creation")
            validate_profile(cfg, urdf, request["execute"])
        session = RosMotion(request, directory)
        if request["operation"] == "inspect":
            report = session.inspect()
        elif request["operation"] == "stop":
            receipt = request["owned_goal"]
            if (receipt["action_name"] != cfg["action_name"] or receipt["robot_id"] != cfg.get("robot_id")
                    or receipt["ros_domain_id"] != os.environ.get("ROS_DOMAIN_ID", "0")):
                raise MotionError("Stop receipt does not match this robot/action/ROS domain")
            owned_goal_uuid(receipt["goal_uuid"])
            session.require_server()
            reply = session.cancel_owned(receipt["goal_uuid"])
            report = {"status": "stopped" if reply["stop_verified"] else "stop_unconfirmed", **reply}
        else:
            with motion_lock(directory/request["lock_path"]):
                report = session.move(urdf)
    except (Exception, KeyboardInterrupt) as exc:
        report = {"status": "blocked", "error": str(exc) or "Interrupted before goal submission", "motion_command_sent": False}
    finally:
        if session:
            session.close()
    from core import __version__
    report.update(toolkit_version=__version__, backend="ros2", target=request["config"]["target"], sources=SOURCES,
                  is_real_robot_acceptance=False)
    write_json(directory/"result.json", report)
    print(json.dumps({k: v for k, v in report.items() if k != "plan"}, ensure_ascii=False), flush=True)
    return 0 if report["status"] in ("completed", "planned", "inspected", "stopped") else 2


if __name__ == "__main__":
    import sys
    raise SystemExit(main(sys.argv[1]))

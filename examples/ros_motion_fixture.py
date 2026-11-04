"""Isolated ROS protocol fixture. Ideal interpolation, NOT robot physics/hardware.

Refuses the normal robot graph: localhost, domain 200..232, /robotcal_test/ only.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.joint_motion import quintic, write_json


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--namespace", required=True)
    p.add_argument("--directory", required=True)
    p.add_argument("--mode", choices=["normal", "reject", "abort", "stale", "hold-drift"], default="normal")
    args = p.parse_args()
    domain = int(os.environ.get("ROS_DOMAIN_ID", "-1"))
    if not 200 <= domain <= 232 or os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not args.namespace.startswith("/robotcal_test/"):
        raise SystemExit("This fixture requires an isolated test ROS domain and namespace")
    import rclpy
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node("fixture", namespace=args.namespace)
    directory = Path(args.directory)
    directory.mkdir(parents=True, exist_ok=True)
    state = {"q": {"joint1": .1, "joint2": -.1, "slider": .01},
             "v": {"joint1": 0.0, "joint2": 0.0, "slider": 0.0}, "active": False, "freeze": False}
    publisher = node.create_publisher(JointState, args.namespace+"/joint_states", 10)
    group = ReentrantCallbackGroup()

    def publish():
        if state["freeze"]:
            return
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = list(state["q"])
        message.position = [state["q"][n] for n in message.name]
        message.velocity = [state["v"][n] for n in message.name]
        publisher.publish(message)
    node.create_timer(.01, publish, callback_group=group)

    def accept(goal):
        return GoalResponse.REJECT if args.mode == "reject" or state["active"] else GoalResponse.ACCEPT

    def execute(handle):
        state["active"] = True
        goal = handle.request
        names = list(goal.trajectory.joint_names)
        record = {"uuid": bytes(handle.goal_id.uuid).hex(), "joint_names": names,
                  "start": list(goal.trajectory.points[0].positions), "target": list(goal.trajectory.points[-1].positions)}
        with (directory/"goals.jsonl").open("a") as f:
            f.write(json.dumps(record)+"\n")
        result = FollowJointTrajectory.Result()
        try:
            if set(names) != set(state["q"]):
                handle.abort(); result.error_code = -2; return result
            start = dict(zip(names, goal.trajectory.points[0].positions))
            target = dict(zip(names, goal.trajectory.points[-1].positions))
            last = goal.trajectory.points[-1].time_from_start
            duration = last.sec+last.nanosec*1e-9
            stamp = goal.trajectory.header.stamp
            begins = stamp.sec+stamp.nanosec*1e-9
            while rclpy.ok():
                elapsed = node.get_clock().now().nanoseconds*1e-9-begins
                if handle.is_cancel_requested:
                    state["v"] = {n: 0.0 for n in names}
                    handle.canceled(); return result
                if elapsed < 0:
                    time.sleep(.005); continue
                if args.mode == "abort" and elapsed > .2:
                    state["v"] = {n: 0.0 for n in names}
                    result.error_code = -4; result.error_string = "Injected protocol fixture abort"
                    handle.abort(); return result
                if args.mode == "stale" and elapsed > .2:
                    state["freeze"] = True
                values = {n: quintic(start[n], target[n], duration, elapsed) for n in names}
                state["q"] = {n: v[0] for n, v in values.items()}
                state["v"] = {n: v[1] for n, v in values.items()}
                if args.mode == "hold-drift" and elapsed > .2:
                    state["q"]["joint2"] += .04
                if elapsed >= duration:
                    handle.succeed(); return result
                time.sleep(.005)
            return result
        finally:
            state["active"] = False

    server = ActionServer(node, FollowJointTrajectory, args.namespace+"/controller/follow_joint_trajectory",
        execute_callback=execute, goal_callback=accept, cancel_callback=lambda h: CancelResponse.ACCEPT,
        callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    write_json(directory/"ready.json", {"mode": args.mode, "namespace": args.namespace, "domain": domain,
                                      "hardware_accessed": False, "physics_simulated": False})
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

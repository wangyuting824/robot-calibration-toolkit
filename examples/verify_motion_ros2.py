"""Exercise the real ROS action transport against an isolated nonhardware fixture."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.motion_cli import create_demo
from core.joint_motion import write_json


def wait_file(path, process, seconds=10):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if path.is_file():
            try:
                return json.loads(path.read_text())
            except json.JSONDecodeError:
                pass
        if process.poll() is not None:
            raise RuntimeError("Protocol fixture/process exited before producing its expected record")
        time.sleep(.03)
    raise RuntimeError(f"Timed out waiting for {path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--destination", required=True)
    p.add_argument("--ros-python", default="/usr/bin/python3")
    args = p.parse_args()
    destination = Path(args.destination).resolve()
    create_demo(destination)
    base = yaml.safe_load((destination/"mujoco.yaml").read_text())
    env = dict(os.environ, ROS_DOMAIN_ID="221", ROS_LOCALHOST_ONLY="1")
    rows = []
    for mode in ("preview", "normal", "reject", "abort", "stale", "hold-drift", "interrupt", "cancel"):
        case = destination/mode
        case.mkdir()
        namespace = "/robotcal_test/t"+uuid.uuid4().hex
        config = dict(base, backend="ros2", target="protocol_test", test_domain_id=221,
            urdf="../robot.urdf", joint_state_topic=namespace+"/joint_states", state_publisher_node=namespace+"/fixture",
            action_name=namespace+"/controller/follow_joint_trajectory", action_server_node=namespace+"/fixture",
            max_state_age_s=1.0, max_timestamp_skew_s=2.0)
        (case/"profile.yaml").write_text(yaml.safe_dump(config))
        log = (case/"fixture.log").open("w")
        fixture = subprocess.Popen([args.ros_python, str(ROOT/"examples/ros_motion_fixture.py"), "--namespace", namespace,
            "--directory", str(case/"fixture"), "--mode", "normal" if mode in ("preview", "interrupt", "cancel") else mode],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_file(case/"fixture/ready.json", fixture)
            command = ["bash", str(ROOT/"控制关节.sh"), "move", "--config", str(case/"profile.yaml"),
                "--joint", "joint1", "--delta-deg", "5", "--duration", "6" if mode in ("cancel", "interrupt") else "2",
                "--execute", "--ros-python", args.ros_python, "--run-dir", str(case/"run")]
            if mode == "preview":
                command.remove("--execute")
            if mode in ("cancel", "interrupt"):
                with (case/"move.log").open("w") as output:
                    process = subprocess.Popen(command, env=env, stdout=output, stderr=subprocess.STDOUT)
                    receipt = wait_file(case/"run/goal.json", process)
                    time.sleep(.5)
                    if mode == "cancel":
                        stopped = subprocess.run(["bash", str(ROOT/"控制关节.sh"), "stop", "--run", str(case/"run"),
                            "--ros-python", args.ros_python], env=env, capture_output=True, text=True, timeout=20)
                        (case/"stop.log").write_text(stopped.stdout+stopped.stderr)
                        assert stopped.returncode == 0, stopped.stdout+stopped.stderr
                        assert process.wait(timeout=15) == 0
                    else:
                        process.send_signal(signal.SIGINT)
                        assert process.wait(timeout=20) == 2
                report = json.loads((case/"run/result.json").read_text())
                if mode == "cancel":
                    assert report["status"] == "stopped" and report["stop_verified"]
                else:
                    assert report["status"] == "interrupted" and report["cancellation"]["stop_verified"], report
            else:
                completed = subprocess.run(command, env=env, capture_output=True, text=True, timeout=25)
                (case/"move.log").write_text(completed.stdout+completed.stderr)
                report = json.loads((case/"run/result.json").read_text())
                if mode == "preview":
                    assert completed.returncode == 0 and report["status"] == "planned", report
                    assert not (case/"fixture/goals.jsonl").exists()
                elif mode == "normal":
                    assert completed.returncode == 0 and report["status"] == "completed", report
                    assert abs(report["final_positions"]["joint1"]-(.1+5*3.141592653589793/180)) < .002
                    assert abs(report["final_positions"]["joint2"]+.1) < .005
                    captured = [json.loads(line) for line in (case/"fixture/goals.jsonl").read_text().splitlines()]
                    assert captured[0]["joint_names"] == config["controller_joints"]
                    assert captured[0]["start"][1] == captured[0]["target"][1] == -.1
                else:
                    assert completed.returncode != 0 and report["status"] in ("failed", "blocked"), report
                    if mode == "stale":
                        assert report["cancellation"]["stop_verified"] is False
                    if mode in ("abort", "hold-drift"):
                        assert report["cancellation"]["stop_verified"] is True, report
            rows.append({"case": mode, "status": report["status"], "passed": True})
            print(rows[-1], flush=True)
        finally:
            fixture.send_signal(signal.SIGINT)
            try:
                fixture.wait(timeout=5)
            except subprocess.TimeoutExpired:
                fixture.kill(); fixture.wait()
            log.close()
    write_json(destination/"verification.json", {"status": "passed", "cases": rows, "hardware_accessed": False,
        "domain": 221, "localhost_only": True, "qualification": "Real ROS transport and cancellation protocol; ideal fixture, not dynamics or hardware validation"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

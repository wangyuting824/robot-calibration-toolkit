"""Operator entry point for relative/absolute single-axis motion."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid

from core.joint_motion import MotionError, SOURCES, digest, motion_lock, validate_profile, write_json

ROOT = Path(__file__).resolve().parents[1]


def parser():
    p = argparse.ArgumentParser(description="关节动作工具：从实际当前位置移动；默认只预览，--execute才发送动作。")
    subs = p.add_subparsers(dest="operation", required=True)
    demo = subs.add_parser("demo", help="生成可直接运行的MuJoCo示例；不接硬件")
    demo.add_argument("--destination", required=True)
    for op in ("inspect", "move"):
        s = subs.add_parser(op, help="查询状态/接口" if op == "inspect" else "指定关节相对移动或到指定位置")
        s.add_argument("--config", required=True)
        s.add_argument("--run-dir", help="新建日志目录，不覆盖旧记录")
        s.add_argument("--ros-python", help="已安装ROS的Python解释器，默认/usr/bin/python3")
        if op == "move":
            s.add_argument("--joint", required=True)
            units = s.add_mutually_exclusive_group(required=True)
            for flag in ("delta-deg", "target-deg", "delta-mm", "target-mm"):
                units.add_argument("--"+flag, type=float)
            s.add_argument("--duration", type=float, required=True, help="运动时间，秒；不满足速度/加速度约束则拒绝")
            s.add_argument("--execute", action="store_true", help="明确执行本次动作")
            s.add_argument("--viewer", action="store_true", help="仅MuJoCo，打开实时查看器")
    s = subs.add_parser("stop", help="取消本工具某次动作；只针对记录中的goal UUID")
    s.add_argument("--run", required=True, help="动作日志目录或goal.json路径")
    s.add_argument("--ros-python")
    return p


def create_demo(destination):
    import yaml
    path = Path(destination).resolve()
    path.mkdir(parents=True, exist_ok=False)
    path.joinpath("robot.urdf").write_text('''<?xml version="1.0"?>
<robot name="robotcal_motion_demo">
 <link name="base"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
 <link name="link1"><inertial><origin xyz="0.15 0 0"/><mass value="1"/><inertia ixx="0.001" iyy="0.009" izz="0.009" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0.15 0 0"/><geometry><box size="0.3 0.04 0.04"/></geometry><material name="blue"><color rgba="0.1 0.4 0.9 1"/></material></visual></link>
 <joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="20" velocity="0.6"/></joint>
 <link name="link2"><inertial><origin xyz="0.125 0 0"/><mass value="0.5"/><inertia ixx="0.0005" iyy="0.003" izz="0.003" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0.125 0 0"/><geometry><box size="0.25 0.03 0.03"/></geometry><material name="green"><color rgba="0.1 0.8 0.4 1"/></material></visual></link>
 <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="0.6"/></joint>
 <link name="slide"><inertial><mass value="0.2"/><inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/></inertial><visual><geometry><box size="0.04 0.04 0.04"/></geometry></visual></link>
 <joint name="slider" type="prismatic"><parent link="link2"/><child link="slide"/><origin xyz="0.25 0 0"/><axis xyz="1 0 0"/><limit lower="-0.05" upper="0.05" effort="25" velocity="0.02"/></joint>
</robot>''', encoding="utf-8")
    config = {"schema_version": 1, "backend": "mujoco", "target": "simulation", "robot_id": "SIMULATION_DEMO_ONLY",
        "urdf": "robot.urdf", "controller_joints": ["joint1", "joint2", "slider"], "limits": {},
        "state_timeout_s": 3, "max_state_age_s": .3, "max_timestamp_skew_s": 1,
        "server_timeout_s": 3, "goal_time_tolerance_s": 3, "cancel_timeout_s": 3,
        "settle_s": .15, "point_interval_s": .05, "lead_time_s": .2,
        "simulation": {"physics_dt": .002, "gravity": [0, 0, 0], "disable_contacts": True,
            "bias_feedforward": True, "initial_positions": {"joint1": .1, "joint2": -.1, "slider": .01}, "drives": {}}}
    for n in config["controller_joints"]:
        rotary = n != "slider"
        config["limits"][n] = {"lower": -1 if rotary else -.05, "upper": 1 if rotary else .05,
            "max_velocity": .6 if rotary else .02, "max_acceleration": 1 if rotary else .05,
            "path_tolerance": .08 if rotary else .005, "goal_tolerance": .002 if rotary else .0002,
            "hold_tolerance": .005 if rotary else .0005, "start_tolerance": .001 if rotary else .0001,
            "stopped_velocity_tolerance": .005 if rotary else .0005, "source": "Synthetic simulation example; not any physical robot's limits"}
        config["simulation"]["drives"][n] = {"stiffness": 80 if rotary else 200, "damping": 10 if rotary else 20}
    path.joinpath("mujoco.yaml").write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"仿真示例已生成：{path}\n关节：joint1、joint2（角度）；slider（位移）。所有参数仅用于示例。")
    return 0


def run_ros(request_path, interpreter=None):
    interpreter = interpreter or os.environ.get("ROBOTCAL_ROS_PYTHON", "/usr/bin/python3")
    env = dict(os.environ)
    env.pop("PYTHONHOME", None)
    env["PYTHONPATH"] = str(ROOT)+(os.pathsep+env["ROBOTCAL_ROS_PYTHONPATH"] if env.get("ROBOTCAL_ROS_PYTHONPATH") else "")
    env["PYTHONNOUSERSITE"] = "1"
    process = subprocess.Popen([interpreter, "-m", "core.motion_ros2", str(request_path)], env=env, start_new_session=True)
    try:
        return process.wait()
    except KeyboardInterrupt:
        # Worker is in a separate session; exactly one interrupt requests its owned-goal cancellation.
        process.send_signal(signal.SIGINT)
        try:
            return process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            print("ROS进程未确认结束。停止状态未知；请使用现场既定停止方式。", file=sys.stderr)
            return 2


def main(argv=None):
    args = parser().parse_args(argv)
    directory, created = None, False
    try:
        if args.operation == "demo":
            return create_demo(args.destination)
        if args.operation == "stop":
            old = Path(args.run).resolve()
            old = old.parent if old.is_file() else old
            previous = json.loads((old/"request.json").read_text())
            if previous.get("operation") != "move" or not previous.get("execute"):
                raise MotionError("This directory is not an executed motion run")
            if previous["config"]["backend"] == "mujoco":
                (old/"STOP_REQUESTED").write_text("Operator requested simulated stop\n")
                print("已请求仿真停止；查看原动作result.json中的stop_verified。")
                return 0
            receipt = json.loads((old/"goal.json").read_text())
            if receipt.get("request_sha256") != digest(old/"request.json"):
                raise MotionError("Owned-goal receipt does not match the original motion request")
            if receipt.get("state") == "rejected":
                print("本次目标已被控制器拒绝，没有已接收的本次动作可取消。")
                return 0
            directory = old/"stop_requests"/uuid.uuid4().hex
            directory.mkdir(parents=True)
            created = True
            request = {"operation": "stop", "config": previous["config"], "owned_goal": receipt}
            write_json(directory/"request.json", request)
            return run_ros(directory/"request.json", args.ros_python)
        import yaml
        config_path = Path(args.config).resolve()
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if not isinstance(config, dict) or config.get("schema_version") != 1:
            raise MotionError("A schema_version: 1 motion configuration is required")
        if config.get("backend") not in ("ros2", "mujoco"):
            raise MotionError("backend must be ros2 or mujoco")
        if config["backend"] == "ros2" and os.environ.get("ROBOTCAL_DISABLE_NETWORK") == "1":
            raise MotionError("ROS transport is unavailable with ROBOTCAL_DISABLE_NETWORK=1")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        directory = Path(args.run_dir).resolve() if args.run_dir else config_path.parent/"motion_runs"/(stamp+"_"+args.operation)
        directory.mkdir(parents=True, exist_ok=False)
        created = True
        request = {"operation": args.operation, "config_path": os.path.relpath(config_path, directory),
            "config_sha256": digest(config_path), "config": config, "execute": getattr(args, "execute", False),
            "viewer": getattr(args, "viewer", False), "sources": SOURCES}
        from core import __version__
        request["toolkit_version"] = __version__
        if config.get("urdf"):
            model = (config_path.parent/config["urdf"]).resolve()
            config["urdf"] = os.path.relpath(model, directory)
            request["urdf_sha256"] = digest(model)
        if args.operation == "move":
            if not config.get("urdf"):
                raise MotionError("Configure urdf relative to the configuration file")
            validate_profile(config, model, args.execute)
            selected = next((flag for flag in ("delta_deg", "target_deg", "delta_mm", "target_mm") if getattr(args, flag) is not None))
            request["motion"] = {"joint": args.joint, "value": getattr(args, selected), "unit": selected.split("_")[1],
                                 "relative": selected.startswith("delta"), "duration": args.duration}
        if request["viewer"] and config["backend"] != "mujoco":
            raise MotionError("--viewer is only implemented for the MuJoCo backend")
        # Host/user/controller lock, shared by separate copies of the toolkit.
        import hashlib
        import tempfile
        identity = config["backend"]+"|"+str(config.get("action_name"))+"|"+os.environ.get("ROS_DOMAIN_ID", "0")
        lock_id = hashlib.sha256(identity.encode()).hexdigest()[:24]
        lock = Path(tempfile.gettempdir())/("robotcal-motion-"+str(os.getuid()))/(lock_id+".lock")
        request["lock_path"] = os.path.relpath(lock, directory)
        write_json(directory/"request.json", request)
        print(f"本次记录：{directory}\n模式：{config.get('target')} / {config['backend']} / {'执行' if request['execute'] else '只读查询或动作预览'}", flush=True)
        if config["backend"] == "ros2":
            return run_ros(directory/"request.json", args.ros_python)
        from core.motion_mujoco import execute
        with motion_lock(directory/request["lock_path"]):
            result = execute(request, directory)
        print(json.dumps({k: v for k, v in result.items() if k != "plan"}, ensure_ascii=False))
        return 0 if result["status"] in ("completed", "planned", "inspected", "stopped") else 2
    except (Exception, KeyboardInterrupt) as exc:
        report = {"status": "blocked", "error": str(exc) or "Interrupted before starting", "sources": SOURCES}
        if created:
            write_json(directory/"result.json", report)
        print(f"未执行/未完成：{report['error']}", file=sys.stderr)
        return 2

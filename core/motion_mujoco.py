"""Physical MuJoCo stepping for the motion CLI; never imports a hardware driver."""
import csv
import math
from pathlib import Path
import time

from core.joint_motion import (MotionError, SOURCES, check_state, check_tracking,
    number, plan_motion, quintic, validate_profile, write_json)


def run(request, directory):
    import mujoco
    from core.mujoco_model import prepare
    cfg = request["config"]
    urdf = (directory/cfg["urdf"]).resolve()
    joints = validate_profile(cfg, urdf, request["execute"])
    prepare(urdf, directory/"mujoco")
    model = mujoco.MjModel.from_xml_path(str(directory/"mujoco/robot.urdf"))
    settings = cfg.get("simulation", {})
    model.opt.timestep = number(settings.get("physics_dt"), "simulation.physics_dt", True)
    if model.opt.timestep > .01:
        raise MotionError("Simulation timestep must be <= 0.01 s")
    if settings.get("disable_contacts") is True:
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    gravity = settings.get("gravity")
    if not isinstance(gravity, list) or len(gravity) != 3:
        raise MotionError("Declare the simulation gravity vector")
    model.opt.gravity[:] = [number(v) for v in gravity]
    names = cfg["controller_joints"]
    if model.nq != len(names) or model.nv != len(names):
        raise MotionError("MuJoCo motion currently requires all scalar independent joints in one group; floating/mimic/loop mechanisms need a matching controller")
    indices = {}
    for name in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise MotionError(f"MuJoCo missing joint {name}")
        indices[name] = (model.jnt_qposadr[jid], model.jnt_dofadr[jid])
    data = mujoco.MjData(model)
    for name, (q, _) in indices.items():
        data.qpos[q] = number(settings.get("initial_positions", {}).get(name), name+" initial position")
    mujoco.mj_forward(model, data)
    positions = lambda: {n: float(data.qpos[q]) for n, (q, _) in indices.items()}
    velocities = lambda: {n: float(data.qvel[v]) for n, (_, v) in indices.items()}
    if request["operation"] == "inspect":
        return {"status": "inspected", "positions": positions(), "joint_types": {n: joints[n]["type"] for n in names}}
    plan = plan_motion(cfg, joints, positions(), **request["motion"])
    write_json(directory/"plan.json", plan)
    if not request["execute"]:
        return {"status": "planned", "plan": plan, "motion_command_sent": False}
    viewer = None
    if request.get("viewer"):
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    log = (directory/"feedback.csv").open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(log, fieldnames=["t", "joint", "q", "dq", "q_command", "dq_command", "applied_effort"])
    writer.writeheader()

    def step(target, desired_velocity):
        before = time.monotonic()
        for n, (qi, vi) in indices.items():
            gains = settings.get("drives", {}).get(n, {})
            kp, kd = number(gains.get("stiffness"), n+" stiffness", True), number(gains.get("damping"), n+" damping", True)
            force = kp*(target[n]-data.qpos[qi])+kd*(desired_velocity[n]-data.qvel[vi])
            if settings.get("bias_feedforward") is True:
                force += data.qfrc_bias[vi]
            effort = joints[n]["effort"]
            data.qfrc_applied[vi] = min(effort, max(-effort, force))
            writer.writerow({"t": data.time, "joint": n, "q": data.qpos[qi], "dq": data.qvel[vi],
                             "q_command": target[n], "dq_command": desired_velocity[n], "applied_effort": data.qfrc_applied[vi]})
        mujoco.mj_step(model, data)
        check_state(cfg, positions())
        if not all(math.isfinite(v) for v in velocities().values()):
            raise MotionError("Nonfinite simulation velocity")
        if viewer:
            viewer.sync()
            time.sleep(max(0, model.opt.timestep-(time.monotonic()-before)))

    def brake():
        hold = positions()
        deadline, stable = data.time+number(cfg["cancel_timeout_s"]), None
        while data.time < deadline:
            step(hold, {n: 0.0 for n in names})
            quiet = all(abs(v) <= number(cfg["limits"][n]["stopped_velocity_tolerance"]) for n, v in velocities().items())
            stable = (stable if stable is not None else data.time) if quiet else None
            if stable is not None and data.time-stable >= number(cfg["settle_s"]):
                return True
        return False

    try:
        deadline, stable = plan["duration_s"]+number(cfg["goal_time_tolerance_s"]), None
        while data.time <= deadline:
            if (directory/"STOP_REQUESTED").exists() or (viewer and not viewer.is_running()):
                stopped = brake()
                return {"status": "stopped" if stopped else "stop_unconfirmed", "stop_verified": stopped, "plan": plan,
                        "final_positions": positions(), "motion_command_sent": True}
            desired = {n: quintic(plan["start"][n], plan["target"][n], plan["duration_s"], data.time) for n in names}
            step({n: v[0] for n, v in desired.items()}, {n: v[1] for n, v in desired.items()})
            check_tracking(cfg, plan, positions(), data.time)
            quiet = data.time >= plan["duration_s"] and all(
                abs(positions()[n]-plan["target"][n]) <= number(cfg["limits"][n]["goal_tolerance"])
                and abs(velocities()[n]) <= number(cfg["limits"][n]["stopped_velocity_tolerance"]) for n in names)
            stable = (stable if stable is not None else data.time) if quiet else None
            if stable is not None and data.time-stable >= number(cfg["settle_s"]):
                final = positions()
                return {"status": "completed", "plan": plan, "final_positions": final,
                        "errors": {n: final[n]-plan["target"][n] for n in names}, "motion_command_sent": True}
        raise MotionError("MuJoCo did not reach and settle at the target")
    except KeyboardInterrupt:
        stopped = brake()
        return {"status": "interrupted", "stop_verified": stopped, "motion_command_sent": True, "final_positions": positions()}
    finally:
        log.close()
        if viewer:
            viewer.close()


def execute(request, directory):
    try:
        report = run(request, directory)
    except Exception as exc:
        report = {"status": "failed", "error": str(exc)}
    from core import __version__
    report.update(toolkit_version=__version__, backend="mujoco", target="simulation", sources=SOURCES,
                  hardware_control_commands_sent=False, is_real_robot_acceptance=False)
    write_json(directory/"result.json", report)
    return report

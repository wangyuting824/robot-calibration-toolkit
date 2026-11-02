"""Compare independent FK code with actual Isaac 6.1.0 tensor link poses.

Simulation-only state resets and zero-gravity stepping. This verifies software
and import semantics; identical URDF inputs cannot establish real geometry.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from adapters.isaac_adapter import IsaacAdapter, numpy_value
from core.io import InputError, load, number, result, write_csv
from core.model import Model
from core.metrics import summary


def run(args):
    model = Model(args.urdf)
    config = load(args.config)
    poses = config.get("joint_positions", [])
    if not poses:
        raise InputError("Provide explicit simulation test joint_positions")
    for pose in poses:
        if set(pose) != set(model.independent):
            raise InputError("Each simulation pose must specify all independent URDF joints")
        model.check_q(pose)
    tolerance = load(args.acceptance).get("runtime_kinematics", {})
    p_tol, r_tol = tolerance.get("max_position_m"), tolerance.get("max_orientation_rad")
    if any(v is None or number(v) < 0 for v in (p_tol, r_tol)):
        raise InputError("Declare nonnegative runtime software comparison tolerances")
    adapter = IsaacAdapter({**config.get("options", {}), "stage": args.stage, "observe_rigid_bodies": list(model.links)})
    errors = []
    try:
        if not adapter.modern or not np.allclose(adapter.runtime_info["gravity_m_s2"], 0):
            raise InputError("Runtime FK validation requires Isaac 6.1.0 and authored zero gravity")
        if set(model.independent) - set(adapter.names) or set(adapter.names) - set(model.movable):
            raise InputError("Imported articulation DOFs do not map to the nominal URDF")
        import omni.usd
        from pxr import UsdPhysics
        from isaacsim.core.experimental.utils import backend as backend_utils
        stage = omni.usd.get_context().get_stage()
        bodies = {}
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in model.links:
                if prim.GetName() in bodies:
                    raise InputError("Ambiguous rigid body names")
                bodies[prim.GetName()] = str(prim.GetPath())
            for drive_name in ("angular", "linear"):
                drive = UsdPhysics.DriveAPI(prim, drive_name)
                if drive and (number(drive.GetStiffnessAttr().Get()) != 0 or number(drive.GetDampingAttr().Get()) != 0):
                    raise InputError("Disable drives in this simulation-only FK test asset")
        if set(bodies) != set(model.links):
            raise InputError("Runtime test requires an explicit rigid-body mapping for every tested link")
        names = list(model.links)
        rigid = adapter.rigid_view
        for index, q in enumerate(poses):
            expanded = model.positions(q)
            values = [[expanded[n] for n in adapter.names]]
            adapter.robot.set_dof_positions(values)
            adapter.robot.set_dof_velocities(np.zeros((1, len(adapter.names))))
            adapter.robot.set_dof_position_targets(values)
            adapter.manager.step(steps=2)
            state = adapter.read_state()
            observed_q = {n: state["q:" + n] for n in model.independent}
            predicted = model.transforms(observed_q)
            with backend_utils.use_backend("tensor"):
                xyz, quat = rigid.get_world_poses()
            xyz, quat = numpy_value(xyz), numpy_value(quat)
            actual = {}
            for n, p, r in zip(names, xyz, quat):
                matrix = np.eye(4)
                matrix[:3, :3] = Rotation.from_quat([*r[1:], r[0]]).as_matrix()
                matrix[:3, 3] = p
                actual[n] = matrix
            root_inverse = np.linalg.inv(actual[model.root])
            for name in names:
                measured = root_inverse @ actual[name]
                position = float(np.linalg.norm(measured[:3, 3] - predicted[name][:3, 3]))
                angle = float(Rotation.from_matrix(predicted[name][:3, :3].T @ measured[:3, :3]).magnitude())
                errors.append({"sample": index, "link": name, "position_error_m": position,
                               "orientation_error_rad": angle})
        write_csv(args.errors_csv, errors)
        p_stats = summary([r["position_error_m"] for r in errors])
        r_stats = summary([r["orientation_error_rad"] for r in errors])
        passed = p_stats["max_abs"] <= number(p_tol) and r_stats["max_abs"] <= number(r_tol)
        return result("validate-runtime-kinematics", {"metrics": {"position_m": p_stats, "orientation_rad": r_stats},
            "poses": len(poses), "links_per_pose": len(names), "errors_csv": str(args.errors_csv),
            "runtime_info": adapter.runtime_info, "backend": "Isaac RigidPrim tensor transforms (not USD authored poses)",
            "hardware_commands_sent": False, "is_real_robot_acceptance": False,
            "scope": "Forward kinematics and import consistency on a nominal model; no real robot calibration"},
            [args.urdf, args.stage, args.config, args.acceptance], "simulation_pass" if passed else "failed")
    finally:
        try:
            adapter.stop()
        finally:
            adapter.close()

"""Static USD/URDF consistency, not a physical-response or collision-clearance test."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from adapters.isaac_adapter import start_app
from core.io import InputError, load, number, result
from core.model import Model, origin
from core.schemas import inertia_matrix


def run(args):
    model = Model(args.urdf)
    tolerance = load(args.acceptance).get("asset", {})
    app = None
    try:
        from pxr import Usd, UsdGeom, UsdPhysics
    except ImportError:
        app = start_app(True)
    try:
        from pxr import Usd, UsdGeom, UsdPhysics
        stage = Usd.Stage.Open(str(args.stage))
        if stage is None:
            raise InputError("Cannot open USD scene")
        errors, checks = [], {}
        if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) > 1e-12 or abs(UsdPhysics.GetStageKilogramsPerUnit(stage) - 1) > 1e-12:
            errors.append("Scene must use meters and kilograms")
        if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
            errors.append("Scene must use Z up to match the toolkit frame convention")
        joints, bodies = {}, {}
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint) and prim.GetName() in model.joints:
                if prim.GetName() in joints:
                    errors.append("Ambiguous duplicate USD joint names")
                joints[prim.GetName()] = prim
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in model.links:
                if prim.GetName() in bodies:
                    errors.append("Ambiguous duplicate USD rigid-body names")
                bodies[prim.GetName()] = prim
        needed = ("limit_tolerance_rad_or_m", "axis_tolerance_rad", "anchor_tolerance_m",
                  "mass_tolerance_kg", "com_tolerance_m", "inertia_tolerance_kg_m2")
        configured = all(tolerance.get(k) is not None for k in needed)
        if configured and any(number(tolerance[k]) < 0 for k in needed):
            raise InputError("Asset consistency tolerances must be nonnegative")
        threshold = number(tolerance["limit_tolerance_rad_or_m"]) if configured else None
        zero_poses = model.transforms({n: 0 for n in model.independent})
        cache = UsdGeom.XformCache()

        def world_pose(prim):
            return np.array(cache.GetLocalToWorldTransform(prim), dtype=float).T

        def quat_matrix(value):
            return Rotation.from_quat([*value.GetImaginary(), value.GetReal()]).as_matrix()

        root_inverse = np.linalg.inv(world_pose(bodies[model.root])) if model.root in bodies else None
        if root_inverse is None:
            errors.append("Root rigid body missing; verify fixed-link/name mapping")
        for name in model.movable:
            prim = joints.get(name)
            if prim is None:
                errors.append(f"Missing movable joint: {name}")
                continue
            kind = model.joints[name].get("type")
            angular = kind != "prismatic"
            if angular and not prim.IsA(UsdPhysics.RevoluteJoint) or not angular and not prim.IsA(UsdPhysics.PrismaticJoint):
                errors.append(f"Joint type mismatch: {name}")
                continue
            joint = UsdPhysics.RevoluteJoint(prim) if angular else UsdPhysics.PrismaticJoint(prim)
            checks[name] = {}
            if kind != "continuous":
                lower, upper = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
                factor = math.pi / 180 if angular else 1
                source = model.joints[name].find("limit")
                residual = max(abs(number(lower) * factor - number(source.get("lower"))),
                               abs(number(upper) * factor - number(source.get("upper"))))
                checks[name]["max_limit_error_rad_or_m"] = residual
                if configured and residual > threshold:
                    errors.append(f"Limit mismatch: {name}")
            api = UsdPhysics.Joint(prim)
            targets0, targets1 = api.GetBody0Rel().GetTargets(), api.GetBody1Rel().GetTargets()
            parent = model.joints[name].find("parent").get("link")
            child = model.joints[name].find("child").get("link")
            if len(targets0) != 1 or len(targets1) != 1 or targets0[0].name != parent or targets1[0].name != child:
                errors.append(f"Body relationship mismatch: {name}")
                continue
            if root_inverse is not None:
                body = stage.GetPrimAtPath(targets0[0])
                parent_actual = root_inverse @ world_pose(body)
                expected = zero_poses[parent] @ origin(model.joints[name].find("origin"))
                axis_token = str(joint.GetAxisAttr().Get())
                if axis_token not in ("X", "Y", "Z"):
                    errors.append(f"Unknown axis token: {name}")
                    continue
                axis = np.eye(3)[("X", "Y", "Z").index(axis_token)]
                actual_axis = parent_actual[:3, :3] @ quat_matrix(api.GetLocalRot0Attr().Get()) @ axis
                expected_axis = expected[:3, :3] @ model.axis(name)
                axis_error = math.acos(float(np.clip(actual_axis @ expected_axis, -1, 1)))
                position = parent_actual[:3, :3] @ np.array(api.GetLocalPos0Attr().Get()) + parent_actual[:3, 3]
                anchor_error = float(np.linalg.norm(position - expected[:3, 3]))
                checks[name].update(axis_error_rad=axis_error, anchor_error_m=anchor_error)
                if configured and (axis_error > number(tolerance["axis_tolerance_rad"]) or anchor_error > number(tolerance["anchor_tolerance_m"])):
                    errors.append(f"Axis/anchor mismatch: {name}")
        mass_checks = {}
        for name, link in model.links.items():
            if name not in bodies:
                errors.append(f"Missing rigid body: {name}")
                continue
            source = link.find("inertial")
            if source is None:
                errors.append(f"Missing source inertia: {name}")
                continue
            api = UsdPhysics.MassAPI(bodies[name])
            if not api:
                errors.append(f"Missing explicit mass properties: {name}")
                continue
            mass_error = abs(number(api.GetMassAttr().Get()) - number(source.find("mass").get("value")))
            source_pose = origin(source.find("origin"))
            actual_com = np.array(api.GetCenterOfMassAttr().Get())
            diagonal = np.array(api.GetDiagonalInertiaAttr().Get())
            if not np.all(np.isfinite(actual_com)) or not np.all(np.isfinite(diagonal)) or np.any(diagonal <= 0):
                errors.append(f"Unresolved or invalid explicit COM/inertia: {name}")
                continue
            com_error = float(np.linalg.norm(actual_com - source_pose[:3, 3]))
            attrs = source.find("inertia").attrib
            source_tensor = inertia_matrix([attrs[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")])
            source_tensor = source_pose[:3, :3] @ source_tensor @ source_pose[:3, :3].T
            principal = quat_matrix(api.GetPrincipalAxesAttr().Get())
            actual_tensor = principal @ np.diag(diagonal) @ principal.T
            inertia_error = float(np.max(abs(actual_tensor - source_tensor)))
            mass_checks[name] = {"mass_error_kg": mass_error, "com_error_m": com_error, "inertia_error_kg_m2": inertia_error}
            if configured and (mass_error > number(tolerance["mass_tolerance_kg"]) or com_error > number(tolerance["com_tolerance_m"]) or inertia_error > number(tolerance["inertia_tolerance_kg_m2"])):
                errors.append(f"Mass/COM/inertia mismatch: {name}")
        return result("validate-isaac-asset", {"errors": errors, "checks": checks, "mass_checks": mass_checks,
                      "scope": "Static USD/URDF units, joint types/bodies/limits/axes/anchors, link masses/COM/inertias",
                      "not_checked": ["runtime FK", "self collision clearance", "closed-loop response", "dynamics"]},
                      [args.urdf, args.stage, args.acceptance], "failed" if errors else "passed" if configured else "unverified")
    finally:
        if app is not None:
            app.close()

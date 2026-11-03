"""Import with documented NVIDIA 6.1.0 APIs; explicit 5.1.0 legacy option."""
from pathlib import Path
import math
import os

from adapters.isaac_adapter import start_app, runtime_version, TARGET_VERSION
from core.io import InputError, load, number, result, sha256, vector
from core.model import Model


def run(args):
    model = Model(args.urdf)
    config = load(args.config)
    manifest_path = Path(args.urdf).parent / "model_manifest.yaml"
    manifest = load(manifest_path)
    if manifest.get("urdf_sha256") != sha256(args.urdf) or manifest.get("draft") or not manifest.get("definition_ready") or not manifest.get("inertial_parameters_confirmed"):
        raise InputError("Build a confirmed model with separately confirmed inertias before importing a physical asset")
    mechanism = manifest.get("mechanisms", {})
    if mechanism.get("status") != "confirmed":
        raise InputError("Mechanism definition is unconfirmed")
    if not isinstance(config.get("fix_base"), bool):
        raise InputError("Explicitly configure fix_base for this test")
    if config.get("physics_backend") != "physx":
        raise InputError("This scene authoring implementation targets PhysX; other backends require their own validated mapping")
    if mechanism.get("closed_loops") and not config.get("constraint_layer"):
        raise InputError("Supply a validated USD constraint layer for the actual closed-loop mechanism")
    output = Path(args.destination).resolve()
    output.mkdir(parents=True, exist_ok=True)
    app = start_app(config.get("headless", True))
    try:
        version = runtime_version(config.get("expected_isaac_sim_version", TARGET_VERSION))
        import omni.kit.app
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, PhysxSchema
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        import isaacsim.asset.importer.urdf as importer_api
        if version["isaac_sim_version"] == TARGET_VERSION:
            if not hasattr(importer_api, "URDFImporter"):
                raise InputError("Isaac Sim 6.1.0 URDFImporter API is unavailable")
            api_used = "NVIDIA 6.1.0 URDFImporter"
            imported = importer_api.URDFImporter(importer_api.URDFImporterConfig(
                urdf_path=str(Path(args.urdf).resolve()), usd_path=str(output / "asset"),
                fix_base=config["fix_base"], merge_fixed_joints=False, merge_mesh=False,
                collision_from_visuals=False, allow_self_collision=bool(config.get("self_collision", False)),
                run_multi_physics_conversion=True,
            )).import_urdf()
        else:
            import omni.kit.commands
            api_used = "NVIDIA 5.1 URDFParseAndImportFile"
            ok, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            if not ok:
                raise InputError("Installed NVIDIA importer cannot create its configuration")
            import_config.merge_fixed_joints = False
            import_config.import_inertia_tensor = True
            import_config.make_default_prim = True
            import_config.fix_base = config["fix_base"]
            import_config.collision_from_visuals = False
            import_config.self_collision = bool(config.get("self_collision", False))
            asset_dir = output / "asset"
            asset_dir.mkdir(parents=True, exist_ok=True)
            imported = str(asset_dir / "robot.usd")
            ok, _ = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(Path(args.urdf).resolve()),
                                              import_config=import_config, dest_path=imported)
            if not ok:
                raise InputError("NVIDIA 5.1 importer failed")
        asset = Path(imported).resolve()
        if not asset.is_file():
            raise InputError("NVIDIA importer did not produce the declared USD file")
        asset_stage = Usd.Stage.Open(str(asset))
        default = asset_stage.GetDefaultPrim()
        if not default:
            raise InputError("Imported asset has no default prim")
        scene = output / "scene.usda"
        stage = Usd.Stage.CreateNew(str(scene))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        robot = stage.DefinePrim("/World/Robot", "Xform")
        robot.GetReferences().AddReference(os.path.relpath(asset, output))
        if config.get("base_xyz_rpy") is not None:
            from scipy.spatial.transform import Rotation
            base = vector(config["base_xyz_rpy"], 6, "base_xyz_rpy")
            q = Rotation.from_euler("xyz", base[3:]).as_quat()
            xform = UsdGeom.Xformable(robot)
            xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, base[:3])))
            xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), Gf.Vec3d(*map(float, q[:3]))))
        variants = robot.GetVariantSets()
        if variants.HasVariantSet("Physics"):
            physics_variant = variants.GetVariantSet("Physics")
            if "physx" not in physics_variant.GetVariantNames():
                raise InputError("Imported asset has no PhysX physics variant")
            # NVIDIA's 6.1 importer tests explicitly select this variant too.
            physics_variant.SetVariantSelection("physx")
        scene_physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        gravity = config.get("gravity", [0, 0, -9.81])
        magnitude = math.sqrt(sum(number(v) ** 2 for v in gravity))
        direction = [number(v) / magnitude for v in gravity] if magnitude else [0, 0, -1]
        scene_physics.CreateGravityDirectionAttr(Gf.Vec3f(*direction))
        scene_physics.CreateGravityMagnitudeAttr(magnitude)
        physics = PhysxSchema.PhysxSceneAPI.Apply(scene_physics.GetPrim())
        dt = number(config.get("physics_dt"))
        if dt <= 0:
            raise InputError("physics_dt must be positive")
        stage.SetTimeCodesPerSecond(1 / dt)
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(1_000_000)
        physics.CreateTimeStepsPerSecondAttr(int(round(1 / dt)))
        if config.get("constraint_layer"):
            constraint = Path(config["constraint_layer"]).resolve()
            if not constraint.is_file() or not config.get("constraint_reference"):
                raise InputError("Constraint layer and engineering reference are required")
            # Explicit layer must use scene paths; do not invent generic closure joints.
            from shutil import copy2
            copy2(constraint, output / "constraints.usda")
            stage.GetRootLayer().subLayerPaths.append("constraints.usda")
        drives = config.get("drives", {})
        joint_paths = {}
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint) or prim.GetName() not in model.joints:
                continue
            name = prim.GetName()
            joint_paths[name] = str(prim.GetPath())
            kind = model.joints[name].get("type")
            if kind == "fixed":
                continue
            angular = kind in ("revolute", "continuous")
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular" if angular else "linear")
            gains = drives.get(name)
            if gains is None:
                drive.CreateStiffnessAttr(0)
                drive.CreateDampingAttr(0)
                continue
            if gains.get("type") != "force":
                raise InputError("Only explicitly specified force-drive gains are authored")
            # OpenUSD angular drives use degrees, whereas input gains are per radian.
            factor = math.pi / 180 if angular else 1.0
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(number(gains["stiffness"]) * factor)
            drive.CreateDampingAttr(number(gains["damping"]) * factor)
            drive.CreateMaxForceAttr(number(gains["max_effort"]))
        if set(drives) - set(joint_paths):
            raise InputError("Drive configuration contains joints not found after import")
        if config.get("tool_frame"):
            from scipy.spatial.transform import Rotation
            tool = config["tool_frame"]
            parents = [p for p in stage.Traverse() if p.GetName() == tool["parent_link"] and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(parents) != 1 or not tool.get("name", "").isidentifier():
                raise InputError("Tool frame needs one rigid parent and an identifier name")
            pose = vector(tool["xyz_rpy"], 6, "tool xyz_rpy")
            q = Rotation.from_euler("xyz", pose[3:]).as_quat()
            path = str(parents[0].GetPath()) + "/" + tool["name"]
            if stage.GetPrimAtPath(path):
                raise InputError("Tool frame path already exists")
            xform = UsdGeom.Xform.Define(stage, path)
            xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, pose[:3])))
            xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), Gf.Vec3d(*map(float, q[:3]))))
        for material in config.get("materials", []):
            path = material["path"]
            surface = UsdShade.Material.Define(stage, path)
            api = UsdPhysics.MaterialAPI.Apply(surface.GetPrim())
            api.CreateStaticFrictionAttr(number(material["static_friction"]))
            api.CreateDynamicFrictionAttr(number(material["dynamic_friction"]))
            api.CreateRestitutionAttr(number(material["restitution"]))
            for shape in material["collision_paths"]:
                prim = stage.GetPrimAtPath(shape)
                if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI):
                    raise InputError(f"Material target is not a collision prim: {shape}")
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(surface, materialPurpose="physics")
        stage.GetRootLayer().Save()
        articulation_paths = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        return result("build-isaac-asset", {**version, "asset": str(asset), "scene": str(scene), "api": api_used,
                      "joint_paths": joint_paths, "articulation_paths": articulation_paths,
                      "unconfigured_drives_disabled": sorted(set(model.independent) - set(drives)),
                      "configuration": config, "runtime_physics_validated": False}, [args.urdf, args.config, manifest_path], "built")
    finally:
        app.close()

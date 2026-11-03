"""Optional MuJoCo nominal-model check using the official URDF compiler.

This module never imports a robot driver. Compiler/FK checks are software tests,
not measurements of physical axes, joint limits, inertia or accuracy.
"""
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from core.io import InputError, save, sha256, write_csv
from core.model import Model, origin
from core.units_and_frames import pose_error

SOURCES = {
    "urdf_compiler": "https://mujoco.readthedocs.io/en/3.17.0/modeling.html#urdf-extensions",
    "compiler_and_equality": "https://mujoco.readthedocs.io/en/3.17.0/XMLreference.html",
    "python": "https://mujoco.readthedocs.io/en/3.17.0/python.html",
    "dae_conversion": "https://trimesh.org/trimesh.exchange.dae.html",
}


def prepare(urdf, destination):
    """Copy resolved local assets; convert DAE geometry to OBJ for the compiler.

    DAE node transforms are baked into vertices by trimesh; URDF scales (including
    mirrors) remain on each mesh. External textures require a separate pipeline.
    No mesh-derived or repaired inertia is allowed. Original input is untouched.
    """
    import mujoco
    source = Path(urdf).resolve()
    nominal = Model(source)
    target = Path(destination).resolve()
    target.mkdir(parents=True, exist_ok=False)
    (target / "meshes").mkdir()
    tree = ET.parse(source)
    root = tree.getroot()
    if root.find("mujoco") is not None:
        raise InputError("Existing MuJoCo extensions need explicit review before this test")
    assets = {}
    geometry_names = set()
    for index, geom in enumerate(root.findall(".//visual") + root.findall(".//collision")):
        if geom.get("name"):
            name = geom.get("name")
            if name in geometry_names:
                name = f"robotcal_geom_{index}_{name}"
                geom.set("name", name)
            geometry_names.add(name)
    for mesh in root.findall(".//mesh"):
        name = mesh.get("filename", "")
        if "://" in name:
            raise InputError("Resolve package/remote mesh URIs before MuJoCo preparation")
        path = (source.parent / name).resolve()
        if not path.is_file():
            raise InputError(f"Missing mesh: {path}")
        key = sha256(path)
        if key not in assets:
            suffix = path.suffix.lower()
            if suffix not in (".dae", ".stl", ".obj", ".msh"):
                raise InputError(f"Unsupported mesh format: {suffix}")
            output = target / "meshes" / (key[:16] + (".obj" if suffix == ".dae" else suffix))
            info = {"source": str(path), "source_sha256": key, "conversion": "copy"}
            if suffix == ".dae":
                import trimesh
                dae = ET.parse(path)
                if any(e.tag.rsplit("}", 1)[-1] == "image" for e in dae.getroot().iter()):
                    raise InputError("Textured DAE requires explicit texture/material conversion")
                scene = trimesh.load_scene(path, process=False)
                geometry = scene.to_mesh()
                if not len(geometry.faces) or not np.isfinite(geometry.vertices).all():
                    raise InputError("DAE did not produce finite triangle geometry")
                geometry.export(output, file_type="obj", include_texture=False, include_color=False)
                info.update(conversion="DAE scene geometry to OBJ; URDF material retained",
                    trimesh_version=trimesh.__version__, vertices=len(geometry.vertices), faces=len(geometry.faces),
                    source_scene_bounds=scene.bounds.tolist(), output_bounds=geometry.bounds.tolist())
            elif suffix == ".stl":
                # Official MuJoCo docs accept binary STL, not ASCII STL.
                with path.open("rb") as handle:
                    header=handle.read(84)
                binary = len(header)==84 and 84+50*struct.unpack("<I",header[80:84])[0]==path.stat().st_size
                over_stl_face_limit = binary and struct.unpack("<I",header[80:84])[0] > 200000
                if binary and not over_stl_face_limit:
                    shutil.copy2(path,output)
                else:
                    import trimesh
                    geometry=trimesh.load_mesh(path,process=False)
                    if not isinstance(geometry,trimesh.Trimesh) or not len(geometry.faces) or not np.isfinite(geometry.vertices).all():
                        raise InputError("STL did not produce finite triangle geometry")
                    if over_stl_face_limit:
                        output=output.with_suffix(".obj")
                        geometry.export(output,file_type="obj",include_texture=False,include_color=False,include_normals=False,digits=17)
                    else:
                        geometry.export(output,file_type="stl")
                    info.update(conversion="Binary STL to OBJ for MuJoCo STL decoder face limit; no processing or decimation" if over_stl_face_limit else "ASCII STL to binary STL; no processing or decimation",
                        trimesh_version=trimesh.__version__,vertices=len(geometry.vertices),faces=len(geometry.faces),
                        output_bounds=geometry.bounds.tolist(),
                        max_vertex_float32_roundoff=float(np.max(abs(geometry.vertices-geometry.vertices.astype(np.float32).astype(np.float64)))))
            else:
                if suffix == ".obj" and any(line.lstrip().startswith("mtllib ") for line in path.read_text().splitlines()):
                    raise InputError("OBJ material dependencies require explicit packaging")
                shutil.copy2(path, output)
            info.update(path=str(output.relative_to(target)), sha256=sha256(output))
            assets[key] = info
        mesh.set("filename", assets[key]["path"])
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(extension, "compiler", strippath="false", fusestatic="false", discardvisual="false",
        inertiafromgeom="false", balanceinertia="false", boundmass="0", boundinertia="0")
    ET.SubElement(extension, "option", gravity="0 0 0", timestep="0.002")
    modified = target / "robot.urdf"
    tree.write(modified, encoding="utf-8", xml_declaration=True)
    compiled = mujoco.MjModel.from_xml_path(str(modified))
    mjcf = target / "robot.xml"
    mujoco.mj_saveLastXML(str(mjcf), compiled)
    # Make saved MJCF relocatable and preserve fixed-link names on recompile.
    saved = ET.parse(mjcf)
    saved.find("compiler").set("fusestatic", "false")
    for mesh in saved.findall(".//asset/mesh"):
        path = Path(mesh.get("file"))
        if path.is_absolute():
            mesh.set("file", str(path.relative_to(target)))
    saved.write(mjcf, encoding="utf-8", xml_declaration=True)
    info = {"status": "built", "backend": "MuJoCo", "version": mujoco.__version__,
        "source_urdf": str(source), "source_sha256": sha256(source), "mjcf_sha256": sha256(mjcf),
        "runtime_model": "robot.urdf", "readable_mjcf": "robot.xml", "mjcf_export_is_runtime": False,
        "serialization_note": "Run the prepared URDF directly. mj_saveLastXML may round geometry/frame numbers in its readable MJCF export.",
        "assets": list(assets.values()), "sources": SOURCES,
        "is_real_robot_acceptance": False, "hardware_control_commands_sent": False,
        "scope": "Fixed base, zero gravity nominal model; URDF materials and collision meshes retained; mesh collision uses MuJoCo convex geometry"}
    save(target / "preparation.json", info)
    validate_static(nominal, compiled)
    return modified


def validate_static(nominal, model):
    """Reject missing or changed joints, frames, limits, mass and full inertia."""
    import mujoco
    expected = set(nominal.movable)
    found = {model.joint(i).name for i in range(model.njnt)}
    if expected != found:
        raise InputError(f"MuJoCo joint mismatch: {expected ^ found}")
    for name in nominal.links:
        body = model.body(name)
        inertial = nominal.links[name].find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        attrs = inertial.find("inertia").attrib
        xx, yy, zz, xy, xz, yz = [float(attrs[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        tensor = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        local = origin(inertial.find("origin"))
        rotation = Rotation.from_quat(np.roll(body.iquat, -1)).as_matrix()
        if not (np.isclose(body.mass[0], mass, atol=1e-10, rtol=1e-5)
                and np.allclose(body.ipos, local[:3, 3], atol=1e-7)
                and np.allclose(rotation @ np.diag(body.inertia) @ rotation.T,
                    local[:3, :3] @ tensor @ local[:3, :3].T, atol=1e-8, rtol=1e-4)):
            raise InputError(f"MuJoCo mass/COM/inertia mismatch: {name}")
    mimic_count = 0
    for name in nominal.movable:
        joint, source = model.joint(name), nominal.joints[name]
        kind = mujoco.mjtJoint.mjJNT_SLIDE if source.get("type") == "prismatic" else mujoco.mjtJoint.mjJNT_HINGE
        if joint.type[0] != kind or not np.allclose(joint.axis, nominal.axis(name), atol=1e-10):
            raise InputError(f"MuJoCo axis/type mismatch: {name}")
        bounded = source.get("type") != "continuous"
        if bool(joint.limited[0]) != bounded:
            raise InputError(f"MuJoCo limit enable mismatch: {name}")
        if bounded and not np.allclose(joint.range, [float(source.find("limit").get(k)) for k in ("lower", "upper")], atol=1e-9, rtol=1e-5):
            raise InputError(f"MuJoCo range mismatch: {name}")
        mimic = source.find("mimic")
        if mimic is not None:
            mimic_count += 1
            parent = model.joint(mimic.get("joint"))
            a, b = joint.id, parent.id
            offset, ratio = float(mimic.get("offset", 0)), float(mimic.get("multiplier", 1))
            matches = []
            for i in range(model.neq):
                if model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT and model.eq_obj1id[i] == a and model.eq_obj2id[i] == b:
                    # Equality polynomials operate about reference positions.
                    coefficient = offset - model.qpos0[joint.qposadr[0]] + ratio * model.qpos0[parent.qposadr[0]]
                    matches.append(np.allclose(model.eq_data[i, :5], [coefficient, ratio, 0, 0, 0], atol=1e-9))
            if not any(matches):
                raise InputError(f"MuJoCo mimic equality missing/changed: {name}; use a supported compiler version")
    return {"links": len(nominal.links), "movable_joints": len(expected), "independent_joints": len(nominal.independent),
        "mimic_equalities": mimic_count, "mass_com_inertia_axes_limits": "passed"}


def default_poses(nominal):
    middle = {}
    bounds = {}
    for name in nominal.independent:
        joint = nominal.joints[name]
        bounds[name] = [-1., 1.] if joint.get("type") == "continuous" else [float(joint.find("limit").get(k)) for k in ("lower", "upper")]
        middle[name] = sum(bounds[name]) / 2
    poses = [middle]
    for name in nominal.independent:
        for sign in (-1, 1):
            q = dict(middle)
            q[name] += sign * min(.05, (bounds[name][1] - bounds[name][0]) * .1)
            nominal.check_q(q)
            poses.append(q)
    return poses


def set_pose(nominal, model, data, q):
    import mujoco
    nominal.check_q(q)
    for name, value in nominal.positions(q).items():
        data.qpos[model.joint(name).qposadr[0]] = value
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


def compare_fk(nominal, model, poses, errors_csv):
    import mujoco
    data = mujoco.MjData(model)
    rows = []
    for index, q in enumerate(poses):
        set_pose(nominal, model, data, q)
        for name, expected in nominal.transforms(q).items():
            body = data.body(name)
            actual = np.eye(4)
            actual[:3, :3] = body.xmat.reshape(3, 3)
            actual[:3, 3] = body.xpos
            error = pose_error(expected, actual)
            rows.append({"pose": index, "link": name, "position_m": float(np.linalg.norm(error[:3])),
                "orientation_rad": float(np.linalg.norm(error[3:]))})
    if not rows:
        raise InputError("Provide at least one complete joint pose")
    write_csv(errors_csv, rows)
    position = max(r["position_m"] for r in rows)
    orientation = max(r["orientation_rad"] for r in rows)
    return {"status": "simulation_pass" if position < 1e-7 and orientation < 1e-7 else "failed",
        "poses": len(poses), "links_per_pose": len(nominal.links), "comparisons": len(rows),
        "max_position_m": position, "max_orientation_rad": orientation,
        "threshold_scope": "Numerical software tolerances only; not physical calibration accuracy"}


def physics_smoke(nominal, model):
    """Bounded local engine step, zero gravity/contact disabled, tiny test effort.

    No actuator/controller tuning or physical response equivalence is claimed.
    """
    import mujoco
    data = mujoco.MjData(model)
    set_pose(nominal, model, data, default_poses(nominal)[0])
    model.opt.gravity[:] = 0
    old_flags = model.opt.disableflags
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    before = data.qpos.copy()
    try:
        first = nominal.independent[0]
        data.qfrc_applied[model.joint(first).dofadr[0]] = .001
        for _ in range(100):
            mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise InputError("Nonfinite MuJoCo physics state")
        delta = float(np.max(np.abs(data.qpos - before)))
        if data.time <= 0 or delta <= 1e-10 or np.any(data.warning.number):
            raise InputError("MuJoCo stepping produced no finite response or numerical warnings")
        return {"status": "simulation_pass", "steps": 100, "duration_s": float(data.time),
            "applied_test_effort": .001, "max_position_change": delta,
            "gravity": [0, 0, 0], "contact_enabled": False, "hardware_control_commands_sent": False,
            "scope": "Local engine smoke test, not physical dynamics or trajectory validation"}
    finally:
        model.opt.disableflags = old_flags

"""Prepare an explicit OpenArm nominal-model test, without changing its workspace.

The reproduced vendor parameters are test inputs, not measured robot parameters.
Empty fixed frames are preserved in a sidecar; no fictitious mass is assigned.
"""
from copy import deepcopy
from pathlib import Path
import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.io import InputError, save, sha256
from core.model import Model, origin
from core.schemas import create_workbook, validate_parameters


def prepare(source, description_root, destination):
    source, package, output = Path(source).resolve(), Path(description_root).resolve(), Path(destination).resolve()
    if output.exists() and any(output.iterdir()):
        raise InputError("Choose a new OpenArm test destination")
    output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output / "source_model.urdf")
    original = Model(source)
    xml = deepcopy(original.xml)
    frames = []
    for name, link in original.links.items():
        if link.find("inertial") is not None:
            continue
        incident = [j for j in original.joints.values() if name in (j.find("parent").get("link"), j.find("child").get("link"))]
        if link.findall("visual") or link.findall("collision") or len(incident) != 1 or incident[0].get("type") != "fixed":
            raise InputError(f"Unsupported massless physical link; do not invent inertia: {name}")
        joint = incident[0]
        if name == original.root and not np.allclose(origin(joint.find("origin")), np.eye(4), atol=1e-12):
            raise InputError("This OpenArm test only removes an identity world mount")
        frames.append({"link": name, "fixed_joint_xml": ET.tostring(joint, encoding="unicode"),
                       "reason": "Empty fixed coordinate frame; excluded only from physical import model"})
        xml.remove(next(l for l in xml.findall("link") if l.get("name") == name))
        xml.remove(next(j for j in xml.findall("joint") if j.get("name") == joint.get("name")))
    for tag in list(xml):
        if tag.tag not in ("link", "joint", "material"):
            xml.remove(tag)
    mesh_hashes = {}
    for mesh in xml.findall(".//mesh"):
        uri = mesh.get("filename", "")
        prefix = "package://openarm_description/"
        if not uri.startswith(prefix):
            raise InputError(f"Expected an explicitly mapped OpenArm package URI: {uri}")
        path = (package / uri[len(prefix):]).resolve()
        if not path.is_relative_to(package) or not path.is_file():
            raise InputError(f"Missing package mesh: {uri}")
        if path.suffix.lower() == ".dae" and ET.parse(path).getroot().findall(".//{*}library_images/{*}image"):
            raise InputError("DAE textures need explicit dependency packaging before this test")
        relative = "meshes/" + sha256(path)[:12] + "_" + path.name
        target = output / relative
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(path, target)
        mesh.set("filename", relative)
        mesh_hashes[uri] = {"sha256": sha256(path), "local_path": relative}
    ET.indent(xml)
    baseline = output / "nominal_physical.urdf"
    ET.ElementTree(xml).write(baseline, encoding="utf-8", xml_declaration=True)
    model = Model(baseline)
    source_label = "OpenArm nominal URDF sha256:" + sha256(source) + "; SIMULATION TEST ONLY, not measured"
    records = {"robot_info": {"name": model.xml.get("name"), "units": "m rad kg s",
                 "validation_scope": "nominal_model_software_test_only"}, "links": [], "joints": [], "geometry": []}
    for name, link in model.links.items():
        inertia = link.find("inertial")
        pose = inertia.find("origin")
        attrs = inertia.find("inertia").attrib
        records["links"].append({"name": name, "mass": float(inertia.find("mass").get("value")),
            "com_xyz": pose.get("xyz", "0 0 0") if pose is not None else "0 0 0",
            "com_rpy": pose.get("rpy", "0 0 0") if pose is not None else "0 0 0",
            "inertia": " ".join(attrs[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
            "status": "confirmed", "source": source_label, "inertial_status": "confirmed", "inertial_source": source_label})
        records["geometry"].append({"target_link": name, "source_link": name, "origin_xyz": "0 0 0", "origin_rpy": "0 0 0"})
    for name, joint in model.joints.items():
        pose, axis, limit = joint.find("origin"), joint.find("axis"), joint.find("limit")
        row = {"name": name, "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
               "type": joint.get("type"), "origin_xyz": pose.get("xyz", "0 0 0") if pose is not None else "0 0 0",
               "origin_rpy": pose.get("rpy", "0 0 0") if pose is not None else "0 0 0",
               "status": "confirmed", "source": source_label}
        if axis is not None: row["axis"] = axis.get("xyz")
        if limit is not None:
            row.update({k: float(limit.get(k)) for k in ("lower", "upper", "velocity", "effort") if limit.get(k) is not None})
        mimic = joint.find("mimic")
        if mimic is not None:
            row.update(mimic_joint=mimic.get("joint"), mimic_multiplier=float(mimic.get("multiplier", 1)), mimic_offset=float(mimic.get("offset", 0)))
        records["joints"].append(row)
    checked = validate_parameters(records, require_inertia=True)
    if not checked["ready"]:
        raise InputError(f"OpenArm nominal model failed parameter validation: {checked}")
    save(output / "parameters.yaml", records)
    create_workbook(output / "parameters.xlsx", {**records, "robot_info": [{"key": k, "value": v} for k, v in records["robot_info"].items()]})
    save(output / "mechanisms.yaml", {"status": "confirmed", "source": source_label,
        "scope": "Reproduce the supplied nominal URDF tree and its mimic relationships only", "closed_loops": []})
    save(output / "isaac.yaml", {"expected_isaac_sim_version": "6.1.0", "physics_backend": "physx", "fix_base": True,
        "headless": True, "physics_dt": 1 / 240, "gravity": [0, 0, 0], "self_collision": False,
        "drives": {}, "test_scope": "Kinematic software validation; drives disabled, no dynamics calibration"})
    save(output / "acceptance.yaml", {"asset": {k: v for k, v in (
        ("limit_tolerance_rad_or_m", 1e-5), ("axis_tolerance_rad", 1e-4), ("anchor_tolerance_m", 1e-5),
        ("mass_tolerance_kg", 1e-5), ("com_tolerance_m", 1e-5), ("inertia_tolerance_kg_m2", 1e-6))}})
    inventory = {"source_urdf_sha256": sha256(source), "source_links": len(original.links), "source_joints": len(original.joints),
        "physics_links": len(model.links), "physics_joints": len(model.joints), "independent_joints": model.independent,
        "mimic_joints": [n for n in model.movable if model.joints[n].find("mimic") is not None],
        "excluded_empty_fixed_frames": frames, "mesh_sources": mesh_hashes, "data_kind": "nominal_model",
        "is_real_robot_acceptance": False, "hardware_commands_sent": False}
    save(output / "preparation.json", inventory)
    return inventory


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--urdf", required=True)
    p.add_argument("--description-root", required=True)
    p.add_argument("--destination", required=True)
    args = p.parse_args()
    info = prepare(args.urdf, args.description_root, args.destination)
    print(f"OpenArm nominal test prepared: {info['physics_links']} physical links, {len(info['independent_joints'])} independent joints; no real-robot acceptance")

from pathlib import Path
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation

from core.geometry import attach_geometry
from core.io import InputError, load, number, result, vector, sha256
from core.model import Model
from core.optimization import decode
from core.schemas import parameters, validate_parameters
from core.units_and_frames import transform


def text_vector(value, length):
    return " ".join(format(v, ".16g") for v in vector(value, length, "vector"))


def run(args):
    data = parameters(args.parameters)
    checked = validate_parameters(data, args.require_inertia)
    if checked["errors"] or (checked["pending"] and not args.draft):
        raise InputError(f"Model definition is not ready: {checked}")
    baseline = Model(args.baseline)
    mechanism = load(args.mechanisms) if args.mechanisms else {}
    mechanisms_confirmed = mechanism.get("status") == "confirmed" and bool(mechanism.get("source"))
    if not args.draft and not mechanisms_confirmed:
        raise InputError("A non-draft model requires confirmed mechanisms.yaml, including an explicit closed-loop decision")
    output_dir = Path(args.destination)
    output_dir.mkdir(parents=True, exist_ok=True)
    model_name = data.get("robot_info", {}).get("name", "robot")
    root = ET.Element("robot", name=str(model_name))
    links = {}
    for row in data["links"]:
        element = links[row["name"]] = ET.SubElement(root, "link", name=row["name"])
        if row.get("mass") is not None:
            inertia = ET.SubElement(element, "inertial")
            ET.SubElement(inertia, "origin", xyz=text_vector(row["com_xyz"], 3), rpy=text_vector(row["com_rpy"], 3))
            ET.SubElement(inertia, "mass", value=str(number(row["mass"])))
            ET.SubElement(inertia, "inertia", **dict(zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), text_vector(row["inertia"], 6).split())))
    for row in data.get("geometry", []):
        if row["target_link"] not in links or row["source_link"] not in baseline.links:
            raise InputError("Geometry mapping references an unknown link")
        attach_geometry(links[row["target_link"]], baseline.links[row["source_link"]],
                        transform(row["origin_xyz"], row["origin_rpy"]), baseline.path.parent, output_dir)
    offsets, deltas, calibration = {}, {}, None
    if args.calibration:
        calibration = load(args.calibration)
        if calibration.get("diagnostics", {}).get("identifiable") is not True:
            raise InputError("Only identifiable calibration results may update a model")
        _, _, offsets, deltas = decode(calibration["config"], calibration["specifications"], calibration["values"])
    for row in data["joints"]:
        name, kind = row["name"], row["type"]
        element = ET.SubElement(root, "joint", name=name, type=kind)
        ET.SubElement(element, "parent", link=row["parent"])
        ET.SubElement(element, "child", link=row["child"])
        pose = transform(row["origin_xyz"], row["origin_rpy"])
        if name in deltas:
            delta = deltas[name]
            pose = pose @ transform(delta[:3], delta[3:])
        ET.SubElement(element, "origin", xyz=text_vector(pose[:3, 3], 3), rpy=text_vector(Rotation.from_matrix(pose[:3, :3]).as_euler("xyz"), 3))
        if kind != "fixed":
            ET.SubElement(element, "axis", xyz=text_vector(row["axis"], 3))
            limits = {k: str(number(row[k])) for k in ("velocity", "effort")}
            if kind != "continuous":
                limits.update({k: str(number(row[k])) for k in ("lower", "upper")})
            ET.SubElement(element, "limit", **limits)
            dynamics = {k: str(number(row[k])) for k in ("damping", "friction") if row.get(k) is not None}
            if dynamics:
                ET.SubElement(element, "dynamics", **dynamics)
            if row.get("mimic_joint"):
                ET.SubElement(element, "mimic", joint=row["mimic_joint"],
                              multiplier=str(number(row["mimic_multiplier"])), offset=str(number(row["mimic_offset"])))
    ET.indent(root, space="  ")
    path = output_dir / "robot.urdf"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    Model(path)
    # Joint offsets belong in the feedback mapping, not also baked into origins.
    from core.io import save
    save(output_dir / "joint_offsets.yaml", {"convention": "q_model = q_feedback + offset", "offsets": offsets})
    if calibration:
        save(output_dir / "calibration.yaml", calibration)
    inertial_confirmed = all(r.get("inertial_status") == "confirmed" and r.get("inertial_source") for r in data["links"])
    save(output_dir / "model_manifest.yaml", {"urdf_sha256": sha256(path), "parameters_sha256": sha256(args.parameters),
         "baseline_sha256": sha256(args.baseline), "draft": args.draft, "definition_ready": checked["ready"],
         "inertial_parameters_confirmed": bool(inertial_confirmed), "mechanisms": mechanism,
         "joint_offsets": offsets})
    return result("build-urdf", {"urdf": str(path), "draft": args.draft,
                               "inertial_parameters_confirmed": bool(inertial_confirmed),
                               "pending": checked["pending"], "joint_offsets": offsets,
                               "base_tool_corrections": "calibration.yaml; use in the scene/TF, not twice in joint feedback"},
                  [args.parameters, args.baseline, args.calibration, args.mechanisms], "draft" if args.draft else "built")

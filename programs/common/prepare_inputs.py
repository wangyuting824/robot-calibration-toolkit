"""Extract supplied model and make an explicitly unconfirmed workbook."""
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET

from core.io import InputError, result, save, sha256
from core.model import Model
from core.schemas import create_workbook


def run(args):
    target = Path(args.destination)
    if Path(args.workbook).exists():
        raise InputError("Workbook already exists; choose a new path to preserve existing parameter edits")
    if target.exists() and any(target.iterdir()):
        raise InputError("Destination must be empty; existing work is never overwritten")
    target.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.archive) as archive:
        if sum(info.file_size for info in archive.infolist()) > 2_000_000_000:
            raise InputError("Archive is larger than the 2 GB extraction limit")
        for info in archive.infolist():
            path = (target / info.filename).resolve()
            if not path.is_relative_to(target.resolve()) or (info.external_attr >> 16) & 0o170000 == 0o120000:
                raise InputError("Archive contains an unsafe path or symbolic link")
        archive.extractall(target)
    files = list(target.rglob("*.urdf"))
    if len(files) != 1:
        raise InputError("Expected exactly one baseline URDF")
    model = Model(files[0])
    records = {"robot_info": [{"key": "name", "value": model.xml.get("name")},
                               {"key": "units", "value": "m rad kg s"}], "links": [], "joints": [], "geometry": []}
    for name, element in model.links.items():
        # Preserve baseline values as unconfirmed; do not infer real masses.
        item = {"name": name, "status": "unconfirmed", "source": "baseline_test_defaults",
                "inertial_status": "unconfirmed", "inertial_source": "baseline_test_defaults"}
        inertia = element.find("inertial")
        if inertia is not None:
            o = inertia.find("origin")
            item.update(mass=float(inertia.find("mass").get("value")),
                        com_xyz=o.get("xyz", "0 0 0") if o is not None else "0 0 0",
                        com_rpy=o.get("rpy", "0 0 0") if o is not None else "0 0 0",
                        inertia=" ".join(inertia.find("inertia").get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
        records["links"].append(item)
        records["geometry"].append({"target_link": name, "source_link": name, "origin_xyz": "0 0 0", "origin_rpy": "0 0 0"})
    for name, element in model.joints.items():
        o, axis, limits = element.find("origin"), element.find("axis"), element.find("limit")
        item = {"name": name, "parent": element.find("parent").get("link"), "child": element.find("child").get("link"),
                "type": element.get("type"), "origin_xyz": o.get("xyz", "0 0 0") if o is not None else "0 0 0",
                "origin_rpy": o.get("rpy", "0 0 0") if o is not None else "0 0 0",
                "axis": axis.get("xyz") if axis is not None else None,
                "status": "unconfirmed", "source": "baseline_test_defaults"}
        if limits is not None:
            for key in ("lower", "upper", "velocity", "effort"):
                item[key] = float(limits.get(key)) if limits.get(key) is not None else None
        mimic = element.find("mimic")
        if mimic is not None:
            item.update(mimic_joint=mimic.get("joint"), mimic_multiplier=float(mimic.get("multiplier", 1)),
                        mimic_offset=float(mimic.get("offset", 0)))
        records["joints"].append(item)
    create_workbook(args.workbook, records)
    return result("prepare-inputs", {"baseline_urdf": str(files[0]), "workbook": str(args.workbook),
                                    "confirmed": False, "missing_structure_workbook": "STEP候选连杆与连接关系确认.xlsx"}, [args.archive])

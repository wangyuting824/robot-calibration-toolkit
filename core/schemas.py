"""Workbook schema and physical admissibility checks; no guessed unknown values."""
from pathlib import Path
import numpy as np
from openpyxl import Workbook, load_workbook

from core.io import InputError, load, number, vector

SHEETS = {
    "robot_info": ["key", "value"],
    "links": ["name", "mass", "com_xyz", "com_rpy", "inertia", "status", "source", "inertial_status", "inertial_source"],
    "joints": ["name", "k_id", "parent", "child", "type", "origin_xyz", "origin_rpy", "axis", "lower", "upper", "velocity", "effort", "damping", "friction", "mimic_joint", "mimic_multiplier", "mimic_offset", "status", "source"],
    "geometry": ["target_link", "source_link", "origin_xyz", "origin_rpy"],
    "frames": ["name", "parent", "xyz", "rpy", "status", "source"],
    "actuators": ["joint", "device_id", "encoder_location", "raw_unit", "scale", "offset", "control_mode", "status", "source"],
    "mechanisms": ["name", "type", "description", "status", "source"],
    "contacts": ["name", "description", "status", "source"],
    "sources": ["id", "file_or_url", "revision", "description"],
}


def create_workbook(path, records=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    book = Workbook()
    book.remove(book.active)
    for name, fields in SHEETS.items():
        sheet = book.create_sheet(name)
        sheet.append(fields)
        for row in (records or {}).get(name, []):
            sheet.append([row.get(field) for field in fields])
        sheet.freeze_panes = "A2"
        sheet.auto_filter.ref = sheet.dimensions
        for column in sheet.columns:
            sheet.column_dimensions[column[0].column_letter].width = 22
    book.save(path)


def parameters(path):
    if Path(path).suffix != ".xlsx":
        return load(path)
    book = load_workbook(path, read_only=True, data_only=False)
    output = {}
    for name, fields in SHEETS.items():
        if name not in book.sheetnames:
            raise InputError(f"Workbook missing sheet: {name}")
        rows = list(book[name].values)
        if not rows or list(rows[0]) != fields:
            raise InputError(f"Workbook headers differ: {name}")
        output[name] = []
        for values in rows[1:]:
            if all(v is None for v in values):
                continue
            if any(isinstance(v, str) and v.startswith("=") for v in values):
                raise InputError("Formula cells are not accepted as measured values")
            output[name].append(dict(zip(fields, values)))
    output["robot_info"] = {r["key"]: r["value"] for r in output["robot_info"]}
    return output


def inertia_matrix(values):
    xx, yy, zz, xy, xz, yz = vector(values, 6, "inertia xx yy zz xy xz yz")
    matrix = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    eigen = np.linalg.eigvalsh(matrix)
    if eigen[0] <= 0 or eigen[-1] > eigen[0] + eigen[1] + 1e-9:
        raise InputError("Inertia must be positive definite and satisfy the principal-moment triangle inequality")
    return matrix


def validate_parameters(data, require_inertia=False):
    errors, pending = [], []
    if data.get("robot_info", {}).get("units") != "m rad kg s":
        errors.append("robot_info.units must explicitly declare 'm rad kg s'")
    links, joints = data.get("links", []), data.get("joints", [])
    if not links or not joints:
        errors.append("links/joints must not be empty")
    for kind, rows in (("link", links), ("joint", joints)):
        names = [row.get("name") for row in rows]
        if len(set(names)) != len(names) or any(not n for n in names):
            errors.append(f"Duplicate/missing {kind} names")
        for row in rows:
            if row.get("status") != "confirmed" or not row.get("source"):
                pending.append(f"{kind}:{row.get('name')}: status/source unconfirmed")
    known = {r.get("name") for r in links}
    children, edges = {}, {}
    for row in joints:
        name = row.get("name")
        try:
            parent, child = row.get("parent"), row.get("child")
            if parent not in known or child not in known or parent == child or child in children:
                raise InputError("Invalid parent/child or multiple parents")
            children[child] = parent
            edges.setdefault(parent, []).append(child)
            vector(row.get("origin_xyz"), 3, "origin_xyz")
            vector(row.get("origin_rpy"), 3, "origin_rpy")
            kind = row.get("type")
            if kind not in ("fixed", "revolute", "prismatic", "continuous"):
                raise InputError("Unsupported joint type")
            if kind != "fixed":
                if abs(np.linalg.norm(vector(row.get("axis"), 3, "axis")) - 1) > 1e-6:
                    raise InputError("Axis is not a unit vector")
                if kind != "continuous" and number(row.get("lower")) >= number(row.get("upper")):
                    raise InputError("Invalid lower/upper limits")
                if number(row.get("velocity")) <= 0 or number(row.get("effort")) <= 0:
                    raise InputError("Velocity/effort must be positive")
        except InputError as exc:
            errors.append(f"joint:{name}: {exc}")
    roots = known - set(children)
    if len(roots) != 1:
        errors.append("URDF definition must have exactly one root; physical loops go in mechanisms.yaml")
    else:
        visited = set()
        stack = list(roots)
        while stack:
            current = stack.pop()
            if current in visited:
                errors.append("Cycle in URDF definition")
                break
            visited.add(current)
            stack.extend(edges.get(current, []))
        if visited != known:
            errors.append("Disconnected links or cycle")
    for row in links:
        try:
            if require_inertia and (row.get("inertial_status") != "confirmed" or not row.get("inertial_source")):
                pending.append(f"link:{row.get('name')}: inertial parameters lack separate confirmation/source")
            has_mass = row.get("mass") is not None
            if require_inertia or has_mass:
                if number(row.get("mass")) <= 0:
                    raise InputError("Mass must be positive")
                vector(row.get("com_xyz"), 3, "com_xyz")
                vector(row.get("com_rpy"), 3, "com_rpy")
                inertia_matrix(row.get("inertia"))
        except InputError as exc:
            errors.append(f"link:{row.get('name')}: {exc}")
    named_joints = {row.get("name"): row for row in joints}
    for row in joints:
        if row.get("mimic_joint"):
            current, visited = row, set()
            try:
                number(row.get("mimic_multiplier"))
                number(row.get("mimic_offset"))
                while current.get("mimic_joint"):
                    if current["name"] in visited or current.get("mimic_joint") not in named_joints:
                        raise InputError("Unknown/cyclic mimic leader")
                    visited.add(current["name"])
                    current = named_joints[current["mimic_joint"]]
                    if current["type"] == "fixed":
                        raise InputError("Mimic leader must be movable")
            except InputError as exc:
                errors.append(f"joint:{row.get('name')}: {exc}")
    return {"errors": errors, "pending": pending, "ready": not errors and not pending,
            "links": len(links), "joints": len(joints),
            "movable_joints": sum(r.get("type") != "fixed" for r in joints)}

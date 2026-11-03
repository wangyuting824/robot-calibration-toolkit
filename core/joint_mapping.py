"""Position-coordinate conversion with explicit provenance and no drive access.

Affine position/velocity semantics follow ros2_control SimpleTransmission.
This implementation never infers a sign from range membership, maps effort,
clips values, extrapolates, or claims physical acceptance from a candidate.
"""
from pathlib import Path

from core.io import InputError, load, number, sha256, evidence_paths
from core.model import Model


def checked_evidence(entry, relative_to):
    if not isinstance(entry, dict) or not entry.get("path") or not entry.get("sha256"):
        raise InputError("Evidence requires path and SHA-256")
    path = (Path(relative_to) / entry["path"]).resolve()
    if not path.is_file() or sha256(path) != entry["sha256"]:
        raise InputError(f"Missing or changed evidence: {path}")
    return path


class JointMapping:
    def __init__(self, profile_path, urdf, allow_candidate=False):
        self.path = Path(profile_path).resolve()
        self.profile = load(self.path)
        self.model = Model(urdf)
        p = self.profile
        if p.get("kind") != "joint_feedback_mapping" or p.get("schema_version") != 1:
            raise InputError("Expected schema_version=1 joint_feedback_mapping")
        if p.get("urdf_sha256") != sha256(urdf):
            raise InputError("Mapping was built for a different URDF")
        for key in ("robot_id", "input_coordinates_id", "source"):
            if not isinstance(p.get(key), str) or not p[key].strip():
                raise InputError(f"Mapping requires {key}")
        if p.get("unmapped_policy") != "preserve_unverified":
            raise InputError("Explicit unmapped_policy=preserve_unverified is required")
        self.qualified = False
        validation = None
        if p.get("status") == "validated_reference":
            report = checked_evidence(p.get("validation"), self.path.parent)
            validation = load(report)
            if validation.get("command") != "validate-joint-reference" or validation.get("status") not in ("passed", "synthetic_pass", "simulation_pass"):
                raise InputError("Mapping requires a passing independent joint-reference validation")
            if not validation.get("inputs") or validation.get("urdf_sha256") != p["urdf_sha256"]:
                raise InputError("Validation evidence does not bind this URDF")
            evidence_paths(validation, report)
            self.qualified = validation["status"] == "passed" and validation.get("training_data_kind") == "real" and validation.get("dataset", {}).get("data_kind") == "real"
            for key in ("robot_id", "input_coordinates_id"):
                if validation.get(key) != p[key]:
                    raise InputError(f"Mapping/validation {key} differs")
        elif p.get("status") != "candidate":
            raise InputError("Mapping status must be candidate or validated_reference")
        if not self.qualified and not allow_candidate:
            raise InputError("Candidate/synthetic mapping requires explicit --allow-candidate; simulation use only")
        self.entries = {}
        targets = set()
        for entry in p.get("joints", []):
            source, target = entry.get("source_joint"), entry.get("target_joint")
            if not source or source in self.entries or target in targets or target not in self.model.independent:
                raise InputError("Mapping needs unique source/target names and independent URDF targets")
            scale, offset = number(entry.get("scale")), number(entry.get("offset"))
            if abs(scale) < 1e-12:
                raise InputError("Mapping scale cannot be zero")
            unit = "m" if self.model.joints[target].get("type") == "prismatic" else "rad"
            if entry.get("raw_unit") not in ("rad", "m") or entry.get("reference_unit") != unit:
                raise InputError("Declare SI source units and the correct URDF target unit")
            domain = entry.get("valid_raw_range")
            if not isinstance(domain, list) or len(domain) != 2 or number(domain[0]) >= number(domain[1]):
                raise InputError("Declare an ordered valid_raw_range; extrapolation is unsupported")
            if validation is not None:
                accepted = validation.get("joints", {}).get(source)
                fields = ("source_joint", "target_joint", "raw_unit", "reference_unit", "valid_raw_range", "scale", "offset")
                if accepted is None or any(entry.get(k) != accepted.get(k) for k in fields):
                    raise InputError("Mapping coefficients/domain differ from validated result")
            self.entries[source] = dict(entry, scale=scale, offset=offset)
            targets.add(target)
        if not self.entries:
            raise InputError("Mapping has no joint entries")
        if validation is not None and set(self.entries) != set(validation.get("joints", {})):
            raise InputError("Mapping joint coverage differs from its validation")

    def metadata(self, meta):
        if meta.get("mapping_profile_sha256") or meta.get("mapping_applied"):
            raise InputError("Feedback is already mapped; refusing double application")
        for key in ("robot_id", "input_coordinates_id"):
            source_key = "coordinates_id" if key == "input_coordinates_id" else key
            if meta.get(source_key) != self.profile[key]:
                raise InputError(f"Feedback {source_key} does not match mapping input")
        units = meta.get("joint_units", {})
        if any(units.get(name) != e["raw_unit"] for name, e in self.entries.items()):
            raise InputError("Feedback joint_units do not match mapping source units")
        output_units = dict(units)
        for name, e in self.entries.items():
            output_units.pop(name, None)
            output_units[e["target_joint"]] = e["reference_unit"]
        kind = meta["data_kind"] if self.qualified else "synthetic" if meta["data_kind"] == "synthetic" else "simulation"
        return {**meta, "dataset_id": meta["dataset_id"] + "-mapped", "origin_source": meta["source"],
            "origin_data_kind": meta["data_kind"], "data_kind": kind,
            "coordinates_id": "urdf:" + self.profile["urdf_sha256"], "joint_units": output_units,
            "mapping_applied": True, "mapping_profile_sha256": sha256(self.path),
            "mapping_profile": str(self.path), "mapping_reference_validated": self.qualified,
            "joint_mapping_verified": False, "is_real_robot_acceptance": False,
            "scope": "Selected position coordinates converted; unmapped joints remain unverified; effort is not converted",
            "hardware_control_commands_sent": False}

    def row(self, row):
        if "q_source" in row or row.get("mapping_applied") not in (None, "", False, "False", "false"):
            raise InputError("Feedback row is already mapped")
        name, q = row.get("joint"), number(row.get("q"))
        if name not in self.entries and name in {e["target_joint"] for e in self.entries.values()}:
            raise InputError("Unmapped source name would collide with a mapped target")
        out = {**row, "source_joint": name, "q_source": q,
            "dq_source": row.get("dq", ""), "effort_source": row.get("effort", ""),
            "mapping_applied": name in self.entries, "joint_mapping_verified": False}
        if name not in self.entries:
            return out
        entry = self.entries[name]
        lower, upper = map(number, entry["valid_raw_range"])
        if not lower <= q <= upper:
            raise InputError(f"{name}: feedback outside mapping domain; no extrapolation")
        value = entry["scale"] * q + entry["offset"]
        joint = self.model.joints[entry["target_joint"]]
        if joint.get("type") != "continuous":
            limits = joint.find("limit")
            if not number(limits.get("lower")) <= value <= number(limits.get("upper")):
                raise InputError(f"{name}: converted position outside URDF limits; no clipping")
        out.update(joint=entry["target_joint"], q=value,
            dq=entry["scale"] * number(row["dq"]) if row.get("dq") not in (None, "") else "",
            effort="", joint_mapping_verified=self.qualified, effort_reference_calibrated=False)
        return out

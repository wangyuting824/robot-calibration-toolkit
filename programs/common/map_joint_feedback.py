from pathlib import Path

from core.io import InputError, dataset, result, save, sha256, write_csv
from core.joint_mapping import JointMapping


def run(args):
    if Path(args.output_data).resolve() == Path(args.output_manifest).resolve():
        raise InputError("Dataset and manifest must use different output paths")
    if Path(args.output_data).exists() or Path(args.output_manifest).exists():
        raise InputError("Choose new output paths; raw/mapped records must not be overwritten")
    rows, meta = dataset(args.data, args.manifest, ("sequence", "joint", "q"))
    if not meta.get("sha256"):
        raise InputError("Feedback manifest requires SHA-256")
    mapping = JointMapping(args.mapping, args.urdf, args.allow_candidate)
    converted_meta = mapping.metadata(meta)
    converted = [mapping.row(row) for row in rows]
    if set(mapping.entries) - {row["joint"] for row in rows}:
        raise InputError("Dataset is missing configured source joints")
    seen = set()
    for row in converted:
        key = (row["sequence"], row["joint"])
        if key in seen:
            raise InputError("Mapping produced duplicate joints in a message")
        seen.add(key)
    write_csv(args.output_data, converted)
    converted_meta.update(dataset_id=meta["dataset_id"] + "-mapped", sha256=sha256(args.output_data),
        source_dataset_sha256=sha256(args.data), source_manifest_sha256=sha256(args.manifest))
    save(args.output_manifest, converted_meta)
    return result("map-joint-feedback", {"rows": len(converted), "dataset": converted_meta,
        "mapping_reference_validated": mapping.qualified, "hardware_control_commands_sent": False,
        "is_real_robot_acceptance": False}, [args.data, args.manifest, args.mapping, args.urdf],
        "computed" if mapping.qualified else "simulation_pass")

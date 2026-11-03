"""Package explicitly listed artifacts. Final delivery requires current real-data acceptance."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
from core.io import InputError, load, result, save, sha256, evidence_paths


def run(args):
    config = load(args.config)
    reports = [load(p) for p in config.get("reports", [])]
    by_command = {}
    for report in reports:
        by_command.setdefault(report.get("command"), []).append(report)
    if args.final:
        required = ["validate-structure", "build-urdf", "verify-joint-limits", "validate-kinematics"]
        if config.get("stage") == "dynamics":
            required += ["validate-isaac-asset", "validate-dynamics"]
        for command in required:
            items = by_command.get(command, [])
            accepted = [r for r in items if r.get("status") in ("ready", "built", "passed") and not r.get("draft")]
            if not accepted:
                raise InputError(f"Final delivery requires an accepted result: {command}")
            if command in ("validate-kinematics", "verify-joint-limits"):
                if not any(r.get("dataset", {}).get("data_kind") == "real" for r in accepted):
                    raise InputError(f"Final delivery requires real measurements: {command}")
            if command == "validate-kinematics" and not any(r.get("training_data_kind") == "real" for r in accepted):
                raise InputError("Final calibrated-model delivery requires traceable real calibration observations")
            if command == "validate-dynamics":
                required_metrics = set(config.get("required_dynamics_metrics", []))
                if not required_metrics:
                    raise InputError("Declare required_dynamics_metrics for final dynamics delivery")
                if not any(required_metrics <= set(r.get("required_metrics", [])) and r.get("real_dataset", {}).get("data_kind") == "real" for r in accepted):
                    raise InputError("Dynamics evidence does not cover the declared real-data acceptance scope")
        for report, path in zip(reports, config.get("reports", [])):
            evidence_paths(report, path)
    destination = Path(args.destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise InputError("Delivery archive already exists; select a new versioned name")
    artifacts = config.get("artifacts", [])
    categories = {item.get("delivery") for item in artifacts}
    if categories != {"D01", "D02", "D03", "D04", "D05", "D06"}:
        raise InputError("Explicitly list artifacts for all D01–D06 categories (draft D03 may contain readiness evidence)")
    entries, archive_names = [], set()
    for item in artifacts:
        original = Path(item["path"])
        source = original.resolve()
        if not source.exists() or original.is_symlink():
            raise InputError(f"Missing/unsafe artifact: {source}")
        files = [source] if source.is_file() else sorted(p for p in source.rglob("*") if p.is_file())
        if not files:
            raise InputError(f"Empty delivery category: {item['delivery']}")
        for file in files:
            if file.is_symlink() or file.suffix in (".pyc",) or "__pycache__" in file.parts:
                continue
            relative = Path(item.get("name", source.name)) if source.is_file() else Path(item.get("name", source.name)) / file.relative_to(source)
            if relative.is_absolute() or ".." in relative.parts:
                raise InputError("Unsafe archive destination name")
            name = (Path(item["delivery"]) / relative).as_posix()
            if name in archive_names:
                raise InputError(f"Duplicate archive path: {name}")
            archive_names.add(name)
            entries.append((file, name))
    manifest = result("package-delivery", {"delivery_mode": "final" if args.final else "draft",
                      "stage": config.get("stage"), "files": [{"path": n, "sha256": sha256(p)} for p, n in entries]},
                      [args.config, *config.get("reports", [])], "packaged")
    with ZipFile(destination, "w", ZIP_DEFLATED) as archive:
        for path, name in entries:
            archive.write(path, name)
        import json
        archive.writestr("MANIFEST.json", json.dumps(manifest, ensure_ascii=False, indent=2))
    manifest["archive"] = str(destination)
    return manifest

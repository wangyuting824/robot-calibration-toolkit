"""Strict, portable input/output and run provenance."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml


class InputError(ValueError):
    """An input or a required capability is missing or inconsistent."""


def load(path):
    path = Path(path)
    if not path.is_file():
        raise InputError(f"Missing file: {path}")
    try:
        with path.open(encoding="utf-8") as handle:
            data = json.load(handle) if path.suffix == ".json" else yaml.safe_load(handle)
    except (json.JSONDecodeError, yaml.YAMLError) as exc:
        raise InputError(f"Invalid JSON/YAML: {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise InputError(f"Expected a mapping: {path}")
    return data


def native(value):
    if isinstance(value, np.ndarray):
        return native(value.tolist())
    if isinstance(value, np.generic):
        return native(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        raise InputError("Non-finite output is not a valid result")
    if isinstance(value, dict):
        return {str(k): native(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [native(v) for v in value]
    return value


def save(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = native(data)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        if path.suffix == ".json":
            json.dump(data, handle, ensure_ascii=False, indent=2, allow_nan=False)
        else:
            yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
    temporary.replace(path)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def csv_rows(path, required=()):
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or any(not name or not name.strip() for name in reader.fieldnames) or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise InputError(f"CSV headers must be nonempty and unique: {path}")
        missing = set(required) - set(reader.fieldnames or [])
        if missing:
            raise InputError(f"Missing CSV columns: {sorted(missing)}")
        rows = list(reader)
        if any(None in row or any(value is None for value in row.values()) for row in rows):
            raise InputError(f"CSV row length differs from header: {path}")
    if not rows:
        raise InputError(f"Empty dataset: {path}")
    return rows


def write_csv(path, rows):
    rows = list(rows)
    if not rows:
        raise InputError("Cannot export an empty dataset")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def number(value, label="value"):
    if isinstance(value, bool):
        raise InputError(f"{label}: boolean is not a number")
    try:
        result = float(value)
    except (ValueError, TypeError) as exc:
        raise InputError(f"{label}: expected a finite number, got {value!r}") from exc
    if not math.isfinite(result):
        raise InputError(f"{label}: must be finite")
    return result


def vector(value, length, label):
    if isinstance(value, str):
        value = value.split()
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != length:
        raise InputError(f"{label}: expected {length} components")
    return np.array([number(v, label) for v in value])


def dataset(path, manifest_path, required=(), role=None):
    meta = load(manifest_path)
    for key in ("dataset_id", "role", "data_kind", "units", "clock", "source"):
        if not meta.get(key):
            raise InputError(f"Dataset manifest is missing {key}")
    if meta["units"] != {"length": "m", "angle": "rad", "time": "s", "effort": "N_or_Nm"}:
        raise InputError("Normalize dataset units to m, rad, s, N_or_Nm before use")
    if meta["data_kind"] not in ("synthetic", "real", "simulation"):
        raise InputError("data_kind must be synthetic, real, or simulation")
    if meta["role"] not in ("calibration", "validation"):
        raise InputError("Dataset role must be calibration or validation")
    if role and meta["role"] != role:
        raise InputError(f"Expected dataset role={role}, got {meta['role']}")
    if meta.get("sha256") and sha256(path) != meta["sha256"]:
        raise InputError("Dataset hash does not match its manifest")
    return csv_rows(path, required), meta


def result(command, payload, paths=(), status="computed"):
    from core import __version__
    return {
        "schema_version": 1,
        "toolkit_version": __version__,
        "command": command,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "inputs": [{"path": str(p), "sha256": sha256(p)} for p in paths if p and Path(p).is_file()],
        **payload,
    }


def portable_inputs(report, report_path):
    """Anchor new evidence to its report, never to a workstation or caller cwd."""
    parent = Path(report_path).resolve().parent
    report["inputs"] = [dict(e, path=os.path.relpath(Path(e["path"]).resolve(), parent))
                        for e in report.get("inputs", [])]
    report["input_path_base"] = "report_directory"
    return report


def evidence_paths(report, report_path, verify=True):
    """Read new relative evidence; retain legacy cwd/absolute evidence semantics."""
    parent = Path(report_path).resolve().parent if report.get("input_path_base") == "report_directory" else Path.cwd()
    paths = []
    for entry in report.get("inputs", []):
        if not isinstance(entry, dict) or not entry.get("path") or not entry.get("sha256"):
            raise InputError("Evidence requires path and SHA-256")
        path = (parent / entry["path"]).resolve()
        if verify and (not path.is_file() or sha256(path) != entry["sha256"]):
            raise InputError(f"Missing or changed evidence: {path}")
        paths.append(path)
    return paths

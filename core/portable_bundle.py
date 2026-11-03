"""Build and verify a relocatable Linux bundle from an explicitly staged runtime."""
from datetime import datetime, timezone
from pathlib import Path
import json
import os
import shutil
import subprocess
import tarfile

from core.io import InputError, sha256

SOURCE_DIRS = ("core", "programs", "adapters", "configs", "templates", "examples", "docs", "tests", ".github")
SOURCE_FILES = ("field.py", "run.py", "pyproject.toml", "README.md", "LICENSE", "启动工具包.sh", "离线使用先读.html")


def allowed(path):
    return not path.is_symlink() and "__pycache__" not in path.parts and path.suffix not in (".pyc", ".pyo") and path.name != ".machine.json"


def build_bundle(root, destination, project=None):
    runtime = root/"runtime/python/bin/python3"
    if not runtime.is_file() or not (root/"wheelhouse").is_dir():
        raise InputError("缺少完整runtime/python与wheelhouse；先运行 docs 中的准备离线环境步骤")
    if destination.exists() or destination.is_relative_to(root/"runtime"):
        raise InputError("请选择一个不存在的工具包目录")
    destination.mkdir(parents=True)
    for name in SOURCE_DIRS:
        shutil.copytree(root/name, destination/name, ignore=lambda p, names: [n for n in names if n == "__pycache__" or n.endswith((".pyc", ".pyo"))])
    for name in SOURCE_FILES:
        if (root/name).is_file():
            shutil.copy2(root/name, destination/name)
    for name in ("runtime", "wheelhouse"):
        for p in (root/name).rglob("*"):
            if p.is_symlink() and not p.resolve().is_relative_to((root/name).resolve()):
                raise InputError("运行环境含外部符号链接，不能制作自包含工具包")
        # Materialize internal aliases so USB filesystems need not support symlinks.
        shutil.copytree(root/name, destination/name, symlinks=False)
    if project:
        shutil.copytree(project, destination/"projects"/project.name,
                        ignore=lambda p, names: [n for n in names if n == ".machine.json" or n == "__pycache__"])
    (destination/"启动工具包.sh").chmod(0o755)
    manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "target": "Ubuntu Linux x86_64; runtime and wheels included; Isaac/ROS/manufacturer drivers external",
        "private_project_included": bool(project), "files": [], "symlinks": []}
    for p in sorted(destination.rglob("*")):
        name = p.relative_to(destination).as_posix()
        if p.is_symlink():
            if not p.resolve().is_relative_to(destination):
                raise InputError(f"运行环境含外部符号链接：{name}")
            manifest["symlinks"].append({"path": name, "target": os.readlink(p)})
        elif p.is_file():
            manifest["files"].append({"path": name, "sha256": sha256(p), "bytes": p.stat().st_size})
    (destination/"MANIFEST.sha256.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    print(f"工具包已生成：{destination}；共 {len(manifest['files'])} 个文件。请运行 check-bundle 与 self-test。")
    return 0


def check_bundle(root):
    manifest = json.loads((root/"MANIFEST.sha256.json").read_text(encoding="utf-8"))
    failures = []
    for item in manifest["files"]:
        p = root/item["path"]
        if not p.is_file() or sha256(p) != item["sha256"]:
            failures.append(item["path"])
    for item in manifest.get("symlinks", []):
        p = root/item["path"]
        if not p.is_symlink() or os.readlink(p) != item["target"] or not p.resolve().is_relative_to(root.resolve()):
            failures.append(item["path"])
    print(json.dumps({"status": "failed" if failures else "verified", "checked_files": len(manifest["files"]),
        "failures": failures, "note": "清单仅验证交接文件完整性，不证明物理参数正确。用户编辑过的项目文件与清单不符属预期，应另存新版本。"}, ensure_ascii=False, indent=2))
    return 2 if failures else 0

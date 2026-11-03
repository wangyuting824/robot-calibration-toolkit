"""Run online once to stage independent CPython and a complete offline wheelhouse.

Use a downloaded official python-build-standalone install_only archive. This
script never copies a Conda/venv environment or the user's Isaac/ROS workspace.
"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def run(archive):
    runtime = ROOT/"runtime"
    if runtime.exists():
        raise RuntimeError("runtime exists; preserve the existing release and use a fresh checkout")
    runtime.mkdir()
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            path = (runtime/member.name).resolve()
            if not path.is_relative_to(runtime.resolve()) or member.isdev():
                raise RuntimeError("Unsafe Python archive")
            if member.issym() and not (path.parent/member.linkname).resolve().is_relative_to(runtime.resolve()):
                raise RuntimeError("External runtime symlink")
        tar.extractall(runtime, filter="data")
    python = runtime/"python/bin/python3"
    wheelhouse = ROOT/"wheelhouse"
    wheelhouse.mkdir(exist_ok=True)
    requirements = ["numpy>=1.24,<3", "scipy>=1.10,<2", "PyYAML>=6,<7", "openpyxl>=3.1,<4",
        "pytest>=8,<10", "mujoco>=3.17,<4", "trimesh>=4.12,<5", "pycollada>=0.9,<1", "Pillow>=10", "usd-core>=24"]
    env = dict(os.environ, PYTHONNOUSERSITE="1")
    env.pop("PYTHONPATH",None)
    env.pop("PYTHONHOME",None)
    subprocess.run([str(python), "-m", "pip", "download", "--only-binary=:all:", "--dest", str(wheelhouse), *requirements], env=env, check=True)
    subprocess.run([str(python), "-m", "pip", "install", "--no-index", "--find-links", str(wheelhouse), *requirements], env=env, check=True)
    installed = subprocess.check_output([str(python), "-m", "pip", "freeze"], env=env, text=True)
    (runtime/"requirements-lock.txt").write_text(installed,encoding="utf-8")
    from hashlib import sha256
    (runtime/"RUNTIME_SOURCE.json").write_text(json.dumps({"provider": "Astral python-build-standalone",
        "official": "https://github.com/astral-sh/python-build-standalone/releases", "archive": Path(archive).name,
        "archive_sha256": sha256(Path(archive).read_bytes()).hexdigest(), "target": "Linux x86_64 CPython3.12",
        "isaac_included": False}, indent=2),encoding="utf-8")
    print("Offline runtime and wheelhouse prepared. Build the bundle using field.py build-bundle.")


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--python-archive",required=True)
    run(p.parse_args().python_archive)

"""Reassign supplied geometry using explicit T_target_source transforms.

STEP contains geometry, not verified joint definitions. This module consumes
the existing tessellated package, or explicit mesh files exported by CAD tools;
it never guesses joint axes or reconstructs missing CAD assembly constraints.
"""
from copy import deepcopy
from pathlib import Path
import hashlib
import shutil
import xml.etree.ElementTree as ET

from scipy.spatial.transform import Rotation
from core.io import InputError
from core.model import origin
from core.units_and_frames import transform


def attach_geometry(target, source, placement, source_dir, output_dir):
    source_dir = Path(source_dir).resolve()
    for kind in ("visual", "collision"):
        for item in source.findall(kind):
            element = deepcopy(item)
            composed = placement @ origin(element.find("origin"))
            old = element.find("origin")
            if old is not None:
                element.remove(old)
            ET.SubElement(element, "origin", xyz=" ".join(map(str, composed[:3, 3])),
                          rpy=" ".join(map(str, Rotation.from_matrix(composed[:3, :3]).as_euler("xyz"))))
            for mesh in element.findall(".//mesh"):
                filename = mesh.get("filename", "")
                if "://" in filename:
                    raise InputError("Resolve package:// or remote mesh URIs before building")
                source_path = (Path(source_dir) / filename).resolve()
                if not source_path.is_relative_to(Path(source_dir).resolve()) or not source_path.is_file():
                    raise InputError(f"Invalid mesh reference: {filename}")
                name = hashlib.sha256(str(source_path.relative_to(source_dir)).encode()).hexdigest()[:12] + "_" + source_path.name
                destination = Path(output_dir) / "meshes" / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source_path, destination)
                mesh.set("filename", "meshes/" + name)
            target.append(element)

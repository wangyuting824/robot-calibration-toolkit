"""URDF tree kinematics. Semantics follow ROS URDF; this is project code, not urdfdom."""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from core.io import InputError, number, vector
from core.units_and_frames import transform


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(element.get("xyz", "0 0 0"), element.get("rpy", "0 0 0"))


class Model:
    def __init__(self, path):
        self.path = Path(path)
        text = self.path.read_text(encoding="utf-8")
        if "<!DOCTYPE" in text or "<!ENTITY" in text:
            raise InputError("DTD and XML entities are unsupported")
        try:
            self.xml = ET.fromstring(text)
        except ET.ParseError as exc:
            raise InputError(f"Invalid URDF XML: {exc}") from exc
        if self.xml.tag != "robot":
            raise InputError("Expected a robot element")
        links = self.xml.findall("link")
        joints = self.xml.findall("joint")
        self.links = {e.get("name"): e for e in links}
        self.joints = {e.get("name"): e for e in joints}
        if len(self.links) != len(links) or len(self.joints) != len(joints) or None in self.links or None in self.joints:
            raise InputError("Duplicate or missing link/joint names")
        self.children = {name: [] for name in self.links}
        self.parent_joint = {}
        for name, joint in self.joints.items():
            if joint.get("type") not in ("fixed", "revolute", "continuous", "prismatic"):
                raise InputError(f"{name}: unsupported joint type {joint.get('type')}")
            try:
                parent = joint.find("parent").get("link")
                child = joint.find("child").get("link")
            except AttributeError as exc:
                raise InputError(f"{name}: missing parent/child") from exc
            if parent not in self.links or child not in self.links or parent == child or child in self.parent_joint:
                raise InputError(f"{name}: invalid parent/child or multiple parents")
            self.children[parent].append(name)
            self.parent_joint[child] = name
            origin(joint.find("origin"))
            if joint.get("type") != "fixed":
                axis = self.axis(name)
                if abs(np.linalg.norm(axis) - 1) > 1e-6:
                    raise InputError(f"{name}: axis must be a unit vector")
                if joint.get("type") != "continuous":
                    limit = joint.find("limit")
                    if limit is None or number(limit.get("lower")) >= number(limit.get("upper")):
                        raise InputError(f"{name}: missing or inverted position limits")
        roots = set(self.links) - set(self.parent_joint)
        if len(roots) != 1:
            raise InputError(f"URDF must have one root, got {sorted(roots)}")
        self.root = roots.pop()
        self.order = []
        visited = {self.root}

        def walk(link):
            for name in self.children[link]:
                child = self.joints[name].find("child").get("link")
                if child in visited:
                    raise InputError("Closed loop in URDF tree; define physical constraints separately")
                visited.add(child)
                self.order.append(name)
                walk(child)
        walk(self.root)
        if visited != set(self.links):
            raise InputError("Disconnected or cyclic URDF")
        self.movable = [n for n in self.order if self.joints[n].get("type") != "fixed"]
        self.independent = [n for n in self.movable if self.joints[n].find("mimic") is None]
        self.positions({n: 0 for n in self.independent})

    def axis(self, name):
        axis = self.joints[name].find("axis")
        return vector(axis.get("xyz") if axis is not None else "1 0 0", 3, f"{name}.axis")

    def positions(self, q):
        values = {}
        resolving = set()

        def resolve(name):
            if name in values:
                return values[name]
            if name in resolving or name not in self.movable:
                raise InputError(f"Invalid mimic dependency: {name}")
            resolving.add(name)
            mimic = self.joints[name].find("mimic")
            if mimic is None:
                if name not in q:
                    raise InputError(f"Missing independent joint position: {name}")
                value = number(q[name], name)
            else:
                value = resolve(mimic.get("joint")) * number(mimic.get("multiplier", 1)) + number(mimic.get("offset", 0))
            resolving.remove(name)
            values[name] = value
            return value
        for name in self.movable:
            resolve(name)
        return values

    def transforms(self, q, offsets=None, origin_deltas=None):
        offsets, origin_deltas = offsets or {}, origin_deltas or {}
        values = self.positions({name: number(q[name]) + offsets.get(name, 0) for name in self.independent})
        poses = {self.root: np.eye(4)}
        for name in self.order:
            joint = self.joints[name]
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            local = origin(joint.find("origin"))
            if name in origin_deltas:
                delta = origin_deltas[name]
                local = local @ transform(delta[:3], delta[3:])
            motion = np.eye(4)
            kind = joint.get("type")
            if kind in ("revolute", "continuous"):
                motion[:3, :3] = Rotation.from_rotvec(self.axis(name) * values[name]).as_matrix()
            elif kind == "prismatic":
                motion[:3, 3] = self.axis(name) * values[name]
            poses[child] = poses[parent] @ local @ motion
        return poses

    def check_q(self, q):
        for name, value in self.positions(q).items():
            if self.joints[name].get("type") != "continuous":
                limit = self.joints[name].find("limit")
                if value < number(limit.get("lower")) - 1e-9 or value > number(limit.get("upper")) + 1e-9:
                    raise InputError(f"{name}: position {value} outside configured limits")

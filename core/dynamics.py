"""Recursive Newton-Euler inverse dynamics (RNEA), spatial [angular, linear].

Algorithm reference: Roy Featherstone, Spatial_v2, ID.m. Independently
implemented here; no vendor code copied. Assumes a fixed-base rigid URDF tree.
It does not estimate individual link inertias from arbitrary motion logs.
"""
import numpy as np
from core.io import InputError, number
from core.model import origin
from core.schemas import inertia_matrix
from core.units_and_frames import skew


def motion_cross(v):
    return np.block([[skew(v[:3]), np.zeros((3, 3))], [skew(v[3:]), skew(v[:3])]])


def spatial_transform(pose):
    r, p = pose[:3, :3].T, pose[:3, 3]
    return np.block([[r, np.zeros((3, 3))], [-r @ skew(p), r]])


def link_inertia(element):
    data = element.find("inertial")
    if data is None:
        raise InputError(f"Missing inertia for {element.get('name')}")
    mass = number(data.find("mass").get("value"))
    if mass <= 0:
        raise InputError("Mass must be positive")
    attr = data.find("inertia").attrib
    matrix = inertia_matrix([attr[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")])
    pose = origin(data.find("origin"))
    matrix = pose[:3, :3] @ matrix @ pose[:3, :3].T
    c = skew(pose[:3, 3])
    return np.block([[matrix + mass * c @ c.T, mass * c], [mass * c.T, mass * np.eye(3)]])


def inverse_dynamics(model, q, dq, ddq, gravity=(0, 0, -9.81)):
    poses = model.transforms(q)
    velocities = {model.root: np.zeros(6)}
    accelerations = {model.root: np.r_[np.zeros(3), -np.asarray(gravity)]}
    forces, transforms, subspaces = {}, {}, {}

    def derivative(name, values):
        mimic = model.joints[name].find("mimic")
        if mimic is None:
            return number(values[name])
        return number(mimic.get("multiplier", 1)) * derivative(mimic.get("joint"), values)

    for name in model.order:
        joint = model.joints[name]
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        local = np.linalg.inv(poses[parent]) @ poses[child]
        x = transforms[name] = spatial_transform(local)
        s = np.zeros(6)
        if joint.get("type") in ("revolute", "continuous"):
            s[:3] = model.axis(name)
        elif joint.get("type") == "prismatic":
            s[3:] = model.axis(name)
        subspaces[name] = s
        rate = derivative(name, dq) if name in model.movable else 0
        accel = derivative(name, ddq) if name in model.movable else 0
        vj = s * rate
        v = velocities[child] = x @ velocities[parent] + vj
        a = accelerations[child] = x @ accelerations[parent] + s * accel + motion_cross(v) @ vj
        inertia = link_inertia(model.links[child])
        forces[child] = inertia @ a - motion_cross(v).T @ inertia @ v
    torques = {}
    for name in reversed(model.order):
        joint = model.joints[name]
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        if name in model.movable:
            torques[name] = float(subspaces[name] @ forces[child])
        if parent != model.root:
            forces[parent] += transforms[name].T @ forces[child]
    output = {name: torques[name] for name in model.independent}
    for name in model.movable:
        mimic = model.joints[name].find("mimic")
        if mimic is not None:
            multiplier, leader = 1.0, name
            while model.joints[leader].find("mimic") is not None:
                item = model.joints[leader].find("mimic")
                multiplier *= number(item.get("multiplier", 1))
                leader = item.get("joint")
            output[leader] += multiplier * torques[name]
    return output

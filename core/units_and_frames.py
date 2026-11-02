"""SI units; active transforms T_parent_child, URDF fixed-axis RPY, xyzw quaternions."""
import numpy as np
from scipy.spatial.transform import Rotation

from core.io import InputError, vector


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    output = np.eye(4)
    output[:3, :3] = Rotation.from_euler("xyz", vector(rpy, 3, "rpy")).as_matrix()
    output[:3, 3] = vector(xyz, 3, "xyz")
    return output


def pose_from_row(row):
    q = vector([row[k] for k in ("qx", "qy", "qz", "qw")], 4, "xyzw")
    if abs(np.linalg.norm(q) - 1) > 1e-3:
        raise InputError("Measured quaternion must be normalized (xyzw)")
    output = np.eye(4)
    output[:3, :3] = Rotation.from_quat(q).as_matrix()
    output[:3, 3] = vector([row[k] for k in ("x", "y", "z")], 3, "position")
    return output


def pose_error(predicted, observed):
    return np.r_[predicted[:3, 3] - observed[:3, 3],
                 Rotation.from_matrix(observed[:3, :3].T @ predicted[:3, :3]).as_rotvec()]


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

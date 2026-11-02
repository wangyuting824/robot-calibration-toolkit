import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from core.dynamics import inverse_dynamics, link_inertia
from core.io import InputError
from core.model import Model, origin


def test_forward_kinematics_against_geometry(demo):
    model = Model(demo / "nominal.urdf")
    poses = model.transforms({"j1": np.pi / 2, "j2": 0})
    np.testing.assert_allclose(poses["arm2"][:3, 3], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(poses["arm2"][:3, :3], Rotation.from_euler("z", np.pi / 2).as_matrix(), atol=1e-12)


@pytest.mark.parametrize("replacement", [
    ('axis xyz="0 1 0"', 'axis xyz="0 0 0"'),
    ('lower="-2" upper="2"', 'lower="2" upper="-2"'),
    ('child link="arm2"', 'child link="base"'),
    ('name="j2"', 'name="j1"'),
])
def test_invalid_model_definitions_rejected(demo, replacement):
    path = demo / "broken.urdf"
    path.write_text((demo / "nominal.urdf").read_text().replace(*replacement), encoding="utf-8")
    with pytest.raises(InputError):
        Model(path)


def test_missing_positions_and_out_of_range(demo):
    model = Model(demo / "nominal.urdf")
    with pytest.raises(InputError):
        model.positions({"j1": 0})
    with pytest.raises(InputError):
        model.check_q({"j1": 0, "j2": 2.1})


def numerical_mass_matrix(model, q):
    names = model.independent
    eps = 1e-6
    matrix = np.zeros((len(names), len(names)))
    poses = model.transforms(q)
    for link in model.links:
        inertia = model.links[link].find("inertial")
        if link == model.root or inertia is None:
            continue
        local = origin(inertia.find("origin"))
        jac_v, jac_w = [], []
        for name in names:
            plus, minus = dict(q), dict(q)
            plus[name] += eps
            minus[name] -= eps
            a, b = model.transforms(plus)[link] @ local, model.transforms(minus)[link] @ local
            jac_v.append((a[:3, 3] - b[:3, 3]) / (2 * eps))
            jac_w.append(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).as_rotvec() / (2 * eps))
        jv, jw = np.array(jac_v).T, np.array(jac_w).T
        mass = float(inertia.find("mass").get("value"))
        attrs = inertia.find("inertia").attrib
        xx, yy, zz, xy, xz, yz = [float(attrs[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        tensor = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        rotation = (poses[link] @ local)[:3, :3]
        matrix += mass * jv.T @ jv + jw.T @ rotation @ tensor @ rotation.T @ jw
    return matrix


@pytest.mark.parametrize("q", [{"j1": 0.3, "j2": -0.4}, {"j1": -0.8, "j2": 0.7}])
def test_rnea_mass_matrix_against_kinetic_energy(demo, q):
    model = Model(demo / "nominal.urdf")
    zero = {n: 0 for n in model.independent}
    columns = []
    for name in model.independent:
        ddq = {**zero, name: 1}
        effort = inverse_dynamics(model, q, zero, ddq, gravity=(0, 0, 0))
        columns.append([effort[n] for n in model.independent])
    matrix = np.array(columns).T
    np.testing.assert_allclose(matrix, numerical_mass_matrix(model, q), atol=1e-8, rtol=1e-7)
    np.testing.assert_allclose(matrix, matrix.T, atol=1e-12)
    assert np.linalg.eigvalsh(matrix).min() > 0


def test_rnea_gravity_against_potential_gradient(demo):
    model = Model(demo / "nominal.urdf")
    q, zero = {"j1": 0.2, "j2": 0.4}, {"j1": 0, "j2": 0}
    gravity = np.array([0.4, -0.2, -9.81])

    def potential(position):
        poses = model.transforms(position)
        return sum(-float(el.find("inertial/mass").get("value")) * gravity @ (poses[name] @ origin(el.find("inertial/origin")))[:3, 3]
                   for name, el in model.links.items())
    effort = inverse_dynamics(model, q, zero, zero, gravity)
    for name in model.independent:
        a, b = dict(q), dict(q)
        a[name] += 1e-6
        b[name] -= 1e-6
        assert effort[name] == pytest.approx((potential(a) - potential(b)) / 2e-6, abs=1e-7)


def test_rnea_coriolis_power_identity(demo):
    model = Model(demo / "nominal.urdf")
    q, dq, zero = {"j1": 0.2, "j2": 0.7}, {"j1": 0.8, "j2": -0.4}, {"j1": 0, "j2": 0}
    eps = 1e-4
    a = {n: q[n] + eps * dq[n] for n in q}
    b = {n: q[n] - eps * dq[n] for n in q}
    mdot = (numerical_mass_matrix(model, a) - numerical_mass_matrix(model, b)) / (2 * eps)
    velocity = np.array(list(dq.values()))
    c = np.array(list(inverse_dynamics(model, q, dq, zero, (0, 0, 0)).values()))
    assert velocity @ c == pytest.approx(0.5 * velocity @ mdot @ velocity, abs=1e-5)


def test_mimic_positions_and_forces(demo):
    root = ET.parse(demo / "nominal.urdf")
    ET.SubElement(root.getroot().find("joint[@name='j2']"), "mimic", joint="j1", multiplier="-0.5", offset="0.1")
    path = demo / "mimic.urdf"
    root.write(path)
    model = Model(path)
    assert model.independent == ["j1"]
    assert model.positions({"j1": 0.4})["j2"] == pytest.approx(-0.1)
    effort = inverse_dynamics(model, {"j1": 0.4}, {"j1": 0}, {"j1": 1}, (0, 0, 0))
    assert effort["j1"] == pytest.approx(numerical_mass_matrix(model, {"j1": 0.4})[0, 0], rel=1e-7)

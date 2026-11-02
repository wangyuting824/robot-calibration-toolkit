"""Version guards and static USD regression; no claim of 6.1.0 runtime validation."""
from pathlib import Path
from types import SimpleNamespace as Args
import pytest
from core.io import InputError, load, save
from adapters.isaac_adapter import check_version
from programs.stage_03_isaac_dynamics import validate_isaac_asset


def test_sdk_version_must_match_requested_target():
    check_version("6.1.0")
    with pytest.raises(InputError, match="expected 6.1.0, installed 5.1.0"):
        check_version("5.1.0")
    check_version("5.1.0", "5.1.0")
    with pytest.raises(InputError, match="No documented adapter"):
        check_version("6.2.0", "6.2.0")


@pytest.mark.parametrize("damage", [None, "limit", "mass", "axis", "units"])
@pytest.mark.parametrize("sdk", ["51", "61"])
def test_static_reference_and_intentional_damage(demo, damage, sdk):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    source = Path(__file__).parent / f"fixtures/isaac{sdk}_synthetic_reference.usda"
    stage = Usd.Stage.Open(str(source))
    target = demo / "reference.usda"
    stage.GetRootLayer().Export(str(target))
    stage = Usd.Stage.Open(str(target))
    joint = next(p for p in stage.Traverse() if p.GetName() == "j1")
    body = next(p for p in stage.Traverse() if p.GetName() == "arm1")
    if damage == "limit":
        UsdPhysics.RevoluteJoint(joint).CreateUpperLimitAttr(30)
    elif damage == "mass":
        UsdPhysics.MassAPI(body).CreateMassAttr(9)
    elif damage == "axis":
        UsdPhysics.Joint(joint).CreateLocalRot0Attr(Gf.Quatf(0.70710678, Gf.Vec3f(0.70710678, 0, 0)))
    elif damage == "units":
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    stage.GetRootLayer().Save()
    acceptance = load(demo / "acceptance.yaml")
    acceptance["asset"] = {k: 1e-5 for k in ("limit_tolerance_rad_or_m", "axis_tolerance_rad", "anchor_tolerance_m",
        "mass_tolerance_kg", "com_tolerance_m", "inertia_tolerance_kg_m2")}
    save(demo / "asset_acceptance.yaml", acceptance)
    report = validate_isaac_asset.run(Args(urdf=demo / "nominal.urdf", stage=target, acceptance=demo / "asset_acceptance.yaml"))
    assert report["status"] == ("failed" if damage else "passed")
    assert "runtime FK" in report["not_checked"]

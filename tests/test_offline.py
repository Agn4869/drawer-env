from pathlib import Path
import json

from pxr import Usd, UsdPhysics
from drawer_interaction.scene import build_scene
from drawer_interaction.schedule import DrawerCommandSchedule


ROOT = Path(__file__).resolve().parents[1]


def test_schedule_interpolates_and_respects_limits():
    spec = {
        "schema_version": 1,
        "mode": "joint_drive",
        "interpolation": "linear",
        "after_end": "hold",
        "keyframes": [{"time_s": 0.0, "value": 0.0}, {"time_s": 1.0, "value": 0.2}],
    }
    schedule = DrawerCommandSchedule.from_spec(spec, limits_m=(0.0, 0.25))
    assert schedule.evaluate(0.5)["position_m"] == 0.1
    assert schedule.evaluate(2.0)["position_m"] == 0.2


def test_scene_authors_bundle(tmp_path):
    manifest = build_scene(ROOT / "config/scenes/drawer.yaml", tmp_path / "drawer")
    assert manifest["status"] == "authored" and manifest["physics_validated"] is False
    stage = Usd.Stage.Open(str(tmp_path / "drawer/scene.usda"))
    assert stage
    binding = json.loads((tmp_path / "drawer/bindings.json").read_text())["objects"]["drawer"]
    joint = UsdPhysics.PrismaticJoint.Get(stage, binding["joint_prim"])
    assert joint and list(joint.GetBody0Rel().GetTargets()) == [binding["fixed_body_prim"]]
    assert list(joint.GetBody1Rel().GetTargets()) == [binding["body_prim"]]
    assert joint.GetLowerLimitAttr().Get() == 0.0
    assert joint.GetUpperLimitAttr().Get() == 0.25
    assert UsdPhysics.FixedJoint.Get(stage, binding["anchor_joint_prim"])
    assert UsdPhysics.DriveAPI(joint.GetPrim(), "linear").GetMaxForceAttr().Get() == 40.0

"""Offline OpenUSD authoring for the standalone drawer template."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from .drawer import author_drawer
from .usd_utils import cube, number, physics_material


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def build_scene(config_path, output_dir):
    config_path = Path(config_path).resolve(strict=True)
    output = Path(output_dir).expanduser().absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"refusing to overwrite scene destination: {output}")
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise ValueError("drawer scene requires schema_version: 1")
    world = config.get("world", {})
    if world.get("up_axis", "Z") != "Z" or world.get("meters_per_unit", 1.0) != 1.0:
        raise ValueError("drawer template requires Z-up meter units")
    table = config.get("table", {})
    table_size = [number(value, "table size", positive=True) for value in table.get("size_m", [1.6, 1.2, 0.05])]
    top_z = number(table.get("top_z_m", 0.75), "table top", positive=True)
    if len(table_size) != 3 or top_z <= table_size[2]:
        raise ValueError("table size must contain three positive values and fit below the top")
    drawer_config = dict(config.get("drawer", {}))
    drawer_config.setdefault("position_m", [0.0, 0.0, top_z])
    drawer_config.setdefault("id", "drawer")
    if drawer_config["id"] != "drawer":
        raise ValueError("standalone scene binds exactly one object named drawer")
    if not isinstance(drawer_config.get("position_m"), list) or len(drawer_config["position_m"]) != 3:
        raise ValueError("drawer.position_m must be XYZ")

    output.mkdir(parents=True, exist_ok=False)
    layout_path = output / "layout.usda"
    stage = Usd.Stage.CreateNew(str(layout_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world_prim)
    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(9.81)
    material = physics_material(stage, "/World/PhysicsMaterial")
    cube(stage, "/World/Ground", [3.0, 3.0, 0.05], [0, 0, -0.025], material=material)
    UsdGeom.Xform.Define(stage, "/World/Table")
    cube(stage, "/World/Table/Top", table_size, [0, 0, top_z - table_size[2] / 2], material=material,
         color=(0.45, 0.30, 0.18))
    UsdGeom.Xform.Define(stage, "/World/Lighting")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Dome")
    dome.CreateIntensityAttr(300.0)
    drawer_config["position_m"] = list(drawer_config["position_m"])
    binding = author_drawer(stage, "/World/Objects/drawer", drawer_config)
    stage.GetRootLayer().Save()

    scene_path = output / "scene.usda"
    entry = Usd.Stage.CreateNew(str(scene_path))
    entry.GetRootLayer().subLayerPaths = ["layout.usda"]
    UsdGeom.SetStageUpAxis(entry, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(entry, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(entry, 1.0)
    entry.SetDefaultPrim(entry.GetPrimAtPath("/World"))
    entry.GetRootLayer().Save()
    manifest = {
        "schema_version": 1,
        "scene_id": config.get("scene_id", "standalone_drawer"),
        "status": "authored",
        "physics_validated": False,
        "execution_mode": "usd_only",
        "scene_file": str(scene_path),
        "layout_file": str(layout_path),
        "bindings_file": str(output / "bindings.json"),
        "config_sha256": _sha256(config_path),
        "bindings": {"drawer": binding},
        "files": {name: _sha256(output / name) for name in ("scene.usda", "layout.usda")},
    }
    _write_json(output / "bindings.json", {"schema_version": 1, "objects": {"drawer": binding}})
    manifest["files"]["bindings.json"] = _sha256(output / "bindings.json")
    _write_json(output / "scene_manifest.json", manifest)
    return manifest

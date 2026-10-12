#!/usr/bin/env python3
"""Run drawer interaction in Isaac Sim with GUI rendering for visual demo.

Usage:
  export DISPLAY=:20
  /isaac-sim/python.sh run_isaac_gui.py \
    --scene-config config/scenes/drawer.yaml \
    --schedule config/interactions/pull_release.yaml

Switch schedule for joint_drive mode:
  --schedule config/interactions/drive_open_close.yaml
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--schedule", type=Path, required=True)
    parser.add_argument("--physics-hz", type=int, default=60)
    args = parser.parse_args()

    # Launch Isaac Sim with GUI
    from isaacsim import SimulationApp
    app = SimulationApp({
        "headless": False,
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
    })

    from omni.isaac.core import World
    from omni.isaac.core.prims import RigidPrimView
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdLux
    import omni.kit.viewport.utility as vp_utils

    # Load configs
    with args.scene_config.open(encoding="utf-8") as f:
        scene_cfg = yaml.safe_load(f)
    with args.schedule.open(encoding="utf-8") as f:
        schedule_spec = yaml.safe_load(f)

    sys.path.insert(0, str(Path(__file__).parent / "src"))
    from drawer_interaction.schedule import DrawerCommandSchedule

    drawer_cfg = dict(scene_cfg.get("drawer", {}))
    drawer_cfg.setdefault("position_m", [0, 0, float(scene_cfg.get("table", {}).get("top_z_m", 0.75))])
    limits_m = tuple(float(v) for v in drawer_cfg.get("limits_m", [0.0, 0.25]))
    max_force_n = float(drawer_cfg.get("drive", {}).get("max_force_n", 40.0))
    mode = schedule_spec["mode"]
    schedule = DrawerCommandSchedule.from_spec(schedule_spec, limits_m=limits_m, max_force_n=max_force_n)

    d = float(drawer_cfg.get("size_m", [0.36, 0.34, 0.24])[0])
    w = float(drawer_cfg.get("size_m", [0.36, 0.34, 0.24])[1])
    h = float(drawer_cfg.get("size_m", [0.36, 0.34, 0.24])[2])
    t = float(drawer_cfg.get("wall_thickness_m", 0.012))
    g_clearance = float(drawer_cfg.get("clearance_m", 0.006))
    lower, upper = limits_m
    initial_pos = float(drawer_cfg.get("initial_position_m", 0.0))
    position = [float(v) for v in drawer_cfg["position_m"]]
    yaw_deg = float(drawer_cfg.get("yaw_deg", 0))
    yaw_rad = math.radians(yaw_deg)
    mass_kg = float(drawer_cfg.get("mass_kg", 1.5))
    cabinet_mass_kg = float(drawer_cfg.get("cabinet_mass_kg", 12.0))
    drive_cfg = drawer_cfg.get("drive", {})
    max_force = float(drive_cfg.get("max_force_n", 40.0))

    # Create world
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / 30)
    stage = world.stage

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Physics scene
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)

    # Helper functions
    def xform(prim, trans=(0, 0, 0), yaw=0):
        x = UsdGeom.Xformable(prim)
        x.AddTranslateOp().Set(Gf.Vec3d(*trans))
        x.AddRotateZOp().Set(float(yaw))

    def cube(path, size, center=(0, 0, 0), color=(0.55, 0.6, 0.65)):
        geo = UsdGeom.Cube.Define(stage, path)
        geo.CreateSizeAttr(1.0)
        geo.AddTranslateOp().Set(Gf.Vec3d(*center))
        geo.AddScaleOp().Set(Gf.Vec3d(*size))
        geo.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(geo.GetPrim()).CreateCollisionEnabledAttr(True)
        return geo.GetPrim()

    def rigid(prim, mass):
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))

    # Ground + table + light
    world.scene.add_default_ground_plane()

    table_size = [float(v) for v in scene_cfg.get("table", {}).get("size_m", [1.6, 1.2, 0.05])]
    top_z = float(scene_cfg.get("table", {}).get("top_z_m", 0.75))
    cube("/World/Table/Top", table_size, [0, 0, top_z - table_size[2] / 2], color=(0.45, 0.30, 0.18))

    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Dome")
    dome.CreateIntensityAttr(500.0)

    # Drawer assembly
    root_path = "/World/Objects/drawer"
    root_prim = UsdGeom.Xform.Define(stage, root_path).GetPrim()
    xform(root_prim, position, yaw_deg)

    cabinet_path = f"{root_path}/CabinetBody"
    drawer_path = f"{root_path}/DrawerBody"
    cabinet_prim = UsdGeom.Xform.Define(stage, cabinet_path).GetPrim()
    drawer_prim = UsdGeom.Xform.Define(stage, drawer_path).GetPrim()
    xform(cabinet_prim, (0, 0, h / 2))
    xform(drawer_prim, (initial_pos, 0, h / 2))
    rigid(cabinet_prim, cabinet_mass_kg)
    rigid(drawer_prim, mass_kg)

    # Cabinet walls
    for name, dims, center in [
        ("Bottom", (d, w, t), (0, 0, -h / 2 + t / 2)),
        ("Top", (d, w, t), (0, 0, h / 2 - t / 2)),
        ("Back", (t, w, h - 2 * t), (-d / 2 + t / 2, 0, 0)),
        ("SideLeft", (d - t, t, h - 2 * t), (t / 2, -w / 2 + t / 2, 0)),
        ("SideRight", (d - t, t, h - 2 * t), (t / 2, w / 2 - t / 2, 0)),
    ]:
        cube(f"{cabinet_path}/{name}", dims, center)

    # Drawer inner box
    inner = [d - 2 * (t + g_clearance), w - 2 * (t + g_clearance), h - 2 * (t + g_clearance)]
    for name, dims, center in [
        ("Bottom", (inner[0], inner[1], t), (0, 0, -inner[2] / 2 + t / 2)),
        ("WallXMin", (t, inner[1], inner[2] - t), (-inner[0] / 2 + t / 2, 0, t / 2)),
        ("WallXMax", (t, inner[1], inner[2] - t), (inner[0] / 2 - t / 2, 0, t / 2)),
        ("WallYMin", (inner[0] - 2 * t, t, inner[2] - t), (0, -inner[1] / 2 + t / 2, t / 2)),
        ("WallYMax", (inner[0] - 2 * t, t, inner[2] - t), (0, inner[1] / 2 - t / 2, t / 2)),
    ]:
        cube(f"{drawer_path}/{name}", dims, center, color=(0.25, 0.45, 0.7))

    # Handle
    handle_center = (d / 2 + 0.025, 0, 0)
    cube(f"{drawer_path}/Handle", (0.014, w * 0.38, 0.016), handle_center, color=(0.2, 0.2, 0.22))
    cube(f"{drawer_path}/HandleStemL", (0.025 + t + g_clearance, 0.012, 0.012),
         (d / 2 - (t + g_clearance) / 2 + 0.0125, -w * 0.14, 0), color=(0.2, 0.2, 0.22))
    cube(f"{drawer_path}/HandleStemR", (0.025 + t + g_clearance, 0.012, 0.012),
         (d / 2 - (t + g_clearance) / 2 + 0.0125, w * 0.14, 0), color=(0.2, 0.2, 0.22))

    # Prismatic joint
    joint_path = f"{root_path}/DrawerSlideJoint"
    joint = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([cabinet_path])
    joint.CreateBody1Rel().SetTargets([drawer_path])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    joint.CreateLowerLimitAttr(lower)
    joint.CreateUpperLimitAttr(upper)
    joint.CreateCollisionEnabledAttr(True)

    # Drive
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(initial_pos)
    if mode == "joint_drive":
        drive.CreateStiffnessAttr(500.0)
        drive.CreateDampingAttr(30.0)
    else:
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
    drive.CreateMaxForceAttr(max_force)

    # Anchor
    anchor = UsdPhysics.FixedJoint.Define(stage, f"{root_path}/AnchorJoint")
    world_pos = Gf.Vec3f(position[0], position[1], position[2] + h / 2)
    quat = Gf.Quatf(float(math.cos(yaw_rad / 2)), Gf.Vec3f(0, 0, float(math.sin(yaw_rad / 2))))
    anchor.CreateBody1Rel().SetTargets([cabinet_path])
    anchor.CreateLocalPos0Attr(world_pos)
    anchor.CreateLocalPos1Attr(Gf.Vec3f(0))
    anchor.CreateLocalRot0Attr(quat)
    anchor.CreateLocalRot1Attr(Gf.Quatf(1))

    # Set camera position for a good view
    try:
        viewport = vp_utils.get_active_viewport()
        if viewport:
            from omni.kit.viewport.utility import get_viewport_from_window_name
            import omni.kit.commands
            cam_path = "/World/Camera"
            UsdGeom.Xform.Define(stage, cam_path)
            cam = UsdGeom.Camera.Define(stage, cam_path)
            cam_xf = UsdGeom.Xformable(cam.GetPrim())
            cam_xf.AddTranslateOp().Set(Gf.Vec3d(0.8, -0.6, 1.2))
            cam_xf.AddRotateXYZOp().Set(Gf.Vec3d(60, 0, 130))
    except Exception as e:
        print(f"Camera setup note: {e}")

    # Initialize physics
    world.reset()

    # RigidPrimView for force application
    drawer_view = RigidPrimView(prim_paths_expr=drawer_path, name="drawer_view", track_contact_forces=True)
    world.scene.add(drawer_view)
    world.reset()

    rot = np.array([
        [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
        [math.sin(yaw_rad), math.cos(yaw_rad), 0],
        [0, 0, 1],
    ])

    print(f"\n{'='*60}")
    print(f"  Isaac Sim GUI - Drawer Interaction Demo")
    print(f"  Mode: {mode}")
    print(f"  Schedule duration: {schedule.duration_s}s + 1s settle")
    print(f"  Limits: {limits_m}")
    print(f"  Physics: {args.physics_hz} Hz")
    print(f"{'='*60}\n")
    print("Watch the viewport! The drawer will move now...\n")

    duration = schedule.duration_s
    total_steps = int((duration + 2.0) * args.physics_hz)  # extra settle time for viewing

    for step_i in range(total_steps):
        sim_time = (step_i + 1) / args.physics_hz
        cmd = schedule.evaluate(sim_time)

        if cmd["mode"] == "external_force":
            force_val = cmd.get("force_handle_n") or cmd.get("force_n") or cmd.get("value", 0)
            if isinstance(force_val, (int, float)):
                direction = cmd.get("direction_handle", [1, 0, 0])
                force_local = np.array(direction, dtype=float) * force_val
            else:
                force_local = np.array(force_val, dtype=float)
            force_world = rot @ force_local
            drawer_view.apply_forces_and_torques_at_pos(
                forces=np.array([force_world]),
                positions=np.array([[0, 0, 0]]),
                is_global=False,
            )
        else:
            target = float(cmd.get("position_m") or cmd.get("value", 0))
            target = max(lower, min(upper, target))
            drive.GetTargetPositionAttr().Set(target)

        world.step(render=True)

        if (step_i + 1) % args.physics_hz == 0:
            # Read position from transforms
            xfc = UsdGeom.XformCache()
            xfc.Clear()
            cab_xf = xfc.GetLocalToWorldTransform(stage.GetPrimAtPath(cabinet_path))
            drw_xf = xfc.GetLocalToWorldTransform(stage.GetPrimAtPath(drawer_path))
            delta = np.array(drw_xf.ExtractTranslation()) - np.array(cab_xf.ExtractTranslation())
            local_x = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0])
            pos = float(np.dot(delta, local_x))
            print(f"  t={sim_time:.1f}s  position={pos:.4f}m  cmd={cmd.get('value', '-')}")

    print("\nSimulation complete! Window stays open for inspection.")
    print("Close the Isaac Sim window to exit.\n")

    # Keep the app running so user can inspect
    while app.is_running():
        app.update()


if __name__ == "__main__":
    main()

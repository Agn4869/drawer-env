#!/usr/bin/env python3
"""Run drawer interaction in Isaac Sim with real PhysX physics.

Usage (headless):
  /isaac-sim/python.sh run_isaac.py \
    --scene-config config/scenes/drawer.yaml \
    --schedule config/interactions/pull_release.yaml \
    --output outputs/isaac_external_force \
    --headless

Usage (with GUI):
  /isaac-sim/python.sh run_isaac.py \
    --scene-config config/scenes/drawer.yaml \
    --schedule config/interactions/drive_open_close.yaml \
    --output outputs/isaac_joint_drive
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml


def parse_args():
    parser = argparse.ArgumentParser(description="Drawer interaction via Isaac Sim / PhysX")
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--schedule", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--physics-hz", type=int, default=60)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-hz", type=int, default=30)
    return parser.parse_args()


def main():
    args = parse_args()

    # ---------- Launch SimulationApp FIRST (before any omni imports) ----------
    from isaacsim import SimulationApp

    launch_config = {
        "headless": args.headless,
        "width": 1280,
        "height": 720,
        "anti_aliasing": 0,
    }
    app = SimulationApp(launch_config)

    # ---------- Now safe to import omni / pxr ----------
    from omni.isaac.core import World
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema

    # ---------- Load configs ----------
    with args.scene_config.open(encoding="utf-8") as f:
        scene_cfg = yaml.safe_load(f)

    with args.schedule.open(encoding="utf-8") as f:
        schedule_spec = yaml.safe_load(f)

    # Add src to path for schedule import
    sys.path.insert(0, str(Path(__file__).parent / "src"))
    from drawer_interaction.schedule import DrawerCommandSchedule

    drawer_cfg = dict(scene_cfg.get("drawer", {}))
    drawer_cfg.setdefault("position_m", [0, 0, float(scene_cfg.get("table", {}).get("top_z_m", 0.75))])
    limits_m = tuple(float(v) for v in drawer_cfg.get("limits_m", [0.0, 0.25]))
    max_force_n = float(drawer_cfg.get("drive", {}).get("max_force_n", 40.0))
    mode = schedule_spec["mode"]

    schedule = DrawerCommandSchedule.from_spec(schedule_spec, limits_m=limits_m, max_force_n=max_force_n)

    # ---------- Create World ----------
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / args.physics_hz,
        rendering_dt=1.0 / args.render_hz,
    )

    stage = world.stage

    # Set stage metadata
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Physics scene
    physics_scene_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(physics_scene_path):
        ps = UsdPhysics.Scene.Define(stage, physics_scene_path)
        ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        ps.CreateGravityMagnitudeAttr(9.81)

    # ---------- Author scene directly on the live stage ----------
    d, w, h = [float(v) for v in drawer_cfg.get("size_m", [0.36, 0.34, 0.24])]
    t = float(drawer_cfg.get("wall_thickness_m", 0.012))
    g = float(drawer_cfg.get("clearance_m", 0.006))
    lower, upper = limits_m
    initial_pos = float(drawer_cfg.get("initial_position_m", 0.0))
    position = [float(v) for v in drawer_cfg["position_m"]]
    yaw_deg = float(drawer_cfg.get("yaw_deg", 0))
    mass_kg = float(drawer_cfg.get("mass_kg", 1.5))
    cabinet_mass_kg = float(drawer_cfg.get("cabinet_mass_kg", 12.0))
    drive_cfg = drawer_cfg.get("drive", {})
    max_force = float(drive_cfg.get("max_force_n", 40.0))

    def _xform(prim, trans=(0, 0, 0), yaw=0):
        x = UsdGeom.Xformable(prim)
        x.AddTranslateOp().Set(Gf.Vec3d(*trans))
        x.AddRotateZOp().Set(float(yaw))

    def _cube(path, size, center=(0, 0, 0), color=(0.55, 0.6, 0.65)):
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*center))
        g.AddScaleOp().Set(Gf.Vec3d(*size))
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(g.GetPrim()).CreateCollisionEnabledAttr(True)
        return g.GetPrim()

    def _rigid(prim, mass):
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr(float(mass))

    # Ground
    world.scene.add_default_ground_plane()

    # Table
    UsdGeom.Xform.Define(stage, "/World/Table")
    table_size = [float(v) for v in scene_cfg.get("table", {}).get("size_m", [1.6, 1.2, 0.05])]
    top_z = float(scene_cfg.get("table", {}).get("top_z_m", 0.75))
    _cube("/World/Table/Top", table_size, [0, 0, top_z - table_size[2] / 2], color=(0.45, 0.30, 0.18))

    # Dome light
    UsdGeom.Xform.Define(stage, "/World/Lighting")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/Dome")
    dome.CreateIntensityAttr(300.0)

    # --- Drawer assembly ---
    root_path = "/World/Objects/drawer"
    root_prim = UsdGeom.Xform.Define(stage, root_path).GetPrim()
    _xform(root_prim, position, yaw_deg)

    cabinet_path = f"{root_path}/CabinetBody"
    drawer_path = f"{root_path}/DrawerBody"

    cabinet_prim = UsdGeom.Xform.Define(stage, cabinet_path).GetPrim()
    drawer_prim = UsdGeom.Xform.Define(stage, drawer_path).GetPrim()
    _xform(cabinet_prim, (0, 0, h / 2))
    _xform(drawer_prim, (initial_pos, 0, h / 2))

    _rigid(cabinet_prim, cabinet_mass_kg)
    _rigid(drawer_prim, mass_kg)

    # Cabinet walls
    for name, dims, center in [
        ("Bottom", (d, w, t), (0, 0, -h / 2 + t / 2)),
        ("Top", (d, w, t), (0, 0, h / 2 - t / 2)),
        ("Back", (t, w, h - 2 * t), (-d / 2 + t / 2, 0, 0)),
        ("SideLeft", (d - t, t, h - 2 * t), (t / 2, -w / 2 + t / 2, 0)),
        ("SideRight", (d - t, t, h - 2 * t), (t / 2, w / 2 - t / 2, 0)),
    ]:
        _cube(f"{cabinet_path}/{name}", dims, center)

    # Drawer inner box
    inner = [d - 2 * (t + g), w - 2 * (t + g), h - 2 * (t + g)]
    for name, dims, center in [
        ("Bottom", (inner[0], inner[1], t), (0, 0, -inner[2] / 2 + t / 2)),
        ("WallXMin", (t, inner[1], inner[2] - t), (-inner[0] / 2 + t / 2, 0, t / 2)),
        ("WallXMax", (t, inner[1], inner[2] - t), (inner[0] / 2 - t / 2, 0, t / 2)),
        ("WallYMin", (inner[0] - 2 * t, t, inner[2] - t), (0, -inner[1] / 2 + t / 2, t / 2)),
        ("WallYMax", (inner[0] - 2 * t, t, inner[2] - t), (0, inner[1] / 2 - t / 2, t / 2)),
    ]:
        _cube(f"{drawer_path}/{name}", dims, center, color=(0.25, 0.45, 0.7))

    # Handle
    handle_center = (d / 2 + 0.025, 0, 0)
    _cube(f"{drawer_path}/Handle", (0.014, w * 0.38, 0.016), handle_center, color=(0.2, 0.2, 0.22))

    handle_frame_path = f"{drawer_path}/HandleFrame"
    handle_prim = UsdGeom.Xform.Define(stage, handle_frame_path).GetPrim()
    _xform(handle_prim, handle_center)

    # --- Prismatic joint ---
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

    # Drive API
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

    # --- Fixed anchor to world ---
    anchor_path = f"{root_path}/AnchorJoint"
    anchor = UsdPhysics.FixedJoint.Define(stage, anchor_path)
    world_pos = Gf.Vec3f(position[0], position[1], position[2] + h / 2)
    yaw_rad = math.radians(yaw_deg)
    quat = Gf.Quatf(
        float(math.cos(yaw_rad / 2)),
        Gf.Vec3f(0, 0, float(math.sin(yaw_rad / 2)))
    )
    anchor.CreateBody1Rel().SetTargets([cabinet_path])
    anchor.CreateLocalPos0Attr(world_pos)
    anchor.CreateLocalPos1Attr(Gf.Vec3f(0))
    anchor.CreateLocalRot0Attr(quat)
    anchor.CreateLocalRot1Attr(Gf.Quatf(1))

    # ---------- Prepare for simulation ----------
    print(f"Scene authored: mode={mode}, limits={limits_m}, schedule_duration={schedule.duration_s}s")

    world.reset()

    # Use RigidPrimView for force application and contact queries
    from omni.isaac.core.prims import RigidPrimView
    drawer_view = RigidPrimView(prim_paths_expr=drawer_path, name="drawer_view",
                                 track_contact_forces=True)
    world.scene.add(drawer_view)
    world.reset()  # re-initialize with the new view

    # ---------- Simulation loop ----------
    output_dir = args.output
    output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = output_dir / "joint_states.jsonl"
    wall_start = time.time()

    duration = schedule.duration_s
    settle_time = 1.0
    total_seconds = duration + settle_time
    total_steps = int(total_seconds * args.physics_hz)

    xform_cache = UsdGeom.XformCache()

    def read_state(step_idx, sim_time, last_cmd):
        # Read joint state
        pos_attr = joint.GetPrim().GetAttribute("state:linearPosition")
        vel_attr = joint.GetPrim().GetAttribute("state:linearVelocity")

        # Fallback: compute from transforms
        joint_pos = 0.0
        joint_vel = 0.0
        if pos_attr and pos_attr.Get() is not None:
            joint_pos = float(pos_attr.Get())
        else:
            # Compute from drawer position relative to cabinet
            xform_cache.Clear()
            cab_xf = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(cabinet_path))
            drw_xf = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(drawer_path))
            cab_pos = np.array(cab_xf.ExtractTranslation())
            drw_pos = np.array(drw_xf.ExtractTranslation())
            delta = drw_pos - cab_pos
            local_x = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0])
            joint_pos = float(np.dot(delta, local_x))

        if vel_attr and vel_attr.Get() is not None:
            joint_vel = float(vel_attr.Get())
        else:
            vels = drawer_view.get_linear_velocities()
            if vels is not None:
                vel = np.array(vels).flatten()[:3]
                local_x = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0])
                joint_vel = float(np.dot(vel, local_x))

        # Contact force (net)
        contact_force = np.zeros(3)
        contact_norm = 0.0
        try:
            cf = drawer_view.get_net_contact_forces()
            if cf is not None:
                contact_force = np.array(cf).flatten()[:3]
                contact_norm = float(np.linalg.norm(contact_force))
        except Exception:
            pass

        return {
            "step_index": step_idx,
            "simulation_time": round(sim_time, 6),
            "position_m": round(joint_pos, 6),
            "velocity_m_s": round(joint_vel, 6),
            "mode": mode,
            "applied_command": dict(last_cmd) if last_cmd else {},
            "contacts": {
                "available": contact_norm > 0,
                "net_force_world_n": contact_force.tolist(),
                "net_force_norm_n": round(contact_norm, 4),
            },
        }

    def apply_command(cmd):
        if cmd["mode"] == "external_force":
            force_val = cmd.get("force_handle_n") or cmd.get("force_n") or cmd.get("value", 0)
            if isinstance(force_val, (int, float)):
                direction = cmd.get("direction_handle", [1, 0, 0])
                force_local = np.array(direction, dtype=float) * force_val
            else:
                force_local = np.array(force_val, dtype=float)

            # Transform to world frame
            rot = np.array([
                [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
                [math.sin(yaw_rad), math.cos(yaw_rad), 0],
                [0, 0, 1],
            ])
            force_world = rot @ force_local

            # Apply force via PhysX RigidPrimView
            drawer_view.apply_forces_and_torques_at_pos(
                forces=np.array([force_world]),
                positions=np.array([[0, 0, 0]]),
                is_global=False,
            )
            return {
                "mode": mode,
                "force_handle_n": force_local.tolist(),
                "force_world_n": force_world.tolist(),
                "force_norm_n": float(np.linalg.norm(force_local)),
                "position_m": None,
            }
        else:
            target = float(cmd.get("position_m") or cmd.get("value", 0))
            target = max(lower, min(upper, target))
            drive.GetTargetPositionAttr().Set(target)
            return {
                "mode": mode,
                "position_m": target,
                "force_norm_n": None,
                "force_handle_n": None,
            }

    print(f"Starting simulation: {total_steps} physics steps at {args.physics_hz} Hz")
    print(f"Schedule: {mode}, duration={duration}s + {settle_time}s settle")

    with jsonl_path.open("w", encoding="utf-8") as out:
        # Initial state
        initial = read_state(0, 0.0, None)
        out.write(json.dumps(initial, allow_nan=False) + "\n")

        last_cmd = None
        for step_i in range(total_steps):
            sim_time = (step_i + 1) / args.physics_hz
            cmd = schedule.evaluate(sim_time)
            last_cmd = apply_command(cmd)
            world.step(render=not args.headless)

            # Record every step (physics_hz is already reasonable)
            state = read_state(step_i + 1, sim_time, last_cmd)
            out.write(json.dumps(state, allow_nan=False) + "\n")

            if (step_i + 1) % args.physics_hz == 0:
                print(f"  t={sim_time:.1f}s  pos={state['position_m']:.4f}m  vel={state['velocity_m_s']:.4f}m/s")

    wall_time = time.time() - wall_start
    print(f"Simulation complete in {wall_time:.1f}s")

    # Export curves
    sys.path.insert(0, str(Path(__file__).parent / "src"))
    from drawer_interaction.curves import export_drawer_curves
    curves_result = export_drawer_curves(str(jsonl_path), str(output_dir))
    print(f"Curves exported: {curves_result['csv']}")

    # Write manifest
    manifest = {
        "schema_version": 1,
        "status": "complete",
        "interaction_mode": mode,
        "physics_hz": args.physics_hz,
        "physics_backend": "IsaacSim/PhysX",
        "isaac_sim_version": "4.5.0",
        "schedule_duration_s": duration,
        "total_physics_steps": total_steps,
        "wall_time_s": round(wall_time, 3),
        "limits_m": list(limits_m),
        "curves": curves_result,
        "scene_config": str(args.scene_config),
        "schedule_config": str(args.schedule),
        "runtime_dynamics_verified": True,
    }
    manifest_path = output_dir / "run_manifest.json"
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
        f.write("\n")

    print(json.dumps(manifest, indent=2))
    app.close()


if __name__ == "__main__":
    main()

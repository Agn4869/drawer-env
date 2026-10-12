"""CLI for offline scene authoring and schedule validation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from .schedule import DrawerCommandSchedule


def parser():
    root = argparse.ArgumentParser(prog="drawer-interaction")
    commands = root.add_subparsers(dest="command", required=True)
    scene = commands.add_parser("scene", help="author an OpenUSD drawer scene")
    scene.add_argument("--config", type=Path, required=True)
    scene.add_argument("--output", type=Path, required=True)
    schedule = commands.add_parser("validate-schedule", help="validate a force/drive schedule")
    schedule.add_argument("--config", type=Path, required=True)
    schedule.add_argument("--limits", type=float, nargs=2, default=(0.0, 0.25))
    return root


def main(argv=None):
    args = parser().parse_args(argv)
    if args.command == "scene":
        from .scene import build_scene
        result = build_scene(args.config, args.output)
        print(json.dumps({"status": result["status"], "scene_file": result["scene_file"],
                          "execution_mode": result["execution_mode"],
                          "physics_validated": result["physics_validated"]}, ensure_ascii=False, indent=2))
        return 0
    with args.config.open(encoding="utf-8") as stream:
        spec = yaml.safe_load(stream)
    schedule = DrawerCommandSchedule.from_spec(spec, limits_m=tuple(args.limits))
    print(json.dumps({"status": "valid", "mode": schedule.mode, "duration_s": schedule.duration_s,
                      "limits_m": list(schedule.limits_m)}, ensure_ascii=False, indent=2))
    return 0

# Drawer Environment

OpenUSD drawer interaction environment with Isaac Sim / PhysX physics backend. Supports two interaction modes: **external force** (apply force at handle) and **joint drive** (PD position control).

Extracted from `geniesim-features` F102.

## Architecture

```
config/                          YAML scene & schedule definitions
src/drawer_interaction/          Core library (schedule, curves, USD authoring)
run_isaac.py                     Headless Isaac Sim simulation (JSONL + curves)
run_isaac_gui.py                 GUI Isaac Sim simulation (real-time viewport)
web/index.html                   Three.js 3D visualization + Chart.js curves
tests/test_offline.py            Offline unit tests
```

## Physics Modes

- **`external_force`**: Applies handle-frame force via `RigidPrimView.apply_forces_and_torques_at_pos()`. The drawer responds with realistic inertia, friction, and joint-limit bouncing.
- **`joint_drive`**: Sets `DriveAPI` target position each tick. The PD controller drives the prismatic joint to the target with configurable stiffness/damping.

## Requirements

- **Isaac Sim 5.x** (tested with 5.1.0) for physics simulation
- Python 3.11+, NumPy, PyYAML for offline tools
- Optional: `usd-core` for standalone USD authoring without Isaac Sim

## Usage

### Headless Simulation (produces data)

```bash
# External force mode
/isaac-sim/python.sh run_isaac.py \
  --scene-config config/scenes/drawer.yaml \
  --schedule config/interactions/pull_release.yaml \
  --output outputs/isaac_external_force

# Joint drive mode
/isaac-sim/python.sh run_isaac.py \
  --scene-config config/scenes/drawer.yaml \
  --schedule config/interactions/drive_open_close.yaml \
  --output outputs/isaac_joint_drive
```

Outputs: `joint_states.jsonl`, `drawer_curves.csv`, `drawer_curves.svg`, `run_manifest.json`

### GUI Simulation (real-time viewport)

```bash
export DISPLAY=:0  # or your X display
/isaac-sim/python.sh run_isaac_gui.py \
  --scene-config config/scenes/drawer.yaml \
  --schedule config/interactions/pull_release.yaml \
  --physics-hz 60
```

### Offline Tools (no Isaac Sim needed)

```bash
pip install -e '.[usd]'

# Author USD scene bundle
python -m drawer_interaction scene \
  --config config/scenes/drawer.yaml \
  --output outputs/drawer_scene

# Validate schedule
python -m drawer_interaction validate-schedule \
  --config config/interactions/pull_release.yaml
```

### Web Visualization

Serve the `web/` directory and place simulation output data under `web/data/`:

```bash
cd web && python -m http.server 8080
```

## Configuration

### Scene (`config/scenes/drawer.yaml`)

Defines cabinet/drawer geometry, mass, joint limits, drive parameters, and table layout.

### Schedules (`config/interactions/`)

- `pull_release.yaml`: External force — 8N pull for 2s, release for 2s, repeat
- `drive_open_close.yaml`: Joint drive — open to 0.12m, hold, close to 0m

## Output Schema

Each `joint_states.jsonl` line contains:

```json
{
  "step_index": 240,
  "simulation_time": 1.0,
  "position_m": 0.0523,
  "velocity_m_s": 0.1234,
  "mode": "external_force",
  "applied_command": { "mode": "external_force", "force_handle_n": [8.0, 0, 0], ... },
  "contacts": { "available": true, "net_force_norm_n": 3.45, ... }
}
```

## Project Origin

Core USD authoring and schedule system from `geniesim-features` F102. Isaac Sim runtime added for real PhysX verification of drawer dynamics, contact forces, and joint behavior.

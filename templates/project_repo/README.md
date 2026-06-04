# Project Template

This directory is a starter template for an external project repository that consumes `mujoco_wheeled_uav_simulator` as a Git submodule.

Use it when creating a real project repository around this simulator.

## What This Template Is For

- keep project-specific controllers outside the simulator repository
- launch the simulator through `external/mujoco_wheeled_uav_simulator`
- keep logs and generated XML in the project repository
- allow a project-owned parameter file via `--params-file`

## First Setup

1. Create a new repository for the project.
2. Add this simulator repository as `external/mujoco_wheeled_uav_simulator` via Git submodule.
3. Copy the contents of this template into the new repository root.
4. Replace `my_project_controller` with the real project controller.
5. If needed, add `configs/vehicle/vehicle_params.project.json`.

## Expected Layout

```text
project-repo/
├─ external/
│  └─ mujoco_wheeled_uav_simulator/
├─ analysis/
├─ build/
├─ controllers/
├─ configs/
├─ experiments/
├─ scripts/
├─ logs/
├─ results/
└─ docs/
```

## Replace These Files First

- `controllers/matlab/my_project_controller.m`
- `experiments/matlab/project_contact_trials.m`
- `experiments/matlab/project_formation_trials.m`
- `analysis/matlab/project_contact_review.m`
- `analysis/matlab/project_formation_review.m`
- the controller name inside `scripts/run_controller.ps1`
- the controller name inside `scripts/run_controller.sh`
- the experiment name inside `scripts/run_experiment.ps1`
- the experiment name inside `scripts/run_experiment.sh`
- optionally `configs/vehicle/vehicle_params.project.json`

## Initialize The Simulator Dependency

```powershell
git submodule add <simulator-repository-url> external/mujoco_wheeled_uav_simulator
git submodule update --init --recursive
uv sync --project external/mujoco_wheeled_uav_simulator
```

## Run On Windows

```powershell
./scripts/run_simulator.ps1
./scripts/run_controller.ps1
./scripts/run_experiment.ps1
```

## Run On Ubuntu/Linux

```bash
./scripts/run_simulator.sh
./scripts/run_controller.sh
./scripts/run_experiment.sh
```

## Notes

- `.ps1` is the Windows wrapper and `.sh` is the Ubuntu/Linux wrapper.
- The simulator CLI is still the real entry point; the wrappers only shorten repeated commands.
- Generated MuJoCo XML defaults to `build/generated_xml/` unless the launch scripts override it.
- Project-owned experiment and review entrypoints now live under `experiments/matlab/` and `analysis/matlab/`.
- The external-project style launch flow represented by this template has already been validated once on Windows in this workspace.

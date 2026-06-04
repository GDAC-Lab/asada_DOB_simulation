# Project Integration Workflow

This document describes a practical workflow for running experiments from an external project repository while using this simulator as a Git submodule.

The environment assumed here is:

- Windows or Ubuntu/Linux
- `uv`
- MATLAB

The `.ps1` wrappers shown below are Windows-oriented examples. By contrast, the MATLAB-side `auto_launch` path is intended to work on both Windows and Ubuntu/Linux.

## Operating Principle

The project repository is the daily entry point.

- launch the simulator from the project repository
- launch the project-specific controller from the project repository
- treat the simulator repository as a pinned dependency under `external/mujoco_wheeled_uav_simulator/`

## Example Workflow

### 1. Clone The Project

```powershell
git clone <project-repository-url>
cd <project-repository>
git submodule update --init --recursive
```

### 2. Prepare The Simulator Environment

```powershell
uv sync --project external/mujoco_wheeled_uav_simulator
```

This keeps dependency installation tied to the simulator repository, which already owns the Python environment definition.

### 3. Start The Simulator

Typical example:

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate
```

Formation example:

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate --num-uavs 3
```

Independent instance example:

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate --instance-id 1
```

If the project owns a parameter override file, pass it explicitly.

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate --params-file configs/vehicle/vehicle_params.project.json --generated-xml-dir build/generated_xml
```

## 4. Start The Project Controller

The controller should live in the external project repository, not inside the simulator submodule.

Typical MATLAB pattern:

```matlab
addpath('controllers/matlab');
addpath('external/mujoco_wheeled_uav_simulator');
addpath('external/mujoco_wheeled_uav_simulator/matlab');

my_project_controller
```

If the project controller reuses helper code from the simulator, keep the reuse explicit. Do not edit the simulator sample controllers in place.

## 4.5. Start Project-Owned Experiments And Reviews

Project-specific experiment entrypoints should live in the project repository.

Typical examples:

- `experiments/matlab/project_contact_trials.m`
- `experiments/matlab/project_formation_trials.m`
- `analysis/matlab/project_contact_review.m`
- `analysis/matlab/project_formation_review.m`

These entrypoints may call shared helpers or sample implementations from `external/mujoco_wheeled_uav_simulator`, but the orchestration itself should stay project-owned.

## 5. Store Logs In The Project Repository

Prefer to store project run outputs in the project repository rather than in the simulator submodule.

- good: `project-repo/logs/...`
- avoid for long-term use: `project-repo/external/mujoco_wheeled_uav_simulator/logs/...`

This keeps experiment outputs attached to the project that generated them.

## Recommended PowerShell Wrapper Pattern

Instead of typing raw commands each time, place thin wrapper scripts under `scripts/`.

These `.ps1` examples are for Windows. On Ubuntu/Linux, use equivalent `.sh` wrappers or call the simulator CLI directly. The starter template now includes both.

Example `scripts/run_simulator.ps1`:

```powershell
param(
    [int]$InstanceId = 0,
    [int]$NumUavs = 1
)

$simRoot = "external/mujoco_wheeled_uav_simulator"

if ($NumUavs -gt 1) {
    uv run --project $simRoot mujoco-wheeled-uav-simulator simulate --num-uavs $NumUavs
} else {
    uv run --project $simRoot mujoco-wheeled-uav-simulator simulate --instance-id $InstanceId
}
```

Example `scripts/run_controller.ps1`:

```powershell
matlab -batch "addpath('controllers/matlab'); addpath('external/mujoco_wheeled_uav_simulator'); addpath('external/mujoco_wheeled_uav_simulator/matlab'); my_project_controller"
```

These wrappers should stay thin. They orchestrate process launch and argument selection, but they should not become a second simulator implementation.

## Recommended Reproducibility Record

For each experiment family, store a compact record that includes:

- project commit hash
- simulator submodule commit hash
- launch command used
- controller entry point used
- configuration files used
- output log directory

One practical approach is a Markdown file under `docs/` plus a run-generated metadata JSON or MAT file.

## Updating The Simulator Submodule

Only update the simulator dependency intentionally.

Typical flow:

```powershell
cd external/mujoco_wheeled_uav_simulator
git fetch
git checkout <desired-tag-or-commit>
cd ../..
git add external/mujoco_wheeled_uav_simulator
git commit -m "Update simulator submodule"
```

If the update is required for a reusable capability, make the change in the simulator repository first, then update the project repository to the new pinned commit.

## Recommended First Trial

For the first external project trial:

- keep one controller entry point
- keep one baseline hover experiment
- keep one contact or terrain experiment
- keep one reproducibility note

This is enough to validate the boundary without introducing unnecessary infrastructure too early.

## Starter Template

This repository also includes a working starter scaffold under [templates/project_repo](../templates/project_repo).
# Project Repository Template

This document describes the recommended structure for a research or product repository that consumes this simulator as a dependency.

The intent is:

- keep this repository focused on reusable simulator functionality
- keep project-specific controllers, experiments, evaluation, and publication assets outside
- make experiment reproduction easy by pinning the simulator version through a Git submodule

## Recommended Directory Layout

```text
project-repo/
├─ .gitmodules
├─ external/
│  └─ mujoco_wheeled_uav_simulator/
├─ controllers/
│  ├─ matlab/
│  └─ python/
├─ experiments/
│  ├─ hover_baseline/
│  ├─ contact_trials/
│  └─ formation_trials/
├─ configs/
│  ├─ vehicle/
│  ├─ scenarios/
│  └─ logging/
├─ analysis/
│  ├─ matlab/
│  └─ python/
├─ scripts/
│  ├─ run_simulator.ps1
│  ├─ run_controller.ps1
│  ├─ run_experiment.ps1
│  └─ update_simulator.ps1
├─ logs/
├─ results/
│  ├─ figures/
│  └─ tables/
├─ docs/
│  ├─ reproducibility.md
│  └─ experiment_notes.md
└─ README.md
```

## Directory Roles

### `external/mujoco_wheeled_uav_simulator/`

Contains this simulator repository as a Git submodule.

- treat it as an upstream dependency
- do not store project-specific logic here
- update it intentionally and record the pinned commit used for experiments

### `controllers/`

Contains project-specific control logic.

- keep paper-specific or customer-specific controllers here
- add MATLAB controllers under `controllers/matlab/`
- add Python controllers under `controllers/python/`
- keep the simulator sample controllers separate from these files

### `experiments/`

Defines the experiment catalog for the project.

- one directory per experiment family is usually easiest to maintain
- store experiment entry scripts, scenario definitions, and notes here
- prefer explicit experiment names over generic scratch files

### `configs/`

Contains project-owned configuration layers.

- vehicle variants derived from the simulator defaults
- scenario-specific settings
- logging options
- sweep settings

The important rule is that the project repository owns the experiment configuration, even when the simulator owns the default baseline configuration.

### `analysis/`

Contains project-owned post-processing and evaluation.

- plotting scripts
- metric computation
- figure export for papers or reports
- comparison scripts between runs

### `scripts/`

Contains thin orchestration entry points.

- simulator launch wrappers
- controller launch wrappers
- experiment runners
- helper scripts for updating the submodule to a new commit

Keep these scripts small and declarative. They should call the simulator and project controllers, not reimplement simulator logic.

### `docs/`

Contains project-level operating notes.

- how to reproduce published results
- which submodule commit was used
- which config files were used for each figure or table
- deviations from simulator defaults

## Minimal Git Submodule Setup

Example:

```powershell
git submodule add <simulator-repository-url> external/mujoco_wheeled_uav_simulator
git submodule update --init --recursive
```

In the project README, explicitly state that contributors must clone with submodules or run the init command after cloning.

## Reproducibility Rules

For each important experiment, record at least:

- the project repository commit
- the submodule commit for `external/mujoco_wheeled_uav_simulator`
- the config files used
- the controller entry point used
- the log output directory
- the date of the run

This metadata can live in `docs/reproducibility.md`, in experiment manifests, or inside run-generated metadata files.

## Daily Development Rule

Use the project repository as the main working entry point.

- start experiments from the project repository
- keep project controllers and evaluation there
- only change the simulator submodule when the change is reusable beyond the current project

## Suggested First External Project

When you create the first project repository, keep it intentionally small.

- one controller
- one or two experiment families
- one reproducibility note
- one launch script for simulator and one for controller

That is enough to pressure-test whether the simulator boundary is well chosen before introducing more automation.

## Starter Template In This Repository

A concrete scaffold is included under [templates/project_repo](../templates/project_repo).

It demonstrates:

- a project-owned MATLAB controller
- PowerShell launch wrappers
- shell wrappers for Ubuntu/Linux
- project-side config override location
- project-side log and generated-XML ownership

The template flow has been validated once in this workspace on Windows using the same external-project style launch pattern.

The starter template now also includes project-owned MATLAB experiment and review entrypoints under `experiments/matlab/` and `analysis/matlab/`. These are the preferred places to orchestrate contact trials, formation runs, and post-run review without editing the simulator repository itself.
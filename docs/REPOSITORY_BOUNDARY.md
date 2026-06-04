# Repository Boundary

This repository should be treated as a shared simulator base, not as the final home for project-specific research code.

The long-term operating model is:

- keep this repository as the reusable simulator and sample-controller base
- keep each research or product project in its own repository
- reference this repository from those projects, typically as a Git submodule

## What Belongs In This Repository

The following items are part of the shared simulator base and should remain here.

- MuJoCo model generation and simulator runtime
- UDP communication and shared state/control packet conventions
- common parameter loading from `vehicle_params.json`
- terrain, contact, logging, and analysis primitives that are reusable across projects
- CLI commands such as `simulate` and `check-model`
- minimal sample controllers that demonstrate how to connect external control logic
- basic example experiments and analysis scripts that serve as reference workflows
- public-facing documentation, license, and citation metadata

## What Should Stay In A Project Repository

The following items should normally live outside this repository.

- controllers written for a specific paper, project, or customer requirement
- experiment orchestration tied to one research question
- project-specific configurations, scenario files, or parameter sweeps
- evaluation scripts and plots used only by one project
- publication assets, figure-generation scripts, and result tables
- temporary test code used to explore one project-specific hypothesis

## Core Versus Sample

This repository contains both core simulator components and sample control workflows. They should be treated differently.

### Core Components

Core components define the reusable simulator contract.

- `qav_wheel/`
- `vehicle_params.json`
- `qav_wheel.template.xml`
- top-level launch entry points such as `drone_sim.py`
- shared MATLAB support code under `matlab/shared/`

Changes to core components should be made only when they improve the reusable simulator for multiple future projects.

### Sample Components

Sample components exist to demonstrate usage and provide a baseline.

- thin top-level MATLAB wrappers such as `hovering_controller.m`
- `contact_test_controller.m`
- `multi_uav_formation_controller.m`
- `contact_log_review.m`
- `formation_log_review.m`
- implementation files under `matlab/controllers/`, `matlab/experiments/`, and `matlab/analysis/`

These files are intentionally useful, but they are still examples and reference workflows. They should not become the default location for project-specific controller development.

## Change Acceptance Rule

Before adding new logic to this repository, apply this check.

If the change answers one of the following questions with yes, it likely belongs here.

- Will multiple future projects need this capability?
- Does this change improve the simulator contract itself?
- Does this change reduce duplication across project repositories?
- Does this change make the sample workflows clearer without embedding project-specific assumptions?

If the answer is no, the change likely belongs in the external project repository instead.

## Recommended Future Project Layout

When another repository consumes this simulator, the intended layout is roughly:

```text
project-repo/
├─ external/
│  └─ mujoco_wheeled_uav_simulator/
├─ controllers/
├─ experiments/
├─ configs/
├─ analysis/
├─ results/
└─ docs/
```

This keeps the reusable simulator isolated while allowing each project to own its control logic, experiment setup, and evaluation pipeline.

## Practical Rule For Daily Work

During normal development:

- run project-specific experiments from the project repository
- treat this repository as an upstream dependency
- only modify this repository when the change is genuinely reusable
- push reusable changes back here, rather than maintaining a long-lived private fork inside a project

## Scope Of The Current Refactor

At this stage, the repository is moving toward a clearer split:

- Python runtime and MATLAB shared helpers are being centralized as reusable core APIs
- top-level MATLAB files are being reduced to thin entry wrappers where possible
- project-owned controllers are expected to live in the external project template or downstream repositories

The full directory move of experiment and analysis assets into project repositories is still in progress.

## Next Documents

The next-step operating guidance lives in the following documents.

- [Project Repository Template](PROJECT_REPOSITORY_TEMPLATE.md)
- [Project Integration Workflow](PROJECT_INTEGRATION_WORKFLOW.md)
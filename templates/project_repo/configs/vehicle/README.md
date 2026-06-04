# Vehicle Config Overrides

If the project needs a simulator configuration different from the simulator default, place the project-owned parameter file here.

Suggested filename:

- `vehicle_params.project.json`

Typical workflow:

1. Copy `external/mujoco_wheeled_uav_simulator/vehicle_params.json`.
2. Save the copy as `configs/vehicle/vehicle_params.project.json`.
3. Edit only the project-specific changes.
4. Launch the simulator with `--params-file` pointing to that file.

Recommended scope of overrides:

- project-specific environment and scenario values
- project-specific formation defaults
- controller defaults only when the project intentionally diverges from the simulator baseline

Avoid copying the file repeatedly for one-off experiment scratch changes. Keep durable project defaults here and put temporary sweep logic under `experiments/` or `configs/scenarios/`.

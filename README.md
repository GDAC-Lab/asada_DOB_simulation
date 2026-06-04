# mujoco-wheeled-uav-simulator

日本語のドキュメントは [README.ja.md](README.ja.md) をご覧ください。

`mujoco-wheeled-uav-simulator` is a MuJoCo-based simulator for a wheel-equipped quadrotor, with a Python simulator and MATLAB controllers communicating over UDP. It is intended both as a reusable simulation base for future research and as a reference implementation for reproducing control methods described in papers.

Planned features and open issues are tracked in [BACKLOG.md](BACKLOG.md).

Fidelity-mode usage and publication-oriented logging notes are summarized in [docs/FIDELITY_MODES.md](docs/FIDELITY_MODES.md).

## Highlights

- MuJoCo simulation with a wheel-equipped quadrotor model
- MATLAB hover, contact-test, and formation-control workflows
- Shared parameter source in `vehicle_params.json` for Python and MATLAB
- Support for single-UAV, multi-instance, and single-world multi-UAV runs
- Contact logging and post-run analysis with `.mat` outputs
- Plane, slope, and function-based curved terrain support

## Requirements

- Python 3.12 or newer
- `uv`
- A local environment capable of showing the MuJoCo GUI
- MATLAB

## Quick Start

Install dependencies:

```powershell
uv sync
```

Run the default single-UAV simulation:

```powershell
uv run mujoco-wheeled-uav-simulator simulate
```

For remote-controller experiments, you can split the simulator bind and state-destination IPs:

```powershell
uv run mujoco-wheeled-uav-simulator simulate --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

```matlab
hovering_controller
```

If you prefer the legacy launcher:

```powershell
uv run python drone_sim.py
```

## Common Workflows

### Single-UAV Hover

```powershell
uv run mujoco-wheeled-uav-simulator simulate
```

```matlab
hovering_controller
```

You can also run the hover controller from Python instead of MATLAB:

```powershell
uv run mujoco-wheeled-uav-simulator hover-controller
uv run mujoco-wheeled-uav-simulator hover-controller --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

### PC Simulator + Jetson Python Controller

This repository now treats Jetson-facing evaluation as two separate tracks: timing-faithful remote evaluation even when `realtime_factor < 1`, and near-real-time compute-budget evaluation when `realtime_factor ~= 1`. The concrete workflow is summarized in [docs/JETSON_EVALUATION_WORKFLOW.md](docs/JETSON_EVALUATION_WORKFLOW.md).

For a first remote setup, the simplest recommendation is:

- Run MuJoCo and `simulate` on a Windows or Linux PC with a GUI.
- Run `hover-controller` on the Jetson.
- Clone the same repository on both machines so that `vehicle_params.json` and the packet behavior stay aligned.
- Use `uv` on both sides if it is available in your Jetson environment.

Example setup:

On the PC running MuJoCo:

```powershell
uv sync
uv run mujoco-wheeled-uav-simulator simulate --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

On the Jetson running the Python controller:

```bash
uv sync
uv run mujoco-wheeled-uav-simulator hover-controller --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

In this example, replace `192.168.0.42` with the Jetson IP address and `192.168.0.10` with the PC IP address.

Current recommendation: start by cloning the same repository on the Jetson as well. That keeps the controller logic, packet format, and shared parameters consistent while the remote workflow is still evolving. Later, if Jetson-side installation should be lighter, the next refinement would be a controller-only dependency profile or a small standalone controller package.

Single-UAV controller packet contract used by both MATLAB and Python hover controllers:

- Simulator state packets are JSON objects containing at least `time`, `position`, `velocity`, `angular_velocity_body`, and `rotation_matrix`.
- `position`, `velocity`, and `angular_velocity_body` are 3-element vectors.
- `rotation_matrix` is a flattened 3x3 rotation matrix in row-major order.
- Controller command packets are JSON objects containing either `rotor_thrusts` or `rotor_omega`, each as a 4-element vector.
- `hover-controller` is for single-UAV packets only; if it receives a multi-UAV packet with `uavs`, it stops with an error.
- The current Python `hover-controller` assumes conventional vertical rotors. Fixed-tilt rotor examples remain model-only until the controller-side allocation is generalized.

Baseline and HIL modes:

- `--fidelity-mode baseline` keeps the idealized reference path: no injected network delay, no injected packet loss, and no additional sensor or actuator degradation.
- `--fidelity-mode hil` enables the runtime to honor `network_fidelity` settings from `vehicle_params.json`, including state transmit delay, command receive delay, jitter, packet loss, and stale-command handling.
- Python and MATLAB logs now preserve packet metadata such as `sequence`, `source_state_sequence`, `wall_time_send_ns`, `state_age_ms`, and controller compute time so that remote runs can be evaluated with the same dataset structure.

Example HIL-oriented run:

```powershell
uv run mujoco-wheeled-uav-simulator simulate --fidelity-mode hil --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
uv run mujoco-wheeled-uav-simulator hover-controller --fidelity-mode hil --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

Single-UAV Jetson review entrypoints:

```matlab
jetson_timing_review('logs/hover_20260410_120000.mat')
jetson_compute_budget_review('logs/hover_20260410_120000.mat')
```

### Multi-UAV Formation in One MuJoCo World

```powershell
uv run mujoco-wheeled-uav-simulator simulate --num-uavs 3
```

```matlab
multi_uav_formation_controller('num_uavs', 3)
multi_uav_formation_controller('num_uavs', 3, 'formation_radius', 2.0, 'spawn_radius', 2.0, 'base_height', 1.8)
```

### Multiple Independent Simulator Instances

This mode is mainly useful for isolated single-UAV experiments and comparisons.

```powershell
uv run mujoco-wheeled-uav-simulator simulate --instance-id 0
uv run mujoco-wheeled-uav-simulator simulate --instance-id 1
uv run mujoco-wheeled-uav-simulator simulate --instance-id 0 --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

```matlab
hovering_controller('instance_id', 0)
hovering_controller('instance_id', 1)
contact_test_controller('landing', 'instance_id', 2)
```

### Contact Tests and Log Review

```matlab
contact_test_controller('hover')
contact_test_controller('landing')
contact_test_controller('hard_landing')
contact_test_controller('ground_load')
contact_test_controller('wall')
contact_test_controller('wall_load')
```

```matlab
contact_log_review
contact_log_review('forces', 'logs/generated_log.mat')
contact_log_review('instantaneous', 'logs/generated_log.mat')
contact_log_review('network', 'logs/generated_log.mat')
```

### Model Validation Only

```powershell
uv run mujoco-wheeled-uav-simulator check-model
uv run mujoco-wheeled-uav-simulator check-model --instance-id 1
uv run mujoco-wheeled-uav-simulator check-model --num-uavs 3
```

## Citation

If you use this simulator in academic work, please cite the related paper, preprint, or project page for your implementation. Citation metadata is also provided in [CITATION.cff](CITATION.cff). You can replace the placeholder citation entry there once the publication details are finalized.

## License

This project is released under the [MIT License](LICENSE).

## Repository Boundary

This repository is intended to remain a reusable simulator base plus sample controllers, not the final home for project-specific research logic. The concrete operating policy for future submodule-based use is documented in [docs/REPOSITORY_BOUNDARY.md](docs/REPOSITORY_BOUNDARY.md).

## Advanced Reference

<details>
<summary>Repository layout</summary>

### Top-level files

| File | Role |
|------|------|
| `qav_wheel/` | Main Python simulator package. Terrain generation, XML construction, UDP communication, simulation execution, and CLI entry points are split into modules here. |
| `drone_sim.py` | Thin compatibility wrapper for older launch flows. Internally it delegates to the `qav_wheel` package. |
| `hovering_controller.m` | MATLAB entry point for hover control. The implementation lives in `matlab/controllers/hovering_controller_impl.m`. |
| `contact_test_controller.m` | MATLAB entry point for contact test scenarios. The implementation lives in `matlab/experiments/contact_test_controller_impl.m`. |
| `multi_uav_formation_controller.m` | MATLAB entry point for formation control in a single MuJoCo world containing multiple UAVs. |
| `contact_log_review.m` | Entry point for contact log analysis. The implementation lives in `matlab/analysis/contact_log_review_impl.m`. |
| `formation_log_review.m` | Entry point for reviewing formation-control logs across multiple UAVs. The implementation lives in `matlab/analysis/formation_log_review_impl.m`. |
| `matlab/shared/controller_shared.m` | Shared MATLAB utilities for communication, control calculations, and simulator-launch helpers. |
| `matlab/shared/simulation_logger.m` | MATLAB-side logger class that writes `.mat` logs under `logs/`. |
| `vehicle_params.json` | Shared vehicle, actuator, and environment parameters used by both Python and MATLAB. |
| `qav_wheel.template.xml` | MuJoCo model template. At runtime, `vehicle_params.json` is used to render XML into `build/generated_xml/`. |
| `drone_body.stl` | Vehicle mesh. |
| `pyproject.toml` | Python dependency and packaging metadata. |
| `uv.lock` | Lockfile for `uv`. |

### Python package overview

| Module | Role |
|------|------|
| `qav_wheel/cli.py` | CLI entry point. Dispatches `simulate` and `check-model`. |
| `qav_wheel/config.py` | Loads `vehicle_params.json`. |
| `qav_wheel/contact.py` | Aggregates MuJoCo contact data and builds contact reports. |
| `qav_wheel/model_builder.py` | Computes XML template replacements and writes generated XML artifacts under `build/generated_xml/`. |
| `qav_wheel/network.py` | UDP communication and interpretation of control inputs received from MATLAB. |
| `qav_wheel/paths.py` | Centralizes important repository paths and shared constants. |
| `qav_wheel/simulation.py` | Loads MuJoCo models, configures the viewer, sends state packets, and runs `check-model`. |
| `qav_wheel/surface.py` | Interprets plane, slope, and curved-surface settings, evaluates normals, and generates plane or hfield geometry. |
| `qav_wheel/types.py` | Shared Python dataclasses. |
| `qav_wheel/__init__.py` | Package-level public entry point exposing `main`. |

### MATLAB overview

| Path | Role |
|------|------|
| `matlab/controllers/hovering_controller_impl.m` | Standard hover-control implementation called from `hovering_controller.m`. |
| `matlab/experiments/contact_test_controller_impl.m` | Contact-test controller implementation called from `contact_test_controller.m`. |
| `matlab/experiments/multi_uav_formation_controller_impl.m` | Experimental controller that receives multiple UAV states from one simulator and maintains a circular formation. |
| `matlab/analysis/contact_log_review_impl.m` | Visualization and evaluation of saved contact logs. |
| `matlab/analysis/formation_log_review_impl.m` | Multi-UAV formation log review, including centroid error, slot error, real-time factor, and contact trends. |
| `matlab/shared/controller_shared.m` | Shared helpers for receiving state, computing control, sending commands, and optionally launching the simulator. |
| `matlab/shared/simulation_logger.m` | Logger class for saving state, control input, and contact summaries to `.mat` files. |

</details>

<details>
<summary>Ports, runtime modes, and communication</summary>

In single-instance mode, the Python simulator sends state to `127.0.0.1:5001`, and MATLAB sends per-rotor thrust or rotor-speed commands back to `127.0.0.1:5000`.

In multi-instance mode, ports are offset by `instance_id = i` as follows:

- simulator receive port: `5000 + 2*i`
- simulator state send port: `5001 + 2*i`

For example, with `instance_id = 1`, Python receives control commands on `5002` and sends state on `5003`. MATLAB must use the same `instance_id`, for example `hovering_controller('instance_id', 1)` or `contact_test_controller('landing', 'instance_id', 1)`.

If a MATLAB controller reports that its local UDP port is already in use, the usual cause is another MATLAB session or controller process still holding the same port. Recent versions of the shared controller runtime now stop early with a clearer diagnostic that includes the expected port and, on Windows, the owning process when it can be resolved. In normal use you do not need to change anything, but when running repeated tests it is safer to either fully close the earlier controller session or switch to another `instance_id`.

`multi_uav_formation_controller` is separate from the independent multi-instance flow and is the recommended path for formation experiments. With `simulate --num-uavs N`, one MuJoCo world contains `N` UAVs, and both state packets and control packets are exchanged as arrays.

</details>

<details>
<summary>Parameter management</summary>

Key vehicle and actuator parameters are centralized in `vehicle_params.json`. At the moment, this includes at least:

- gravity and simulation timestep
- arm lengths
- yaw moment coefficient
- maximum rotor thrust
- thrust conversion coefficient `thrust_coefficient`
- initial vehicle position
- main body and wheel dimensions and masses
- contact settings for floor, wall, body, and wheels
- plane or function-based curved surface settings under `environment.surface`
- fidelity settings under `fidelity_mode`, `network_fidelity`, `actuator_dynamics`, `sensor_fidelity`, and `logging_config`

For a publication-oriented workflow, treat `baseline` and `hil` as different experiment modes rather than small option tweaks. The detailed semantics, recommended metrics, and current implementation boundary are documented in [docs/FIDELITY_MODES.md](docs/FIDELITY_MODES.md).
- MuJoCo sensor names and their target body

On the Python side, MuJoCo XML is generated from `vehicle_params.json` and `qav_wheel.template.xml`. By default the output goes to `build/generated_xml/`: `qav_wheel.generated.xml` for `instance_id = 0`, and `qav_wheel.generated.instance_N.xml` for nonzero instance IDs.

On the MATLAB side, the same `vehicle_params.json` is used to load mass, gravity, arm lengths, yaw moment coefficient, maximum thrust, thrust conversion coefficient, and default hover/contact control gains.

Controller defaults are centralized under `controller` in `vehicle_params.json`. At present, at least the following values are read from there:

- `desired_heading`
- `position_gain`, `velocity_gain`
- `attitude_gain`, `angular_velocity_gain`

Default formation-control settings are centralized under `formation` in `vehicle_params.json`. At present, at least the following values are read from there:

- `num_uavs`
- `spawn_radius`
- `base_height`
- `centroid_target_xy`
- `formation_radius`
- `centroid_gain`
- `formation_gain`
- `duration_seconds`
- `idle_sleep_seconds`
- `status_display_interval`

Formation log review can be used like this:

```matlab
formation_log_review
formation_log_review('tracking')
formation_log_review('rtf')
formation_log_review('contacts')
formation_log_review('network')
formation_log_review('overview', 'logs/formation_bundle_20260410_220000.mat')
formation_log_review('overview', 'logs/formation_uav_1_20260410_220000.mat', 'logs/formation_uav_2_20260410_220000.mat', 'logs/formation_uav_3_20260410_220000.mat')
```

Formation runs now keep only the combined `formation_bundle*.mat` file by default, and the default `formation_log_review` path prefers that single-file bundle when it is available.
If you want to keep both the bundle and the per-UAV files, run `multi_uav_formation_controller('formation_log_mode', 'bundle_and_individual')`.
Inside the bundle, logs remain available both as an ordered cell array under `formation_log.logs` and as named fields such as `formation_log.uavs.uav_1`, `formation_log.uavs.uav_2`, and so on.

</details>

<details>
<summary>Curved surface environments</summary>

`vehicle_params.json` supports either a plane or a curved surface of the form `z = h(x, y)` via `environment.surface`.

For routine switching, the simplest approach is to change `environment.surface.mode`.

- `"mode": "plane"` or `"mode": "floor"` for a flat floor
- `"mode": "slope"` for a sloped plane
- `"mode": "paraboloid"`, `"mode": "sinusoidal"`, or `"mode": "gaussian"` for other built-in surfaces

For example, switching between a flat floor and a slope only needs this single field:

```json
"surface": {
	"mode": "plane",
	...
}
```

or

```json
"surface": {
	"mode": "slope",
	...
}
```

`mode` is just a convenience toggle. The detailed shape is still controlled by the existing `type`, `plane`, `height_function`, and `parameters` settings.

By default, `follow_surface_for_initial_position = true`, so `initial_position.z` is interpreted as a height relative to the local surface. This prevents the vehicle from spawning inside the terrain when switching to curved or raised surfaces. If needed, set it to `false` to treat the value as an absolute world coordinate instead.

During ground-contact initialization, the roll angle is chosen to satisfy left and right wheel contact conditions, and an initial pitch angle is added from the terrain gradient `dh/dx`. The initial wheel-to-ground clearance can be adjusted via `environment.surface.initial_wheel_contact_clearance`. The current default is `0.0001` m.

`type = "plane"`:

- uses MuJoCo's plane geom as before
- remains compatible with existing contact-test workflows

`type = "height_function"`:

- the Python side generates terrain from the `height_function` settings
- surfaces that can still be expressed as planes, such as `flat` and `slope`, are automatically converted into MuJoCo plane geoms
- nonplanar shapes such as `paraboloid` and `sinusoidal` are embedded as MuJoCo hfields
- supported function names are currently `flat`, `slope`, `paraboloid`, `sinusoidal`, and `gaussian`

Representative example:

```json
"surface": {
	"type": "height_function",
	"material": "floor_mat",
	"solref": [0.002, 1.0],
	"contact": {
		"contype": 1,
		"conaffinity": 1
	},
	"height_function": {
		"x_range": [-3.0, 3.0],
		"y_range": [-3.0, 3.0],
		"grid_resolution": [121, 121],
		"name": "slope",
		"parameters": {
			"z_offset": 0.0,
			"slope_x": 0.08,
			"slope_y": 0.0
		}
	}
}
```

For now, the project uses named functions with explicit parameters rather than evaluating raw expression strings. This is intentional for safety and maintainability.

`gaussian` is used to create hill- or bowl-shaped surfaces. Its main parameters are:

- `amplitude`: hill height; use a negative value for a depression
- `center_x`, `center_y`: center position
- `sigma_x`, `sigma_y`: spread of the hill

</details>

<details>
<summary>Fixed tilt rotors and input modes</summary>

If you want to represent fixed tilt rotors at the model level, use `actuation.rotors` in `vehicle_params.json`. Each rotor specifies its position and thrust axis in the body frame. The thrust axis is normalized automatically during loading, so the input does not need to be unit length.

```json
"actuation": {
	"command_mode": "omega",
	"max_rotor_thrust": 20.0,
	"yaw_moment_ratio": 0.02,
	"thrust_coefficient": 2.0e-5,
	"rotors": [
		{
			"name": "fr",
			"position_body": [0.075025, -0.100264, 0.0125],
			"thrust_axis_body": [-0.14834, 0.197905, 0.968912],
			"yaw_moment_ratio": 0.02,
			"spin_sign": 1
		}
	]
}
```

`spin_sign` specifies the reaction-torque direction. Use `1` and `-1`.

The default `vehicle_params.json` now uses conventional upward rotor axes (`[0, 0, 1]`) for all four rotors.

A fixed-tilt rotor example is provided in [vehicle_params.tilted_rotor_example.json](vehicle_params.tilted_rotor_example.json). It defines a symmetric 4-rotor layout tilted outward by about 14.3 degrees. To try it, copy that `actuation.rotors` section into `vehicle_params.json` and run `uv run mujoco-wheeled-uav-simulator check-model` to inspect the generated model. At the moment, the controller-side allocation matrix is not yet generalized, so this example should be treated primarily as a MuJoCo model and visualization sample.

The current default in `vehicle_params.json` is `actuation.command_mode = "omega"`. Changing this switches the MATLAB-side message format for both normal control and contact tests.

```json
"actuation": {
	"command_mode": "omega"
}
```

`command_mode = "thrust"`:

- MATLAB sends `rotor_thrusts` directly
- behavior is compatible with the original implementation

`command_mode = "omega"`:

- MATLAB converts controller thrust outputs into `rotor_omega = sqrt(T / k_f)` before sending
- Python converts them back to thrust using `actuation.thrust_coefficient` from `vehicle_params.json` and applies the result in MuJoCo
- there is currently no motor first-order lag or PWM model; those can be added later if needed

</details>

<details>
<summary>Auto launch from MATLAB</summary>

`hovering_controller.m` includes an option to launch the MuJoCo simulator directly from MATLAB. However, for clearer separation of responsibilities and easier debugging, that behavior is disabled by default.

To enable auto-launch, change the following inside `build_simulator_options` in `matlab/shared/controller_shared.m`:

```matlab
'auto_launch', true, ...
```

When enabled, the flow is:

1. MATLAB reserves the UDP receive port.
2. If `.venv\Scripts\python.exe` exists, it launches `drone_sim.py` with that interpreter.
3. If `.venv` does not exist, it tries `uv run python drone_sim.py`.

</details>

<details>
<summary>Logging and analysis</summary>

The MATLAB-side `simulation_logger` saves the following data to `.mat` files:

- `meta`: save time and related metadata
- `config`: controller gains, allocation matrix, targets, and other settings
- `state`: time, position, velocity, angular velocity, and rotation matrix
- `control`: per-rotor thrust, and per-rotor speed when applicable
- `reference`: target position
- `contact`: contact count, contact-force summaries, and per-sample contact details

Logs are written under `logs/`. Since `logs/` is generated output, it is included in `.gitignore`.

You can change the save mode in `build_logging_options` inside `hovering_controller.m`.

- `finalize`: save once on shutdown
- `periodic`: overwrite-save at a fixed interval
- `periodic_and_finalize`: periodic saves plus a final save on shutdown

In addition to overall contact summaries, `contact` includes per-group summaries for `left_wheel`, `right_wheel`, and `surface`. `contact.details` stores, for each time sample, the counterpart geom names, contact position, penetration distance, force and torque in the contact frame, and normal force. For curved-surface contact, `surface_contact`, `surface_height`, and `surface_normal` are also logged. At the moment, the implementation records MuJoCo's per-step contact forces directly and does not perform any impulse post-processing.

Saved contact logs can be reviewed with `contact_log_review.m`.

```matlab
contact_log_review
contact_log_review('noncontact')
contact_log_review('landing', 'logs/hover_20260410_120000.mat')
contact_log_review('wall', 'logs/hover_20260410_121000.mat')
contact_log_review('instantaneous', 'logs/hover_20260410_121000.mat')
contact_log_review('impact_compare', 'logs/soft.mat', 'logs/hard.mat')
```

Main modes:

- `overview`: basic review of the latest log
- `noncontact`: confirm the non-contact baseline
- `landing`: check landing time and floor contact
- `wall`: check wall contact
- `forces`: review total contact force and left/right wheel contact force
- `instantaneous`: raw instantaneous contact-force time series and zoomed view
- `impact_compare`: compare peak values across two logs

</details>

<details>
<summary>Troubleshooting</summary>

- If `uv` is not found, install it and reopen your shell.
- If the MuJoCo window does not appear, make sure you are running in a local environment with GUI support.
- If you see port conflicts when starting MATLAB controllers, check whether stale `udpport` objects remain in the same MATLAB session.
- If you use auto-launch, either `.venv` or `uv` must provide a working Python execution path.

</details>

## Notes

- `vehicle_params.json`, `qav_wheel.template.xml`, and `drone_body.stl` are expected to remain at the repository root. Both the Python package and MATLAB code resolve paths relative to that layout.
- UDP communication is fixed to localhost.
- On startup, the MATLAB side attempts to release stale `udpport` objects from earlier sessions.
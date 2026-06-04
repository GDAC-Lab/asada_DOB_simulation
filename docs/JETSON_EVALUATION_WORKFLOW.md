# Jetson Evaluation Workflow

This document describes the intended evaluation workflow when MuJoCo runs on a PC and the Python controller runs on a Jetson.

## Purpose

The repository supports two different Jetson-oriented evaluation tracks.

- `timing-faithful remote evaluation`: confirm that wall-clock packet timing, age tracking, stale-command behavior, and delay injection remain interpretable even when `realtime_factor` is below one
- `near-real-time compute-budget evaluation`: confirm that the Jetson controller can sustain the target loop budget while the simulator remains close to real time

These are different experimental goals and should be reported separately.

## Shared Setup

Recommended topology:

- run `simulate` on a Windows or Linux PC
- run `hover-controller` on the Jetson
- keep the same repository revision and `vehicle_params.json` on both machines
- bind the simulator to `0.0.0.0` and send state packets to the Jetson IP
- bind the Jetson controller to `0.0.0.0` and target commands back to the PC IP

PC side:

```powershell
uv sync
uv run mujoco-wheeled-uav-simulator simulate --fidelity-mode hil --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

Jetson side:

```bash
uv sync
uv run mujoco-wheeled-uav-simulator hover-controller --fidelity-mode hil --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

Replace `192.168.0.42` with the Jetson IP and `192.168.0.10` with the PC IP.

## Track 1: Timing-Faithful Remote Evaluation

Use this track when the main claim is that the PC simulator and Jetson controller remain wall-clock consistent even if `realtime_factor` drops below one.

Recommended settings:

- use `--fidelity-mode hil`
- enable `network_fidelity`
- configure representative state transmit delay, command receive delay, jitter, and stale-command thresholds
- keep packet metadata and MATLAB logging enabled

Recommended review points:

- verify `state_age_ms` and `command_packet_age_ms` trends
- verify `sequence_gap` and stale-command counts
- check whether the stale policy applied matches the intended safety behavior
- treat reduced `realtime_factor` as acceptable if the runtime instrumentation remains internally consistent

Single-UAV review entrypoints:

```matlab
jetson_timing_review('logs/hover_20260410_120000.mat')
contact_log_review('network', 'logs/hover_20260410_120000.mat')
```

Multi-UAV review entrypoints:

```matlab
formation_log_review('network')
```

## Track 2: Near-Real-Time Compute-Budget Evaluation

Use this track when the main claim is that the Jetson can sustain the target controller rate while the simulator remains near real time.

Recommended settings:

- start from the same remote topology
- prefer moderate scene complexity and logging load first, then scale up
- keep `realtime_factor` close to one for the target scenario
- use `hil` if remote-link timing should remain part of the benchmark, or `baseline` if you need a controller-compute reference without injected disturbances

Recommended review points:

- check `realtime_factor`
- check controller compute time against the control period budget
- review packet age under sustained load
- compare tracking degradation against a `baseline` reference run

Single-UAV review entrypoints:

```matlab
jetson_compute_budget_review('logs/hover_20260410_120000.mat')
contact_log_review('network', 'logs/hover_20260410_120000.mat')
```

Multi-UAV review entrypoints:

```matlab
formation_log_review('rtf')
formation_log_review('network')
```

## Minimum Reported Metrics

For timing-faithful remote evaluation:

- state packet age
- command packet age
- sequence gap rate
- stale-command rate
- timeout count

For near-real-time compute-budget evaluation:

- realtime factor
- controller compute time
- packet age under sustained load
- tracking degradation relative to `baseline`

## Current Boundary

The repository now supports packet metadata, runtime age tracking, stale-command handling, HIL-style network disturbance injection, core actuator dynamics, and additive sensor noise with truth logging. Remaining gaps are experiment-batch automation and publication-ready figure regeneration rather than the basic Jetson-facing timing instrumentation itself.
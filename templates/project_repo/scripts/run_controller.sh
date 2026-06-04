#!/usr/bin/env bash
set -euo pipefail

INSTANCE_ID="${INSTANCE_ID:-0}"
CONTROLLER_NAME="${CONTROLLER_NAME:-my_project_controller}"
AUTO_LAUNCH="${AUTO_LAUNCH:-false}"

matlab -batch "addpath('controllers/matlab'); addpath('external/mujoco_wheeled_uav_simulator'); addpath('external/mujoco_wheeled_uav_simulator/matlab'); addpath('external/mujoco_wheeled_uav_simulator/matlab/shared'); ${CONTROLLER_NAME}('instance_id',${INSTANCE_ID},'auto_launch',${AUTO_LAUNCH});"

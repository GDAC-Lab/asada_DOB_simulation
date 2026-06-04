param(
    [int]$InstanceId = 0,
    [string]$ExperimentName = 'project_contact_trials',
    [string]$Scenario = 'landing'
)

$matlabCommand = "addpath('experiments/matlab'); addpath('controllers/matlab'); addpath('external/mujoco_wheeled_uav_simulator'); addpath('external/mujoco_wheeled_uav_simulator/matlab'); addpath('external/mujoco_wheeled_uav_simulator/matlab/shared'); $ExperimentName('instance_id',$InstanceId,'scenario','$Scenario','auto_launch',true);"
matlab -batch $matlabCommand

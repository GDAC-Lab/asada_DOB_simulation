param(
    [int]$InstanceId = 0,
    [switch]$AutoLaunch,
    [string]$ControllerName = 'my_project_controller'
)

$autoLaunchValue = if ($AutoLaunch.IsPresent) { 'true' } else { 'false' }
$matlabCommand = "addpath('controllers/matlab'); addpath('external/mujoco_wheeled_uav_simulator'); addpath('external/mujoco_wheeled_uav_simulator/matlab'); addpath('external/mujoco_wheeled_uav_simulator/matlab/shared'); $ControllerName('instance_id',$InstanceId,'auto_launch',$autoLaunchValue);"
matlab -batch $matlabCommand

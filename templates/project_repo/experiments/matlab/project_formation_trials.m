function project_formation_trials(varargin)
close all; clc;

runtime_options = parse_runtime_options(varargin{:});
project_directory = fileparts(fileparts(fileparts(mfilename('fullpath'))));
simulator_root = fullfile(project_directory, 'external', 'mujoco_wheeled_uav_simulator');
params_path = resolve_params_path(project_directory, runtime_options.params_path);
generated_xml_directory = fullfile(project_directory, 'build', 'generated_xml');

experiment_directory = fullfile(simulator_root, 'matlab', 'experiments');
shared_directory = fullfile(simulator_root, 'matlab', 'shared');
addpath(experiment_directory, shared_directory);
cleanup_handler = onCleanup(@() rmpath(experiment_directory, shared_directory)); %#ok<NASGU>

multi_uav_formation_controller_impl( ...
    'num_uavs', runtime_options.num_uavs, ...
    'instance_id', runtime_options.instance_id, ...
    'spawn_radius', runtime_options.spawn_radius, ...
    'duration_seconds', runtime_options.duration_seconds, ...
    'auto_launch', runtime_options.auto_launch, ...
    'shutdown_on_exit', runtime_options.shutdown_on_exit, ...
    'target_ip', runtime_options.target_ip, ...
    'params_path', params_path, ...
    'simulator_root', simulator_root, ...
    'generated_xml_directory', generated_xml_directory ...
);
end


function params_path = resolve_params_path(project_directory, override_path)
if ~isempty(override_path)
    params_path = override_path;
    return;
end

candidate = fullfile(project_directory, 'configs', 'vehicle', 'vehicle_params.project.json');
if isfile(candidate)
    params_path = candidate;
    return;
end

params_path = fullfile(project_directory, 'external', 'mujoco_wheeled_uav_simulator', 'vehicle_params.json');
end


function runtime_options = parse_runtime_options(varargin)
parser = inputParser;
addParameter(parser, 'num_uavs', 3, @(value) validateattributes(value, {'numeric'}, {'scalar', 'integer', 'positive'}));
addParameter(parser, 'instance_id', 0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'integer', 'nonnegative'}));
addParameter(parser, 'spawn_radius', 1.5, @(value) validateattributes(value, {'numeric'}, {'scalar', 'positive'}));
addParameter(parser, 'duration_seconds', 20.0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'positive'}));
addParameter(parser, 'target_ip', '127.0.0.1', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'auto_launch', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'shutdown_on_exit', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'params_path', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
parse(parser, varargin{:});

runtime_options = parser.Results;
runtime_options.num_uavs = double(runtime_options.num_uavs);
runtime_options.instance_id = double(runtime_options.instance_id);
runtime_options.spawn_radius = double(runtime_options.spawn_radius);
runtime_options.duration_seconds = double(runtime_options.duration_seconds);
runtime_options.target_ip = char(runtime_options.target_ip);
runtime_options.auto_launch = logical(runtime_options.auto_launch);
runtime_options.shutdown_on_exit = logical(runtime_options.shutdown_on_exit);
runtime_options.params_path = char(runtime_options.params_path);
end
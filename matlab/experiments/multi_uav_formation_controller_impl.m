function multi_uav_formation_controller_impl(varargin)
close all; clc;

runtime_options = parse_runtime_options(varargin{:});
project_directory = fileparts(fileparts(fileparts(mfilename('fullpath'))));
params_path = resolve_runtime_path_option(runtime_options.params_path, fullfile(project_directory, 'vehicle_params.json'));
vehicle_params = controller_shared.load_vehicle_params(project_directory, 'params_path', params_path);
runtime_options = apply_runtime_defaults(runtime_options, vehicle_params.formation);
num_uavs = runtime_options.num_uavs;
desired_offsets_xy = build_circular_offsets(num_uavs, runtime_options.formation_radius);
target_centroid_xy = reshape(double(runtime_options.centroid_target_xy), 2, 1);
controller_session = controller_shared.initialize_controller_session( ...
    project_directory, ...
    runtime_options, ...
    'target_ip', runtime_options.target_ip, ...
    'simulator_root', resolve_runtime_path_option(runtime_options.simulator_root, project_directory), ...
    'params_path', params_path, ...
    'generated_xml_directory', resolve_runtime_path_option(runtime_options.generated_xml_directory, fullfile(project_directory, 'build', 'generated_xml')) ...
);
instance_options = controller_session.instance_options;
controller_socket = controller_session.controller_socket;
socket_cleanup_handler = onCleanup(@() controller_shared.cleanup_controller_socket(controller_socket)); %#ok<NASGU>
simulator_cleanup_handler = onCleanup(@() controller_shared.cleanup_simulator_process(controller_session.simulator_process_id, controller_session.simulator_options)); %#ok<NASGU>

position_gain = runtime_options.position_gain;
velocity_gain = runtime_options.velocity_gain;
attitude_gain = runtime_options.attitude_gain;
angular_velocity_gain = runtime_options.angular_velocity_gain;
desired_heading = runtime_options.desired_heading;

[allocation_matrix, mixer] = controller_shared.build_allocation_and_mixer(vehicle_params);
command_options = controller_shared.build_command_options(vehicle_params.command_mode, vehicle_params.thrust_coefficient, 'fidelity_mode', vehicle_params.fidelity.mode);
runtime_metrics = controller_shared.initialize_runtime_metrics();

loggers = build_loggers(project_directory, num_uavs, instance_options, runtime_options, ...
    vehicle_params, position_gain, velocity_gain, attitude_gain, angular_velocity_gain, desired_heading, allocation_matrix, mixer, command_options, desired_offsets_xy, target_centroid_xy);
logger_cleanup_handler = onCleanup(@() finalize_formation_logging(loggers, project_directory, instance_options)); %#ok<NASGU>

status_display_interval = runtime_options.status_display_interval;
next_status_time = 0.0;
start_time = NaN;
idle_deadline = tic;

fprintf('Starting multi-UAV formation controller for %d UAVs.\n', num_uavs);
    fprintf('Simulator routing -> %s, recv=%d, send=%d\n', instance_options.label, instance_options.controller_local_port, controller_session.target_port);
fprintf('Formation radius: %.2f m, spawn radius: %.2f m, base height: %.2f m, centroid target=[%.2f %.2f] m\n', ...
    runtime_options.formation_radius, runtime_options.spawn_radius, runtime_options.base_height, target_centroid_xy(1), target_centroid_xy(2));

try
    while true
        state_packet = controller_shared.read_latest_state(controller_socket);
        if isempty(state_packet) || ~isfield(state_packet, 'uavs')
            if toc(idle_deadline) >= runtime_options.state_timeout_seconds
                runtime_metrics = controller_shared.note_timeout(runtime_metrics);
                error('No simulator state received within %.1f s.', runtime_options.state_timeout_seconds);
            end
            pause(runtime_options.idle_sleep_seconds);
            continue;
        end

        idle_deadline = tic;

        states = normalize_uav_states(state_packet.uavs, num_uavs);
        if isempty(states)
            pause(runtime_options.idle_sleep_seconds);
            continue;
        end

        simulation_time = double(state_packet.time);
        if isnan(start_time)
            start_time = simulation_time;
        end
        elapsed_simulation_time = simulation_time - start_time;
        if isfinite(runtime_options.duration_seconds) && elapsed_simulation_time >= runtime_options.duration_seconds
            fprintf('Formation run complete at t=%.2f s\n', elapsed_simulation_time);
            break;
        end

        positions_xy = gather_xy_positions(states);
        centroid_xy = mean(positions_xy, 2);
        centroid_error_xy = target_centroid_xy - centroid_xy;
        max_slot_error = 0.0;
        realtime_factors = zeros(num_uavs, 1);
        control_commands = cell(num_uavs, 1);
        state_metrics = controller_shared.get_state_packet_metrics(states{1});
        command_wall_time_ns = controller_shared.wall_time_now_ns();

        for uav_index = 1:num_uavs
            state = states{uav_index};
            current_position = reshape(double(state.position), [], 1);
            current_relative_xy = current_position(1:2) - centroid_xy;
            slot_error_xy = desired_offsets_xy(:, uav_index) - current_relative_xy;
            target_xy = current_position(1:2) ...
                + runtime_options.centroid_gain * centroid_error_xy ...
                + runtime_options.formation_gain * slot_error_xy;
            target_position = [target_xy; runtime_options.base_height];

            compute_timer = tic;
            rotor_thrusts = controller_shared.compute_hover_control( ...
                state, ...
                target_position, ...
                desired_heading, ...
                vehicle_params.mass, ...
                vehicle_params.gravity, ...
                position_gain, ...
                velocity_gain, ...
                attitude_gain, ...
                angular_velocity_gain, ...
                mixer, ...
                vehicle_params.max_rotor_thrust ...
            );
            runtime_metrics = controller_shared.update_runtime_metrics(runtime_metrics, state, toc(compute_timer) * 1.0e3);

            control_command = controller_shared.build_control_command( ...
                rotor_thrusts, ...
                command_options, ...
                'sequence', runtime_metrics.command_sequence, ...
                'source_state_sequence', state_metrics.sequence, ...
                'wall_time_send_ns', command_wall_time_ns, ...
                'controller_compute_ms', runtime_metrics.last_controller_compute_ms, ...
                'state_age_ms', runtime_metrics.last_state_age_ms, ...
                'state_sequence_gap', runtime_metrics.last_state_sequence_gap, ...
                'fidelity_mode', vehicle_params.fidelity.mode ...
            );
            control_commands{uav_index} = control_command;
            loggers{uav_index}.append(state, control_command, target_position);
            realtime_factors(uav_index) = controller_shared.get_realtime_factor(state);
            max_slot_error = max(max_slot_error, norm(slot_error_xy));
        end

        controller_shared.send_multi_uav_control_command(controller_socket, control_commands, command_options, controller_session.target_ip, controller_session.target_port);

        if simulation_time >= next_status_time
            display_status(elapsed_simulation_time, centroid_xy, centroid_error_xy, max_slot_error, realtime_factors, num_uavs);
            next_status_time = simulation_time + status_display_interval;
        end

        pause(runtime_options.idle_sleep_seconds);
    end
catch execution_error
    fprintf('\nMulti-UAV formation controller stopped: %s\n', execution_error.message);
end
end


function runtime_options = parse_runtime_options(varargin)
parser = inputParser;
addParameter(parser, 'num_uavs', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'instance_id', 0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'integer', 'nonnegative'}));
addParameter(parser, 'formation_radius', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'spawn_radius', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'base_height', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'centroid_target_xy', [], @(value) isempty(value) || (isnumeric(value) && numel(value) == 2));
addParameter(parser, 'centroid_gain', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'formation_gain', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'duration_seconds', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'idle_sleep_seconds', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'status_display_interval', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'wait_for_startup_seconds', 3.0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'positive'}));
addParameter(parser, 'state_timeout_seconds', inf, @(value) (isnumeric(value) && isscalar(value) && value > 0) || isinf(value));
addParameter(parser, 'headless', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'simulation_duration_seconds', NaN, @(value) (isnumeric(value) && isscalar(value)) || isempty(value));
addParameter(parser, 'desired_heading', [], @(value) isempty(value) || (isnumeric(value) && numel(value) == 3));
addParameter(parser, 'position_gain', [], @(value) isempty(value) || (isnumeric(value) && numel(value) == 3));
addParameter(parser, 'velocity_gain', [], @(value) isempty(value) || (isnumeric(value) && numel(value) == 3));
addParameter(parser, 'attitude_gain', [], @(value) isempty(value) || (isnumeric(value) && numel(value) == 3));
addParameter(parser, 'angular_velocity_gain', [], @(value) isempty(value) || (isnumeric(value) && numel(value) == 3));
addParameter(parser, 'target_ip', '127.0.0.1', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'auto_launch', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'shutdown_on_exit', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'simulator_root', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'params_path', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'generated_xml_directory', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'formation_log_mode', 'bundle_only', @(value) any(strcmp(char(value), {'bundle_and_individual', 'bundle_only'})));
parse(parser, varargin{:});

runtime_options = parser.Results;
runtime_options.instance_id = double(runtime_options.instance_id);
runtime_options.target_ip = char(runtime_options.target_ip);
runtime_options.wait_for_startup_seconds = double(runtime_options.wait_for_startup_seconds);
runtime_options.state_timeout_seconds = double(runtime_options.state_timeout_seconds);
runtime_options.headless = logical(runtime_options.headless);
runtime_options.simulation_duration_seconds = double(runtime_options.simulation_duration_seconds);
runtime_options.auto_launch = logical(runtime_options.auto_launch);
runtime_options.shutdown_on_exit = logical(runtime_options.shutdown_on_exit);
runtime_options.simulator_root = char(runtime_options.simulator_root);
runtime_options.params_path = char(runtime_options.params_path);
runtime_options.generated_xml_directory = char(runtime_options.generated_xml_directory);
runtime_options.formation_log_mode = char(runtime_options.formation_log_mode);
end


function runtime_options = apply_runtime_defaults(runtime_options, formation_defaults)
runtime_options.num_uavs = resolve_scalar_option(runtime_options.num_uavs, formation_defaults.num_uavs);
runtime_options.formation_radius = resolve_scalar_option(runtime_options.formation_radius, formation_defaults.formation_radius);
runtime_options.spawn_radius = resolve_scalar_option(runtime_options.spawn_radius, formation_defaults.spawn_radius);
runtime_options.base_height = resolve_scalar_option(runtime_options.base_height, formation_defaults.base_height);
runtime_options.centroid_gain = resolve_scalar_option(runtime_options.centroid_gain, formation_defaults.centroid_gain);
runtime_options.formation_gain = resolve_scalar_option(runtime_options.formation_gain, formation_defaults.formation_gain);
runtime_options.duration_seconds = resolve_scalar_option(runtime_options.duration_seconds, formation_defaults.duration_seconds);
runtime_options.idle_sleep_seconds = resolve_scalar_option(runtime_options.idle_sleep_seconds, formation_defaults.idle_sleep_seconds);
runtime_options.status_display_interval = resolve_scalar_option(runtime_options.status_display_interval, formation_defaults.status_display_interval);
runtime_options.centroid_target_xy = resolve_vector_option(runtime_options.centroid_target_xy, formation_defaults.centroid_target_xy, 2);
runtime_options.desired_heading = resolve_vector_option(runtime_options.desired_heading, formation_defaults.desired_heading, 3);
runtime_options.position_gain = resolve_vector_option(runtime_options.position_gain, formation_defaults.position_gain, 3);
runtime_options.velocity_gain = resolve_vector_option(runtime_options.velocity_gain, formation_defaults.velocity_gain, 3);
runtime_options.attitude_gain = resolve_vector_option(runtime_options.attitude_gain, formation_defaults.attitude_gain, 3);
runtime_options.angular_velocity_gain = resolve_vector_option(runtime_options.angular_velocity_gain, formation_defaults.angular_velocity_gain, 3);
end


function value = resolve_scalar_option(value, default_value)
if isempty(value)
    value = default_value;
else
    value = double(value);
end
end


function value = resolve_vector_option(value, default_value, expected_length)
if isempty(value)
    value = reshape(double(default_value), [], 1);
else
    value = reshape(double(value), [], 1);
end
if numel(value) ~= expected_length
    error('Expected vector of length %d.', expected_length);
end
end


function desired_offsets_xy = build_circular_offsets(num_uavs, formation_radius)
angles = 2.0 * pi * (0:(num_uavs - 1)) / num_uavs;
desired_offsets_xy = formation_radius * [cos(angles); sin(angles)];
end


function states = normalize_uav_states(raw_uav_states, expected_count)
states = cell(expected_count, 1);
if numel(raw_uav_states) ~= expected_count
    states = {};
    return;
end

for uav_index = 1:expected_count
    states{uav_index} = raw_uav_states(uav_index);
end
end


function positions_xy = gather_xy_positions(states)
num_uavs = numel(states);
positions_xy = zeros(2, num_uavs);
for uav_index = 1:num_uavs
    position = reshape(double(states{uav_index}.position), [], 1);
    positions_xy(:, uav_index) = position(1:2);
end
end


function loggers = build_loggers(project_directory, num_uavs, instance_options, runtime_options, ...
    vehicle_params, ...
    position_gain, velocity_gain, attitude_gain, angular_velocity_gain, desired_heading, allocation_matrix, mixer, command_options, desired_offsets_xy, target_centroid_xy)
loggers = cell(num_uavs, 1);
controller_config = controller_shared.build_controller_config( ...
    vehicle_params, ...
    'desired_heading', desired_heading, ...
    'position_gain', position_gain, ...
    'velocity_gain', velocity_gain, ...
    'attitude_gain', attitude_gain, ...
    'angular_velocity_gain', angular_velocity_gain ...
);
for uav_index = 1:num_uavs
    logging_options = struct( ...
        'save_mode', 'finalize', ...
        'periodic_interval_seconds', 30.0, ...
        'print_save_events', ~strcmp(runtime_options.formation_log_mode, 'bundle_only'), ...
        'directory_name', 'logs', ...
        'file_prefix', sprintf('formation_uav_%d%s', uav_index, instance_options.file_suffix), ...
        'formation_log_mode', runtime_options.formation_log_mode ...
    );

    config = controller_shared.build_base_logger_config(vehicle_params, controller_config, allocation_matrix, mixer, command_options, instance_options);
    config.controller = 'multi_uav_formation_controller';
    config.uav_index = uav_index;
    config.num_uavs = num_uavs;
    config.formation_radius = runtime_options.formation_radius;
    config.spawn_radius = runtime_options.spawn_radius;
    config.base_height = runtime_options.base_height;
    config.centroid_target_xy = target_centroid_xy;
    config.desired_offset_xy = desired_offsets_xy(:, uav_index);
    config.centroid_gain = runtime_options.centroid_gain;
    config.formation_gain = runtime_options.formation_gain;

    loggers{uav_index} = simulation_logger(project_directory, config, logging_options);
end
end


function finalize_formation_logging(loggers, project_directory, instance_options)
for uav_index = 1:numel(loggers)
    if isempty(loggers{uav_index})
        continue;
    end
    controller_shared.finalize_controller_run(loggers{uav_index});
end

write_combined_formation_log(loggers, project_directory, instance_options);

if strcmp(resolve_formation_log_mode(loggers), 'bundle_only')
    delete_individual_formation_logs(loggers);
end
end


function write_combined_formation_log(loggers, project_directory, instance_options)
log_paths = cellfun(@(logger) char(logger.get_file_path()), loggers, 'UniformOutput', false);
existing_mask = cellfun(@isfile, log_paths);
log_paths = log_paths(existing_mask);
if isempty(log_paths)
    return;
end

loaded_logs = cell(numel(log_paths), 1);
for log_index = 1:numel(log_paths)
    loaded_data = load(log_paths{log_index}, 'log');
    if ~isfield(loaded_data, 'log')
        error('The file does not contain a log variable: %s', log_paths{log_index});
    end
    loaded_log = loaded_data.log;
    loaded_log.source_path = log_paths{log_index};
    loaded_logs{log_index} = loaded_log;
end

uav_indices = cellfun(@(log_entry) double(log_entry.config.uav_index), loaded_logs);
[~, sort_index] = sort(uav_indices);
loaded_logs = loaded_logs(sort_index);
log_paths = log_paths(sort_index);

timestamp = extract_timestamp_from_path(log_paths{1});
if isempty(timestamp)
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end

formation_log = struct();
formation_log.meta = struct( ...
    'format_version', 1, ...
    'created_at', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    'num_uavs', numel(loaded_logs), ...
    'instance_id', instance_options.instance_id, ...
    'instance_label', instance_options.label, ...
    'timestamp', timestamp, ...
    'bundle_layout', 'logs_and_named_uavs' ...
);
formation_log.logs = {loaded_logs};
formation_log.source_paths = {log_paths};
formation_log.uavs = build_named_uav_log_struct(loaded_logs);

log_directory = fullfile(project_directory, 'logs');
bundle_path = fullfile(log_directory, sprintf('formation_bundle%s_%s.mat', instance_options.file_suffix, timestamp));
save(bundle_path, 'formation_log');
fprintf('Combined formation log saved -> %s\n', bundle_path);
end


function named_uavs = build_named_uav_log_struct(loaded_logs)
named_uavs = struct();
for log_index = 1:numel(loaded_logs)
    field_name = sprintf('uav_%d', log_index);
    named_uavs.(field_name) = loaded_logs{log_index};
end
end


function formation_log_mode = resolve_formation_log_mode(loggers)
formation_log_mode = 'bundle_only';
if isempty(loggers) || isempty(loggers{1})
    return;
end
logger_options = loggers{1}.get_options();
if isfield(logger_options, 'formation_log_mode')
    formation_log_mode = char(logger_options.formation_log_mode);
end
end


function delete_individual_formation_logs(loggers)
for log_index = 1:numel(loggers)
    if isempty(loggers{log_index})
        continue;
    end
    file_path = char(loggers{log_index}.get_file_path());
    if isfile(file_path)
        delete(file_path);
    end
end
fprintf('Individual formation logs removed after bundle generation.\n');
end


function timestamp = extract_timestamp_from_path(file_path)
[~, file_name, extension] = fileparts(file_path);
timestamp = regexp([file_name, extension], '\d{8}_\d{6}(?=\.mat$)', 'match', 'once');
if isempty(timestamp)
    timestamp = '';
end
end


function path_value = resolve_runtime_path_option(path_value, default_value)
if isempty(path_value)
    path_value = default_value;
end
path_value = char(path_value);
end


function display_status(elapsed_simulation_time, centroid_xy, centroid_error_xy, max_slot_error, realtime_factors, num_uavs)
average_realtime_factor = mean(realtime_factors);
fprintf('[formation t=%.2f s, avg_rtf=%.2f] centroid=[%.3f %.3f] m, centroid_err=[%.3f %.3f] m, max_slot_err=%.3f m, uavs=%d\n', ...
    elapsed_simulation_time, ...
    average_realtime_factor, ...
    centroid_xy(1), centroid_xy(2), ...
    centroid_error_xy(1), centroid_error_xy(2), ...
    max_slot_error, ...
    num_uavs ...
);
end
function hovering_controller_impl(varargin)
% === MATLAB-side control script ===
close all; clc;

runtime_options = parse_runtime_options(varargin{:});
project_directory = fileparts(fileparts(fileparts(mfilename('fullpath'))));
controller_session = controller_shared.initialize_controller_session( ...
    project_directory, ...
    runtime_options, ...
    'simulator_root', resolve_runtime_path_option(runtime_options.simulator_root, project_directory), ...
    'params_path', resolve_runtime_path_option(runtime_options.params_path, fullfile(project_directory, 'vehicle_params.json')), ...
    'generated_xml_directory', resolve_runtime_path_option(runtime_options.generated_xml_directory, fullfile(project_directory, 'build', 'generated_xml')) ...
);
vehicle_params = controller_session.vehicle_params;
instance_options = controller_session.instance_options;
controller_socket = controller_session.controller_socket;
socket_cleanup_handler = onCleanup(@() controller_shared.cleanup_controller_socket(controller_socket)); %#ok<NASGU>
simulator_cleanup_handler = onCleanup(@() controller_shared.cleanup_simulator_process(controller_session.simulator_process_id, controller_session.simulator_options)); %#ok<NASGU>

controller_config = controller_shared.build_controller_config(vehicle_params, 'target_position', [0.0; 0.0; 1.5]);
[allocation_matrix, mixer] = controller_shared.build_allocation_and_mixer(vehicle_params);
command_options = controller_shared.build_command_options(vehicle_params.command_mode, vehicle_params.thrust_coefficient, 'fidelity_mode', vehicle_params.fidelity.mode);
logging_options = build_logging_options(instance_options);
runtime_metrics = controller_shared.initialize_runtime_metrics();

logger = simulation_logger(project_directory, build_logging_config( ...
    vehicle_params, controller_config, allocation_matrix, mixer, command_options, instance_options ...
), logging_options);
next_log_save_time = logging_options.periodic_interval_seconds;
cleanup_handler = onCleanup(@() controller_shared.finalize_controller_run(logger)); %#ok<NASGU>

status_display_interval = 2.0;
next_status_time = 0.0;
start_time = NaN;
idle_deadline = tic;

fprintf('制御を開始します (%s, recv=%d, send=%d)。終了する場合は Ctrl+C を押してください。\n', ...
    instance_options.label, ...
    instance_options.controller_local_port, ...
    instance_options.simulator_receive_port ...
);
controller_shared.display_logging_behavior(logger);

try
    while true
        state = controller_shared.read_latest_state(controller_socket);
        if isempty(state)
            if toc(idle_deadline) >= runtime_options.state_timeout_seconds
                runtime_metrics = controller_shared.note_timeout(runtime_metrics);
                error('No simulator state received within %.1f s.', runtime_options.state_timeout_seconds);
            end
            continue;
        end

        idle_deadline = tic;

        if isnan(start_time)
            start_time = double(state.time);
        end

        elapsed_simulation_time = double(state.time) - start_time;
        if isfinite(runtime_options.duration_seconds) && elapsed_simulation_time >= runtime_options.duration_seconds
            fprintf('Hover run complete at t=%.2f s\n', elapsed_simulation_time);
            break;
        end

        compute_timer = tic;
        rotor_thrusts = controller_shared.compute_hover_control( ...
            state, ...
            controller_config.target_position, ...
            controller_config.desired_heading, ...
            vehicle_params.mass, ...
            vehicle_params.gravity, ...
            controller_config.position_gain, ...
            controller_config.velocity_gain, ...
            controller_config.attitude_gain, ...
            controller_config.angular_velocity_gain, ...
            mixer, ...
            vehicle_params.max_rotor_thrust ...
        );
        runtime_metrics = controller_shared.update_runtime_metrics(runtime_metrics, state, toc(compute_timer) * 1.0e3);

        control_command = controller_shared.build_control_command( ...
            rotor_thrusts, ...
            command_options, ...
            'sequence', runtime_metrics.command_sequence, ...
            'source_state_sequence', runtime_metrics.last_source_state_sequence, ...
            'wall_time_send_ns', controller_shared.wall_time_now_ns(), ...
            'controller_compute_ms', runtime_metrics.last_controller_compute_ms, ...
            'state_age_ms', runtime_metrics.last_state_age_ms, ...
            'state_sequence_gap', runtime_metrics.last_state_sequence_gap, ...
            'fidelity_mode', vehicle_params.fidelity.mode ...
        );
        controller_shared.send_control_command(controller_socket, control_command, controller_session.target_ip, controller_session.target_port);

        logger.append(state, control_command, controller_config.target_position);

        if should_save_log_periodically(logging_options, state.time, next_log_save_time)
            logger.save_snapshot(double(state.time), 'periodic');
            if logging_options.print_save_events
                fprintf('Simulation log checkpoint saved at t=%.2f s -> %s\n', double(state.time), logger.get_file_path());
            end
            next_log_save_time = next_log_save_time + logging_options.periodic_interval_seconds;
        end

        if state.time >= next_status_time
            display_status(state, controller_config.target_position, control_command, command_options, instance_options.label);
            next_status_time = state.time + status_display_interval;
        end
    end
catch execution_error
    fprintf('\nController stopped: %s\n', execution_error.message);
end
end


function display_status(state, target_position, control_command, command_options, instance_label)
position = reshape(double(state.position), [], 1);
position_error = target_position - position;
angular_velocity = reshape(double(state.angular_velocity_body), [], 1);
command_values = controller_shared.displayed_command_values(control_command, command_options);
realtime_factor = controller_shared.get_realtime_factor(state);
fprintf( ...
    '[%s t=%.2f s, rtf=%.2f] pos=[%.3f %.3f %.3f] m, err=[%.3f %.3f %.3f] m, omega=[%.3f %.3f %.3f] rad/s, cmd=%s [%.3f %.3f %.3f %.3f] %s\n', ...
    instance_label, ...
    state.time, ...
    realtime_factor, ...
    position(1), position(2), position(3), ...
    position_error(1), position_error(2), position_error(3), ...
    angular_velocity(1), angular_velocity(2), angular_velocity(3), ...
    command_options.input_mode, ...
    command_values(1), command_values(2), command_values(3), command_values(4), ...
    controller_shared.command_unit_label(command_options.input_mode) ...
);
end


function config = build_logging_config(vehicle_params, controller_config, allocation_matrix, mixer, command_options, instance_options)
config = controller_shared.build_base_logger_config(vehicle_params, controller_config, allocation_matrix, mixer, command_options, instance_options);
config.target_position = controller_config.target_position;
end


function logging_options = build_logging_options(instance_options)
logging_options = struct( ...
    'save_mode', 'finalize', ...
    'periodic_interval_seconds', 30.0, ...
    'print_save_events', true, ...
    'directory_name', 'logs', ...
    'file_prefix', ['hover' instance_options.file_suffix] ...
);
end


function runtime_options = parse_runtime_options(varargin)
parser = inputParser;
addParameter(parser, 'instance_id', 0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'integer', 'nonnegative'}));
addParameter(parser, 'duration_seconds', inf, @(value) (isnumeric(value) && isscalar(value) && value > 0) || isinf(value));
addParameter(parser, 'wait_for_startup_seconds', 3.0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'positive'}));
addParameter(parser, 'state_timeout_seconds', inf, @(value) (isnumeric(value) && isscalar(value) && value > 0) || isinf(value));
addParameter(parser, 'headless', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'simulation_duration_seconds', NaN, @(value) (isnumeric(value) && isscalar(value)) || isempty(value));
addParameter(parser, 'auto_launch', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'shutdown_on_exit', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'simulator_root', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'params_path', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'generated_xml_directory', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
parse(parser, varargin{:});

runtime_options = parser.Results;
runtime_options.instance_id = double(runtime_options.instance_id);
runtime_options.duration_seconds = double(runtime_options.duration_seconds);
runtime_options.wait_for_startup_seconds = double(runtime_options.wait_for_startup_seconds);
runtime_options.state_timeout_seconds = double(runtime_options.state_timeout_seconds);
runtime_options.headless = logical(runtime_options.headless);
runtime_options.simulation_duration_seconds = double(runtime_options.simulation_duration_seconds);
runtime_options.auto_launch = logical(runtime_options.auto_launch);
runtime_options.shutdown_on_exit = logical(runtime_options.shutdown_on_exit);
runtime_options.simulator_root = char(runtime_options.simulator_root);
runtime_options.params_path = char(runtime_options.params_path);
runtime_options.generated_xml_directory = char(runtime_options.generated_xml_directory);
end


function should_save = should_save_log_periodically(logging_options, simulation_time, next_log_save_time)
supports_periodic = strcmp(logging_options.save_mode, 'periodic') || strcmp(logging_options.save_mode, 'periodic_and_finalize');
has_valid_interval = logging_options.periodic_interval_seconds > 0.0;
should_save = supports_periodic && has_valid_interval && simulation_time >= next_log_save_time;
end


function path_value = resolve_runtime_path_option(path_value, default_value)
if isempty(path_value)
    path_value = default_value;
end
path_value = char(path_value);
end
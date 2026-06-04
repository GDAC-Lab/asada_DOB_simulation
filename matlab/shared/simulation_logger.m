classdef simulation_logger < handle
    properties (Access = private)
        capacity
        count
        file_path
        config
        options
        meta
        finalized
        save_count
        last_simulation_time
        realtime_factor
        state_protocol_version
        state_sequence
        state_wall_time_send_ns
        state_age_ms
        state_fidelity_mode
        time
        position
        velocity
        angular_velocity_body
        angular_velocity_world
        rotation_matrix
        sensor_truth_position
        sensor_truth_velocity
        sensor_truth_angular_velocity_body
        sensor_truth_angular_velocity_world
        sensor_truth_rotation_matrix
        rotor_thrusts
        rotor_omega
        command_protocol_version
        command_sequence
        command_source_state_sequence
        command_wall_time_send_ns
        command_fidelity_mode
        controller_compute_ms
        state_sequence_gap
        actuator_requested_rotor_thrusts
        actuator_applied_rotor_thrusts
        actuator_tracking_error
        target_position
        contact_count
        total_contact_force_magnitude
        max_contact_force_magnitude
        total_contact_normal_force
        max_contact_normal_force
        left_wheel_contact_count
        left_wheel_total_force_magnitude
        left_wheel_total_normal_force
        left_wheel_max_normal_force
        right_wheel_contact_count
        right_wheel_total_force_magnitude
        right_wheel_total_normal_force
        right_wheel_max_normal_force
        surface_contact_count
        surface_total_force_magnitude
        surface_total_normal_force
        surface_max_normal_force
        contact_details
    end

    methods
        function obj = simulation_logger(base_directory, config, options)
            arguments
                base_directory (1, :) char
                config struct
                options struct
            end

            obj.capacity = 5000;
            obj.count = 0;
            obj.finalized = false;
            obj.save_count = 0;
            obj.last_simulation_time = NaN;
            obj.config = config;
            obj.options = options;
            obj.meta = struct( ...
                'controller', 'hovering_controller', ...
                'created_at', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
                'format_version', 3, ...
                'save_count', 0, ...
                'last_save_reason', '', ...
                'last_save_wall_time', '', ...
                'last_save_simulation_time', NaN ...
            );

            log_directory = fullfile(base_directory, options.directory_name);
            if ~exist(log_directory, 'dir')
                mkdir(log_directory);
            end

            timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            obj.file_path = fullfile(log_directory, sprintf('%s_%s.mat', options.file_prefix, timestamp));
            obj.initialize_buffers(obj.capacity);
        end

        function append(obj, state, control_command, target_position)
            next_index = obj.count + 1;
            obj.ensure_capacity(next_index);

            [logged_rotor_thrusts, logged_rotor_omega] = unpack_control_command(control_command);
            contact_payload = extract_contact_payload(state);
            state_network_payload = extract_state_network_payload(state);
            command_network_payload = extract_command_network_payload(control_command);
            sensor_truth_payload = extract_sensor_truth_payload(state);
            actuator_payload = extract_actuator_payload(state);

            obj.time(next_index, 1) = double(state.time);
            obj.last_simulation_time = double(state.time);
            obj.realtime_factor(next_index, 1) = get_realtime_factor(state);
            obj.state_protocol_version(next_index, 1) = state_network_payload.protocol_version;
            obj.state_sequence(next_index, 1) = state_network_payload.sequence;
            obj.state_wall_time_send_ns(next_index, 1) = state_network_payload.wall_time_send_ns;
            obj.state_age_ms(next_index, 1) = state_network_payload.age_ms;
            obj.state_fidelity_mode{next_index, 1} = state_network_payload.fidelity_mode;
            obj.position(next_index, :) = reshape(double(state.position), 1, 3);
            obj.velocity(next_index, :) = reshape(double(state.velocity), 1, 3);
            obj.angular_velocity_body(next_index, :) = reshape(double(state.angular_velocity_body), 1, 3);
            obj.angular_velocity_world(next_index, :) = reshape(double(state.angular_velocity_world), 1, 3);
            obj.rotation_matrix(next_index, :) = reshape(double(state.rotation_matrix), 1, 9);
            obj.sensor_truth_position(next_index, :) = sensor_truth_payload.position;
            obj.sensor_truth_velocity(next_index, :) = sensor_truth_payload.velocity;
            obj.sensor_truth_angular_velocity_body(next_index, :) = sensor_truth_payload.angular_velocity_body;
            obj.sensor_truth_angular_velocity_world(next_index, :) = sensor_truth_payload.angular_velocity_world;
            obj.sensor_truth_rotation_matrix(next_index, :) = sensor_truth_payload.rotation_matrix;
            obj.rotor_thrusts(next_index, :) = reshape(double(logged_rotor_thrusts), 1, 4);
            obj.rotor_omega(next_index, :) = reshape(double(logged_rotor_omega), 1, 4);
            obj.command_protocol_version(next_index, 1) = command_network_payload.protocol_version;
            obj.command_sequence(next_index, 1) = command_network_payload.sequence;
            obj.command_source_state_sequence(next_index, 1) = command_network_payload.source_state_sequence;
            obj.command_wall_time_send_ns(next_index, 1) = command_network_payload.wall_time_send_ns;
            obj.command_fidelity_mode{next_index, 1} = command_network_payload.fidelity_mode;
            obj.controller_compute_ms(next_index, 1) = command_network_payload.controller_compute_ms;
            obj.state_sequence_gap(next_index, 1) = command_network_payload.state_sequence_gap;
            obj.actuator_requested_rotor_thrusts(next_index, :) = actuator_payload.requested_rotor_thrusts;
            obj.actuator_applied_rotor_thrusts(next_index, :) = actuator_payload.applied_rotor_thrusts;
            obj.actuator_tracking_error(next_index, :) = actuator_payload.tracking_error;
            obj.target_position(next_index, :) = reshape(double(target_position), 1, 3);
            obj.contact_count(next_index, 1) = contact_payload.count;
            obj.total_contact_force_magnitude(next_index, 1) = contact_payload.total_force_magnitude;
            obj.max_contact_force_magnitude(next_index, 1) = contact_payload.max_force_magnitude;
            obj.total_contact_normal_force(next_index, 1) = contact_payload.total_normal_force;
            obj.max_contact_normal_force(next_index, 1) = contact_payload.max_normal_force;
            obj.left_wheel_contact_count(next_index, 1) = contact_payload.left_wheel.count;
            obj.left_wheel_total_force_magnitude(next_index, 1) = contact_payload.left_wheel.total_force_magnitude;
            obj.left_wheel_total_normal_force(next_index, 1) = contact_payload.left_wheel.total_normal_force;
            obj.left_wheel_max_normal_force(next_index, 1) = contact_payload.left_wheel.max_normal_force;
            obj.right_wheel_contact_count(next_index, 1) = contact_payload.right_wheel.count;
            obj.right_wheel_total_force_magnitude(next_index, 1) = contact_payload.right_wheel.total_force_magnitude;
            obj.right_wheel_total_normal_force(next_index, 1) = contact_payload.right_wheel.total_normal_force;
            obj.right_wheel_max_normal_force(next_index, 1) = contact_payload.right_wheel.max_normal_force;
            obj.surface_contact_count(next_index, 1) = contact_payload.surface.count;
            obj.surface_total_force_magnitude(next_index, 1) = contact_payload.surface.total_force_magnitude;
            obj.surface_total_normal_force(next_index, 1) = contact_payload.surface.total_normal_force;
            obj.surface_max_normal_force(next_index, 1) = contact_payload.surface.max_normal_force;
            obj.contact_details{next_index, 1} = contact_payload.details;

            obj.count = next_index;
        end

        function save_snapshot(obj, simulation_time, reason)
            arguments
                obj
                simulation_time (1, 1) double = NaN
                reason (1, :) char = 'manual'
            end

            if obj.finalized
                return;
            end

            obj.write_log(simulation_time, reason);
        end

        function finalize(obj, simulation_time)
            arguments
                obj
                simulation_time (1, 1) double = obj.last_simulation_time
            end

            if obj.finalized
                return;
            end

            obj.write_log(simulation_time, 'finalize');
            obj.finalized = true;
        end

        function file_path = get_file_path(obj)
            file_path = obj.file_path;
        end

        function options = get_options(obj)
            options = obj.options;
        end
    end

    methods (Access = private)
        function write_log(obj, simulation_time, reason)
            if nargin < 2
                simulation_time = NaN;
            end
            if nargin < 3
                reason = 'manual';
            end

            log = struct();
            obj.save_count = obj.save_count + 1;
            obj.meta.save_count = obj.save_count;
            obj.meta.last_save_reason = reason;
            obj.meta.last_save_wall_time = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
            obj.meta.last_save_simulation_time = simulation_time;
            log.meta = obj.meta;
            log.config = obj.config;
            log.options = obj.options;
            log.state = struct( ...
                'time', obj.time(1:obj.count, :), ...
                'realtime_factor', obj.realtime_factor(1:obj.count, :), ...
                'protocol_version', obj.state_protocol_version(1:obj.count, :), ...
                'sequence', obj.state_sequence(1:obj.count, :), ...
                'wall_time_send_ns', obj.state_wall_time_send_ns(1:obj.count, :), ...
                'age_ms', obj.state_age_ms(1:obj.count, :), ...
                'fidelity_mode', {obj.state_fidelity_mode(1:obj.count, :)}, ...
                'position', obj.position(1:obj.count, :), ...
                'velocity', obj.velocity(1:obj.count, :), ...
                'angular_velocity_body', obj.angular_velocity_body(1:obj.count, :), ...
                'angular_velocity_world', obj.angular_velocity_world(1:obj.count, :), ...
                'rotation_matrix', obj.rotation_matrix(1:obj.count, :) ...
            );
            log.sensor_truth = struct( ...
                'position', obj.sensor_truth_position(1:obj.count, :), ...
                'velocity', obj.sensor_truth_velocity(1:obj.count, :), ...
                'angular_velocity_body', obj.sensor_truth_angular_velocity_body(1:obj.count, :), ...
                'angular_velocity_world', obj.sensor_truth_angular_velocity_world(1:obj.count, :), ...
                'rotation_matrix', obj.sensor_truth_rotation_matrix(1:obj.count, :) ...
            );
            log.control = struct( ...
                'command_mode', obj.config.command_mode, ...
                'rotor_thrusts', obj.rotor_thrusts(1:obj.count, :), ...
                'rotor_omega', obj.rotor_omega(1:obj.count, :), ...
                'protocol_version', obj.command_protocol_version(1:obj.count, :), ...
                'sequence', obj.command_sequence(1:obj.count, :), ...
                'source_state_sequence', obj.command_source_state_sequence(1:obj.count, :), ...
                'wall_time_send_ns', obj.command_wall_time_send_ns(1:obj.count, :), ...
                'fidelity_mode', {obj.command_fidelity_mode(1:obj.count, :)}, ...
                'controller_compute_ms', obj.controller_compute_ms(1:obj.count, :), ...
                'state_sequence_gap', obj.state_sequence_gap(1:obj.count, :) ...
            );
            log.actuator = struct( ...
                'requested_rotor_thrusts', obj.actuator_requested_rotor_thrusts(1:obj.count, :), ...
                'applied_rotor_thrusts', obj.actuator_applied_rotor_thrusts(1:obj.count, :), ...
                'tracking_error', obj.actuator_tracking_error(1:obj.count, :) ...
            );
            log.network = struct( ...
                'state_age_ms', obj.state_age_ms(1:obj.count, :), ...
                'state_sequence', obj.state_sequence(1:obj.count, :), ...
                'command_sequence', obj.command_sequence(1:obj.count, :), ...
                'command_source_state_sequence', obj.command_source_state_sequence(1:obj.count, :), ...
                'state_sequence_gap', obj.state_sequence_gap(1:obj.count, :), ...
                'controller_compute_ms', obj.controller_compute_ms(1:obj.count, :) ...
            );
            log.reference = struct( ...
                'target_position', obj.target_position(1:obj.count, :) ...
            );
            log.contact = struct( ...
                'count', obj.contact_count(1:obj.count, :), ...
                'total_force_magnitude', obj.total_contact_force_magnitude(1:obj.count, :), ...
                'max_force_magnitude', obj.max_contact_force_magnitude(1:obj.count, :), ...
                'total_normal_force', obj.total_contact_normal_force(1:obj.count, :), ...
                'max_normal_force', obj.max_contact_normal_force(1:obj.count, :), ...
                'left_wheel', struct( ...
                    'count', obj.left_wheel_contact_count(1:obj.count, :), ...
                    'total_force_magnitude', obj.left_wheel_total_force_magnitude(1:obj.count, :), ...
                    'total_normal_force', obj.left_wheel_total_normal_force(1:obj.count, :), ...
                    'max_normal_force', obj.left_wheel_max_normal_force(1:obj.count, :) ...
                ), ...
                'right_wheel', struct( ...
                    'count', obj.right_wheel_contact_count(1:obj.count, :), ...
                    'total_force_magnitude', obj.right_wheel_total_force_magnitude(1:obj.count, :), ...
                    'total_normal_force', obj.right_wheel_total_normal_force(1:obj.count, :), ...
                    'max_normal_force', obj.right_wheel_max_normal_force(1:obj.count, :) ...
                ), ...
                'surface', struct( ...
                    'count', obj.surface_contact_count(1:obj.count, :), ...
                    'total_force_magnitude', obj.surface_total_force_magnitude(1:obj.count, :), ...
                    'total_normal_force', obj.surface_total_normal_force(1:obj.count, :), ...
                    'max_normal_force', obj.surface_max_normal_force(1:obj.count, :) ...
                ), ...
                'details', {obj.contact_details(1:obj.count, :)} ...
            );

            save(obj.file_path, 'log');
        end

        function initialize_buffers(obj, capacity)
            obj.time = nan(capacity, 1);
            obj.realtime_factor = nan(capacity, 1);
            obj.state_protocol_version = nan(capacity, 1);
            obj.state_sequence = nan(capacity, 1);
            obj.state_wall_time_send_ns = nan(capacity, 1);
            obj.state_age_ms = nan(capacity, 1);
            obj.state_fidelity_mode = cell(capacity, 1);
            obj.position = nan(capacity, 3);
            obj.velocity = nan(capacity, 3);
            obj.angular_velocity_body = nan(capacity, 3);
            obj.angular_velocity_world = nan(capacity, 3);
            obj.rotation_matrix = nan(capacity, 9);
            obj.sensor_truth_position = nan(capacity, 3);
            obj.sensor_truth_velocity = nan(capacity, 3);
            obj.sensor_truth_angular_velocity_body = nan(capacity, 3);
            obj.sensor_truth_angular_velocity_world = nan(capacity, 3);
            obj.sensor_truth_rotation_matrix = nan(capacity, 9);
            obj.rotor_thrusts = nan(capacity, 4);
            obj.rotor_omega = nan(capacity, 4);
            obj.command_protocol_version = nan(capacity, 1);
            obj.command_sequence = nan(capacity, 1);
            obj.command_source_state_sequence = nan(capacity, 1);
            obj.command_wall_time_send_ns = nan(capacity, 1);
            obj.command_fidelity_mode = cell(capacity, 1);
            obj.controller_compute_ms = nan(capacity, 1);
            obj.state_sequence_gap = nan(capacity, 1);
            obj.actuator_requested_rotor_thrusts = nan(capacity, 4);
            obj.actuator_applied_rotor_thrusts = nan(capacity, 4);
            obj.actuator_tracking_error = nan(capacity, 4);
            obj.target_position = nan(capacity, 3);
            obj.contact_count = zeros(capacity, 1);
            obj.total_contact_force_magnitude = nan(capacity, 1);
            obj.max_contact_force_magnitude = nan(capacity, 1);
            obj.total_contact_normal_force = nan(capacity, 1);
            obj.max_contact_normal_force = nan(capacity, 1);
            obj.left_wheel_contact_count = zeros(capacity, 1);
            obj.left_wheel_total_force_magnitude = nan(capacity, 1);
            obj.left_wheel_total_normal_force = nan(capacity, 1);
            obj.left_wheel_max_normal_force = nan(capacity, 1);
            obj.right_wheel_contact_count = zeros(capacity, 1);
            obj.right_wheel_total_force_magnitude = nan(capacity, 1);
            obj.right_wheel_total_normal_force = nan(capacity, 1);
            obj.right_wheel_max_normal_force = nan(capacity, 1);
            obj.surface_contact_count = zeros(capacity, 1);
            obj.surface_total_force_magnitude = nan(capacity, 1);
            obj.surface_total_normal_force = nan(capacity, 1);
            obj.surface_max_normal_force = nan(capacity, 1);
            obj.contact_details = cell(capacity, 1);
        end

        function ensure_capacity(obj, required_capacity)
            if required_capacity <= obj.capacity
                return;
            end

            new_capacity = max(required_capacity, obj.capacity * 2);
            obj.time(new_capacity, 1) = nan;
            obj.realtime_factor(new_capacity, 1) = nan;
            obj.state_protocol_version(new_capacity, 1) = nan;
            obj.state_sequence(new_capacity, 1) = nan;
            obj.state_wall_time_send_ns(new_capacity, 1) = nan;
            obj.state_age_ms(new_capacity, 1) = nan;
            obj.state_fidelity_mode{new_capacity, 1} = '';
            obj.position(new_capacity, 3) = nan;
            obj.velocity(new_capacity, 3) = nan;
            obj.angular_velocity_body(new_capacity, 3) = nan;
            obj.angular_velocity_world(new_capacity, 3) = nan;
            obj.rotation_matrix(new_capacity, 9) = nan;
            obj.sensor_truth_position(new_capacity, 3) = nan;
            obj.sensor_truth_velocity(new_capacity, 3) = nan;
            obj.sensor_truth_angular_velocity_body(new_capacity, 3) = nan;
            obj.sensor_truth_angular_velocity_world(new_capacity, 3) = nan;
            obj.sensor_truth_rotation_matrix(new_capacity, 9) = nan;
            obj.rotor_thrusts(new_capacity, 4) = nan;
            obj.rotor_omega(new_capacity, 4) = nan;
            obj.command_protocol_version(new_capacity, 1) = nan;
            obj.command_sequence(new_capacity, 1) = nan;
            obj.command_source_state_sequence(new_capacity, 1) = nan;
            obj.command_wall_time_send_ns(new_capacity, 1) = nan;
            obj.command_fidelity_mode{new_capacity, 1} = '';
            obj.controller_compute_ms(new_capacity, 1) = nan;
            obj.state_sequence_gap(new_capacity, 1) = nan;
            obj.actuator_requested_rotor_thrusts(new_capacity, 4) = nan;
            obj.actuator_applied_rotor_thrusts(new_capacity, 4) = nan;
            obj.actuator_tracking_error(new_capacity, 4) = nan;
            obj.target_position(new_capacity, 3) = nan;
            obj.contact_count(new_capacity, 1) = 0;
            obj.total_contact_force_magnitude(new_capacity, 1) = nan;
            obj.max_contact_force_magnitude(new_capacity, 1) = nan;
            obj.total_contact_normal_force(new_capacity, 1) = nan;
            obj.max_contact_normal_force(new_capacity, 1) = nan;
            obj.left_wheel_contact_count(new_capacity, 1) = 0;
            obj.left_wheel_total_force_magnitude(new_capacity, 1) = nan;
            obj.left_wheel_total_normal_force(new_capacity, 1) = nan;
            obj.left_wheel_max_normal_force(new_capacity, 1) = nan;
            obj.right_wheel_contact_count(new_capacity, 1) = 0;
            obj.right_wheel_total_force_magnitude(new_capacity, 1) = nan;
            obj.right_wheel_total_normal_force(new_capacity, 1) = nan;
            obj.right_wheel_max_normal_force(new_capacity, 1) = nan;
            obj.surface_contact_count(new_capacity, 1) = 0;
            obj.surface_total_force_magnitude(new_capacity, 1) = nan;
            obj.surface_total_normal_force(new_capacity, 1) = nan;
            obj.surface_max_normal_force(new_capacity, 1) = nan;
            obj.contact_details{new_capacity, 1} = [];
            obj.capacity = new_capacity;
        end
    end
end


function value = get_contact_summary_field(state, field_name)
value = 0.0;
if ~isfield(state, 'contact_summary')
    return;
end
if ~isfield(state.contact_summary, field_name)
    return;
end

value = double(state.contact_summary.(field_name));
end


function value = get_realtime_factor(state)
value = 0.0;
if ~isfield(state, 'realtime_factor')
    return;
end

value = double(state.realtime_factor);
end


function value = get_nested_contact_summary_field(state, group_name, field_name)
value = 0.0;
if ~isfield(state, 'contact_summary')
    return;
end
if ~isfield(state.contact_summary, group_name)
    return;
end

group = state.contact_summary.(group_name);
if ~isfield(group, field_name)
    return;
end

value = double(group.(field_name));
end


function contact_payload = extract_contact_payload(state)
contact_payload = struct( ...
    'count', get_contact_summary_field(state, 'count'), ...
    'total_force_magnitude', get_contact_summary_field(state, 'total_force_magnitude'), ...
    'max_force_magnitude', get_contact_summary_field(state, 'max_force_magnitude'), ...
    'total_normal_force', get_contact_summary_field(state, 'total_normal_force'), ...
    'max_normal_force', get_contact_summary_field(state, 'max_normal_force'), ...
    'left_wheel', build_contact_group_payload(state, 'left_wheel'), ...
    'right_wheel', build_contact_group_payload(state, 'right_wheel'), ...
    'surface', build_contact_group_payload(state, 'surface'), ...
    'details', get_contact_details(state) ...
);
end


function group_payload = build_contact_group_payload(state, group_name)
group_payload = struct( ...
    'count', get_nested_contact_summary_field(state, group_name, 'count'), ...
    'total_force_magnitude', get_nested_contact_summary_field(state, group_name, 'total_force_magnitude'), ...
    'total_normal_force', get_nested_contact_summary_field(state, group_name, 'total_normal_force'), ...
    'max_normal_force', get_nested_contact_summary_field(state, group_name, 'max_normal_force') ...
);
end


function details = get_contact_details(state)
details = struct([]);
if ~isfield(state, 'contacts')
    return;
end

details = state.contacts;
end


function [rotor_thrusts, rotor_omega] = unpack_control_command(control_command)
rotor_thrusts = nan(1, 4);
rotor_omega = nan(1, 4);

if isnumeric(control_command)
    rotor_thrusts = reshape(double(control_command), 1, 4);
    return;
end

if ~isstruct(control_command)
    error('control_command must be numeric or struct.');
end

if isfield(control_command, 'rotor_thrusts')
    rotor_thrusts = reshape(double(control_command.rotor_thrusts), 1, 4);
end

if isfield(control_command, 'rotor_omega')
    rotor_omega = reshape(double(control_command.rotor_omega), 1, 4);
end
end


function payload = extract_state_network_payload(state)
payload = struct( ...
    'protocol_version', 1.0, ...
    'sequence', NaN, ...
    'wall_time_send_ns', NaN, ...
    'age_ms', NaN, ...
    'fidelity_mode', '' ...
);

if ~isfield(state, 'packet_metrics')
    return;
end

metrics = state.packet_metrics;
payload.protocol_version = get_struct_numeric(metrics, 'protocol_version', 1.0);
payload.sequence = get_struct_numeric(metrics, 'sequence', NaN);
payload.wall_time_send_ns = get_struct_numeric(metrics, 'wall_time_send_ns', NaN);
payload.age_ms = get_struct_numeric(metrics, 'age_ms', NaN);
payload.fidelity_mode = get_struct_char(metrics, 'fidelity_mode', '');
end


function payload = extract_command_network_payload(control_command)
payload = struct( ...
    'protocol_version', 1.0, ...
    'sequence', NaN, ...
    'source_state_sequence', NaN, ...
    'wall_time_send_ns', NaN, ...
    'fidelity_mode', '', ...
    'controller_compute_ms', NaN, ...
    'state_sequence_gap', NaN ...
);

if isfield(control_command, 'packet_metadata')
    metadata = control_command.packet_metadata;
    payload.protocol_version = get_struct_numeric(metadata, 'protocol_version', 1.0);
    payload.sequence = get_struct_numeric(metadata, 'sequence', NaN);
    payload.source_state_sequence = get_struct_numeric(metadata, 'source_state_sequence', NaN);
    payload.wall_time_send_ns = get_struct_numeric(metadata, 'wall_time_send_ns', NaN);
    payload.fidelity_mode = get_struct_char(metadata, 'fidelity_mode', '');
end

if isfield(control_command, 'runtime_metrics')
    metrics = control_command.runtime_metrics;
    payload.controller_compute_ms = get_struct_numeric(metrics, 'controller_compute_ms', NaN);
    payload.state_sequence_gap = get_struct_numeric(metrics, 'state_sequence_gap', NaN);
end
end


function value = get_struct_numeric(input_struct, field_name, default_value)
value = default_value;
if ~isstruct(input_struct) || ~isfield(input_struct, field_name)
    return;
end

raw_value = input_struct.(field_name);
if isempty(raw_value)
    return;
end

value = double(raw_value);
end


function value = get_struct_char(input_struct, field_name, default_value)
value = default_value;
if ~isstruct(input_struct) || ~isfield(input_struct, field_name)
    return;
end

raw_value = input_struct.(field_name);
if ischar(raw_value)
    value = raw_value;
elseif isstring(raw_value) && isscalar(raw_value)
    value = char(raw_value);
end
end


function payload = extract_sensor_truth_payload(state)
payload = struct( ...
    'position', nan(1, 3), ...
    'velocity', nan(1, 3), ...
    'angular_velocity_body', nan(1, 3), ...
    'angular_velocity_world', nan(1, 3), ...
    'rotation_matrix', nan(1, 9) ...
);

if ~isfield(state, 'sensor_truth')
    return;
end

truth = state.sensor_truth;
payload.position = get_struct_row_vector(truth, 'position', 3);
payload.velocity = get_struct_row_vector(truth, 'velocity', 3);
payload.angular_velocity_body = get_struct_row_vector(truth, 'angular_velocity_body', 3);
payload.angular_velocity_world = get_struct_row_vector(truth, 'angular_velocity_world', 3);
payload.rotation_matrix = get_struct_row_vector(truth, 'rotation_matrix', 9);
end


function payload = extract_actuator_payload(state)
payload = struct( ...
    'requested_rotor_thrusts', nan(1, 4), ...
    'applied_rotor_thrusts', nan(1, 4), ...
    'tracking_error', nan(1, 4) ...
);

if ~isfield(state, 'actuator')
    return;
end

actuator = state.actuator;
payload.requested_rotor_thrusts = get_struct_row_vector(actuator, 'requested_rotor_thrusts', 4);
payload.applied_rotor_thrusts = get_struct_row_vector(actuator, 'applied_rotor_thrusts', 4);
payload.tracking_error = get_struct_row_vector(actuator, 'tracking_error', 4);
end


function value = get_struct_row_vector(input_struct, field_name, expected_length)
value = nan(1, expected_length);
if ~isstruct(input_struct) || ~isfield(input_struct, field_name)
    return;
end

raw_value = reshape(double(input_struct.(field_name)), 1, []);
if numel(raw_value) ~= expected_length
    return;
end

value = raw_value;
end
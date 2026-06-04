classdef controller_shared
    methods (Static)
        function state = read_latest_state(controller_socket)
            if controller_socket.NumDatagramsAvailable == 0
                state = [];
                return;
            end

            data_packet = read(controller_socket, controller_socket.NumDatagramsAvailable, "string");
            latest_packet = data_packet(end).Data;
            state = jsondecode(latest_packet);
            state = controller_shared.attach_state_packet_metrics(state);
        end

        function rotor_thrusts = compute_hover_control(state, target_position, desired_heading, mass, gravity, position_gain, velocity_gain, attitude_gain, angular_velocity_gain, mixer, max_rotor_thrust)
            position = controller_shared.state_vector(state.position);
            velocity = controller_shared.state_vector(state.velocity);
            angular_velocity = controller_shared.state_vector(state.angular_velocity_body);
            rotation_matrix = reshape(double(state.rotation_matrix), [3, 3])';
            desired_heading = controller_shared.normalize_vector(desired_heading, [1.0; 0.0; 0.0]);

            position_error = target_position - position;
            velocity_error = -velocity;

            desired_force = position_gain .* position_error ...
                + velocity_gain .* velocity_error ...
                + [0.0; 0.0; mass * gravity];

            body_z_axis = rotation_matrix(:, 3);
            collective_thrust = max(0.0, dot(desired_force, body_z_axis));

            desired_body_z = controller_shared.normalize_vector(desired_force, [0.0; 0.0; 1.0]);
            desired_body_y = cross(desired_body_z, desired_heading);
            if norm(desired_body_y) < 1e-6
                desired_body_y = cross(desired_body_z, [0.0; 1.0; 0.0]);
            end
            desired_body_y = desired_body_y / norm(desired_body_y);
            desired_body_x = cross(desired_body_y, desired_body_z);
            desired_body_x = desired_body_x / norm(desired_body_x);
            desired_rotation = [desired_body_x, desired_body_y, desired_body_z];

            attitude_error_matrix = 0.5 * (desired_rotation' * rotation_matrix - rotation_matrix' * desired_rotation);
            attitude_error = [attitude_error_matrix(3, 2); attitude_error_matrix(1, 3); attitude_error_matrix(2, 1)];
            moment_command = -attitude_gain .* attitude_error - angular_velocity_gain .* angular_velocity;

            wrench = [collective_thrust; moment_command];
            rotor_thrusts = min(max_rotor_thrust, max(0.0, mixer * wrench));
        end

        function command_options = build_command_options(command_mode, thrust_coefficient, varargin)
            parser = inputParser;
            addParameter(parser, 'fidelity_mode', 'baseline', @(value) ischar(value) || (isstring(value) && isscalar(value)));
            parse(parser, varargin{:});

            controller_shared.validate_command_mode(command_mode);
            command_options = struct( ...
                'input_mode', command_mode, ...
                'thrust_coefficient', thrust_coefficient, ...
                'fidelity_mode', char(parser.Results.fidelity_mode) ...
            );
        end

        function control_command = build_control_command(rotor_thrusts, command_options, varargin)
            parser = inputParser;
            addParameter(parser, 'sequence', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
            addParameter(parser, 'source_state_sequence', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
            addParameter(parser, 'wall_time_send_ns', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
            addParameter(parser, 'controller_compute_ms', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
            addParameter(parser, 'state_age_ms', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
            addParameter(parser, 'state_sequence_gap', [], @(value) isempty(value) || (isnumeric(value) && isscalar(value)));
            addParameter(parser, 'fidelity_mode', command_options.fidelity_mode, @(value) ischar(value) || (isstring(value) && isscalar(value)));
            parse(parser, varargin{:});

            control_command = struct('rotor_thrusts', rotor_thrusts(:)');
            if strcmp(command_options.input_mode, 'omega')
                control_command.rotor_omega = controller_shared.thrust_to_rotor_omega(rotor_thrusts, command_options.thrust_coefficient)';
            end

            if ~isempty(parser.Results.sequence)
                control_command.packet_metadata = struct( ...
                    'protocol_version', 2, ...
                    'sequence', double(parser.Results.sequence), ...
                    'source_state_sequence', double(parser.Results.source_state_sequence), ...
                    'wall_time_send_ns', double(parser.Results.wall_time_send_ns), ...
                    'fidelity_mode', char(parser.Results.fidelity_mode) ...
                );
            end
            control_command.runtime_metrics = struct( ...
                'controller_compute_ms', controller_shared.default_numeric(parser.Results.controller_compute_ms), ...
                'state_age_ms', controller_shared.default_numeric(parser.Results.state_age_ms), ...
                'state_sequence_gap', controller_shared.default_numeric(parser.Results.state_sequence_gap) ...
            );
        end

        function send_control_command(controller_socket, control_command, target_ip, target_port)
            message = struct();
            if isfield(control_command, 'packet_metadata')
                metadata = control_command.packet_metadata;
                message.protocol_version = double(metadata.protocol_version);
                message.sequence = double(metadata.sequence);
                message.source_state_sequence = double(metadata.source_state_sequence);
                message.wall_time_send_ns = double(metadata.wall_time_send_ns);
                message.fidelity_mode = char(metadata.fidelity_mode);
            end
            if strcmp(controller_shared.control_command_mode(control_command), 'omega')
                message.rotor_omega = control_command.rotor_omega;
            else
                message.rotor_thrusts = control_command.rotor_thrusts;
            end
            write(controller_socket, jsonencode(message), "string", target_ip, target_port);
        end

        function send_multi_uav_control_command(controller_socket, control_commands, command_options, target_ip, target_port)
            num_uavs = numel(control_commands);
            command_matrix = zeros(num_uavs, 4);
            for uav_index = 1:num_uavs
                command_values = controller_shared.displayed_command_values(control_commands{uav_index}, command_options);
                command_matrix(uav_index, :) = reshape(double(command_values), 1, 4);
            end

            message = struct();
            if num_uavs >= 1 && isstruct(control_commands{1}) && isfield(control_commands{1}, 'packet_metadata')
                metadata = control_commands{1}.packet_metadata;
                message.protocol_version = double(metadata.protocol_version);
                message.sequence = double(metadata.sequence);
                message.source_state_sequence = double(metadata.source_state_sequence);
                message.wall_time_send_ns = double(metadata.wall_time_send_ns);
                message.fidelity_mode = char(metadata.fidelity_mode);
            end
            if strcmp(command_options.input_mode, 'omega')
                message.rotor_omegas = command_matrix;
            else
                message.rotor_thrusts = command_matrix;
            end
            write(controller_socket, jsonencode(message), "string", target_ip, target_port);
        end

        function values = displayed_command_values(control_command, command_options)
            if strcmp(command_options.input_mode, 'omega')
                values = reshape(double(control_command.rotor_omega), [], 1);
                return;
            end

            values = reshape(double(control_command.rotor_thrusts), [], 1);
        end

        function unit_label = command_unit_label(input_mode)
            if strcmp(input_mode, 'omega')
                unit_label = 'rad/s';
                return;
            end

            unit_label = 'N';
        end

        function vehicle_params = load_vehicle_params(project_directory, varargin)
            parser = inputParser;
            addParameter(parser, 'params_path', fullfile(project_directory, 'vehicle_params.json'), @(value) ischar(value) || (isstring(value) && isscalar(value)));
            parse(parser, varargin{:});

            params_path = char(parser.Results.params_path);
            raw_params = jsondecode(fileread(params_path));

            vehicle_params = struct( ...
                'mass', double(raw_params.drone.body_box.mass) + 2.0 * double(raw_params.drone.wheels.mass), ...
                'gravity', abs(double(raw_params.simulation.gravity(3))), ...
                'arm_x', abs(double(raw_params.drone.arm.x)), ...
                'arm_y', abs(double(raw_params.drone.arm.y)), ...
                'yaw_moment_ratio', abs(double(raw_params.actuation.yaw_moment_ratio)), ...
                'max_rotor_thrust', double(raw_params.actuation.max_rotor_thrust), ...
                'thrust_coefficient', double(raw_params.actuation.thrust_coefficient), ...
                'command_mode', char(raw_params.actuation.command_mode), ...
                'controller', controller_shared.parse_controller_params(raw_params), ...
                'rotors', controller_shared.parse_rotor_params(raw_params), ...
                'formation', controller_shared.parse_formation_params(raw_params), ...
                'fidelity', controller_shared.parse_fidelity_params(raw_params) ...
            );
        end

        function [allocation_matrix, mixer] = build_allocation_and_mixer(vehicle_params)
            allocation_matrix = [ ...
                1.0, 1.0, 1.0, 1.0; ...
                -vehicle_params.arm_y, vehicle_params.arm_y, -vehicle_params.arm_y, vehicle_params.arm_y; ...
                -vehicle_params.arm_x, -vehicle_params.arm_x, vehicle_params.arm_x, vehicle_params.arm_x; ...
                vehicle_params.yaw_moment_ratio, -vehicle_params.yaw_moment_ratio, -vehicle_params.yaw_moment_ratio, vehicle_params.yaw_moment_ratio ...
            ];
            mixer = pinv(allocation_matrix);
        end

        function controller_session = initialize_controller_session(project_directory, runtime_options, varargin)
            parser = inputParser;
            addParameter(parser, 'target_ip', '127.0.0.1', @(value) ischar(value) || (isstring(value) && isscalar(value)));
            addParameter(parser, 'simulator_root', project_directory, @(value) ischar(value) || (isstring(value) && isscalar(value)));
            addParameter(parser, 'params_path', fullfile(project_directory, 'vehicle_params.json'), @(value) ischar(value) || (isstring(value) && isscalar(value)));
            addParameter(parser, 'generated_xml_directory', fullfile(project_directory, 'build', 'generated_xml'), @(value) ischar(value) || (isstring(value) && isscalar(value)));
            parse(parser, varargin{:});

            vehicle_params = controller_shared.load_vehicle_params(project_directory, 'params_path', char(parser.Results.params_path));
            instance_options = controller_shared.build_instance_options(runtime_options.instance_id);
            controller_shared.release_stale_controller_socket(instance_options.controller_local_port);
            controller_shared.assert_udp_port_available(instance_options.controller_local_port, instance_options.instance_id);
            controller_socket = udpport("datagram", "IPv4", "LocalPort", instance_options.controller_local_port);

            simulator_options = controller_shared.build_simulator_options( ...
                project_directory, ...
                instance_options, ...
                'simulator_root', char(parser.Results.simulator_root), ...
                'params_path', char(parser.Results.params_path), ...
                'generated_xml_directory', char(parser.Results.generated_xml_directory) ...
            );
            simulator_options.auto_launch = controller_shared.get_struct_field(runtime_options, 'auto_launch', false);
            simulator_options.shutdown_on_exit = controller_shared.get_struct_field(runtime_options, 'shutdown_on_exit', false);
            simulator_options.num_uavs = controller_shared.get_struct_field(runtime_options, 'num_uavs', simulator_options.num_uavs);
            simulator_options.spawn_radius = controller_shared.get_struct_field(runtime_options, 'spawn_radius', simulator_options.spawn_radius);
            simulator_options.wait_for_startup_seconds = controller_shared.get_struct_field(runtime_options, 'wait_for_startup_seconds', simulator_options.wait_for_startup_seconds);
            simulator_options.headless = controller_shared.get_struct_field(runtime_options, 'headless', false);
            simulator_options.simulation_duration_seconds = controller_shared.get_struct_field(runtime_options, 'simulation_duration_seconds', simulator_options.simulation_duration_seconds);
            simulator_process_id = controller_shared.launch_simulator_if_requested(simulator_options);

            controller_session = struct( ...
                'vehicle_params', vehicle_params, ...
                'instance_options', instance_options, ...
                'controller_socket', controller_socket, ...
                'target_ip', char(parser.Results.target_ip), ...
                'target_port', instance_options.simulator_receive_port, ...
                'simulator_options', simulator_options, ...
                'simulator_process_id', simulator_process_id ...
            );
        end

        function controller_config = build_controller_config(vehicle_params, varargin)
            parser = inputParser;
            defaults = vehicle_params.controller;
            addParameter(parser, 'target_position', [0.0; 0.0; 1.5], @(value) isnumeric(value) && numel(value) == 3);
            addParameter(parser, 'desired_heading', defaults.desired_heading, @(value) isnumeric(value) && numel(value) == 3);
            addParameter(parser, 'position_gain', defaults.position_gain, @(value) isnumeric(value) && numel(value) == 3);
            addParameter(parser, 'velocity_gain', defaults.velocity_gain, @(value) isnumeric(value) && numel(value) == 3);
            addParameter(parser, 'attitude_gain', defaults.attitude_gain, @(value) isnumeric(value) && numel(value) == 3);
            addParameter(parser, 'angular_velocity_gain', defaults.angular_velocity_gain, @(value) isnumeric(value) && numel(value) == 3);
            parse(parser, varargin{:});

            controller_config = struct( ...
                'target_position', reshape(double(parser.Results.target_position), [], 1), ...
                'desired_heading', reshape(double(parser.Results.desired_heading), [], 1), ...
                'position_gain', reshape(double(parser.Results.position_gain), [], 1), ...
                'velocity_gain', reshape(double(parser.Results.velocity_gain), [], 1), ...
                'attitude_gain', reshape(double(parser.Results.attitude_gain), [], 1), ...
                'angular_velocity_gain', reshape(double(parser.Results.angular_velocity_gain), [], 1) ...
            );
        end

        function config = build_base_logger_config(vehicle_params, controller_config, allocation_matrix, mixer, command_options, instance_options)
            config = struct( ...
                'mass', vehicle_params.mass, ...
                'gravity', vehicle_params.gravity, ...
                'arm_x', vehicle_params.arm_x, ...
                'arm_y', vehicle_params.arm_y, ...
                'yaw_moment_ratio', vehicle_params.yaw_moment_ratio, ...
                'max_rotor_thrust', vehicle_params.max_rotor_thrust, ...
                'thrust_coefficient', vehicle_params.thrust_coefficient, ...
                'rotor_geometry', vehicle_params.rotors, ...
                'position_gain', controller_config.position_gain, ...
                'velocity_gain', controller_config.velocity_gain, ...
                'attitude_gain', controller_config.attitude_gain, ...
                'angular_velocity_gain', controller_config.angular_velocity_gain, ...
                'desired_heading', controller_config.desired_heading, ...
                'allocation_matrix', allocation_matrix, ...
                'mixer', mixer, ...
                'command_mode', command_options.input_mode, ...
                'fidelity_mode', vehicle_params.fidelity.mode, ...
                'network_fidelity', vehicle_params.fidelity.network, ...
                'instance_id', instance_options.instance_id, ...
                'instance_label', instance_options.label ...
            );
        end

        function runtime_metrics = initialize_runtime_metrics()
            runtime_metrics = struct( ...
                'last_state_sequence', NaN, ...
                'last_state_age_ms', NaN, ...
                'last_state_sequence_gap', 0.0, ...
                'state_sequence_gap_count', 0.0, ...
                'timeout_count', 0.0, ...
                'last_controller_compute_ms', NaN, ...
                'command_sequence', 0.0, ...
                'last_source_state_sequence', NaN ...
            );
        end

        function runtime_metrics = update_runtime_metrics(runtime_metrics, state, controller_compute_ms)
            state_metrics = controller_shared.get_state_packet_metrics(state);
            previous_sequence = runtime_metrics.last_state_sequence;
            current_sequence = state_metrics.sequence;

            if ~isnan(previous_sequence) && ~isnan(current_sequence)
                sequence_gap = max(0.0, current_sequence - previous_sequence - 1.0);
            else
                sequence_gap = 0.0;
            end

            runtime_metrics.last_state_sequence = current_sequence;
            runtime_metrics.last_state_age_ms = state_metrics.age_ms;
            runtime_metrics.last_state_sequence_gap = sequence_gap;
            runtime_metrics.state_sequence_gap_count = runtime_metrics.state_sequence_gap_count + sequence_gap;
            runtime_metrics.last_controller_compute_ms = double(controller_compute_ms);
            runtime_metrics.command_sequence = runtime_metrics.command_sequence + 1.0;
            runtime_metrics.last_source_state_sequence = current_sequence;
        end

        function runtime_metrics = note_timeout(runtime_metrics)
            runtime_metrics.timeout_count = runtime_metrics.timeout_count + 1.0;
        end

        function time_ns = wall_time_now_ns()
            time_ns = floor(posixtime(datetime('now', 'TimeZone', 'UTC')) * 1.0e9);
        end

        function state_metrics = get_state_packet_metrics(state)
            default_metrics = struct( ...
                'protocol_version', 1.0, ...
                'sequence', NaN, ...
                'wall_time_send_ns', NaN, ...
                'fidelity_mode', '', ...
                'age_ms', NaN ...
            );
            if ~isstruct(state) || ~isfield(state, 'packet_metrics')
                state_metrics = default_metrics;
                return;
            end

            metrics = state.packet_metrics;
            state_metrics = struct( ...
                'protocol_version', controller_shared.get_struct_field(metrics, 'protocol_version', 1.0), ...
                'sequence', controller_shared.default_numeric(controller_shared.get_struct_field(metrics, 'sequence', NaN)), ...
                'wall_time_send_ns', controller_shared.default_numeric(controller_shared.get_struct_field(metrics, 'wall_time_send_ns', NaN)), ...
                'fidelity_mode', char(controller_shared.get_struct_field(metrics, 'fidelity_mode', '')), ...
                'age_ms', controller_shared.default_numeric(controller_shared.get_struct_field(metrics, 'age_ms', NaN)) ...
            );
        end

        function instance_options = build_instance_options(instance_id)
            arguments
                instance_id (1, 1) double {mustBeInteger, mustBeNonnegative} = 0
            end

            simulator_receive_port = 5000 + 2 * instance_id;
            controller_local_port = simulator_receive_port + 1;
            file_suffix = '';
            if instance_id ~= 0
                file_suffix = sprintf('_i%d', instance_id);
            end

            instance_options = struct( ...
                'instance_id', double(instance_id), ...
                'label', sprintf('instance=%d', instance_id), ...
                'simulator_receive_port', simulator_receive_port, ...
                'controller_local_port', controller_local_port, ...
                'file_suffix', file_suffix ...
            );
        end

        function simulator_options = build_simulator_options(project_directory, instance_options, varargin)
            parser = inputParser;
            addParameter(parser, 'simulator_root', project_directory, @(value) ischar(value) || (isstring(value) && isscalar(value)));
            addParameter(parser, 'params_path', fullfile(project_directory, 'vehicle_params.json'), @(value) ischar(value) || (isstring(value) && isscalar(value)));
            addParameter(parser, 'generated_xml_directory', fullfile(project_directory, 'build', 'generated_xml'), @(value) ischar(value) || (isstring(value) && isscalar(value)));
            parse(parser, varargin{:});

            simulator_root = char(parser.Results.simulator_root);
            simulator_options = struct( ...
                'auto_launch', false, ...
                'wait_for_startup_seconds', 3.0, ...
                'simulator_receive_port', instance_options.simulator_receive_port, ...
                'shutdown_on_exit', false, ...
                'headless', false, ...
                'simulation_duration_seconds', NaN, ...
                'instance_id', instance_options.instance_id, ...
                'num_uavs', 1, ...
                'spawn_radius', 1.5, ...
                'working_directory', project_directory, ...
                'simulator_root', simulator_root, ...
                'python_executable', controller_shared.get_default_python_executable(simulator_root), ...
                'params_path', char(parser.Results.params_path), ...
                'generated_xml_directory', char(parser.Results.generated_xml_directory) ...
            );
        end

        function display_logging_behavior(logger)
            logging_options = logger.get_options();
            fprintf('Logging policy: mode=%s, interval=%.2f s, path=%s\n', ...
                logging_options.save_mode, ...
                logging_options.periodic_interval_seconds, ...
                logger.get_file_path() ...
            );
        end

        function finalize_controller_run(logger)
            logging_options = logger.get_options();
            supports_finalize = strcmp(logging_options.save_mode, 'finalize') || strcmp(logging_options.save_mode, 'periodic_and_finalize');
            if ~supports_finalize
                return;
            end

            logger.finalize();
            if logging_options.print_save_events
                fprintf('Simulation log saved at shutdown -> %s\n', logger.get_file_path());
            end
        end

        function simulator_process_id = launch_simulator_if_requested(simulator_options)
            simulator_process_id = [];
            if ~simulator_options.auto_launch
                return;
            end

            if controller_shared.does_udp_port_exist(simulator_options.simulator_receive_port)
                fprintf('MuJoCo simulator appears to be already running on UDP port %d.\n', simulator_options.simulator_receive_port);
                return;
            end

            [command_text, launch_mode] = controller_shared.build_simulator_launch_command(simulator_options);
            fprintf('Launching MuJoCo simulator from MATLAB using %s.\n', launch_mode);

            [status, command_output] = system(command_text);
            if status ~= 0
                error('Failed to launch MuJoCo simulator: %s', strtrim(command_output));
            end

            simulator_process_id = str2double(strtrim(command_output));
            if isnan(simulator_process_id)
                simulator_process_id = [];
            end

            controller_shared.wait_for_simulator_startup(simulator_options);
        end

        function cleanup_simulator_process(simulator_process_id, simulator_options)
            if isempty(simulator_process_id) || ~simulator_options.shutdown_on_exit
                return;
            end

            if ispc
                kill_command = sprintf( ...
                    'powershell -NoProfile -Command "Stop-Process -Id %d -Force -ErrorAction SilentlyContinue"', ...
                    simulator_process_id ...
                );
            elseif isunix
                kill_command = sprintf( ...
                    'bash -lc "kill %d >/dev/null 2>&1 || true"', ...
                    simulator_process_id ...
                );
            else
                warning('Simulator auto-shutdown is not implemented for this operating system.');
                return;
            end
            system(kill_command);
        end

        function release_stale_controller_socket(local_port)
            try
                evalin('base', 'clear controller_socket');
            catch
            end

            if exist('udpportfind', 'file') ~= 2
                return;
            end

            try
                stale_sockets = udpportfind("LocalPort", local_port);
            catch
                stale_sockets = [];
            end

            if isempty(stale_sockets)
                return;
            end

            fprintf('Releasing stale UDP socket on port %d before controller startup.\n', local_port);
            for socket_index = 1:numel(stale_sockets)
                try
                    clear stale_sockets(socket_index);
                catch
                end
            end

            try
                delete(stale_sockets);
            catch
            end
        end

        function cleanup_controller_socket(controller_socket)
            try
                clear controller_socket;
            catch
            end

            try
                delete(controller_socket);
            catch
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

        function state = attach_state_packet_metrics(state)
            if ~isstruct(state)
                return;
            end

            packet_metrics = controller_shared.extract_packet_metrics(state);
            if isfield(state, 'uavs')
                for uav_index = 1:numel(state.uavs)
                    state.uavs(uav_index).packet_metrics = packet_metrics;
                end
                return;
            end
            state.packet_metrics = packet_metrics;
        end
    end

    methods (Static, Access = private)
        function fidelity_params = parse_fidelity_params(raw_params)
            fidelity_mode = 'baseline';
            if isfield(raw_params, 'fidelity_mode')
                if ischar(raw_params.fidelity_mode) || (isstring(raw_params.fidelity_mode) && isscalar(raw_params.fidelity_mode))
                    fidelity_mode = char(raw_params.fidelity_mode);
                elseif isstruct(raw_params.fidelity_mode) && isfield(raw_params.fidelity_mode, 'mode')
                    fidelity_mode = char(raw_params.fidelity_mode.mode);
                end
            end

            network_section = struct();
            if isfield(raw_params, 'network_fidelity') && isstruct(raw_params.network_fidelity)
                network_section = raw_params.network_fidelity;
            end

            fidelity_params = struct( ...
                'mode', fidelity_mode, ...
                'network', struct( ...
                    'enabled', logical(controller_shared.get_struct_field(network_section, 'enabled', false)), ...
                    'state_tx_latency_ms', controller_shared.get_optional_scalar(network_section, 'state_tx_latency_ms', 0.0), ...
                    'command_rx_latency_ms', controller_shared.get_optional_scalar(network_section, 'command_rx_latency_ms', 0.0), ...
                    'packet_loss_percent', controller_shared.get_optional_scalar(network_section, 'packet_loss_percent', 0.0), ...
                    'jitter_std_dev_ms', controller_shared.get_optional_scalar(network_section, 'jitter_std_dev_ms', 0.0), ...
                    'stale_command_threshold_ms', controller_shared.default_numeric(controller_shared.get_struct_field(network_section, 'stale_command_threshold_ms', NaN)), ...
                    'stale_command_policy', char(controller_shared.get_struct_field(network_section, 'stale_command_policy', 'hold-last-command')) ...
                ) ...
            );
        end

        function packet_metrics = extract_packet_metrics(packet_struct)
            wall_time_send_ns = controller_shared.get_struct_field(packet_struct, 'wall_time_send_ns', NaN);
            receive_time_ns = controller_shared.wall_time_now_ns();
            age_ms = NaN;
            if ~isempty(wall_time_send_ns) && ~isnan(double(wall_time_send_ns))
                age_ms = max(0.0, (double(receive_time_ns) - double(wall_time_send_ns)) / 1.0e6);
            end

            packet_metrics = struct( ...
                'protocol_version', double(controller_shared.get_struct_field(packet_struct, 'protocol_version', 1.0)), ...
                'sequence', controller_shared.default_numeric(controller_shared.get_struct_field(packet_struct, 'sequence', NaN)), ...
                'wall_time_send_ns', controller_shared.default_numeric(wall_time_send_ns), ...
                'fidelity_mode', char(controller_shared.get_struct_field(packet_struct, 'fidelity_mode', '')), ...
                'age_ms', controller_shared.default_numeric(age_ms) ...
            );
        end

        function rotor_params = parse_rotor_params(raw_params)
            default_yaw_moment_ratio = abs(double(raw_params.actuation.yaw_moment_ratio));
            if isfield(raw_params.actuation, 'rotors') && ~isempty(raw_params.actuation.rotors)
                raw_rotors = raw_params.actuation.rotors;
                if ~isstruct(raw_rotors)
                    error('actuation.rotors must be an array of structs.');
                end

                rotor_params = repmat(struct( ...
                    'name', '', ...
                    'position_body', zeros(3, 1), ...
                    'thrust_axis_body', zeros(3, 1), ...
                    'yaw_moment_ratio', 0.0, ...
                    'spin_sign', 1.0 ...
                ), numel(raw_rotors), 1);

                for rotor_index = 1:numel(raw_rotors)
                    raw_rotor = raw_rotors(rotor_index);
                    position_body = controller_shared.get_required_vector(raw_rotor, 'position_body', 3, sprintf('actuation.rotors(%d)', rotor_index));
                    thrust_axis_body = controller_shared.normalize_vector( ...
                        controller_shared.get_required_vector(raw_rotor, 'thrust_axis_body', 3, sprintf('actuation.rotors(%d)', rotor_index)), ...
                        [0.0; 0.0; 1.0] ...
                    );
                    spin_sign = controller_shared.get_optional_scalar(raw_rotor, 'spin_sign', 1.0);
                    if abs(spin_sign) < 1.0e-9
                        error('actuation.rotors(%d).spin_sign must be non-zero.', rotor_index);
                    end

                    rotor_params(rotor_index) = struct( ...
                        'name', char(raw_rotor.name), ...
                        'position_body', position_body, ...
                        'thrust_axis_body', thrust_axis_body, ...
                        'yaw_moment_ratio', controller_shared.get_optional_scalar(raw_rotor, 'yaw_moment_ratio', default_yaw_moment_ratio), ...
                        'spin_sign', sign(spin_sign) ...
                    );
                end
                return;
            end

            arm_x = abs(double(raw_params.drone.arm.x));
            arm_y = abs(double(raw_params.drone.arm.y));
            propeller_z = double(raw_params.drone.propeller.z);
            rotor_params = [ ...
                struct('name', 'fr', 'position_body', [arm_x; -arm_y; propeller_z], 'thrust_axis_body', [0.0; 0.0; 1.0], 'yaw_moment_ratio', default_yaw_moment_ratio, 'spin_sign', 1.0); ...
                struct('name', 'fl', 'position_body', [arm_x; arm_y; propeller_z], 'thrust_axis_body', [0.0; 0.0; 1.0], 'yaw_moment_ratio', default_yaw_moment_ratio, 'spin_sign', -1.0); ...
                struct('name', 'br', 'position_body', [-arm_x; -arm_y; propeller_z], 'thrust_axis_body', [0.0; 0.0; 1.0], 'yaw_moment_ratio', default_yaw_moment_ratio, 'spin_sign', -1.0); ...
                struct('name', 'bl', 'position_body', [-arm_x; arm_y; propeller_z], 'thrust_axis_body', [0.0; 0.0; 1.0], 'yaw_moment_ratio', default_yaw_moment_ratio, 'spin_sign', 1.0) ...
            ];
        end

        function formation_params = parse_formation_params(raw_params)
            controller_defaults = controller_shared.parse_controller_params(raw_params);
            formation_params = struct( ...
                'num_uavs', 3, ...
                'spawn_radius', 1.5, ...
                'base_height', 1.5, ...
                'centroid_target_xy', [0.0; 0.0], ...
                'formation_radius', 1.5, ...
                'centroid_gain', 0.8, ...
                'formation_gain', 1.2, ...
                'duration_seconds', 20.0, ...
                'idle_sleep_seconds', 0.001, ...
                'status_display_interval', 2.0, ...
                'desired_heading', controller_defaults.desired_heading, ...
                'position_gain', controller_defaults.position_gain, ...
                'velocity_gain', controller_defaults.velocity_gain, ...
                'attitude_gain', controller_defaults.attitude_gain, ...
                'angular_velocity_gain', controller_defaults.angular_velocity_gain ...
            );

            if ~isfield(raw_params, 'formation')
                return;
            end

            raw_formation = raw_params.formation;
            formation_params.num_uavs = controller_shared.get_optional_scalar(raw_formation, 'num_uavs', formation_params.num_uavs);
            formation_params.spawn_radius = controller_shared.get_optional_scalar(raw_formation, 'spawn_radius', formation_params.spawn_radius);
            formation_params.base_height = controller_shared.get_optional_scalar(raw_formation, 'base_height', formation_params.base_height);
            formation_params.formation_radius = controller_shared.get_optional_scalar(raw_formation, 'formation_radius', formation_params.formation_radius);
            formation_params.centroid_gain = controller_shared.get_optional_scalar(raw_formation, 'centroid_gain', formation_params.centroid_gain);
            formation_params.formation_gain = controller_shared.get_optional_scalar(raw_formation, 'formation_gain', formation_params.formation_gain);
            formation_params.duration_seconds = controller_shared.get_optional_scalar(raw_formation, 'duration_seconds', formation_params.duration_seconds);
            formation_params.idle_sleep_seconds = controller_shared.get_optional_scalar(raw_formation, 'idle_sleep_seconds', formation_params.idle_sleep_seconds);
            formation_params.status_display_interval = controller_shared.get_optional_scalar(raw_formation, 'status_display_interval', formation_params.status_display_interval);
            formation_params.centroid_target_xy = controller_shared.get_optional_vector(raw_formation, 'centroid_target_xy', formation_params.centroid_target_xy, 2);
            formation_params.desired_heading = controller_shared.get_optional_vector(raw_formation, 'desired_heading', formation_params.desired_heading, 3);
            formation_params.position_gain = controller_shared.get_optional_vector(raw_formation, 'position_gain', formation_params.position_gain, 3);
            formation_params.velocity_gain = controller_shared.get_optional_vector(raw_formation, 'velocity_gain', formation_params.velocity_gain, 3);
            formation_params.attitude_gain = controller_shared.get_optional_vector(raw_formation, 'attitude_gain', formation_params.attitude_gain, 3);
            formation_params.angular_velocity_gain = controller_shared.get_optional_vector(raw_formation, 'angular_velocity_gain', formation_params.angular_velocity_gain, 3);
        end

        function controller_params = parse_controller_params(raw_params)
            controller_params = struct( ...
                'desired_heading', [1.0; 0.0; 0.0], ...
                'position_gain', [3.0; 3.0; 6.0], ...
                'velocity_gain', [2.2; 2.2; 4.0], ...
                'attitude_gain', [0.8; 0.8; 0.25], ...
                'angular_velocity_gain', [0.12; 0.12; 0.08] ...
            );

            if ~isfield(raw_params, 'controller')
                return;
            end

            raw_controller = raw_params.controller;
            controller_params.desired_heading = controller_shared.get_optional_vector(raw_controller, 'desired_heading', controller_params.desired_heading, 3);
            controller_params.position_gain = controller_shared.get_optional_vector(raw_controller, 'position_gain', controller_params.position_gain, 3);
            controller_params.velocity_gain = controller_shared.get_optional_vector(raw_controller, 'velocity_gain', controller_params.velocity_gain, 3);
            controller_params.attitude_gain = controller_shared.get_optional_vector(raw_controller, 'attitude_gain', controller_params.attitude_gain, 3);
            controller_params.angular_velocity_gain = controller_shared.get_optional_vector(raw_controller, 'angular_velocity_gain', controller_params.angular_velocity_gain, 3);
        end

        function value = get_optional_scalar(input_struct, field_name, default_value)
            value = default_value;
            if ~isfield(input_struct, field_name)
                return;
            end
            value = double(input_struct.(field_name));
        end

        function value = get_struct_field(input_struct, field_name, default_value)
            value = default_value;
            if ~isstruct(input_struct) || ~isfield(input_struct, field_name)
                return;
            end
            value = input_struct.(field_name);
        end

        function value = default_numeric(value)
            if isempty(value)
                value = NaN;
                return;
            end
            value = double(value);
        end

        function value = get_optional_vector(input_struct, field_name, default_value, expected_length)
            value = default_value;
            if ~isfield(input_struct, field_name)
                return;
            end
            candidate = reshape(double(input_struct.(field_name)), [], 1);
            if numel(candidate) ~= expected_length
                error('formation.%s must have %d elements.', field_name, expected_length);
            end
            value = candidate;
        end

        function value = get_required_vector(input_struct, field_name, expected_length, context_label)
            if ~isfield(input_struct, field_name)
                error('%s.%s must be present.', context_label, field_name);
            end

            value = reshape(double(input_struct.(field_name)), [], 1);
            if numel(value) ~= expected_length
                error('%s.%s must have %d elements.', context_label, field_name, expected_length);
            end
        end

        function column_vector = state_vector(value)
            column_vector = reshape(double(value), [], 1);
        end

        function normalized = normalize_vector(vector, fallback)
            vector_norm = norm(vector);
            if vector_norm < 1e-6
                normalized = fallback;
                return;
            end

            normalized = vector / vector_norm;
        end

        function rotor_omega = thrust_to_rotor_omega(rotor_thrusts, thrust_coefficient)
            if thrust_coefficient <= 0.0
                error('thrust_coefficient must be positive when input_mode is omega.');
            end

            rotor_omega = sqrt(max(0.0, rotor_thrusts) ./ thrust_coefficient);
        end

        function mode_name = control_command_mode(control_command)
            if isfield(control_command, 'rotor_omega')
                mode_name = 'omega';
                return;
            end

            mode_name = 'thrust';
        end

        function validate_command_mode(command_mode)
            if strcmp(command_mode, 'thrust') || strcmp(command_mode, 'omega')
                return;
            end

            error('Unsupported command_mode: %s', command_mode);
        end

        function [command_text, launch_mode] = build_simulator_launch_command(simulator_options)
            instance_id = round(simulator_options.instance_id);
            num_uavs = round(simulator_options.num_uavs);
            spawn_radius = simulator_options.spawn_radius;

            if ispc
                [command_text, launch_mode] = controller_shared.build_windows_simulator_launch_command(simulator_options, instance_id, num_uavs, spawn_radius);
                return;
            end

            if isunix
                [command_text, launch_mode] = controller_shared.build_unix_simulator_launch_command(simulator_options, instance_id, num_uavs, spawn_radius);
                return;
            end

            error('Simulator auto-launch is not implemented for this operating system.');
        end

        function wait_for_simulator_startup(simulator_options)
            deadline = tic;
            while toc(deadline) < simulator_options.wait_for_startup_seconds
                if controller_shared.does_udp_port_exist(simulator_options.simulator_receive_port)
                    fprintf('MuJoCo simulator is ready on UDP port %d.\n', simulator_options.simulator_receive_port);
                    return;
                end
                pause(0.1);
            end

            fprintf('MuJoCo simulator launch was requested, but UDP port %d did not open within %.1f s.\n', ...
                simulator_options.simulator_receive_port, ...
                simulator_options.wait_for_startup_seconds ...
            );
        end

        function exists_flag = does_udp_port_exist(local_port)
            if ispc
                command_text = sprintf([ ...
                    'powershell -NoProfile -Command "if (Get-NetUDPEndpoint -LocalPort %d -ErrorAction SilentlyContinue) { Write-Output 1 } else { Write-Output 0 }"' ...
                ], local_port);
            elseif isunix
                command_text = sprintf([ ...
                    'bash -lc "if command -v ss >/dev/null 2>&1; then if ss -lun | awk ''{print \$5}'' | grep -Eq '':%d$''; then echo 1; else echo 0; fi; elif command -v netstat >/dev/null 2>&1; then if netstat -lun 2>/dev/null | awk ''{print \$4}'' | grep -Eq '':%d$''; then echo 1; else echo 0; fi; else echo 0; fi"' ...
                ], local_port, local_port);
            else
                exists_flag = false;
                return;
            end
            [status, command_output] = system(command_text);
            exists_flag = status == 0 && strcmp(strtrim(command_output), '1');
        end

        function assert_udp_port_available(local_port, instance_id)
            if ~controller_shared.does_udp_port_exist(local_port)
                return;
            end

            owner_summary = controller_shared.describe_udp_port_owner(local_port);
            error([ ...
                'UDP port %d is already in use before the controller socket could bind. ', ...
                'This usually means another MATLAB controller process is still running or a manual session owns the port. ', ...
                'Use a different instance_id, stop the other controller, or fully close the MATLAB session that owns the socket. ', ...
                'instance_id=%d expects controller_local_port=%d. %s' ...
            ], local_port, instance_id, local_port, owner_summary);
        end

        function owner_summary = describe_udp_port_owner(local_port)
            owner_summary = 'Owner process could not be resolved.';
            if ispc
                command_text = sprintf([ ...
                    'powershell -NoProfile -Command "$endpoint = Get-NetUDPEndpoint -LocalPort %d -ErrorAction SilentlyContinue | Select-Object -First 1; ' ...
                    'if (-not $endpoint) { return }; ' ...
                    '$process = Get-Process -Id $endpoint.OwningProcess -ErrorAction SilentlyContinue; ' ...
                    'if ($process) { Write-Output (''Owner process: '' + $process.ProcessName + '' (PID '' + $process.Id + '')'') } else { Write-Output (''Owner PID: '' + $endpoint.OwningProcess) }"' ...
                ], local_port);
            elseif isunix
                command_text = sprintf([ ...
                    'bash -lc "if command -v lsof >/dev/null 2>&1; then lsof -nP -iUDP:%d 2>/dev/null | tail -n +2 | head -n 1 | awk ''{print \"Owner process: \" $1 \" (PID \" $2 \")\"}''; fi"' ...
                ], local_port);
            else
                return;
            end

            [status, command_output] = system(command_text);
            if status ~= 0
                return;
            end

            command_output = strtrim(command_output);
            if ~isempty(command_output)
                owner_summary = command_output;
            end
        end

        function python_executable = get_default_python_executable(simulator_root)
            if ispc
                python_executable = fullfile(simulator_root, '.venv', 'Scripts', 'python.exe');
                return;
            end

            python_executable = fullfile(simulator_root, '.venv', 'bin', 'python');
        end

        function [command_text, launch_mode] = build_windows_simulator_launch_command(simulator_options, instance_id, num_uavs, spawn_radius)
            working_directory = controller_shared.escape_powershell_string(simulator_options.working_directory);
            simulator_root = controller_shared.escape_powershell_string(simulator_options.simulator_root);
            params_path = controller_shared.escape_powershell_string(simulator_options.params_path);
            generated_xml_directory = controller_shared.escape_powershell_string(simulator_options.generated_xml_directory);
            headless_runner_path = controller_shared.escape_powershell_string(fullfile(simulator_options.simulator_root, 'tools', 'headless_simulator_runner.py'));

            if isfile(simulator_options.python_executable)
                python_executable = controller_shared.escape_powershell_string(simulator_options.python_executable);
                if controller_shared.get_struct_field(simulator_options, 'headless', false) && isfinite(controller_shared.get_struct_field(simulator_options, 'simulation_duration_seconds', NaN))
                    duration_seconds = double(controller_shared.get_struct_field(simulator_options, 'simulation_duration_seconds', NaN));
                    command_text = sprintf([ ...
                        'powershell -NoProfile -Command "$processArgs = @(''%s'',''--instance-id'',''%d'',''--num-uavs'',''%d'',''--spawn-radius'',''%.9g'',''--params-file'',''%s'',''--generated-xml-dir'',''%s'',''--duration-seconds'',''%.9g''); $process = Start-Process -FilePath ''%s'' -ArgumentList $processArgs -WorkingDirectory ''%s'' -PassThru; Write-Output $process.Id"' ...
                    ], headless_runner_path, instance_id, num_uavs, spawn_radius, params_path, generated_xml_directory, duration_seconds, python_executable, simulator_root);
                    launch_mode = '.venv python headless_simulator_runner.py';
                    return;
                end
                extra_arguments = controller_shared.build_simulator_cli_arguments(simulator_options);
                command_text = sprintf([ ...
                    'powershell -NoProfile -Command "$processArgs = @(''-m'',''qav_wheel.cli'',''simulate'',''--instance-id'',''%d'',''--num-uavs'',''%d'',''--spawn-radius'',''%.9g'',''--params-file'',''%s'',''--generated-xml-dir'',''%s'') + %s; $process = Start-Process -FilePath ''%s'' -ArgumentList $processArgs -WorkingDirectory ''%s'' -PassThru; Write-Output $process.Id"' ...
                ], instance_id, num_uavs, spawn_radius, params_path, generated_xml_directory, extra_arguments, python_executable, simulator_root);
                launch_mode = '.venv python -m qav_wheel.cli';
                return;
            end

            extra_arguments = controller_shared.build_simulator_cli_arguments(simulator_options);
            command_text = sprintf([ ...
                'powershell -NoProfile -Command "$processArgs = @(''run'',''--project'',''%s'',''mujoco-wheeled-uav-simulator'',''simulate'',''--instance-id'',''%d'',''--num-uavs'',''%d'',''--spawn-radius'',''%.9g'',''--params-file'',''%s'',''--generated-xml-dir'',''%s'') + %s; $process = Start-Process -FilePath ''uv'' -ArgumentList $processArgs -WorkingDirectory ''%s'' -PassThru; Write-Output $process.Id"' ...
            ], simulator_root, instance_id, num_uavs, spawn_radius, params_path, generated_xml_directory, extra_arguments, working_directory);
            launch_mode = 'uv run --project';
        end

        function [command_text, launch_mode] = build_unix_simulator_launch_command(simulator_options, instance_id, num_uavs, spawn_radius)
            working_directory = controller_shared.escape_bash_double_quoted_string(simulator_options.working_directory);
            simulator_root = controller_shared.escape_bash_double_quoted_string(simulator_options.simulator_root);
            params_path = controller_shared.escape_bash_double_quoted_string(simulator_options.params_path);
            generated_xml_directory = controller_shared.escape_bash_double_quoted_string(simulator_options.generated_xml_directory);
            extra_arguments = controller_shared.build_unix_simulator_cli_arguments(simulator_options);

            if isfile(simulator_options.python_executable)
                python_executable = controller_shared.escape_bash_double_quoted_string(simulator_options.python_executable);
                command_text = sprintf([ ...
                    'bash -lc "cd \"%s\"; nohup \"%s\" -m qav_wheel.cli simulate --instance-id %d --num-uavs %d --spawn-radius %.9g --params-file \"%s\" --generated-xml-dir \"%s\"%s >/dev/null 2>&1 & echo $!"' ...
                ], simulator_root, python_executable, instance_id, num_uavs, spawn_radius, params_path, generated_xml_directory, extra_arguments);
                launch_mode = '.venv python -m qav_wheel.cli';
                return;
            end

            command_text = sprintf([ ...
                'bash -lc "cd \"%s\"; nohup uv run --project \"%s\" mujoco-wheeled-uav-simulator simulate --instance-id %d --num-uavs %d --spawn-radius %.9g --params-file \"%s\" --generated-xml-dir \"%s\"%s >/dev/null 2>&1 & echo $!"' ...
            ], working_directory, simulator_root, instance_id, num_uavs, spawn_radius, params_path, generated_xml_directory, extra_arguments);
            launch_mode = 'uv run --project';
        end

        function cli_arguments = build_simulator_cli_arguments(simulator_options)
            cli_arguments = '{}';
            argument_list = {};
            if controller_shared.get_struct_field(simulator_options, 'headless', false)
                argument_list{end + 1} = '''--headless'''; %#ok<AGROW>
            end
            duration_seconds = controller_shared.get_struct_field(simulator_options, 'simulation_duration_seconds', NaN);
            if isfinite(duration_seconds)
                argument_list{end + 1} = '''--duration-seconds'''; %#ok<AGROW>
                argument_list{end + 1} = sprintf('''%.9g''', double(duration_seconds)); %#ok<AGROW>
            end
            if ~isempty(argument_list)
                cli_arguments = ['@(' strjoin(argument_list, ',') ')'];
            end
        end

        function cli_arguments = build_unix_simulator_cli_arguments(simulator_options)
            cli_arguments = '';
            if controller_shared.get_struct_field(simulator_options, 'headless', false)
                cli_arguments = [cli_arguments ' --headless']; %#ok<AGROW>
            end
            duration_seconds = controller_shared.get_struct_field(simulator_options, 'simulation_duration_seconds', NaN);
            if isfinite(duration_seconds)
                cli_arguments = sprintf('%s --duration-seconds %.9g', cli_arguments, double(duration_seconds));
            end
        end

        function escaped_text = escape_powershell_string(text)
            escaped_text = strrep(text, '''', '''''');
        end

        function escaped_text = escape_bash_double_quoted_string(text)
            escaped_text = char(text);
            escaped_text = strrep(escaped_text, '\', '\\');
            escaped_text = strrep(escaped_text, '"', '\"');
            escaped_text = strrep(escaped_text, '$', '\$');
            escaped_text = strrep(escaped_text, '`', '\`');
        end
    end
end
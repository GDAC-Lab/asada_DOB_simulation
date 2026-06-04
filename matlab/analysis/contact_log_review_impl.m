function contact_log_review_impl(mode, primary_log_path, secondary_log_path)
% Review saved contact logs for the main validation scenarios.
%
% Usage examples:
%   contact_log_review
%   contact_log_review('noncontact')
%   contact_log_review('landing', 'logs/hover_20260410_120000.mat')
%   contact_log_review('wall', 'logs/hover_20260410_121000.mat')
%   contact_log_review('forces', 'logs/hover_20260410_121000.mat')
%   contact_log_review('instantaneous', 'logs/hover_20260410_121000.mat')
%   contact_log_review('impact_compare', 'logs/soft.mat', 'logs/hard.mat')

arguments
    mode (1, :) char = 'overview'
    primary_log_path (1, :) char = ''
    secondary_log_path (1, :) char = ''
end

project_directory = fileparts(fileparts(fileparts(mfilename('fullpath'))));

if strcmp(mode, 'impact_compare')
    primary_log = load_log(resolve_log_path(project_directory, primary_log_path));
    secondary_log = load_log(resolve_log_path(project_directory, secondary_log_path));
    compare_impact_logs(primary_log, secondary_log);
    return;
end

log_data = load_log(resolve_log_path(project_directory, primary_log_path));
execute_review_mode(mode, log_data);
end


function execute_review_mode(mode, log_data)
mode_actions = build_review_mode_actions(log_data);
if ~isfield(mode_actions, mode)
    error('Unsupported mode: %s', mode);
end

mode_actions.(mode)();
end


function mode_actions = build_review_mode_actions(log_data)
mode_actions = struct();
mode_actions.overview = @() show_overview(log_data);
mode_actions.noncontact = @() run_review_with_overview(log_data, @evaluate_noncontact);
mode_actions.landing = @() run_review_with_overview(log_data, @evaluate_landing);
mode_actions.wall = @() run_review_with_overview(log_data, @evaluate_wall_contact);
mode_actions.forces = @() run_review_with_overview(log_data, @plot_contact_force_summary);
mode_actions.instantaneous = @() run_review_with_overview(log_data, @plot_instantaneous_contact_forces);
mode_actions.network = @() run_review_with_overview(log_data, @plot_network_and_fidelity_review);
mode_actions.dob = @() run_review_with_overview(log_data, @evaluate_dob); %DOB用に追加
end


function run_review_with_overview(log_data, review_action)
show_overview(log_data);
review_action(log_data);
end


function resolved_path = resolve_log_path(project_directory, requested_path)
if ~isempty(requested_path)
    if isfile(requested_path)
        resolved_path = requested_path;
        return;
    end

    candidate_path = fullfile(project_directory, requested_path);
    if isfile(candidate_path)
        resolved_path = candidate_path;
        return;
    end

    error('Log file not found: %s', requested_path);
end

log_directory = fullfile(project_directory, 'logs');
log_listing = dir(fullfile(log_directory, '*.mat'));
if isempty(log_listing)
    error('No .mat logs were found in %s', log_directory);
end

[~, newest_index] = max([log_listing.datenum]);
resolved_path = fullfile(log_listing(newest_index).folder, log_listing(newest_index).name);
fprintf('Using latest log: %s\n', resolved_path);
end


function log_data = load_log(log_path)
loaded_data = load(log_path, 'log');
if ~isfield(loaded_data, 'log')
    error('The file does not contain a log variable: %s', log_path);
end

log_data = loaded_data.log;
log_data.source_path = log_path;
end


function show_overview(log_data)
time = log_data.state.time;
contact = log_data.contact;

fprintf('\n=== Contact Log Overview ===\n');
fprintf('Source: %s\n', log_data.source_path);
fprintf('Samples: %d\n', numel(time));
fprintf('Duration: %.3f s\n', time(end) - time(1));
fprintf('Max contact count: %d\n', max(contact.count));
fprintf('Max contact force magnitude: %.6f\n', max(contact.max_force_magnitude));
fprintf('Max total normal force: %.6f\n', max(contact.total_normal_force));
fprintf('Max contact normal force: %.6f\n', max(contact.max_normal_force));
fprintf('Max left-wheel normal force: %.6f\n', max(contact.left_wheel.total_normal_force));
fprintf('Max right-wheel normal force: %.6f\n', max(contact.right_wheel.total_normal_force));
display_network_overview(log_data);

figure('Name', 'Contact Log Review', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot_scalar_series(time, contact.count, 'Contact Count', 'Count');

subplot(3, 1, 2);
plot_scalar_series(time, contact.max_normal_force, 'Max Contact Normal Force', 'Normal Force');

subplot(3, 1, 3);
plot_scalar_series(time, contact.total_force_magnitude, 'Total Contact Force Magnitude', 'Force Magnitude');

display_contact_pairs(log_data);
end


function display_contact_pairs(log_data)
pair_labels = strings(0, 1);
details = log_data.contact.details;
for index = 1:numel(details)
    sample_contacts = details{index};
    if isempty(sample_contacts)
        continue;
    end

    for contact_index = 1:numel(sample_contacts)
        pair_labels(end + 1, 1) = compose_pair_label(sample_contacts(contact_index)); %#ok<AGROW>
    end
end

if isempty(pair_labels)
    fprintf('Observed contact pairs: none\n');
    return;
end

unique_pairs = unique(pair_labels);
fprintf('Observed contact pairs:\n');
for pair_index = 1:numel(unique_pairs)
    fprintf('  %s\n', unique_pairs(pair_index));
end
end


function evaluate_noncontact(log_data)
contact = log_data.contact;
has_contact = any(contact.count > 0);
fprintf('\n=== Non-contact Evaluation ===\n');
if has_contact
    fprintf('Result: contact detected unexpectedly.\n');
    fprintf('Max contact count: %d\n', max(contact.count));
else
    fprintf('Result: no contact detected.\n');
end
end


function evaluate_landing(log_data)
contact = log_data.contact;
wheel_floor_detected = has_contact_pair(log_data, 'floor', 'left_wheel') || ...
    has_contact_pair(log_data, 'floor', 'right_wheel');
contact_started = find(contact.count > 0, 1, 'first');

fprintf('\n=== Landing Evaluation ===\n');
if isempty(contact_started)
    fprintf('Result: no landing contact detected.\n');
    return;
end

fprintf('First contact time: %.3f s\n', log_data.state.time(contact_started));
fprintf('Wheel-floor contact detected: %s\n', boolean_label(wheel_floor_detected));
fprintf('Max total normal force: %.6f\n', max(contact.total_normal_force));
fprintf('Max left-wheel normal force: %.6f\n', max(contact.left_wheel.total_normal_force));
fprintf('Max right-wheel normal force: %.6f\n', max(contact.right_wheel.total_normal_force));
fprintf('Max normal force: %.6f\n', max(contact.max_normal_force));
plot_landing_review(log_data, contact_started);
plot_contact_force_summary(log_data, contact_started);
end


function evaluate_wall_contact(log_data)
wall_detected = has_contact_pair(log_data, 'wall', 'left_wheel') || ...
    has_contact_pair(log_data, 'wall', 'right_wheel') || ...
    has_contact_pair(log_data, 'wall', 'floor') || ...
    has_contact_pair(log_data, 'wall', 'geom_');

fprintf('\n=== Wall Contact Evaluation ===\n');
fprintf('Wall contact detected: %s\n', boolean_label(wall_detected));
if wall_detected
    fprintf('Max contact force magnitude: %.6f\n', max(log_data.contact.max_force_magnitude));
end
end


function compare_impact_logs(primary_log, secondary_log)
fprintf('\n=== Impact Comparison ===\n');
fprintf('Log A: %s\n', primary_log.source_path);
fprintf('Log B: %s\n', secondary_log.source_path);

max_force_a = max(primary_log.contact.max_force_magnitude);
max_force_b = max(secondary_log.contact.max_force_magnitude);
max_normal_a = max(primary_log.contact.max_normal_force);
max_normal_b = max(secondary_log.contact.max_normal_force);

fprintf('Log A max force magnitude: %.6f\n', max_force_a);
fprintf('Log B max force magnitude: %.6f\n', max_force_b);
fprintf('Log A max normal force: %.6f\n', max_normal_a);
fprintf('Log B max normal force: %.6f\n', max_normal_b);
fprintf('Log A max total normal force: %.6f\n', max(primary_log.contact.total_normal_force));
fprintf('Log B max total normal force: %.6f\n', max(secondary_log.contact.total_normal_force));

if max_normal_a > max_normal_b
    fprintf('Result: Log A has the stronger peak normal contact.\n');
elseif max_normal_b > max_normal_a
    fprintf('Result: Log B has the stronger peak normal contact.\n');
else
    fprintf('Result: both logs have the same peak normal contact.\n');
end
end


function detected = has_contact_pair(log_data, token_a, token_b)
detected = false;
details = log_data.contact.details;
for index = 1:numel(details)
    sample_contacts = details{index};
    if isempty(sample_contacts)
        continue;
    end

    for contact_index = 1:numel(sample_contacts)
        pair_label = compose_pair_label(sample_contacts(contact_index));
        if contains(pair_label, token_a) && contains(pair_label, token_b)
            detected = true;
            return;
        end
    end
end
end


function label = compose_pair_label(contact_entry)
label = string(contact_entry.geom1) + ' <-> ' + string(contact_entry.geom2);
end


function label = boolean_label(value)
if value
    label = 'yes';
else
    label = 'no';
end
end


function plot_landing_review(log_data, contact_started)
time = log_data.state.time;
position_z = log_data.state.position(:, 3);
target_z = extract_target_z(log_data);
contact_count = log_data.contact.count;
max_normal_force = log_data.contact.max_normal_force;

figure('Name', 'Landing Review', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot_dual_series(time, position_z, target_z, 'Altitude vs Target Altitude', 'Z [m]', {'position z', 'target z'}, contact_started);

subplot(3, 1, 2);
plot_scalar_series(time, contact_count, 'Contact Count', 'Count', contact_started);

subplot(3, 1, 3);
plot_scalar_series(time, max_normal_force, 'Max Normal Force', 'Force', contact_started);
end


function target_z = extract_target_z(log_data)
if ~isfield(log_data, 'reference') || ~isfield(log_data.reference, 'target_position')
    target_z = nan(size(log_data.state.time));
    return;
end

target_z = log_data.reference.target_position(:, 3);
end


function plot_contact_force_summary(log_data, contact_started)
if nargin < 2
    contact_started = [];
end

time = log_data.state.time;
contact = log_data.contact;

figure('Name', 'Contact Force Summary', 'NumberTitle', 'off');
subplot(5, 1, 1);
plot_scalar_series(time, contact.count, 'Contact Count', 'Count', contact_started);

subplot(5, 1, 2);
plot_scalar_series(time, contact.max_normal_force, 'Max Normal Force', 'Force', contact_started);

subplot(5, 1, 3);
plot_normal_force_series( ...
    time, ...
    contact.total_normal_force, ...
    contact.left_wheel.total_normal_force, ...
    contact.right_wheel.total_normal_force, ...
    'Total / Per-Wheel Normal Force', ...
    contact_started ...
);

subplot(5, 1, 4);
plot_scalar_series(time, contact.max_force_magnitude, 'Max Contact Force Magnitude', 'Force', contact_started);

subplot(5, 1, 5);
plot_scalar_series(time, contact.total_force_magnitude, 'Total Contact Force Magnitude', 'Force', contact_started);
end


function plot_instantaneous_contact_forces(log_data)
time = log_data.state.time;
contact = log_data.contact;
contact_indices = find(contact.count > 0);

figure('Name', 'Instantaneous Contact Forces', 'NumberTitle', 'off');
subplot(4, 1, 1);
plot_normal_force_series( ...
    time, ...
    contact.total_normal_force, ...
    contact.left_wheel.total_normal_force, ...
    contact.right_wheel.total_normal_force, ...
    'Instantaneous Normal Force Time Series' ...
);

subplot(4, 1, 2);
plot_scalar_series(time, contact.total_force_magnitude, 'Instantaneous Total Contact Force Magnitude', 'Force');

subplot(4, 1, 3);
plot_scalar_series(time, contact.max_normal_force, 'Instantaneous Peak Normal Force per Step', 'Force');

subplot(4, 1, 4);
plot_scalar_series(time, contact.count, 'Instantaneous Contact Count', 'Count', [], true);

if isempty(contact_indices)
    fprintf('No contact interval was found for instantaneous-force zooming.\n');
    return;
end

zoom_start_index = max(contact_indices(1) - 50, 1);
zoom_end_index = min(contact_indices(end) + 50, numel(time));
plot_instantaneous_contact_zoom(log_data, zoom_start_index, zoom_end_index);
end


function display_network_overview(log_data)
network = get_network_log(log_data);
if isempty(network)
    fprintf('Network metrics: not available in this log format.\n');
    return;
end

fprintf('Mean state age: %.3f ms\n', mean(network.state_age_ms, 'omitnan'));
fprintf('Max state age: %.3f ms\n', max(network.state_age_ms, [], 'omitnan'));
fprintf('Max state sequence gap: %.0f\n', max(network.state_sequence_gap, [], 'omitnan'));
fprintf('Mean controller compute time: %.3f ms\n', mean(network.controller_compute_ms, 'omitnan'));

if has_actuator_log(log_data)
    actuator_error = log_data.actuator.tracking_error;
    fprintf('Max actuator tracking error: %.6f N\n', max(abs(actuator_error), [], 'all'));
end

if has_sensor_truth_log(log_data)
    sensor_position_error = vecnorm(log_data.state.position - log_data.sensor_truth.position, 2, 2);
    fprintf('Mean sensor position error: %.6f m\n', mean(sensor_position_error, 'omitnan'));
    fprintf('Max sensor position error: %.6f m\n', max(sensor_position_error, [], 'omitnan'));
end
end


function plot_network_and_fidelity_review(log_data)
network = get_network_log(log_data);
if isempty(network)
    error('This log does not include log.network metrics.');
end

time = log_data.state.time;
figure('Name', 'Network And Fidelity Review', 'NumberTitle', 'off');

subplot_count = 3 + double(has_actuator_log(log_data)) + double(has_sensor_truth_log(log_data));
subplot_index = 1;

subplot(subplot_count, 1, subplot_index);
subplot_index = subplot_index + 1;
plot_scalar_series(time, network.state_age_ms, 'State Packet Age', 'Age [ms]');

subplot(subplot_count, 1, subplot_index);
subplot_index = subplot_index + 1;
plot_scalar_series(time, network.state_sequence_gap, 'State Sequence Gap', 'Gap');

subplot(subplot_count, 1, subplot_index);
subplot_index = subplot_index + 1;
plot_scalar_series(time, network.controller_compute_ms, 'Controller Compute Time', 'Time [ms]');

if has_actuator_log(log_data)
    subplot(subplot_count, 1, subplot_index);
    subplot_index = subplot_index + 1;
    tracking_error = log_data.actuator.tracking_error;
    plot(time, tracking_error, 'LineWidth', 1.0);
    grid on;
    title('Actuator Tracking Error');
    ylabel('Error [N]');
    legend({'rotor 1', 'rotor 2', 'rotor 3', 'rotor 4'}, 'Location', 'best');
end

if has_sensor_truth_log(log_data)
    subplot(subplot_count, 1, subplot_index);
    sensor_position_error = vecnorm(log_data.state.position - log_data.sensor_truth.position, 2, 2);
    plot_scalar_series(time, sensor_position_error, 'Measured vs Truth Position Error', 'Error [m]');
end

fprintf('\n=== Network And Fidelity Review ===\n');
display_network_overview(log_data);
end


function network = get_network_log(log_data)
network = [];
if ~isfield(log_data, 'network')
    return;
end
network = log_data.network;
end


function tf = has_actuator_log(log_data)
tf = isfield(log_data, 'actuator') && isfield(log_data.actuator, 'tracking_error');
end


function tf = has_sensor_truth_log(log_data)
tf = isfield(log_data, 'sensor_truth') && isfield(log_data.sensor_truth, 'position');
end


function plot_instantaneous_contact_zoom(log_data, start_index, end_index)
time = log_data.state.time(start_index:end_index);
contact = log_data.contact;

figure('Name', 'Instantaneous Contact Forces Zoom', 'NumberTitle', 'off');
subplot(4, 1, 1);
plot_normal_force_series( ...
    time, ...
    contact.total_normal_force(start_index:end_index), ...
    contact.left_wheel.total_normal_force(start_index:end_index), ...
    contact.right_wheel.total_normal_force(start_index:end_index), ...
    'Zoomed Normal Force Time Series' ...
);

subplot(4, 1, 2);
plot_scalar_series(time, contact.total_force_magnitude(start_index:end_index), 'Zoomed Total Contact Force Magnitude', 'Force');

subplot(4, 1, 3);
plot_scalar_series(time, contact.max_normal_force(start_index:end_index), 'Zoomed Peak Normal Force per Step', 'Force');

subplot(4, 1, 4);
plot_scalar_series(time, contact.count(start_index:end_index), 'Zoomed Contact Count', 'Count', [], true);
end


function plot_scalar_series(time, values, title_text, y_label, contact_started, use_stairs)
if nargin < 5
    contact_started = [];
end
if nargin < 6
    use_stairs = false;
end

if use_stairs
    stairs(time, values, 'LineWidth', 1.2);
else
    plot(time, values, 'LineWidth', 1.2);
end

apply_contact_marker(contact_started, time);
grid on;
xlabel('Time [s]');
ylabel(y_label);
title(title_text);
end


function plot_normal_force_series(time, total_force, left_force, right_force, title_text, contact_started)
if nargin < 6
    contact_started = [];
end

plot(time, total_force, 'LineWidth', 1.4);
hold on;
plot(time, left_force, '--', 'LineWidth', 1.2);
plot(time, right_force, '--', 'LineWidth', 1.2);
apply_contact_marker(contact_started, time);
grid on;
xlabel('Time [s]');
ylabel('Force');
title(title_text);
legend({'total', 'left wheel', 'right wheel'}, 'Location', 'best');
end


function plot_dual_series(time, primary_values, secondary_values, title_text, y_label, legend_labels, contact_started)
if nargin < 7
    contact_started = [];
end

plot(time, primary_values, 'LineWidth', 1.4);
hold on;
plot(time, secondary_values, '--', 'LineWidth', 1.2);
apply_contact_marker(contact_started, time);
grid on;
xlabel('Time [s]');
ylabel(y_label);
title(title_text);
legend(legend_labels, 'Location', 'best');
end


function apply_contact_marker(contact_started, time)
if ~isempty(contact_started)
    hold on;
    xline(time(contact_started), ':r', 'First contact');
end
end


function evaluate_dob(log_data)

time = log_data.state.time;

ux = log_data.custom.ux;
Fhat = log_data.custom.Fhat;
lambda_hat = log_data.custom.lambda_hat;
lambda_true = log_data.contact.max_normal_force;

% --- DOB成立 ---
figure;
plot(time, ux, 'b'); hold on;
plot(time, -Fhat, 'r--');
legend('u_x','-Fhat');
title('DOB consistency');
grid on;

% --- 接触力 ---
figure;
plot(time, lambda_true, 'b'); hold on;
plot(time, lambda_hat, 'r--');
legend('lambda true','lambda hat');
title('Contact force comparison');
grid on;

% --- 不確かさ ---
d = lambda_true - ux;

figure;
plot(time, d);
title('Estimated d_{\Sigma,x}');
grid on;

end

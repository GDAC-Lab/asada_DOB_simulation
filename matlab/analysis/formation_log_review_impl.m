function formation_log_review_impl(mode, varargin)
arguments
    mode (1, :) char = 'overview'
end
arguments (Repeating)
    varargin
end

project_directory = fileparts(fileparts(fileparts(mfilename('fullpath'))));
log_paths = resolve_formation_log_paths(project_directory, varargin{:});
logs = load_formation_logs(log_paths);

mode_actions = struct();
mode_actions.overview = @() show_overview(logs);
mode_actions.tracking = @() show_tracking_review(logs);
mode_actions.rtf = @() show_realtime_factor_review(logs);
mode_actions.contacts = @() show_contact_review(logs);
mode_actions.network = @() show_network_review(logs);

if ~isfield(mode_actions, mode)
    error('Unsupported mode: %s', mode);
end

mode_actions.(mode)();
end


function log_paths = resolve_formation_log_paths(project_directory, varargin)
if ~isempty(varargin)
    log_paths = cellfun(@(value) resolve_single_log_path(project_directory, char(value)), varargin, 'UniformOutput', false);
    return;
end

log_directory = fullfile(project_directory, 'logs');
bundle_listing = dir(fullfile(log_directory, 'formation_bundle*.mat'));
if ~isempty(bundle_listing)
    [~, newest_bundle_index] = max([bundle_listing.datenum]);
    selected_bundle = bundle_listing(newest_bundle_index);
    log_paths = {fullfile(selected_bundle.folder, selected_bundle.name)};
    fprintf('Using combined formation log %s\n', log_paths{1});
    return;
end

log_listing = dir(fullfile(log_directory, 'formation_uav_*.mat'));
if isempty(log_listing)
    error('No formation logs were found in %s', log_directory);
end

timestamps = cellfun(@extract_timestamp_from_name, {log_listing.name}, 'UniformOutput', false);
valid_mask = ~cellfun(@isempty, timestamps);
log_listing = log_listing(valid_mask);
timestamps = timestamps(valid_mask);
if isempty(log_listing)
    error('No timestamped formation logs were found in %s', log_directory);
end

[~, newest_index] = max(cellfun(@datenum_from_timestamp, timestamps));
target_timestamp = timestamps{newest_index};
timestamp_mask = strcmp(timestamps, target_timestamp);
selected_logs = log_listing(timestamp_mask);
log_paths = fullfile({selected_logs.folder}, {selected_logs.name});
fprintf('Using formation logs with timestamp %s\n', target_timestamp);
for log_index = 1:numel(log_paths)
    fprintf('  %s\n', log_paths{log_index});
end
end


function resolved_path = resolve_single_log_path(project_directory, requested_path)
if isfile(requested_path)
    resolved_path = requested_path;
    return;
end

candidate_path = fullfile(project_directory, requested_path);
if isfile(candidate_path)
    resolved_path = candidate_path;
    return;
end

error('Formation log file not found: %s', requested_path);
end


function timestamp = extract_timestamp_from_name(file_name)
token = regexp(file_name, '\d{8}_\d{6}(?=\.mat$)', 'match', 'once');
if isempty(token)
    timestamp = '';
    return;
end
timestamp = token;
end


function value = datenum_from_timestamp(timestamp)
value = datenum(timestamp, 'yyyymmdd_HHMMSS');
end


function logs = load_formation_logs(log_paths)
logs = {};
for log_index = 1:numel(log_paths)
    loaded_data = load(log_paths{log_index});
    if isfield(loaded_data, 'formation_log')
        bundled_logs = extract_bundled_logs(loaded_data.formation_log, log_paths{log_index});
        for bundled_index = 1:numel(bundled_logs)
            loaded_log = bundled_logs{bundled_index};
            if ~isfield(loaded_log, 'source_path')
                loaded_log.source_path = log_paths{log_index};
            end
            logs{end + 1, 1} = loaded_log; %#ok<AGROW>
        end
        continue;
    end

    if ~isfield(loaded_data, 'log')
        error('The file does not contain a log or formation_log variable: %s', log_paths{log_index});
    end
    loaded_log = loaded_data.log;
    loaded_log.source_path = log_paths{log_index};
    logs{end + 1, 1} = loaded_log; %#ok<AGROW>
end

uav_indices = cellfun(@(log_entry) double(log_entry.config.uav_index), logs);
[~, sort_index] = sort(uav_indices);
logs = logs(sort_index);
end


function bundled_logs = extract_bundled_logs(formation_log, source_path)
if isfield(formation_log, 'uavs') && isstruct(formation_log.uavs)
    uav_field_names = fieldnames(formation_log.uavs);
    uav_field_names = sort_named_uav_fields(uav_field_names);
    bundled_logs = cell(numel(uav_field_names), 1);
    for field_index = 1:numel(uav_field_names)
        bundled_logs{field_index} = formation_log.uavs.(uav_field_names{field_index});
    end
    return;
end

if isfield(formation_log, 'logs')
    bundled_logs = formation_log.logs;
    if iscell(bundled_logs) && isscalar(bundled_logs)
        bundled_logs = bundled_logs{1};
    end
    if iscell(bundled_logs)
        return;
    end
end

error('Combined formation log has an unsupported layout: %s', source_path);
end


function sorted_field_names = sort_named_uav_fields(field_names)
field_indices = zeros(numel(field_names), 1);
for field_index = 1:numel(field_names)
    tokens = regexp(field_names{field_index}, '^uav_(\d+)$', 'tokens', 'once');
    if isempty(tokens)
        field_indices(field_index) = inf;
    else
        field_indices(field_index) = str2double(tokens{1});
    end
end

[~, sort_index] = sort(field_indices);
sorted_field_names = field_names(sort_index);
end


function show_overview(logs)
metrics = build_formation_metrics(logs);
fprintf('\n=== Formation Log Overview ===\n');
fprintf('UAVs: %d\n', metrics.num_uavs);
fprintf('Common samples: %d\n', metrics.sample_count);
fprintf('Duration: %.3f s\n', metrics.time(end) - metrics.time(1));
fprintf('Max centroid error norm: %.6f m\n', max(metrics.centroid_error_norm));
fprintf('Max slot error norm: %.6f m\n', max(metrics.max_slot_error_norm));
fprintf('Mean slot error norm: %.6f m\n', mean(metrics.mean_slot_error_norm));
fprintf('Mean realtime factor: %.6f\n', mean(metrics.mean_realtime_factor));
if ~isempty(metrics.state_age_ms)
    fprintf('Mean state age: %.3f ms\n', mean(metrics.mean_state_age_ms, 'omitnan'));
    fprintf('Max state sequence gap: %.0f\n', max(metrics.max_state_sequence_gap, [], 'omitnan'));
    fprintf('Mean controller compute time: %.3f ms\n', mean(metrics.mean_controller_compute_ms, 'omitnan'));
end

figure('Name', 'Formation Overview', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(metrics.time, metrics.centroid_xy(:, 1), 'LineWidth', 1.2);
hold on;
plot(metrics.time, metrics.centroid_xy(:, 2), 'LineWidth', 1.2);
plot(metrics.time, repmat(metrics.centroid_target_xy(1), metrics.sample_count, 1), '--', 'LineWidth', 1.0);
plot(metrics.time, repmat(metrics.centroid_target_xy(2), metrics.sample_count, 1), '--', 'LineWidth', 1.0);
grid on;
title('Formation Centroid');
ylabel('Position [m]');
legend({'centroid x', 'centroid y', 'target x', 'target y'}, 'Location', 'best');

subplot(3, 1, 2);
plot(metrics.time, metrics.centroid_error_norm, 'LineWidth', 1.2);
grid on;
title('Centroid Error Norm');
ylabel('Error [m]');

subplot(3, 1, 3);
plot(metrics.time, metrics.max_slot_error_norm, 'LineWidth', 1.2);
hold on;
plot(metrics.time, metrics.mean_slot_error_norm, 'LineWidth', 1.2);
grid on;
title('Slot Error Norm');
xlabel('Time [s]');
ylabel('Error [m]');
legend({'max slot error', 'mean slot error'}, 'Location', 'best');
end


function show_tracking_review(logs)
metrics = build_formation_metrics(logs);
figure('Name', 'Formation Tracking Review', 'NumberTitle', 'off');
subplot(2, 1, 1);
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.time, metrics.slot_error_norms(:, uav_index), 'LineWidth', 1.1);
end
grid on;
title('Per-UAV Slot Error Norm');
ylabel('Error [m]');
legend(build_uav_legend(metrics.num_uavs), 'Location', 'best');

subplot(2, 1, 2);
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.positions_xy(:, 1, uav_index), metrics.positions_xy(:, 2, uav_index), 'LineWidth', 1.1);
    plot(metrics.positions_xy(1, 1, uav_index), metrics.positions_xy(1, 2, uav_index), 'o');
end
grid on;
axis equal;
title('XY Trajectories');
xlabel('X [m]');
ylabel('Y [m]');
legend(build_uav_legend(metrics.num_uavs), 'Location', 'best');
end


function show_realtime_factor_review(logs)
metrics = build_formation_metrics(logs);
figure('Name', 'Formation Realtime Factor', 'NumberTitle', 'off');
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.time, metrics.realtime_factors(:, uav_index), 'LineWidth', 1.1);
end
plot(metrics.time, metrics.mean_realtime_factor, 'k--', 'LineWidth', 1.4);
grid on;
title('Realtime Factor');
xlabel('Time [s]');
ylabel('RTF');
legend([build_uav_legend(metrics.num_uavs), {'mean'}], 'Location', 'best');
end


function show_contact_review(logs)
metrics = build_formation_metrics(logs);
figure('Name', 'Formation Contact Review', 'NumberTitle', 'off');
subplot(2, 1, 1);
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.time, metrics.contact_counts(:, uav_index), 'LineWidth', 1.1);
end
grid on;
title('Per-UAV Contact Count');
ylabel('Count');
legend(build_uav_legend(metrics.num_uavs), 'Location', 'best');

subplot(2, 1, 2);
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.time, metrics.max_normal_forces(:, uav_index), 'LineWidth', 1.1);
end
grid on;
title('Per-UAV Max Normal Force');
xlabel('Time [s]');
ylabel('Force');
legend(build_uav_legend(metrics.num_uavs), 'Location', 'best');
end


function show_network_review(logs)
metrics = build_formation_metrics(logs);
if isempty(metrics.state_age_ms)
    error('These formation logs do not include network metrics.');
end

subplot_count = 3;
figure('Name', 'Formation Network Review', 'NumberTitle', 'off');

subplot(subplot_count, 1, 1);
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.time, metrics.state_age_ms(:, uav_index), 'LineWidth', 1.1);
end
plot(metrics.time, metrics.mean_state_age_ms, 'k--', 'LineWidth', 1.4);
grid on;
title('State Packet Age');
ylabel('Age [ms]');
legend([build_uav_legend(metrics.num_uavs), {'mean'}], 'Location', 'best');

subplot(subplot_count, 1, 2);
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.time, metrics.state_sequence_gap(:, uav_index), 'LineWidth', 1.1);
end
grid on;
title('State Sequence Gap');
ylabel('Gap');
legend(build_uav_legend(metrics.num_uavs), 'Location', 'best');

subplot(subplot_count, 1, 3);
hold on;
for uav_index = 1:metrics.num_uavs
    plot(metrics.time, metrics.controller_compute_ms(:, uav_index), 'LineWidth', 1.1);
end
plot(metrics.time, metrics.mean_controller_compute_ms, 'k--', 'LineWidth', 1.4);
grid on;
title('Controller Compute Time');
xlabel('Time [s]');
ylabel('Time [ms]');
legend([build_uav_legend(metrics.num_uavs), {'mean'}], 'Location', 'best');
end


function metrics = build_formation_metrics(logs)
num_uavs = numel(logs);
sample_count = min(cellfun(@(log_entry) numel(log_entry.state.time), logs));
if sample_count == 0
    error('Formation logs contain no samples.');
end

time = logs{1}.state.time(1:sample_count);
positions_xy = zeros(sample_count, 2, num_uavs);
realtime_factors = nan(sample_count, num_uavs);
contact_counts = zeros(sample_count, num_uavs);
max_normal_forces = zeros(sample_count, num_uavs);
slot_error_norms = zeros(sample_count, num_uavs);
desired_offsets_xy = zeros(2, num_uavs);
state_age_ms = [];
state_sequence_gap = [];
controller_compute_ms = [];

if isfield(logs{1}, 'network')
    state_age_ms = nan(sample_count, num_uavs);
    state_sequence_gap = nan(sample_count, num_uavs);
    controller_compute_ms = nan(sample_count, num_uavs);
end

for uav_index = 1:num_uavs
    log_entry = logs{uav_index};
    positions_xy(:, :, uav_index) = log_entry.state.position(1:sample_count, 1:2);
    if isfield(log_entry.state, 'realtime_factor')
        realtime_factors(:, uav_index) = log_entry.state.realtime_factor(1:sample_count);
    end
    contact_counts(:, uav_index) = log_entry.contact.count(1:sample_count);
    max_normal_forces(:, uav_index) = log_entry.contact.max_normal_force(1:sample_count);
    desired_offsets_xy(:, uav_index) = reshape(double(log_entry.config.desired_offset_xy), 2, 1);
    if isfield(log_entry, 'network')
        state_age_ms(:, uav_index) = log_entry.network.state_age_ms(1:sample_count);
        state_sequence_gap(:, uav_index) = log_entry.network.state_sequence_gap(1:sample_count);
        controller_compute_ms(:, uav_index) = log_entry.network.controller_compute_ms(1:sample_count);
    end
end

centroid_xy = squeeze(mean(positions_xy, 3));
centroid_target_xy = reshape(double(logs{1}.config.centroid_target_xy), 1, 2);
centroid_error_xy = repmat(centroid_target_xy, sample_count, 1) - centroid_xy;
centroid_error_norm = sqrt(sum(centroid_error_xy.^2, 2));

for uav_index = 1:num_uavs
    current_relative_xy = positions_xy(:, :, uav_index) - centroid_xy;
    slot_error_xy = repmat(desired_offsets_xy(:, uav_index)', sample_count, 1) - current_relative_xy;
    slot_error_norms(:, uav_index) = sqrt(sum(slot_error_xy.^2, 2));
end

metrics = struct( ...
        'num_uavs', num_uavs, ...
        'sample_count', sample_count, ...
        'time', time, ...
        'positions_xy', positions_xy, ...
        'centroid_xy', centroid_xy, ...
        'centroid_target_xy', centroid_target_xy, ...
        'centroid_error_norm', centroid_error_norm, ...
        'slot_error_norms', slot_error_norms, ...
        'max_slot_error_norm', max(slot_error_norms, [], 2), ...
        'mean_slot_error_norm', mean(slot_error_norms, 2), ...
        'realtime_factors', realtime_factors, ...
        'mean_realtime_factor', mean(realtime_factors, 2, 'omitnan'), ...
        'contact_counts', contact_counts, ...
        'max_normal_forces', max_normal_forces, ...
        'state_age_ms', state_age_ms, ...
        'mean_state_age_ms', mean(state_age_ms, 2, 'omitnan'), ...
        'state_sequence_gap', state_sequence_gap, ...
        'max_state_sequence_gap', max(state_sequence_gap, [], 2), ...
        'controller_compute_ms', controller_compute_ms, ...
        'mean_controller_compute_ms', mean(controller_compute_ms, 2, 'omitnan') ...
    );
end


function legend_entries = build_uav_legend(num_uavs)
legend_entries = arrayfun(@(uav_index) sprintf('uav %d', uav_index), 1:num_uavs, 'UniformOutput', false);
end


function contact_test_controller_zdob_wall(varargin)

% ===== plotオプション取得 =====
plot_enable = false;

if any(strcmp(varargin, 'plot_enable'))
    idx = find(strcmp(varargin, 'plot_enable'), 1);

    plot_enable = varargin{idx+1};

    %引数から削除
    varargin(idx:idx+1) = [];
end

close all; 
clc;

[scenario_name, runtime_options] = parse_inputs(varargin{:});
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

controller_config = controller_shared.build_controller_config(vehicle_params);
[allocation_matrix, mixer] = controller_shared.build_allocation_and_mixer(vehicle_params);
command_options = controller_shared.build_command_options(vehicle_params.command_mode, vehicle_params.thrust_coefficient, 'fidelity_mode', vehicle_params.fidelity.mode);
runtime_metrics = controller_shared.initialize_runtime_metrics();

scenario = build_test_scenario(scenario_name);
% ===== この実行用の個別ログフォルダ名 =====
run_timestamp = datestr(now, 'yyyymmdd_HHMMSS');
run_folder_name = [scenario.name '_' run_timestamp];

logging_options = build_logging_options(scenario, instance_options, run_folder_name);

logger = simulation_logger(project_directory, build_logging_config( ...
    vehicle_params, controller_config, scenario.waypoints(end, 2:4)', allocation_matrix, mixer, scenario, command_options, instance_options ...
), logging_options);

cleanup_handler = onCleanup(@() controller_shared.finalize_controller_run(logger)); %#ok<NASGU>

status_display_interval = 1.0;
next_status_time = 0.0;
scenario_start_time = NaN;
idle_deadline = tic;

fprintf('Starting contact test scenario: %s (%s, recv=%d, send=%d)\n', ...
    scenario.name, ...
    instance_options.label, ...
    instance_options.controller_local_port, ...
    instance_options.simulator_receive_port ...
);
controller_shared.display_logging_behavior(logger);

%%% 初期条件：z方向DOB + 壁押し補正 %%%
% dt = 0.001;

% z方向DOB
prev_state_time = NaN;
az_lpf = 0;
az_lpf_initialized = false;

% ===== zDOB用：前ステップ入力保存 =====
Fz_cmd_prev = NaN;
DeltaF_true_prev = 0;
DeltaF_z_true_prev = 0;
DeltaF_n_true_prev = 0;

% DOB設定
Dz_hat = 0;
Lz = 20;              % まず50ではなく20くらい推奨
tau_az = 0.05;        % 加速度LPF時定数 [s]
Dz_limit = 30;        % Dz_hatの安全上限 [N]
thrust_scale_max = 1.6;

% ===== 人工推力誤差（割合）=====
inject_thrust_error = true;

thrust_error_ratio = -0.1;  %%マイナスだから1割減

% 壁方向・壁面鉛直方向
% 壁へ押し付ける正方向を -x とする
n_wall = [-1; 0; 0];
t_wall_z = [0; 0; 1];

sin_min = 0.08;
cos_min = 0.30;

% 推力スケール暴走防止
scale_rate_limit = 1.25;
thrust_scale_prev = 1.0;


%%%データ配列の整理%%%
time_log = [];
pitch_log = [];

% zDOB用ログ
Dz_hat_log = [];
DeltaF_hat_log = [];
DeltaF_n_hat_log = [];

% 人工推力ログ
DeltaF_true_log = [];
DeltaF_n_true_log = [];
DeltaF_z_true_log = [];

% 壁押し評価用ログ
lambda_true_log = [];
lambda_true_sum_log = [];
Fn_nom_log = [];
Fn_hat_log = [];
Fn_actual_log = [];
lambda_req_log = [];


%%%%%%%%%%%%%%%%%%%%%%%%%ここからループ開始%%%%%%%%%%%%%%%%%%%%%%%%
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

        if isnan(scenario_start_time)
            scenario_start_time = double(state.time);
        end

        scenario_time = double(state.time) - scenario_start_time;
        if scenario_time > scenario.duration_seconds
            fprintf('Scenario complete at t=%.2f s\n', scenario_time);
            break;
        end

        target_position = interpolate_waypoints(scenario.waypoints, scenario_time);
        compute_timer = tic;
        rotor_thrusts = controller_shared.compute_hover_control( ...
            state, ...
            target_position, ...
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


% ============================================================
% z方向DOBに基づく推力誤差推定 + 壁法線方向押し付け補正
% ============================================================

% ===== 状態取得 =====
R = reshape(double(state.rotation_matrix), 3, 3);
z_axis = R(:,3);

vel = double(state.velocity(:));
vz = vel(3);

% ===== 公称総推力とワールド座標推力 =====
rotor_thrusts_nom = rotor_thrusts;
total_thrust_nom = sum(rotor_thrusts_nom);
force_world_nom = total_thrust_nom * z_axis;

% ===== 壁法線方向（n）・鉛直方向（z）成分 =====
Fn_nom = dot(force_world_nom, n_wall);      % 壁方向を正とした公称押し付け力
Fz_nom = dot(force_world_nom, t_wall_z);    % 鉛直方向の公称推力

% ===== ピッチ相当角 =====
% z_axis を壁方向 n_wall と鉛直方向 t_wall_z に分解
sin_th = dot(z_axis, n_wall);
cos_th = dot(z_axis, t_wall_z);

theta = atan2(sin_th, cos_th);
pitch_deg = rad2deg(theta);

% ゼロ割り防止
if sin_th >= 0
    sin_sign = 1;
else
    sin_sign = -1;
end

if cos_th >= 0
    cos_sign = 1;
else
    cos_sign = -1;
end

sin_th_safe = sin_sign * max(abs(sin_th), sin_min);
cos_th_safe = cos_sign * max(abs(cos_th), cos_min);

% ============================================================
% z方向DOB
% m*az = Fz_nom - mg + Dz
% Dz = DeltaF*cos(theta) + dz
% ============================================================

% if first_velocity_sample
%     vz_prev = vz;
%     first_velocity_sample = false;
% end
% 
% az_meas = (vz - vz_prev) / dt;
% vz_prev = vz;
current_time = double(state.time);

dt_meas = 0.001;
if isnan(prev_state_time)
    prev_state_time = current_time;
    vz_prev = vz;
    az_meas = 0;
else
    dt_meas = current_time - prev_state_time;

    % 異常dt対策
    if dt_meas <= 0 || dt_meas > 0.1
        dt_meas = 0.001;
    end

    az_raw = (vz - vz_prev) / dt_meas;

    % 加速度LPF
    if ~az_lpf_initialized
        az_lpf = az_raw;
        az_lpf_initialized = true;
    else
        alpha = exp(-dt_meas / tau_az);
        az_lpf = alpha * az_lpf + (1 - alpha) * az_raw;
    end

    az_meas = az_lpf;

    vz_prev = vz;
    prev_state_time = current_time;
end

% ============================================================
% z方向残差：前ステップの公称入力を使う
% ============================================================

if isnan(Fz_cmd_prev)
    Fz_model_used = Fz_nom;
    DeltaF_true_used = 0;
    DeltaF_z_true_used = 0;
    DeltaF_n_true_used = 0;
else
    Fz_model_used = Fz_cmd_prev;
    DeltaF_true_used = DeltaF_true_prev;
    DeltaF_z_true_used = DeltaF_z_true_prev;
    DeltaF_n_true_used = DeltaF_n_true_prev;
end

rz = vehicle_params.mass * az_meas ...
     - (Fz_model_used - vehicle_params.mass * vehicle_params.gravity);


% 一次ローパスDOBの厳密離散化
a_dob = exp(-Lz * dt_meas);
Dz_hat = a_dob * Dz_hat + (1 - a_dob) * rz;

% 推定値の飽和
Dz_hat = max(min(Dz_hat, Dz_limit), -Dz_limit);

% ============================================================
% Dz_hat から推力誤差を推定
% Dz_hat ≈ DeltaF*cos(theta) + dz
% ============================================================

DeltaF_hat = Dz_hat / cos_th_safe;

% 推力誤差を壁法線方向へ射影
DeltaF_n_hat = DeltaF_hat * sin_th;

% ============================================================
% 接触維持に必要な押し付け力
% ============================================================
%%摩擦ベースではなく姿勢ベースでreqを決定
theta_req = deg2rad(10);
lambda_req = vehicle_params.mass * vehicle_params.gravity * tan(theta_req);

% ===== 推力不足を見込んだ制御用要求値 =====
thrust_error_bound = 0.10;   % 10%推力不足を想定
lambda_req_eff = lambda_req / (1 - thrust_error_bound);

% ===== 壁押しに必要な総推力 =====
% Fn_hat = F*sin(theta) + DeltaF_n_hat >= lambda_req
thrust_required_wall = (lambda_req_eff - DeltaF_n_hat) / sin_th_safe;


% sinが小さい/負の場合は危険なので補正を止める
if sin_th <= sin_min
    thrust_required_wall = total_thrust_nom;
end

% 高度制御用の公称推力と壁押し要求の大きい方を採用
total_thrust_cmd = max(total_thrust_nom, thrust_required_wall);

% 推力スケール
thrust_scale_raw = total_thrust_cmd / max(total_thrust_nom, 1.0e-6);

% 急激な増加を制限
thrust_scale = min(thrust_scale_raw, thrust_scale_prev * scale_rate_limit);
thrust_scale = min(thrust_scale, thrust_scale_max);
thrust_scale = max(thrust_scale, 0.0);
thrust_scale_prev = thrust_scale;


% ============================================================
% 制御器が意図したロータ推力
% ============================================================
rotor_thrusts_cmd = rotor_thrusts_nom * thrust_scale;

% 公称指令の飽和
rotor_thrusts_cmd = max(rotor_thrusts_cmd, 0);
rotor_thrusts_cmd = min(rotor_thrusts_cmd, vehicle_params.max_rotor_thrust);
total_thrust_cmd = sum(rotor_thrusts_cmd);

% ============================================================
% 補正後の公称押し付け力を再計算
% ============================================================
force_world_cmd = total_thrust_cmd * z_axis;

Fn_cmd_nom = dot(force_world_cmd, n_wall);
Fz_cmd_nom = dot(force_world_cmd, t_wall_z);

% 次ステップのzDOB残差計算用に保存
Fz_cmd_current = Fz_cmd_nom;

% 推定された推力誤差を含む，補正後の実効押し付け力
Fn_hat_cmd = Fn_cmd_nom + DeltaF_n_hat;


% ============================================================
% 人工的な割合推力誤差
% F_act = (1 + alpha) F_cmd
% ============================================================
if inject_thrust_error
    rotor_thrusts_actual = rotor_thrusts_cmd * (1 + thrust_error_ratio);
else
    rotor_thrusts_actual = rotor_thrusts_cmd;
end

% 実際に送る推力の飽和
rotor_thrusts_actual = max(rotor_thrusts_actual, 0);
rotor_thrusts_actual = min(rotor_thrusts_actual, vehicle_params.max_rotor_thrust);

% ============================================================
% 実際に送る推力による壁方向成分
% ============================================================
total_thrust_actual = sum(rotor_thrusts_actual);
force_world_actual = total_thrust_actual * z_axis;

Fn_actual = dot(force_world_actual, n_wall);

% 飽和後の真の推力誤差
DeltaF_true = sum(rotor_thrusts_actual) - sum(rotor_thrusts_cmd);
DeltaF_n_true = DeltaF_true * sin_th;
DeltaF_z_true = DeltaF_true * cos_th;

% ============================================================
% 次ステップ評価用に保存
% ============================================================
Fz_cmd_prev = Fz_cmd_current;

DeltaF_true_prev = DeltaF_true;
DeltaF_z_true_prev = DeltaF_z_true;
DeltaF_n_true_prev = DeltaF_n_true;


% 実際にシミュレータへ送る推力
rotor_thrusts = rotor_thrusts_actual;

% ============================================================
% 接触力真値
% 複数接触に対応するため max と sum を両方記録
% ============================================================

lambda_true = 0;
lambda_true_sum = 0;
contact_count = controller_shared.get_contact_summary_field(state, 'count');

if isfield(state, 'contact_summary') && ~isempty(state.contact_summary)
    try
        lambda_all = [state.contact_summary.max_normal_force];

        if ~isempty(lambda_all)
            lambda_true = max(lambda_all);
            lambda_true_sum = sum(lambda_all);
        end
    catch
        lambda_true = controller_shared.get_contact_summary_field(state, 'max_normal_force');
        lambda_true_sum = lambda_true;
    end
end

% グラフに出す推定押し付け力は，推力補正後の値を使う
Fn_hat = Fn_hat_cmd;


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
        % logger.append(state, control_command, target_posiion);

        if state.time >= next_status_time
            display_status(state, target_position, control_command, command_options, scenario.name, scenario_time, instance_options.label);
            next_status_time = state.time + status_display_interval;
        end

       % ===== ログ保存 =====
time_log(end+1) = scenario_time;
pitch_log(end+1) = pitch_deg;

% zDOBログ
Dz_hat_log(end+1) = Dz_hat;
DeltaF_hat_log(end+1) = DeltaF_hat;
DeltaF_n_hat_log(end+1) = DeltaF_n_hat;

% 推力誤差真値ログ
DeltaF_true_log(end+1) = DeltaF_true;
DeltaF_n_true_log(end+1) = DeltaF_n_true;
DeltaF_z_true_log(end+1) = DeltaF_z_true;

% 壁押し評価ログ
lambda_true_log(end+1) = lambda_true;
lambda_true_sum_log(end+1) = lambda_true_sum;
Fn_nom_log(end+1) = Fn_nom;
Fn_hat_log(end+1) = Fn_hat;
Fn_actual_log(end+1) = Fn_actual;
lambda_req_log(end+1) = lambda_req;


    end
catch execution_error
    fprintf('\nScenario controller stopped:\n');
    disp(getReport(execution_error, 'extended', 'hyperlinks', 'on'));
end


% ============================================================
% ログ保存フォルダ作成
% ============================================================
log_root = fullfile(project_directory, 'logs');
log_dir = fullfile(log_root, run_folder_name);

if ~exist(log_dir, 'dir')
    mkdir(log_dir);
end

% ============================================================
% mat保存：plot_enableに関係なく必ず保存
% ============================================================
save(fullfile(log_dir, 'dob_log.mat'), ...
    'time_log', ...
    'pitch_log', ...
    'Dz_hat_log', ...
    'DeltaF_hat_log', ...
    'DeltaF_n_hat_log', ...
    'DeltaF_true_log', ...
    'DeltaF_n_true_log', ...
    'DeltaF_z_true_log', ...
    'lambda_true_log', ...
    'lambda_true_sum_log', ...
    'Fn_nom_log', ...
    'Fn_hat_log', ...
    'Fn_actual_log', ...
    'lambda_req_log');

fprintf('MAT log saved -> %s\n', fullfile(log_dir, 'dob_log.mat'));

% ============================================================
% グラフ描画
% ============================================================
if plot_enable
    % --- ピッチ角 ---
    fig1 = figure;
    plot(time_log, pitch_log, 'LineWidth', 2)
    title('ピッチ角 [deg]')
    grid on
    % xlim([0 max(time_log)])
    ylim([-10 30])  
    
exportgraphics(fig1, fullfile(log_dir, 'ピッチ角.png'), ...
    'Resolution', 300, ...
    'BackgroundColor', 'white');

% --- z方向DOB ---
fig2 = figure;
plot(time_log, Dz_hat_log, 'b', 'LineWidth', 2); hold on
plot(time_log, DeltaF_z_true_log, 'g--', 'LineWidth', 2)

legend('Dz hat','DeltaF z true', 'Location', 'best')
title('z方向DOBによる推力誤差成分推定')
xlabel('time [s]')
ylabel('Force [N]')
grid on
ylim([-3 2])

exportgraphics(fig2, fullfile(log_dir, 'z方向DOB推定.png'), ...
    'Resolution', 300, ...
    'BackgroundColor', 'white');

% --- 接触維持評価 ---
fig3 = figure;
plot(time_log, Fn_hat_log, 'r', 'LineWidth', 2); hold on
plot(time_log, Fn_actual_log, 'm-.', 'LineWidth', 2)
plot(time_log, lambda_req_log, 'k--', 'LineWidth', 2)

legend('Fn hat after scale', ...
       'Fn actual', ...
       'lambda req', ...
       'Location', 'best')

title('接触維持条件')
xlabel('time [s]')
ylabel('Force [N]')
grid on

exportgraphics(fig3, fullfile(log_dir, '接触維持条件.png'), ...
    'Resolution', 300, ...
    'BackgroundColor', 'white');
end
end


function scenario = build_test_scenario(scenario_name)
switch scenario_name
    case 'hover'
        scenario = struct( ...
            'name', 'hover', ...
            'duration_seconds', 8.0, ...
            'waypoints', [ ...
                0.0, 0.0, 0.0, 1.5; ...
                8.0, 0.0, 0.0, 1.5 ...
            ] ...
        );
case 'wall_load'
scenario = struct( ...
    'name', 'wallrunning', ...
    'duration_seconds', 10.0, ...
    'waypoints', [ ...
        0.0   2.67  0   0.15;   % 壁の下（ほぼ接触）
        3.0   3.30  0   0.15;   % 押し付け開始
        5.0   3.30  0   1.2;    % 登る
        10.0  3.30  0   1.2
    ] ...
);


    otherwise
        error('Unsupported scenario: %s', scenario_name);
end
end


function target_position = interpolate_waypoints(waypoints, scenario_time)
if scenario_time <= waypoints(1, 1)
    target_position = waypoints(1, 2:4)';
    return;
end

if scenario_time >= waypoints(end, 1)
    target_position = waypoints(end, 2:4)';
    return;
end

for waypoint_index = 1:(size(waypoints, 1) - 1)
    start_waypoint = waypoints(waypoint_index, :);
    end_waypoint = waypoints(waypoint_index + 1, :);
    if scenario_time < end_waypoint(1)
        interval = end_waypoint(1) - start_waypoint(1);
        alpha = (scenario_time - start_waypoint(1)) / interval;
        target_position = ((1.0 - alpha) * start_waypoint(2:4) + alpha * end_waypoint(2:4))';
        return;
    end
end

target_position = waypoints(end, 2:4)';
end


function display_status(state, target_position, control_command, command_options, scenario_name, scenario_time, instance_label)
position = reshape(double(state.position), [], 1);
position_error = target_position - position;
contact_count = controller_shared.get_contact_summary_field(state, 'count');
max_normal_force = controller_shared.get_contact_summary_field(state, 'max_normal_force');
command_values = controller_shared.displayed_command_values(control_command, command_options);
realtime_factor = controller_shared.get_realtime_factor(state);
fprintf( ...
    '[%s %s t=%.2f s, rtf=%.2f] pos=[%.3f %.3f %.3f] m, target=[%.3f %.3f %.3f] m, err=[%.3f %.3f %.3f] m, contacts=%d, maxFn=%.3f, cmd=%s [%.3f %.3f %.3f %.3f] %s\n', ...
    scenario_name, ...
    instance_label, ...
    scenario_time, ...
    realtime_factor, ...
    position(1), position(2), position(3), ...
    target_position(1), target_position(2), target_position(3), ...
    position_error(1), position_error(2), position_error(3), ...
    contact_count, ...
    max_normal_force, ...
    command_options.input_mode, ...
    command_values(1), command_values(2), command_values(3), command_values(4), ...
    controller_shared.command_unit_label(command_options.input_mode) ...
);
end


function config = build_logging_config(vehicle_params, controller_config, target_position, allocation_matrix, mixer, scenario, command_options, instance_options)
config = controller_shared.build_base_logger_config(vehicle_params, controller_config, allocation_matrix, mixer, command_options, instance_options);
config.target_position = target_position;
config.scenario = scenario;
end


function logging_options = build_logging_options(scenario, instance_options, run_folder_name)

if nargin < 3 || isempty(run_folder_name)
    directory_name = 'logs';
else
    directory_name = fullfile('logs', run_folder_name);
end

logging_options = struct( ...
    'save_mode', 'finalize', ...
    'periodic_interval_seconds', 30.0, ...
    'print_save_events', true, ...
    'directory_name', directory_name, ...
    'file_prefix', ['contact_test_' scenario.name instance_options.file_suffix] ...
);
end


function [scenario_name, runtime_options] = parse_inputs(varargin)
scenario_name = 'wall_load';
parse_start_index = 1;
if ~isempty(varargin)
    first_argument = varargin{1};
    if ischar(first_argument) || (isstring(first_argument) && isscalar(first_argument))
        scenario_name = char(first_argument);
        parse_start_index = 2;
    end
end

parser = inputParser;
addParameter(parser, 'instance_id', 0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'integer', 'nonnegative'}));
addParameter(parser, 'wait_for_startup_seconds', 3.0, @(value) validateattributes(value, {'numeric'}, {'scalar', 'positive'}));
addParameter(parser, 'state_timeout_seconds', inf, @(value) (isnumeric(value) && isscalar(value) && value > 0) || isinf(value));
addParameter(parser, 'headless', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'simulation_duration_seconds', NaN, @(value) (isnumeric(value) && isscalar(value)) || isempty(value));
addParameter(parser, 'auto_launch', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'shutdown_on_exit', false, @(value) islogical(value) || (isnumeric(value) && isscalar(value)));
addParameter(parser, 'simulator_root', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'params_path', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
addParameter(parser, 'generated_xml_directory', '', @(value) ischar(value) || (isstring(value) && isscalar(value)));
parse(parser, varargin{parse_start_index:end});

runtime_options = parser.Results;
runtime_options.instance_id = double(runtime_options.instance_id);
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


function path_value = resolve_runtime_path_option(path_value, default_value)
if isempty(path_value)
    path_value = default_value;
end
path_value = char(path_value);
end
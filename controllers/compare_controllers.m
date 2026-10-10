
function comparison = compare_controllers(PARAM, gait, dist)
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Comparative Analysis: Baseline PID vs Sliding Mode Control (SMC)
%
% Compares PID and SMC using the same plant, desired gait, and disturbance.
%
% Usage:
%   comparison = compare_controllers();
%   comparison = compare_controllers(PARAM, gait, dist);

%% 1. Load parameters and inputs safely

if nargin < 1 || isempty(PARAM)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    PARAM = load_project_parameters(project_root);
end

if nargin < 2 || isempty(gait)
    addpath(PARAM.dirs.trajectories);
    gait = generate_gait(PARAM);
end

% Generate disturbance directly if none is supplied.
% A separate disturbance_model.m file is not required.
if nargin < 3 || isempty(dist)
    t_dist = PARAM.simulation.t;

    dist.t_start = PARAM.disturbance.start_time;
    dist.t_end = PARAM.disturbance.end_time;
    dist.magnitude = PARAM.disturbance.magnitude;

    dist.tau_dist = zeros(size(t_dist));

    disturbance_idx = (t_dist >= dist.t_start) & ...
                      (t_dist <= dist.t_end);

    dist.tau_dist(disturbance_idx) = dist.magnitude;
end

% Validate and standardize input dimensions.
t = PARAM.simulation.t(:);
dt = PARAM.simulation.Ts;
N_steps = numel(t);

if numel(dist.tau_dist) ~= N_steps
    error('Disturbance vector must match the simulation time vector.');
end

dist.tau_dist = dist.tau_dist(:);

if numel(gait.theta_rad) ~= N_steps || ...
   numel(gait.theta_dot_rad) ~= N_steps || ...
   numel(gait.theta_ddot_rad) ~= N_steps || ...
   numel(gait.theta_deg) ~= N_steps
    error('Gait vectors must match the simulation time vector.');
end

%% 2. System parameters

J = PARAM.joint.inertia;
B = PARAM.joint.damping;
K = PARAM.joint.stiffness;

tau_sat = PARAM.actuator.max_torque;

theta_d_rad = gait.theta_rad(:);
theta_dot_d_rad = gait.theta_dot_rad(:);
theta_ddot_d_rad = gait.theta_ddot_rad(:);
theta_d_deg = gait.theta_deg(:);

tau_dist_zero = zeros(N_steps, 1);
tau_dist_active = dist.tau_dist;

%% 3. PID and SMC simulation functions

% PID controller with filtered derivative and anti-windup.
function [th_deg, err_deg, u_hist] = sim_pid(d_torque)

    th = zeros(N_steps, 1);
    th_dot = zeros(N_steps, 1);
    u_hist = zeros(N_steps, 1);
    err_rad = zeros(N_steps, 1);

    Kp = PARAM.pid.Kp;
    Ki = PARAM.pid.Ki;
    Kd = PARAM.pid.Kd;
    Nf = PARAM.pid.N;

    e_i = 0;
    de_f = 0;

    for i = 1:N_steps-1

        e = theta_d_rad(i) - th(i);
        err_rad(i) = e;

        up = Kp * e;

        e_i = e_i + e * dt;
        ui = Ki * e_i;

        de_raw = theta_dot_d_rad(i) - th_dot(i);
        de_f = de_f + (Nf * (de_raw - de_f)) * dt;
        ud = Kd * de_f;

        u_raw = up + ui + ud;
        u_app = max(-tau_sat, min(tau_sat, u_raw));

        u_hist(i) = u_app;

        % Conditional integration anti-windup.
        if (u_raw > tau_sat && e > 0) || ...
           (u_raw < -tau_sat && e < 0)
            e_i = e_i - e * dt;
        end

        th_ddot = (u_app + d_torque(i) ...
            - B * th_dot(i) - K * th(i)) / J;

        th_dot(i+1) = th_dot(i) + th_ddot * dt;
        th(i+1) = th(i) + th_dot(i+1) * dt;
    end

    err_rad(end) = theta_d_rad(end) - th(end);
    u_hist(end) = u_hist(end-1);

    th_deg = rad2deg(th);
    err_deg = rad2deg(err_rad);
end

% Sliding Mode Controller with model-based equivalent torque.
function [th_deg, err_deg, u_hist, s_hist] = sim_smc(d_torque)

    th = zeros(N_steps, 1);
    th_dot = zeros(N_steps, 1);
    u_hist = zeros(N_steps, 1);
    s_hist = zeros(N_steps, 1);
    err_rad = zeros(N_steps, 1);

    lam = PARAM.smc.lambda;
    Ksw = PARAM.smc.K_sw;
    phi = PARAM.smc.phi;

    for i = 1:N_steps-1

        e = theta_d_rad(i) - th(i);
        e_dot = theta_dot_d_rad(i) - th_dot(i);

        err_rad(i) = e;

        s = e_dot + lam * e;
        s_hist(i) = s;

        u_eq = J * (theta_ddot_d_rad(i) + lam * e_dot) ...
             + B * th_dot(i) + K * th(i);

        if isfield(PARAM.smc, 'use_tanh') && ~PARAM.smc.use_tanh
            u_sw = Ksw * max(-1, min(1, s / phi));
        else
            u_sw = Ksw * tanh(s / phi);
        end

        u_raw = u_eq + u_sw;
        u_app = max(-tau_sat, min(tau_sat, u_raw));

        u_hist(i) = u_app;

        th_ddot = (u_app + d_torque(i) ...
            - B * th_dot(i) - K * th(i)) / J;

        th_dot(i+1) = th_dot(i) + th_ddot * dt;
        th(i+1) = th(i) + th_dot(i+1) * dt;
    end

    err_rad(end) = theta_d_rad(end) - th(end);
    u_hist(end) = u_hist(end-1);
    s_hist(end) = s_hist(end-1);

    th_deg = rad2deg(th);
    err_deg = rad2deg(err_rad);
end

%% 4. Execute scenarios

% Scenario A: nominal gait, no disturbance.
[pid_nom_th, pid_nom_err, pid_nom_tau] = sim_pid(tau_dist_zero);
[smc_nom_th, smc_nom_err, smc_nom_tau, smc_nom_s] = ...
    sim_smc(tau_dist_zero);

% Scenario B: gait tracking with disturbance.
[pid_dst_th, pid_dst_err, pid_dst_tau] = sim_pid(tau_dist_active);
[smc_dst_th, smc_dst_err, smc_dst_tau, smc_dst_s] = ...
    sim_smc(tau_dist_active);

%% 5. Calculate performance metrics

% Nominal PID metrics.
metrics.pid_nom_rmse = sqrt(mean(pid_nom_err.^2));
metrics.pid_nom_mae = mean(abs(pid_nom_err));
metrics.pid_nom_max = max(abs(pid_nom_err));
metrics.pid_nom_eng = trapz(t, pid_nom_tau.^2);

% Nominal SMC metrics.
metrics.smc_nom_rmse = sqrt(mean(smc_nom_err.^2));
metrics.smc_nom_mae = mean(abs(smc_nom_err));
metrics.smc_nom_max = max(abs(smc_nom_err));
metrics.smc_nom_eng = trapz(t, smc_nom_tau.^2);

% Full simulation with disturbance.
metrics.pid_dst_rmse = sqrt(mean(pid_dst_err.^2));
metrics.pid_dst_mae = mean(abs(pid_dst_err));
metrics.pid_dst_max = max(abs(pid_dst_err));
metrics.pid_dst_eng = trapz(t, pid_dst_tau.^2);

metrics.smc_dst_rmse = sqrt(mean(smc_dst_err.^2));
metrics.smc_dst_mae = mean(abs(smc_dst_err));
metrics.smc_dst_max = max(abs(smc_dst_err));
metrics.smc_dst_eng = trapz(t, smc_dst_tau.^2);

% Metrics specifically inside the disturbance window (inclusive).
w_idx = (t >= dist.t_start) & (t <= dist.t_end);

if any(w_idx)
    metrics.pid_w_rmse = sqrt(mean(pid_dst_err(w_idx).^2));
    metrics.smc_w_rmse = sqrt(mean(smc_dst_err(w_idx).^2));

    metrics.pid_w_peak_err = max(abs(pid_dst_err(w_idx)));
    metrics.smc_w_peak_err = max(abs(smc_dst_err(w_idx)));
else
    metrics.pid_w_rmse = NaN;
    metrics.smc_w_rmse = NaN;
    metrics.pid_w_peak_err = NaN;
    metrics.smc_w_peak_err = NaN;
end

%% 6. Post-disturbance recovery time

% Recovery means the absolute tracking error stays within +/- threshold
% continuously for at least hold_duration after the disturbance ends.
% If that never happens before the simulation ends, recovery is NaN
% (not observed). Do not invent a recovery time.

post_idx = find(t > dist.t_end);

if isfield(PARAM, 'metrics') && isfield(PARAM.metrics, 'recovery_threshold_deg')
    nom_threshold = PARAM.metrics.recovery_threshold_deg;
else
    nom_threshold = 2.0;
end
if isfield(PARAM, 'metrics') && isfield(PARAM.metrics, 'recovery_hold_s')
    hold_duration = PARAM.metrics.recovery_hold_s;
else
    hold_duration = 0.1;
end
hold_samples = round(hold_duration / dt) + 1;

metrics.pid_rec_time = NaN;
metrics.smc_rec_time = NaN;
metrics.recovery_threshold_deg = nom_threshold;
metrics.recovery_hold_s = hold_duration;

if numel(post_idx) >= hold_samples

    % PID recovery.
    for k = 1:(numel(post_idx) - hold_samples + 1)

        window_idx = post_idx(k:k + hold_samples - 1);

        if all(abs(pid_dst_err(window_idx)) <= nom_threshold)
            metrics.pid_rec_time = ...
                t(post_idx(k)) - dist.t_end;
            break;
        end
    end

    % SMC recovery.
    for k = 1:(numel(post_idx) - hold_samples + 1)

        window_idx = post_idx(k:k + hold_samples - 1);

        if all(abs(smc_dst_err(window_idx)) <= nom_threshold)
            metrics.smc_rec_time = ...
                t(post_idx(k)) - dist.t_end;
            break;
        end
    end
end

%% 7. Calculate percentage changes

% Positive RMSE reduction means SMC RMSE is lower than PID RMSE.
if metrics.pid_nom_rmse > 0
    metrics.pct_rmse_reduction = ...
        (1 - metrics.smc_nom_rmse / metrics.pid_nom_rmse) * 100;
else
    metrics.pct_rmse_reduction = NaN;
end

if metrics.pid_nom_mae > 0
    metrics.pct_mae_reduction = ...
        (1 - metrics.smc_nom_mae / metrics.pid_nom_mae) * 100;
else
    metrics.pct_mae_reduction = NaN;
end

if isfinite(metrics.pid_w_peak_err) && metrics.pid_w_peak_err > 0
    metrics.pct_dist_err_reduction = ...
        (1 - metrics.smc_w_peak_err / metrics.pid_w_peak_err) * 100;
else
    metrics.pct_dist_err_reduction = NaN;
end

%% 8. Figure 10: Nominal controller comparison

hFig10 = figure( ...
    'Name', 'PID vs SMC Nominal Tracking Comparison', ...
    'NumberTitle', 'off', ...
    'Units', 'pixels', ...
    'Position', [100, 100, 950, 750], ...
    'Color', 'w');

subplot(3, 1, 1);

plot(t, theta_d_deg, 'k--', 'LineWidth', 2.0, ...
    'DisplayName', 'Desired angle');
hold on;

plot(t, pid_nom_th, 'b-', 'LineWidth', 1.6, ...
    'DisplayName', sprintf('PID RMSE = %.2f deg', ...
    metrics.pid_nom_rmse));

plot(t, smc_nom_th, 'm-', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('SMC RMSE = %.2f deg', ...
    metrics.smc_nom_rmse));

grid on;
grid minor;
xlim([0, PARAM.simulation.time]);
ylabel('Joint Angle (deg)');
title('Nominal Gait Tracking: PID vs SMC');
legend('Location', 'northeast');

subplot(3, 1, 2);

plot(t, pid_nom_err, 'b-', 'LineWidth', 1.5, ...
    'DisplayName', 'PID error');
hold on;

plot(t, smc_nom_err, 'm-', 'LineWidth', 1.8, ...
    'DisplayName', 'SMC error');

yline(0, 'k--', 'DisplayName', 'Zero error');

grid on;
grid minor;
xlim([0, PARAM.simulation.time]);
ylabel('Tracking Error (deg)');
legend('Location', 'best');

subplot(3, 1, 3);

plot(t, pid_nom_tau, 'b-', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('PID energy = %.1f', ...
    metrics.pid_nom_eng));
hold on;

plot(t, smc_nom_tau, 'm-', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('SMC energy = %.1f', ...
    metrics.smc_nom_eng));

yline(tau_sat, 'r--', 'DisplayName', 'Upper torque limit');
yline(-tau_sat, 'r--', 'DisplayName', 'Lower torque limit');

grid on;
grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)');
ylabel('Torque (N m)');
legend('Location', 'northeast');

fig10_path = fullfile(PARAM.dirs.results, ...
    'fig10_pid_vs_smc_nominal.png');

try
    saveas(hFig10, fig10_path);
    fprintf('Saved nominal comparison plot to: %s\n', fig10_path);
catch ME
    warning('Could not save figure 10: %s', ME.message);
end

%% 9. Figure 11: Disturbance rejection comparison

hFig11 = figure( ...
    'Name', 'PID vs SMC Disturbance Rejection Comparison', ...
    'NumberTitle', 'off', ...
    'Units', 'pixels', ...
    'Position', [130, 80, 950, 780], ...
    'Color', 'w');

subplot(3, 1, 1);

plot(t, theta_d_deg, 'k--', 'LineWidth', 2.0, ...
    'DisplayName', 'Desired angle');
hold on;

plot(t, pid_dst_th, 'b-', 'LineWidth', 1.6, ...
    'DisplayName', 'PID with disturbance');

plot(t, smc_dst_th, 'm-', 'LineWidth', 1.8, ...
    'DisplayName', 'SMC with disturbance');

patch([dist.t_start, dist.t_end, dist.t_end, dist.t_start], ...
    [min(theta_d_deg)-5, min(theta_d_deg)-5, ...
     max(theta_d_deg)+5, max(theta_d_deg)+5], ...
    [1, 0.8, 0.8], ...
    'FaceAlpha', 0.3, ...
    'EdgeColor', 'none', ...
    'DisplayName', 'Disturbance interval');

grid on;
grid minor;
xlim([0, PARAM.simulation.time]);
ylabel('Joint Angle (deg)');

title(sprintf('Disturbance Rejection: %.1f N m from %.1f to %.1f s', ...
    dist.magnitude, dist.t_start, dist.t_end));

legend('Location', 'northeast');

subplot(3, 1, 2);

plot(t, pid_dst_err, 'b-', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('PID peak error = %.2f deg', ...
    metrics.pid_w_peak_err));
hold on;

plot(t, smc_dst_err, 'm-', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('SMC peak error = %.2f deg', ...
    metrics.smc_w_peak_err));

yline(0, 'k--', 'DisplayName', 'Zero error');

patch([dist.t_start, dist.t_end, dist.t_end, dist.t_start], ...
    [-50, -50, 50, 50], ...
    [1, 0.8, 0.8], ...
    'FaceAlpha', 0.20, ...
    'EdgeColor', 'none', ...
    'HandleVisibility', 'off');

grid on;
grid minor;
xlim([0, PARAM.simulation.time]);
ylim([-40, 40]);
ylabel('Tracking Error (deg)');
legend('Location', 'best');

subplot(3, 1, 3);

plot(t, pid_dst_tau, 'b-', 'LineWidth', 1.5, ...
    'DisplayName', 'PID torque');
hold on;

plot(t, smc_dst_tau, 'm-', 'LineWidth', 1.5, ...
    'DisplayName', 'SMC torque');

yline(tau_sat, 'r--', 'DisplayName', 'Torque limits');
yline(-tau_sat, 'r--', 'HandleVisibility', 'off');

grid on;
grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)');
ylabel('Torque (N m)');
title('Actuator Torque During Disturbance');
legend('Location', 'northeast');

fig11_path = fullfile(PARAM.dirs.results, ...
    'fig11_pid_vs_smc_disturbance.png');

try
    saveas(hFig11, fig11_path);
    fprintf('Saved disturbance comparison plot to: %s\n', fig11_path);
catch ME
    warning('Could not save figure 11: %s', ME.message);
end

%% 10. Figure 12: Performance metrics bar chart

hFig12 = figure( ...
    'Name', 'Performance Metrics Bar Chart', ...
    'NumberTitle', 'off', ...
    'Units', 'pixels', ...
    'Position', [160, 120, 900, 520], ...
    'Color', 'w');

categories = {'Nominal RMSE (deg)', 'Nominal MAE (deg)', ...
    'Nominal Max Error (deg)', 'Dist. Window Peak (deg)'};

pid_bars = [metrics.pid_nom_rmse, ...
            metrics.pid_nom_mae, ...
            metrics.pid_nom_max, ...
            metrics.pid_w_peak_err];

smc_bars = [metrics.smc_nom_rmse, ...
            metrics.smc_nom_mae, ...
            metrics.smc_nom_max, ...
            metrics.smc_w_peak_err];

bar_data = [pid_bars; smc_bars]';

hB = bar(bar_data, 'grouped');

hB(1).FaceColor = [0.2, 0.4, 0.8];
hB(2).FaceColor = [0.85, 0.2, 0.5];

set(gca, 'XTickLabel', categories, ...
    'FontSize', 9, ...
    'FontWeight', 'bold');

ylabel('Error Magnitude (degrees)');
title({'Tracking Performance: PID vs SMC'; ...
    'First three bars: full-horizon nominal gait. Fourth bar: disturbance-window peak only.'});

legend({'Baseline PID', 'Sliding Mode Control'}, ...
    'Location', 'northwest');

grid on;
grid minor;
hold on;

% Label each bar using its actual position.
for i = 1:numel(categories)
    x_pid = hB(1).XEndPoints(i);
    x_smc = hB(2).XEndPoints(i);

    text(x_pid, pid_bars(i), sprintf('%.2f', pid_bars(i)), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', 9);

    text(x_smc, smc_bars(i), sprintf('%.2f', smc_bars(i)), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', 9);
end

ylim([0, max([pid_bars, smc_bars]) * 1.15 + eps]);

fig12_path = fullfile(PARAM.dirs.results, ...
    'fig12_metrics_comparison_barchart.png');

try
    saveas(hFig12, fig12_path);
    fprintf('Saved metrics bar chart to: %s\n', fig12_path);
catch ME
    warning('Could not save figure 12: %s', ME.message);
end

%% 11. Package outputs

comparison.metrics = metrics;

comparison.pid_nom = struct( ...
    'theta', pid_nom_th, ...
    'error', pid_nom_err, ...
    'tau', pid_nom_tau);

comparison.smc_nom = struct( ...
    'theta', smc_nom_th, ...
    'error', smc_nom_err, ...
    'tau', smc_nom_tau, ...
    'sliding_surface', smc_nom_s);

comparison.pid_dst = struct( ...
    'theta', pid_dst_th, ...
    'error', pid_dst_err, ...
    'tau', pid_dst_tau);

comparison.smc_dst = struct( ...
    'theta', smc_dst_th, ...
    'error', smc_dst_err, ...
    'tau', smc_dst_tau, ...
    'sliding_surface', smc_dst_s);

%% 12. Print summary

fprintf('\n============================================================\n');
fprintf(' CONTROLLER COMPARISON: PID VS SMC\n');
fprintf('============================================================\n');

fprintf('NOMINAL GAIT TRACKING\n');
fprintf('PID: RMSE %.4f deg | MAE %.4f deg | Max %.4f deg | Energy %.3f\n', ...
    metrics.pid_nom_rmse, metrics.pid_nom_mae, ...
    metrics.pid_nom_max, metrics.pid_nom_eng);

fprintf('SMC: RMSE %.4f deg | MAE %.4f deg | Max %.4f deg | Energy %.3f\n', ...
    metrics.smc_nom_rmse, metrics.smc_nom_mae, ...
    metrics.smc_nom_max, metrics.smc_nom_eng);

fprintf('SMC change vs PID: RMSE reduction %.2f%%; MAE reduction %.2f%%\n', ...
    metrics.pct_rmse_reduction, metrics.pct_mae_reduction);

fprintf('\nDISTURBANCE: %.2f N m from %.2f to %.2f s\n', ...
    dist.magnitude, dist.t_start, dist.t_end);

fprintf('PID disturbance-window RMSE: %.4f deg\n', metrics.pid_w_rmse);
fprintf('SMC disturbance-window RMSE: %.4f deg\n', metrics.smc_w_rmse);

fprintf('PID disturbance peak error: %.4f deg\n', metrics.pid_w_peak_err);
fprintf('SMC disturbance peak error: %.4f deg\n', metrics.smc_w_peak_err);

if isnan(metrics.pid_rec_time)
    fprintf('PID recovery time: not observed (|e| did not stay within +/-%.1f deg for %.2f s after t=%.2f s)\n', ...
        nom_threshold, hold_duration, dist.t_end);
else
    fprintf('PID recovery time: %.4f s (threshold +/-%.1f deg, hold %.2f s)\n', ...
        metrics.pid_rec_time, nom_threshold, hold_duration);
end
if isnan(metrics.smc_rec_time)
    fprintf('SMC recovery time: not observed (|e| did not stay within +/-%.1f deg for %.2f s after t=%.2f s)\n', ...
        nom_threshold, hold_duration, dist.t_end);
else
    fprintf('SMC recovery time: %.4f s (threshold +/-%.1f deg, hold %.2f s)\n', ...
        metrics.smc_rec_time, nom_threshold, hold_duration);
end

fprintf('Peak disturbance error reduction: %.2f%%\n', ...
    metrics.pct_dist_err_reduction);

fprintf('============================================================\n');

end


function PARAM = load_project_parameters(project_root)
% Loads the parameter script in its own local workspace.
% This avoids running the script directly in the main function workspace.

param_file = fullfile(project_root, 'scripts', 'project_parameters.m');

if ~exist(param_file, 'file')
    error('compare_controllers:MissingParameters', ...
        'Cannot find parameter file: %s', param_file);
end

SUPPRESS_PARAM_DISPLAY = true; %#ok<NASGU>
run(param_file);

if ~exist('PARAM', 'var')
    error('compare_controllers:MissingPARAM', ...
        'project_parameters.m did not create PARAM.');
end

end


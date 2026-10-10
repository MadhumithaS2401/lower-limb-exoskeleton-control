function aan_result = design_aan(PARAM, gait, dist)
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 7 - Assist-as-Needed (AAN) Adaptive Control Strategy
%
% Purpose:
% Implements a bounded, smooth Assist-as-Needed (AAN) rehabilitation control
% strategy. The exoskeleton scales its intervention torque dynamically based on
% the patient's kinematic tracking error.
%
% Clinical & Control Rationale:
%   - Promotes neuroplasticity and active patient effort during motor recovery.
%   - Low Tracking Error (|e| <= e_low)  --> Low Assistance (alpha = alpha_min)
%   - High Tracking Error (|e| >= e_high) --> Full Assistance (alpha = alpha_max)
%   - Disturbance / Spasm               --> Assistance surges adaptively to alpha_max
%
% Mathematical Assistance Formulation:
%   e_smooth(t): First-order low-pass filtered absolute error (deg)
%   alpha_target(t) = piecewise_linear(e_smooth, [e_low, e_high], [alpha_min, alpha_max])
%   tau_applied(t) = alpha(t) * tau_controller(t)
%
% Usage:
%   aan_result = design_aan();
%   aan_result = design_aan(PARAM, gait, dist);

%% 1. Load Parameters if Not Provided
if nargin < 1 || isempty(PARAM)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    SUPPRESS_PARAM_DISPLAY = true;
    run(fullfile(project_root, 'scripts', 'project_parameters.m'));
end

if nargin < 2 || isempty(gait)
    addpath(PARAM.dirs.trajectories);
    gait = generate_gait(PARAM);
end

if nargin < 3 || isempty(dist)
    addpath(PARAM.dirs.models);
    dist = disturbance_model(PARAM);
end

%% 2. System and Algorithm Parameters
J = PARAM.joint.inertia;
B = PARAM.joint.damping;
K = PARAM.joint.stiffness;
tau_sat = PARAM.actuator.max_torque;

t = PARAM.simulation.t;
dt = PARAM.simulation.Ts;
N_steps = length(t);

theta_d_rad = gait.theta_rad;
theta_dot_d_rad = gait.theta_dot_rad;
theta_d_deg = gait.theta_deg;
tau_dist = dist.tau_dist;

alpha_min = PARAM.aan.alpha_min;       % 0.20
alpha_max = PARAM.aan.alpha_max;       % 1.00
e_low_deg = PARAM.aan.error_low_deg;   % 2.0 deg
e_high_deg = PARAM.aan.error_high_deg; % 10.0 deg
tau_f = PARAM.aan.filter_tau;          % 0.20 s

Kp = PARAM.pid.Kp;
Ki = PARAM.pid.Ki;
Kd = PARAM.pid.Kd;
N_filter = PARAM.pid.N;

%% 3. Simulation A: Assist-as-Needed (AAN) Dynamic Simulation
theta_aan = zeros(N_steps, 1);
theta_dot_aan = zeros(N_steps, 1);
tau_ctrl_aan = zeros(N_steps, 1);
tau_applied_aan = zeros(N_steps, 1);
alpha_hist = zeros(N_steps, 1);
e_smooth_hist = zeros(N_steps, 1);
error_rad_aan = zeros(N_steps, 1);

e_int_aan = 0.0;
de_filt_aan = 0.0;
e_smooth = 0.0;
alpha_curr = alpha_min;

for k = 1:N_steps-1
    % Tracking error
    e = theta_d_rad(k) - theta_aan(k);
    error_rad_aan(k) = e;
    e_deg_abs = abs(rad2deg(e));
    
    % Low-pass filter for error smoothing: de_smooth/dt = (e_deg_abs - e_smooth) / tau_f
    e_smooth = e_smooth + (dt / tau_f) * (e_deg_abs - e_smooth);
    e_smooth_hist(k) = e_smooth;
    
    % Piecewise linear target assistance mapping
    if e_smooth <= e_low_deg
        alpha_target = alpha_min;
    elseif e_smooth >= e_high_deg
        alpha_target = alpha_max;
    else
        alpha_target = alpha_min + (alpha_max - alpha_min) * ...
            ((e_smooth - e_low_deg) / (e_high_deg - e_low_deg));
    end
    
    % Rate limit / smooth alpha transitions (first-order lag with tau = 0.05 s)
    alpha_curr = alpha_curr + (dt / 0.05) * (alpha_target - alpha_curr);
    alpha_curr = max(alpha_min, min(alpha_max, alpha_curr));
    alpha_hist(k) = alpha_curr;
    
    % PID Controller computation with error derivative
    u_p = Kp * e;
    e_int_aan = e_int_aan + e * dt;
    u_i = Ki * e_int_aan;
    e_dot_raw = theta_dot_d_rad(k) - theta_dot_aan(k);
    de_filt_aan = de_filt_aan + (N_filter * (e_dot_raw - de_filt_aan)) * dt;
    u_d = Kd * de_filt_aan;
    
    u_raw = u_p + u_i + u_d;
    tau_ctrl_aan(k) = u_raw;
    
    % Modulate control torque with adaptive assistance level
    u_aan = alpha_curr * u_raw;
    
    % Apply saturation limit
    u_applied = max(-tau_sat, min(tau_sat, u_aan));
    tau_applied_aan(k) = u_applied;
    
    % Plant dynamics with applied torque AND external disturbance
    theta_ddot = (1 / J) * (u_applied + tau_dist(k) - B * theta_dot_aan(k) - K * theta_aan(k));
    
    % Numerical integration
    theta_dot_aan(k+1) = theta_dot_aan(k) + theta_ddot * dt;
    theta_aan(k+1)     = theta_aan(k) + theta_dot_aan(k+1) * dt;
end

error_rad_aan(end) = theta_d_rad(end) - theta_aan(end);
alpha_hist(end) = alpha_hist(end-1);
tau_applied_aan(end) = tau_applied_aan(end-1);
e_smooth_hist(end) = e_smooth_hist(end-1);

theta_deg_aan = rad2deg(theta_aan);
error_deg_aan = rad2deg(error_rad_aan);

%% 4. Simulation B: Fixed Baseline Control (Full 100% Assistance, alpha = 1.0)
theta_full = zeros(N_steps, 1);
theta_dot_full = zeros(N_steps, 1);
tau_applied_full = zeros(N_steps, 1);
error_rad_full = zeros(N_steps, 1);

e_int_full = 0.0;
de_filt_full = 0.0;

for k = 1:N_steps-1
    e = theta_d_rad(k) - theta_full(k);
    error_rad_full(k) = e;
    
    u_p = Kp * e;
    e_int_full = e_int_full + e * dt;
    u_i = Ki * e_int_full;
    e_dot_raw = theta_dot_d_rad(k) - theta_dot_full(k);
    de_filt_full = de_filt_full + (N_filter * (e_dot_raw - de_filt_full)) * dt;
    u_d = Kd * de_filt_full;
    
    u_raw = u_p + u_i + u_d;
    u_applied = max(-tau_sat, min(tau_sat, u_raw));
    tau_applied_full(k) = u_applied;
    
    theta_ddot = (1 / J) * (u_applied + tau_dist(k) - B * theta_dot_full(k) - K * theta_full(k));
    
    theta_dot_full(k+1) = theta_dot_full(k) + theta_ddot * dt;
    theta_full(k+1)     = theta_full(k) + theta_dot_full(k+1) * dt;
end

error_rad_full(end) = theta_d_rad(end) - theta_full(end);
tau_applied_full(end) = tau_applied_full(end-1);
theta_deg_full = rad2deg(theta_full);
error_deg_full = rad2deg(error_rad_full);

%% 5. Metric Calculations
rmse_aan_deg = sqrt(mean(error_deg_aan.^2));
mae_aan_deg  = mean(abs(error_deg_aan));
max_error_aan_deg = max(abs(error_deg_aan));
energy_aan = trapz(t, tau_applied_aan.^2);
mean_alpha = mean(alpha_hist);
max_alpha  = max(alpha_hist);

rmse_full_deg = sqrt(mean(error_deg_full.^2));
mae_full_deg  = mean(abs(error_deg_full));
max_error_full_deg = max(abs(error_deg_full));
energy_full = trapz(t, tau_applied_full.^2);

% Safe division for energy change: positive means AAN used less energy.
if energy_full > 0
    energy_savings_pct = (1 - energy_aan / energy_full) * 100;
else
    energy_savings_pct = NaN;
end

alpha_in_bounds = all(alpha_hist >= alpha_min - 1e-12) && all(alpha_hist <= alpha_max + 1e-12);
if ~alpha_in_bounds
    warning('AAN assistance factor left the configured [alpha_min, alpha_max] interval.');
end

d_idx = (t >= dist.t_start) & (t <= dist.t_end);
if any(d_idx)
    max_alpha_dist = max(alpha_hist(d_idx));
    max_error_aan_dist_deg = max(abs(error_deg_aan(d_idx)));
    max_error_full_dist_deg = max(abs(error_deg_full(d_idx)));
else
    max_alpha_dist = NaN;
    max_error_aan_dist_deg = NaN;
    max_error_full_dist_deg = NaN;
end
reached_alpha_max_during_dist = isfinite(max_alpha_dist) && (max_alpha_dist >= alpha_max - 1e-6);

sat_count_aan = sum(abs(tau_applied_aan) >= tau_sat - 1e-9);
sat_count_full = sum(abs(tau_applied_full) >= tau_sat - 1e-9);

%% 6. Visualization - Assist-as-Needed Adaptation
hFigAAN = figure('Name', 'Assist-as-Needed (AAN) Adaptive Control', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [140, 50, 950, 800], 'Color', 'w');

% Subplot 1: Gait Trajectories
subplot(4, 1, 1);
plot(t, theta_d_deg, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired \theta_d'); hold on;
plot(t, theta_deg_full, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Fixed Full Control (\alpha = 1.0)');
plot(t, theta_deg_aan, 'r-', 'LineWidth', 1.8, 'DisplayName', sprintf('Adaptive AAN Control (\\alpha \\in [%.2f, %.2f])', alpha_min, alpha_max));
xline(PARAM.disturbance.start_time, 'm:', 'LineWidth', 1.2, 'DisplayName', sprintf('Disturbance Start (%.1fs)', PARAM.disturbance.start_time));
xline(PARAM.disturbance.end_time, 'm:', 'LineWidth', 1.2, 'DisplayName', sprintf('Disturbance End (%.1fs)', PARAM.disturbance.end_time));
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylabel('Angle (deg)', 'FontSize', 10, 'FontWeight', 'bold');
title('Assist-as-Needed (AAN) Dynamic Rehabilitation Control Evaluation', 'FontSize', 12, 'FontWeight', 'bold');
legend('Desired', 'Full 100% Control', 'Adaptive AAN', 'Location', 'northeast');

% Subplot 2: Tracking Errors
subplot(4, 1, 2);
plot(t, error_deg_full, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Full Control Error'); hold on;
plot(t, error_deg_aan, 'r-', 'LineWidth', 1.8, 'DisplayName', 'AAN Error');
yline(0, 'k--');
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylabel('Error (deg)', 'FontSize', 10, 'FontWeight', 'bold');
legend('Location', 'best');

% Subplot 3: Assistance Factor Adaptation
subplot(4, 1, 3);
plot(t, alpha_hist, 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2.0, 'DisplayName', 'Assistance Factor \alpha(t)'); hold on;
yline(alpha_min, 'k--', sprintf('\\alpha_{min} = %.2f', alpha_min), 'DisplayName', '\alpha_{min}');
yline(alpha_max, 'g--', sprintf('\\alpha_{max} = %.2f', alpha_max), 'DisplayName', '\alpha_{max}');
patch([dist.t_start, dist.t_end, dist.t_end, dist.t_start], ...
      [0, 0, 1.1, 1.1], [1, 0.8, 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
      'DisplayName', sprintf('Disturbance Window (%.1fs-%.1fs)', dist.t_start, dist.t_end));
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylim([0.1, 1.15]);
ylabel('Assistance \alpha', 'FontSize', 10, 'FontWeight', 'bold');
if reached_alpha_max_during_dist
    alpha_title = 'Assistance Factor \alpha(t) (reached \alpha_{max} in the disturbance window)';
else
    alpha_title = sprintf('Assistance Factor \\alpha(t) (disturbance-window peak = %.2f)', max_alpha_dist);
end
title(alpha_title, 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast');

% Subplot 4: Actuator Torque Demand
subplot(4, 1, 4);
plot(t, tau_applied_full, 'b-', 'LineWidth', 1.5, 'DisplayName', '\tau Full (Fixed)'); hold on;
plot(t, tau_applied_aan, 'r-', 'LineWidth', 1.8, 'DisplayName', '\tau AAN (Modulated)');
yline(tau_sat, 'k:'); yline(-tau_sat, 'k:');
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Torque (N\cdot m)', 'FontSize', 10, 'FontWeight', 'bold');
if ~isnan(energy_savings_pct)
    if energy_savings_pct >= 0
        energy_title = sprintf('Control Torque | AAN used %.1f%% less energy than full assistance', energy_savings_pct);
    else
        energy_title = sprintf('Control Torque | AAN used %.1f%% more energy than full assistance', -energy_savings_pct);
    end
    title(energy_title, 'FontSize', 11, 'FontWeight', 'bold');
else
    title('Control Torque Modulation', 'FontSize', 11, 'FontWeight', 'bold');
end
legend('Location', 'northeast');

fig_path = fullfile(PARAM.dirs.results, 'fig09_aan_assistance_adaptation.png');
try
    saveas(hFigAAN, fig_path);
    fprintf('Saved AAN adaptation plot to: %s\n', fig_path);
catch ME
    warning('Could not save fig 9: %s', ME.message);
end

%% 7. Package Output Struct
aan_result.t = t;
aan_result.theta_d_deg = theta_d_deg;
aan_result.theta_deg_aan = theta_deg_aan;
aan_result.theta_deg_full = theta_deg_full;
aan_result.error_deg_aan = error_deg_aan;
aan_result.error_deg_full = error_deg_full;
aan_result.alpha_hist = alpha_hist;
aan_result.e_smooth_hist = e_smooth_hist;
aan_result.tau_applied_aan = tau_applied_aan;
aan_result.tau_applied_full = tau_applied_full;
aan_result.rmse_aan_deg = rmse_aan_deg;
aan_result.mae_aan_deg = mae_aan_deg;
aan_result.max_error_aan_deg = max_error_aan_deg;
aan_result.energy_aan = energy_aan;
aan_result.mean_alpha = mean_alpha;
aan_result.max_alpha = max_alpha;
aan_result.rmse_full_deg = rmse_full_deg;
aan_result.mae_full_deg = mae_full_deg;
aan_result.max_error_full_deg = max_error_full_deg;
aan_result.energy_full = energy_full;
aan_result.energy_savings_pct = energy_savings_pct;
aan_result.alpha_in_bounds = alpha_in_bounds;
aan_result.max_alpha_dist = max_alpha_dist;
aan_result.reached_alpha_max_during_dist = reached_alpha_max_during_dist;
aan_result.max_error_aan_dist_deg = max_error_aan_dist_deg;
aan_result.max_error_full_dist_deg = max_error_full_dist_deg;
aan_result.sat_count_aan = sat_count_aan;
aan_result.sat_count_full = sat_count_full;

%% 8. Console Summary
fprintf('\n============================================================\n');
fprintf(' ASSIST-AS-NEEDED (AAN) ADAPTIVE STRATEGY SIMULATION COMPLETE\n');
fprintf('============================================================\n');
fprintf(' AAN Parameters: alpha in [%.2f, %.2f] | Error Band: [%.1f, %.1f] deg\n', ...
    alpha_min, alpha_max, e_low_deg, e_high_deg);
fprintf(' Performance Comparison (with Disturbance at %.1fs-%.1fs):\n', dist.t_start, dist.t_end);
fprintf('   Fixed 100%% Control : RMSE = %.2f deg | Energy = %.2f N^2*m^2*s\n', ...
    rmse_full_deg, energy_full);
fprintf('   Adaptive AAN Control: RMSE = %.2f deg | Energy = %.2f N^2*m^2*s\n', ...
    rmse_aan_deg, energy_aan);
fprintf('   Mean Assistance     : %.2f (%.1f%% of full intervention)\n', mean_alpha, mean_alpha*100);
fprintf('   Peak Assistance (all t): %.2f\n', max_alpha);
if isfinite(max_alpha_dist)
    fprintf('   Peak Assistance in disturbance window: %.2f (alpha_max reached: %s)\n', ...
        max_alpha_dist, mat2str(reached_alpha_max_during_dist));
    fprintf('   Peak |error| in disturbance window: AAN %.2f deg | Full %.2f deg\n', ...
        max_error_aan_dist_deg, max_error_full_dist_deg);
end
if ~isnan(energy_savings_pct)
    if energy_savings_pct >= 0
        fprintf('   Actuator energy: AAN used %.1f%% less than full assistance\n', energy_savings_pct);
    else
        fprintf('   Actuator energy: AAN used %.1f%% more than full assistance\n', -energy_savings_pct);
    end
end
fprintf('   Torque saturation samples: AAN %d | Full %d\n', sat_count_aan, sat_count_full);
fprintf('============================================================\n');

if nargout == 0
    clear aan_result;
end
end

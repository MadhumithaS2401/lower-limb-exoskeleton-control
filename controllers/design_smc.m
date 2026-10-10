function smc_result = design_smc(PARAM, gait, dist)
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 5 - Advanced Nonlinear Control: Sliding Mode Control (SMC)
%
% Purpose:
% Implements a robust Lyapunov-based Sliding Mode Controller for tracking
% dynamic gait trajectories in the presence of parametric uncertainty and
% external disturbances. A continuous boundary layer (hyperbolic tangent)
% is implemented to eliminate high-frequency chattering.
%
% Theoretical Formulation:
%   Plant: J * theta_ddot + B * theta_dot + K * theta = tau + tau_dist
%   Tracking Error: e(t) = theta_d(t) - theta(t)
%   Sliding Surface: s(t) = e_dot(t) + lambda * e(t),  (lambda > 0)
%
% Equivalent Control (tau_eq):
%   Setting s_dot = 0 under nominal conditions (tau_dist = 0):
%   tau_eq = J * (theta_ddot_d + lambda * e_dot) + B * theta_dot + K * theta
%
% Robust Reaching Control (tau_sw):
%   To guarantee Lyapunov stability V = 0.5 * J * s^2, V_dot <= -eta * |s|:
%   tau_sw = K_sw * tanh(s / phi)   [Boundary layer phi eliminates chattering]
%
% Total Control Torque:
%   tau(t) = clip(tau_eq + tau_sw, -tau_sat, tau_sat)
%
% Usage:
%   smc_result = design_smc();                  % Uses default PARAM & gait
%   smc_result = design_smc(PARAM, gait);       % Uses supplied structs without disturbance
%   smc_result = design_smc(PARAM, gait, dist); % Uses supplied structs with disturbance

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

if nargin >= 3 && ~isempty(dist)
    tau_dist = dist.tau_dist;
else
    tau_dist = zeros(size(PARAM.simulation.t));
end

%% 2. Extract Plant and SMC Parameters
J = PARAM.joint.inertia;
B = PARAM.joint.damping;
K = PARAM.joint.stiffness;
tau_sat = PARAM.actuator.max_torque;

lambda = PARAM.smc.lambda;    % Sliding surface slope (s^-1)
K_sw   = PARAM.smc.K_sw;      % Switching gain (N*m)
phi    = PARAM.smc.phi;       % Boundary layer thickness (rad/s)

%% 3. Simulation Time Base and Reference Trajectories
t = PARAM.simulation.t;
dt = PARAM.simulation.Ts;
N_steps = length(t);

theta_d_rad      = gait.theta_rad;
theta_dot_d_rad  = gait.theta_dot_rad;
theta_ddot_d_rad = gait.theta_ddot_rad;
theta_d_deg      = gait.theta_deg;

%% 4. Preallocate Arrays
theta      = zeros(N_steps, 1);
theta_dot  = zeros(N_steps, 1);
tau        = zeros(N_steps, 1);
tau_eq     = zeros(N_steps, 1);
tau_sw     = zeros(N_steps, 1);
s_surface  = zeros(N_steps, 1);
error_rad  = zeros(N_steps, 1);
error_dot_rad = zeros(N_steps, 1);

%% 5. Numerical Simulation of Closed-Loop SMC
for k = 1:N_steps-1
    % Error states
    e = theta_d_rad(k) - theta(k);
    e_dot = theta_dot_d_rad(k) - theta_dot(k);
    
    error_rad(k) = e;
    error_dot_rad(k) = e_dot;
    
    % Sliding variable: s = e_dot + lambda * e
    s = e_dot + lambda * e;
    s_surface(k) = s;
    
    % Equivalent control torque (cancels nominal dynamics + drives along manifold)
    u_eq = J * (theta_ddot_d_rad(k) + lambda * e_dot) + B * theta_dot(k) + K * theta(k);
    
    % Robust reaching torque with continuous boundary layer to prevent chattering
    if isfield(PARAM.smc, 'use_tanh') && ~PARAM.smc.use_tanh
        u_sw = K_sw * max(-1, min(1, s / phi));
    else
        u_sw = K_sw * tanh(s / phi);
    end
    
    tau_eq(k) = u_eq;
    tau_sw(k) = u_sw;
    
    % Total commanded torque with physical saturation
    u_total = u_eq + u_sw;
    u_applied = max(-tau_sat, min(tau_sat, u_total));
    tau(k) = u_applied;
    
    % Plant acceleration: theta_ddot = (1/J) * (tau + tau_dist - B*theta_dot - K*theta)
    theta_ddot = (1 / J) * (u_applied + tau_dist(k) - B * theta_dot(k) - K * theta(k));
    
    % Semi-implicit Euler integration
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1)     = theta(k) + theta_dot(k+1) * dt;
end

% Final step states
error_rad(end) = theta_d_rad(end) - theta(end);
error_dot_rad(end) = theta_dot_d_rad(end) - theta_dot(end);
s_surface(end) = error_dot_rad(end) + lambda * error_rad(end);
tau(end) = tau(end-1);
tau_eq(end) = tau_eq(end-1);
tau_sw(end) = tau_sw(end-1);

theta_deg = rad2deg(theta);
error_deg = rad2deg(error_rad);
error_dot_deg = rad2deg(error_dot_rad);

%% 6. Calculate Performance Metrics
rmse_deg = sqrt(mean(error_deg.^2));
rmse_rad = sqrt(mean(error_rad.^2));
mae_deg  = mean(abs(error_deg));
max_error_deg = max(abs(error_deg));
control_energy = trapz(t, tau.^2);

%% 7. Visualization - Gait Tracking & Controller Components
hFigSMC = figure('Name', 'SMC Gait Tracking Performance', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [170, 70, 950, 780], 'Color', 'w');

% Subplot 1: Desired vs Actual
subplot(3, 1, 1);
plot(t, theta_d_deg, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired \theta_d'); hold on;
plot(t, theta_deg, 'm-', 'LineWidth', 1.8, 'DisplayName', 'Actual SMC \theta');
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylim([min(theta_d_deg)-5, max(theta_d_deg)+5]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Joint Angle (deg)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Sliding Mode Control Gait Tracking | RMSE = %.4f^\\circ, MAE = %.4f^\\circ', ...
    rmse_deg, mae_deg), 'FontSize', 12, 'FontWeight', 'bold');
legend('Desired Angle', 'Actual SMC Angle', 'Location', 'northeast');

% Subplot 2: Tracking Error
subplot(3, 1, 2);
plot(t, error_deg, 'r-', 'LineWidth', 1.8); hold on;
yline(0, 'k--', 'LineWidth', 1.0);
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Tracking Error e(t) (deg)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('SMC Tracking Error | Max Abs Error = %.4f^\\circ', max_error_deg), ...
    'FontSize', 11, 'FontWeight', 'bold');

% Subplot 3: Control Torque Partition (Equivalent vs Switching)
subplot(3, 1, 3);
plot(t, tau, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Total \tau'); hold on;
plot(t, tau_eq, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Equivalent \tau_{eq}');
plot(t, tau_sw, 'g:', 'LineWidth', 1.2, 'DisplayName', 'Switching \tau_{sw}');
yline(tau_sat, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('Limit (\\pm%.0f N\\cdot m)', tau_sat));
yline(-tau_sat, 'r--', 'LineWidth', 1.0);
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Torque (N\cdot m)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('SMC Torque Decomposition | Control Energy = %.2f N^2\\cdot m^2\\cdot s', control_energy), ...
    'FontSize', 11, 'FontWeight', 'bold');
legend('Total \tau', '\tau_{eq}', '\tau_{sw}', 'Saturation', 'Location', 'northeast');

fig_smc_path = fullfile(PARAM.dirs.results, 'fig06_smc_gait_tracking.png');
try
    saveas(hFigSMC, fig_smc_path);
    fprintf('Saved SMC gait tracking plot to: %s\n', fig_smc_path);
catch ME
    warning('Could not save fig 6: %s', ME.message);
end

%% 8. Visualization - Phase Plane & Sliding Surface
hFigPhase = figure('Name', 'SMC Phase Portrait & Sliding Surface', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [220, 100, 850, 600], 'Color', 'w');

plot(error_deg, error_dot_deg, 'b-', 'LineWidth', 1.8, 'DisplayName', 'State Trajectory (e, de/dt)'); hold on;
e_range = linspace(min(error_deg)-2, max(error_deg)+2, 100);
e_dot_surface = -lambda * e_range;
plot(e_range, e_dot_surface, 'r--', 'LineWidth', 2.0, 'DisplayName', sprintf('Sliding Line s=0 (\\lambda = %.1f)', lambda));
plot(error_deg(1), error_dot_deg(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Initial State');
plot(error_deg(end), error_dot_deg(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Final State');
grid on; grid minor;
xlabel('Tracking Error e (deg)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Error Derivative de/dt (deg/s)', 'FontSize', 11, 'FontWeight', 'bold');
title('Phase Plane Trajectory & Hurwitz Sliding Surface', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

fig_phase_path = fullfile(PARAM.dirs.results, 'fig07_smc_phase_portrait.png');
try
    saveas(hFigPhase, fig_phase_path);
    fprintf('Saved SMC phase portrait plot to: %s\n', fig_phase_path);
catch ME
    warning('Could not save fig 7: %s', ME.message);
end

%% 9. Output Struct
smc_result.lambda = lambda;
smc_result.K_sw = K_sw;
smc_result.phi = phi;
smc_result.t = t;
smc_result.theta_d_rad = theta_d_rad;
smc_result.theta_d_deg = theta_d_deg;
smc_result.theta_rad = theta;
smc_result.theta_deg = theta_deg;
smc_result.theta_dot = theta_dot;
smc_result.error_rad = error_rad;
smc_result.error_deg = error_deg;
smc_result.error_dot_rad = error_dot_rad;
smc_result.error_dot_deg = error_dot_deg;
smc_result.s_surface = s_surface;
smc_result.tau = tau;
smc_result.tau_eq = tau_eq;
smc_result.tau_sw = tau_sw;
smc_result.rmse_deg = rmse_deg;
smc_result.rmse_rad = rmse_rad;
smc_result.mae_deg = mae_deg;
smc_result.max_error_deg = max_error_deg;
smc_result.control_energy = control_energy;

%% 10. Console Output
fprintf('\n============================================================\n');
fprintf(' SLIDING MODE CONTROLLER (SMC) SIMULATION COMPLETE\n');
fprintf('============================================================\n');
fprintf(' SMC Hyperparameters: lambda = %.2f s^-1, K_sw = %.2f N*m, phi = %.4f rad/s\n', ...
    lambda, K_sw, phi);
fprintf(' Tracking Performance Metrics:\n');
fprintf('   RMSE                : %.4f deg (%.6f rad)\n', rmse_deg, rmse_rad);
fprintf('   Mean Absolute Error : %.4f deg\n', mae_deg);
fprintf('   Max Absolute Error  : %.4f deg\n', max_error_deg);
fprintf('   Control Energy      : %.2f N^2*m^2*s\n', control_energy);
fprintf('   Peak Torque Demanded: %.2f N*m (Limit = +/- %.0f N*m)\n', max(abs(tau)), tau_sat);
fprintf('============================================================\n');

if nargout == 0
    clear smc_result;
end
end

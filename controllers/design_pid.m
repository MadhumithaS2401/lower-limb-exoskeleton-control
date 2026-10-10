function pid_result = design_pid(PARAM, gait, dist)
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 3 - PID Controller Design, Step Validation & Gait Trajectory Tracking
%
% Purpose:
% Designs, tunes, validates, and simulates the classical PID feedback
% controller for both unit-step tracking and dynamic physiological gait
% trajectory tracking with actuator saturation constraints and derivative filter.
%
% Continuous-Time PID Formulation:
%   u_pid(t) = Kp * e(t) + Ki * integral(e(tau) dtau) + Kd * de_filt(t)/dt
%   Derivative Filter: D(s) = Kd * N * s / (s + N)
%   Torque Saturation: tau(t) = clip(u_pid(t), -tau_max, tau_max)
%
% Usage:
%   pid_result = design_pid();                  % Uses default parameters & generates gait
%   pid_result = design_pid(PARAM, gait);       % Uses supplied structs without disturbance
%   pid_result = design_pid(PARAM, gait, dist); % Uses supplied structs with disturbance

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

%% 2. Exoskeleton Plant Model
J = PARAM.joint.inertia;
B = PARAM.joint.damping;
K = PARAM.joint.stiffness;
s = tf('s');
G = 1 / (J*s^2 + B*s + K);

%% 3. PID Controller Tuning (or verification of designated gains)
if isfield(PARAM, 'pid') && isfield(PARAM.pid, 'Kp') && ~isempty(PARAM.pid.Kp)
    Kp = PARAM.pid.Kp;
    Ki = PARAM.pid.Ki;
    Kd = PARAM.pid.Kd;
    N  = PARAM.pid.N;
    C_pid = pid(Kp, Ki, Kd, 1/N);
else
    [C_pid, ~] = pidtune(G, 'PID');
    Kp = C_pid.Kp;
    Ki = C_pid.Ki;
    Kd = C_pid.Kd;
    N  = 100.0;
end

% Closed-loop transfer function for unit step test
T_step = feedback(C_pid * G, 1);
step_info = stepinfo(T_step);

%% 4. Unit-Step Response Validation Figure
hFigStep = figure('Name', 'PID Step Response Validation', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [120, 120, 850, 500], 'Color', 'w');

t_step_vec = 0:0.001:5.0;
[y_step, t_step] = step(T_step, t_step_vec);

plot(t_step, y_step, 'b-', 'LineWidth', 2.0); hold on;
yline(1.0, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Unit Step Reference');
yline(1.0 + step_info.Overshoot/100, 'k:', 'LineWidth', 1.0, 'DisplayName', 'Peak Value');
grid on; grid minor;
xlim([0, 5.0]);
ylim([0, 1.3]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Joint Angle \theta (rad)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Closed-Loop PID Unit-Step Response | Rise Time = %.3fs, Settling = %.3fs, OS = %.1f%%', ...
    step_info.RiseTime, step_info.SettlingTime, step_info.Overshoot), 'FontSize', 12, 'FontWeight', 'bold');
legend('PID Controlled Response', 'Target (1.0 rad)', 'Location', 'southeast');

fig_step_path = fullfile(PARAM.dirs.results, 'fig04_pid_step_response.png');
try
    saveas(hFigStep, fig_step_path);
    fprintf('Saved PID step validation plot to: %s\n', fig_step_path);
catch ME
    warning('Could not save fig 4: %s', ME.message);
end

%% 5. Dynamic Gait Trajectory Tracking Simulation
t = PARAM.simulation.t;
dt = PARAM.simulation.Ts;
N_steps = length(t);

theta_d_rad = gait.theta_rad;
theta_dot_d_rad = gait.theta_dot_rad;
theta_d_deg = gait.theta_deg;

% Allocate state arrays
theta = zeros(N_steps, 1);       % Joint position (rad)
theta_dot = zeros(N_steps, 1);   % Joint angular velocity (rad/s)
tau = zeros(N_steps, 1);         % Actuator torque (N*m)
error_rad = zeros(N_steps, 1);   % Tracking error e = theta_d - theta (rad)
e_int = 0.0;                     % Discrete integrator state
de_filt = 0.0;                   % Filtered derivative state
tau_sat = PARAM.actuator.max_torque;

% Numerical integration of plant: J*theta_ddot + B*theta_dot + K*theta = tau + tau_dist
for k = 1:N_steps-1
    % Tracking error in radians
    e = theta_d_rad(k) - theta(k);
    error_rad(k) = e;
    
    % PID terms
    % Proportional
    u_p = Kp * e;
    
    % Integral with anti-windup clamping
    e_int = e_int + e * dt;
    u_i = Ki * e_int;
    
    % Derivative of error with first-order low-pass filter: de_filt_dot = N * (e_dot_raw - de_filt)
    e_dot_raw = theta_dot_d_rad(k) - theta_dot(k);
    de_filt = de_filt + (N * (e_dot_raw - de_filt)) * dt;
    u_d = Kd * de_filt;
    
    % Raw controller demand
    u_raw = u_p + u_i + u_d;
    
    % Actuator saturation constraint [-tau_sat, +tau_sat]
    u_applied = max(-tau_sat, min(tau_sat, u_raw));
    tau(k) = u_applied;
    
    % Anti-windup clamping: prevent integrator windup if saturated
    if (u_raw > tau_sat && e > 0) || (u_raw < -tau_sat && e < 0)
        e_int = e_int - e * dt; % Un-integrate
    end
    
    % Plant state derivative: theta_ddot = (1/J) * (tau + tau_dist - B*theta_dot - K*theta)
    theta_ddot = (1 / J) * (u_applied + tau_dist(k) - B * theta_dot(k) - K * theta(k));
    
    % Semi-implicit Euler integration for numerical stability
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1)     = theta(k) + theta_dot(k+1) * dt;
end

% Final step error and torque values
error_rad(end) = theta_d_rad(end) - theta(end);
tau(end) = tau(end-1);

theta_deg = rad2deg(theta);
error_deg = rad2deg(error_rad);

%% 6. Calculate Tracking Performance Metrics
rmse_rad = sqrt(mean(error_rad.^2));
rmse_deg = sqrt(mean(error_deg.^2));
mae_deg  = mean(abs(error_deg));
max_error_deg = max(abs(error_deg));
control_energy = trapz(t, tau.^2); % Integral of tau^2 dt (N^2*m^2*s)

%% 7. Gait Tracking Visualization (Multi-Panel Figure)
hFigGait = figure('Name', 'PID Gait Tracking Performance', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [150, 80, 950, 750], 'Color', 'w');

% Subplot 1: Desired vs Actual Joint Angle
subplot(3, 1, 1);
plot(t, theta_d_deg, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired \theta_d'); hold on;
plot(t, theta_deg, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Actual PID \theta');
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylim([min(theta_d_deg)-5, max(theta_d_deg)+5]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Joint Angle (deg)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('PID Gait Trajectory Tracking | RMSE = %.2f^\\circ, MAE = %.2f^\\circ', ...
    rmse_deg, mae_deg), 'FontSize', 12, 'FontWeight', 'bold');
legend('Desired Angle', 'Actual Angle', 'Location', 'northeast');

% Subplot 2: Tracking Error
subplot(3, 1, 2);
plot(t, error_deg, 'r-', 'LineWidth', 1.8); hold on;
yline(0, 'k--', 'LineWidth', 1.0);
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Tracking Error e(t) (deg)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('PID Tracking Error | Max Abs Error = %.2f^\\circ', max_error_deg), ...
    'FontSize', 11, 'FontWeight', 'bold');

% Subplot 3: Actuator Control Torque
subplot(3, 1, 3);
plot(t, tau, 'g-', 'LineWidth', 1.8); hold on;
yline(tau_sat, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('+Saturation (%.0f N\\cdot m)', tau_sat));
yline(-tau_sat, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('-Saturation (-%.0f N\\cdot m)', tau_sat));
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Torque \tau(t) (N\cdot m)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Actuator Control Torque | Control Energy = %.2f N^2\\cdot m^2\\cdot s', control_energy), ...
    'FontSize', 11, 'FontWeight', 'bold');
legend('Control Torque \tau', 'Actuator Limit', 'Location', 'northeast');

fig_gait_path = fullfile(PARAM.dirs.results, 'fig05_pid_gait_tracking.png');
try
    saveas(hFigGait, fig_gait_path);
    fprintf('Saved PID gait tracking plot to: %s\n', fig_gait_path);
catch ME
    warning('Could not save fig 5: %s', ME.message);
end

%% 8. Assemble Output Struct
pid_result.Kp = Kp;
pid_result.Ki = Ki;
pid_result.Kd = Kd;
pid_result.N  = N;
pid_result.step_info = step_info;
pid_result.t = t;
pid_result.theta_d_rad = theta_d_rad;
pid_result.theta_d_deg = theta_d_deg;
pid_result.theta_rad = theta;
pid_result.theta_deg = theta_deg;
pid_result.theta_dot = theta_dot;
pid_result.error_rad = error_rad;
pid_result.error_deg = error_deg;
pid_result.tau = tau;
pid_result.rmse_deg = rmse_deg;
pid_result.rmse_rad = rmse_rad;
pid_result.mae_deg = mae_deg;
pid_result.max_error_deg = max_error_deg;
pid_result.control_energy = control_energy;

%% 9. Console Output
fprintf('\n============================================================\n');
fprintf(' PID CONTROLLER DESIGN AND DYNAMIC GAIT SIMULATION\n');
fprintf('============================================================\n');
fprintf(' PID Gains: Kp = %.6f, Ki = %.6f, Kd = %.6f, N = %.1f\n', Kp, Ki, Kd, N);
fprintf(' Step Validation: RiseTime = %.4fs, SettlingTime = %.4fs, Overshoot = %.2f%%\n', ...
    step_info.RiseTime, step_info.SettlingTime, step_info.Overshoot);
fprintf(' Dynamic Gait Tracking Metrics:\n');
fprintf('   RMSE                : %.4f deg (%.4f rad)\n', rmse_deg, rmse_rad);
fprintf('   Mean Absolute Error : %.4f deg\n', mae_deg);
fprintf('   Max Absolute Error  : %.4f deg\n', max_error_deg);
fprintf('   Control Energy      : %.2f N^2*m^2*s\n', control_energy);
fprintf('   Peak Torque Demanded: %.2f N*m (Limit = +/- %.0f N*m)\n', max(abs(tau)), tau_sat);
fprintf('============================================================\n');

if nargout == 0
    clear pid_result;
end
end
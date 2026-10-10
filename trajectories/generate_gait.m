function gait = generate_gait(PARAM)
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 1 - Desired Gait Trajectory Generation
%
% Purpose:
% Generates the physiological reference joint trajectory in the sagittal plane
% (periodic knee/hip flexion-extension motion) with exact analytical position,
% velocity, and acceleration in both degrees and radians.
%
% Theoretical Model:
%   theta_d(t)       = theta_0 + A * sin(2 * pi * f * t)
%   theta_dot_d(t)   = A * (2 * pi * f) * cos(2 * pi * f * t)
%   theta_ddot_d(t)  = -A * (2 * pi * f)^2 * sin(2 * pi * f * t)
%
% Usage:
%   gait = generate_gait();       % Loads default PARAM from scripts
%   gait = generate_gait(PARAM);  % Uses provided PARAM struct

%% 1. Load Parameters if Not Provided
if nargin < 1 || isempty(PARAM)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    SUPPRESS_PARAM_DISPLAY = true;
    run(fullfile(project_root, 'scripts', 'project_parameters.m'));
end

%% 2. Time Base
t = PARAM.simulation.t;
f = PARAM.gait.frequency;
A_deg = PARAM.gait.amplitude;
offset_deg = PARAM.gait.offset;

omega = 2 * pi * f; % Angular frequency (rad/s)

%% 3. Analytical Trajectory Computation
% Position (degrees and radians)
theta_deg = offset_deg + A_deg * sin(omega * t);
theta_rad = deg2rad(theta_deg);

% Velocity (deg/s and rad/s) - exact analytical derivative
theta_dot_deg = A_deg * omega * cos(omega * t);
theta_dot_rad = deg2rad(theta_dot_deg);

% Acceleration (deg/s^2 and rad/s^2) - exact analytical derivative
theta_ddot_deg = -A_deg * (omega^2) * sin(omega * t);
theta_ddot_rad = deg2rad(theta_ddot_deg);

%% 4. Assemble Output Struct
gait.t = t;
gait.f = f;
gait.omega = omega;
gait.theta_deg = theta_deg;
gait.theta_rad = theta_rad;
gait.theta_dot_deg = theta_dot_deg;
gait.theta_dot_rad = theta_dot_rad;
gait.theta_ddot_deg = theta_ddot_deg;
gait.theta_ddot_rad = theta_ddot_rad;
gait.min_deg = min(theta_deg);
gait.max_deg = max(theta_deg);
gait.min_rad = min(theta_rad);
gait.max_rad = max(theta_rad);

%% 5. Plotting and Visualization
hFig = figure('Name', 'Desired Gait Trajectory', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [100, 100, 900, 700], 'Color', 'w');

% Subplot 1: Position
subplot(3, 1, 1);
plot(t, theta_deg, 'b-', 'LineWidth', 2.0); hold on;
yline(offset_deg, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Mean Offset');
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylim([gait.min_deg - 5, gait.max_deg + 5]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Joint Angle \theta_d (deg)', 'FontSize', 11, 'FontWeight', 'bold');
title('Reference Lower-Limb Gait Kinematics (0.5 Hz Cadence)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Desired Angle \theta_d', 'Gait Offset (30^\circ)', 'Location', 'northeast');

% Subplot 2: Velocity
subplot(3, 1, 2);
plot(t, theta_dot_deg, 'r-', 'LineWidth', 1.8);
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Angular Velocity \omega_d (deg/s)', 'FontSize', 11, 'FontWeight', 'bold');
title('Analytical Angular Velocity', 'FontSize', 11, 'FontWeight', 'bold');

% Subplot 3: Acceleration
subplot(3, 1, 3);
plot(t, theta_ddot_deg, 'm-', 'LineWidth', 1.8);
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Angular Acceleration \alpha_d (deg/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title('Analytical Angular Acceleration', 'FontSize', 11, 'FontWeight', 'bold');

%% 6. Save Plot to Results
fig_path = fullfile(PARAM.dirs.results, 'fig01_desired_gait_trajectory.png');
try
    saveas(hFig, fig_path);
    fprintf('Saved gait trajectory plot to: %s\n', fig_path);
catch ME
    warning('Could not save figure: %s', ME.message);
end

%% 7. Console Summary
fprintf('\n============================================================\n');
fprintf(' DESIRED GAIT TRAJECTORY GENERATION COMPLETE\n');
fprintf('============================================================\n');
fprintf(' Simulation Duration : %.2f s (%.0f samples)\n', PARAM.simulation.time, length(t));
fprintf(' Gait Frequency      : %.2f Hz (Cycle Period = %.2f s)\n', f, 1/f);
fprintf(' Joint Angle Range   : [%.2f, %.2f] deg ([%.3f, %.3f] rad)\n', ...
    gait.min_deg, gait.max_deg, gait.min_rad, gait.max_rad);
fprintf(' Peak Angular Vel    : %.2f deg/s (%.3f rad/s)\n', ...
    max(abs(theta_dot_deg)), max(abs(theta_dot_rad)));
fprintf(' Peak Angular Accel  : %.2f deg/s^2 (%.3f rad/s^2)\n', ...
    max(abs(theta_ddot_deg)), max(abs(theta_ddot_rad)));
fprintf('============================================================\n');

if nargout == 0
    clear gait;
end
end
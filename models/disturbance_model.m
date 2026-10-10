function dist = disturbance_model(PARAM)
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 6 - Human-Robot Interaction & External Disturbance Modelling
%
% Purpose:
% Generates a deterministic external disturbance torque simulating human-robot
% interaction phenomena such as involuntary muscle spasms, limb spasticity,
% voluntary resistance during gait rehabilitation, or environmental contact forces.
%
% Mathematical Model:
%   tau_dist(t) = tau_mag,  for t_start <= t <= t_end
%   tau_dist(t) = 0,        otherwise
%
% Parameters:
%   t_start   = 5.0 s  (Midway during the 10-second simulation, swing phase)
%   duration  = 1.0 s
%   t_end     = 6.0 s
%   magnitude = 5.0 N*m (Resistive torque opposing knee/hip flexion)
%
% Usage:
%   dist = disturbance_model();       % Uses default parameters
%   dist = disturbance_model(PARAM);  % Uses supplied PARAM

%% 1. Load Parameters if Not Provided
if nargin < 1 || isempty(PARAM)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    SUPPRESS_PARAM_DISPLAY = true;
    run(fullfile(project_root, 'scripts', 'project_parameters.m'));
end

%% 2. Generate Disturbance Time Vector
t = PARAM.simulation.t;
t_start = PARAM.disturbance.start_time;
duration = PARAM.disturbance.duration;
t_end = PARAM.disturbance.end_time;
mag = PARAM.disturbance.magnitude;

% Pulse disturbance profile (inclusive window: t_start <= t <= t_end)
tau_dist = zeros(size(t));
active_idx = (t >= t_start) & (t <= t_end);
tau_dist(active_idx) = mag;

%% 3. Struct Packaging
dist.t = t;
dist.tau_dist = tau_dist;
dist.t_start = t_start;
dist.t_end = t_end;
dist.duration = duration;
dist.magnitude = mag;
dist.active_idx = active_idx;

%% 4. Visualization
hFig = figure('Name', 'Disturbance Torque Profile', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [180, 150, 850, 480], 'Color', 'w');

plot(t, tau_dist, 'r-', 'LineWidth', 2.0); hold on;
area(t(active_idx), tau_dist(active_idx), 'FaceColor', [1, 0.8, 0.8], 'EdgeColor', 'r', ...
    'DisplayName', 'Active Disturbance Window (5.0s - 6.0s)');
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylim([-1, mag + 2]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Disturbance Torque \tau_{dist} (N\cdot m)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Human-Robot Interaction Disturbance (%.1f N\\cdot m Resistive Torque)', mag), ...
    'FontSize', 12, 'FontWeight', 'bold');
legend('Disturbance Profile', 'Disturbance Interval', 'Location', 'northeast');

fig_path = fullfile(PARAM.dirs.results, 'fig08_disturbance_profile.png');
try
    saveas(hFig, fig_path);
    fprintf('Saved disturbance profile plot to: %s\n', fig_path);
catch ME
    warning('Could not save fig 8: %s', ME.message);
end

%% 5. Console Summary
fprintf('\n============================================================\n');
fprintf(' HUMAN-ROBOT INTERACTION DISTURBANCE PROFILE CONFIGURED\n');
fprintf('============================================================\n');
fprintf(' Waveform Type   : Deterministic Rectangular Torque Pulse\n');
fprintf(' Active Interval : t = %.2f s to %.2f s (Duration = %.2f s)\n', t_start, t_end, duration);
fprintf(' Magnitude       : %.2f N*m\n', mag);
fprintf(' Interpretation  : Resistive interaction torque in sagittal plane\n');
fprintf('============================================================\n');

if nargout == 0
    clear dist;
end
end

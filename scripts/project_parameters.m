%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Central Project Parameters
% Part 1 - System Definition & Global Configuration
%
% This file establishes the centralized configuration parameters for all
% simulation scenarios, controllers, trajectories, and models.
%
% Course: Control Systems Engineering
% Application: Lower-Limb Rehabilitation Exoskeleton (Sagittal Joint Control)

%% =========================================================================
% 1. PROJECT DIRECTORIES SETUP
% ==========================================================================
script_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);

results_dir = fullfile(project_root, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

PARAM.dirs.project_root = project_root;
PARAM.dirs.results = results_dir;
PARAM.dirs.models = fullfile(project_root, 'models');
PARAM.dirs.controllers = fullfile(project_root, 'controllers');
PARAM.dirs.trajectories = fullfile(project_root, 'trajectories');
PARAM.dirs.scripts = fullfile(project_root, 'scripts');
PARAM.dirs.docs = fullfile(project_root, 'docs');

%% =========================================================================
% 2. SIMULATION TIME & SAMPLING
% ==========================================================================
PARAM.simulation.time = 10.0;           % Simulation duration (seconds)
PARAM.simulation.fs = 1000;             % Sampling frequency (Hz)
PARAM.simulation.Ts = 1 / PARAM.simulation.fs; % Sampling period: 0.001 s
PARAM.simulation.t = (0:PARAM.simulation.Ts:PARAM.simulation.time)'; % Column vector
PARAM.simulation.num_steps = length(PARAM.simulation.t);

%% =========================================================================
% 3. REFERENCE GAIT TRAJECTORY PARAMETERS
% ==========================================================================
% Periodic flexion-extension gait reference in sagittal plane
PARAM.gait.frequency = 0.5;             % Gait cadence frequency (Hz, period = 2.0 s)
PARAM.gait.amplitude = 30.0;            % Amplitude in degrees (deg)
PARAM.gait.offset = 30.0;               % Baseline bias/offset in degrees (deg)
PARAM.gait.min_angle = PARAM.gait.offset - PARAM.gait.amplitude; % 0 deg
PARAM.gait.max_angle = PARAM.gait.offset + PARAM.gait.amplitude; % 60 deg

% Radian conversions for dynamic equations and control laws
PARAM.gait.amplitude_rad = deg2rad(PARAM.gait.amplitude); % pi/6 ~ 0.5236 rad
PARAM.gait.offset_rad = deg2rad(PARAM.gait.offset);       % pi/6 ~ 0.5236 rad
PARAM.gait.min_angle_rad = deg2rad(PARAM.gait.min_angle); % 0.0 rad
PARAM.gait.max_angle_rad = deg2rad(PARAM.gait.max_angle); % pi/3 ~ 1.0472 rad

%% =========================================================================
% 4. EXOSKELETON ROTATIONAL JOINT DYNAMICS (PLANT)
% ==========================================================================
% Model: J * d2(theta)/dt2 + B * d(theta)/dt + K * theta = tau + tau_dist
% Units:
%   theta: joint angle (rad)
%   tau: control torque (N*m)
%   J: equivalent rotational moment of inertia (kg*m^2)
%   B: viscous damping coefficient (N*m*s/rad)
%   K: passive joint stiffness (N*m/rad)
PARAM.joint.inertia   = 1.0;            % J (kg*m^2)
PARAM.joint.damping   = 0.5;            % B (N*m*s/rad)
PARAM.joint.stiffness = 5.0;            % K (N*m/rad)

% State-space matrices: x = [theta; theta_dot], u = tau, y = theta
PARAM.joint.A = [0, 1; -PARAM.joint.stiffness / PARAM.joint.inertia, -PARAM.joint.damping / PARAM.joint.inertia];
PARAM.joint.B = [0; 1 / PARAM.joint.inertia];
PARAM.joint.C = [1, 0];
PARAM.joint.D = 0;

% Actuator torque limits (physical saturation)
PARAM.actuator.max_torque = 50.0;       % Maximum torque limit (N*m)
PARAM.actuator.min_torque = -50.0;      % Minimum torque limit (N*m)

%% =========================================================================
% 5. DISTURBANCE & INTERACTION FORCE PARAMETERS
% ==========================================================================
% Simulates user spasticity, voluntary resistance, or external stumbling load
PARAM.disturbance.start_time = 5.0;     % Start time (s)
PARAM.disturbance.duration   = 1.0;     % Active duration (s)
PARAM.disturbance.end_time   = PARAM.disturbance.start_time + PARAM.disturbance.duration; % 6.0 s
PARAM.disturbance.magnitude  = 5.0;     % Disturbance torque magnitude (N*m)
PARAM.disturbance.waveform   = 'pulse'; % 'pulse' or 'smooth_step'

%% =========================================================================
% 6. PID CONTROLLER GAINS & SETTINGS
% ==========================================================================
% Historically tuned via pidtune(G, 'PID') on open-loop plant G(s) = 1/(s^2 + 0.5s + 5)
PARAM.pid.Kp = 42.282161;              % Proportional gain (N*m/rad)
PARAM.pid.Ki = 35.723521;              % Integral gain (N*m/(rad*s))
PARAM.pid.Kd = 9.806628;               % Derivative gain (N*m*s/rad)
PARAM.pid.N  = 100.0;                  % Derivative filter coefficient (rad/s)

%% =========================================================================
% 7. SLIDING MODE CONTROLLER (SMC) PARAMETERS
% ==========================================================================
% Sliding surface: s = e_dot + lambda * e
% Equivalent control: tau_eq = J * (theta_ddot_d + lambda * e_dot) + B * theta_dot + K * theta
% Switching control: tau_sw = K_sw * sat(s / phi) or K_sw * tanh(s / phi)
PARAM.smc.lambda = 8.0;                % Sliding surface slope (s^-1)
PARAM.smc.K_sw   = 10.0;               % Robust switching gain (N*m, K_sw > |tau_dist|)
PARAM.smc.phi    = 0.05;               % Boundary layer thickness (rad/s) to prevent chattering
PARAM.smc.use_tanh = true;             % Use smooth hyperbolic tangent boundary layer

%% =========================================================================
% 8. ASSIST-AS-NEEDED (AAN) ADAPTIVE PARAMETERS
% ==========================================================================
% Error-dependent assistance scaling: alpha in [alpha_min, alpha_max]
PARAM.aan.alpha_min = 0.20;            % 20% baseline assistance for patient active participation
PARAM.aan.alpha_max = 1.00;            % 100% full robot assistance when error exceeds threshold
PARAM.aan.error_low_deg = 2.0;         % Low-error threshold (deg)
PARAM.aan.error_high_deg = 10.0;       % High-error threshold (deg)
PARAM.aan.filter_tau = 0.20;           % Low-pass filter time constant for error smoothing (s)

%% =========================================================================
% 9. METRIC DEFINITIONS (RECOVERY AND CROSS-VALIDATION)
% ==========================================================================
% Recovery: after disturbance end, |tracking error| must remain within
% +/- threshold for a sustained hold duration. If this never occurs before
% the simulation ends, recovery time is reported as not observed (NaN).
PARAM.metrics.recovery_threshold_deg = 2.0;  % In-band error threshold (deg)
PARAM.metrics.recovery_hold_s = 0.10;        % Required continuous in-band duration (s)
% Simulink vs MATLAB joint-angle comparison tolerance (rad).
% Justification: MATLAB gait uses semi-implicit Euler at Ts, while the
% Simulink gait model uses fixed-step ode4 and a continuous PID block.
PARAM.metrics.simulink_theta_tol_rad = 0.05; % ~2.9 deg

%% =========================================================================
% 10. CONSOLE CONFIRMATION DISPLAY (OPTIONAL)
% ==========================================================================
if ~exist('SUPPRESS_PARAM_DISPLAY', 'var') || ~SUPPRESS_PARAM_DISPLAY
    fprintf('\n========================================================\n');
    fprintf(' LOWER-LIMB REHABILITATION EXOSKELETON: PARAMETERS LOADED\n');
    fprintf('========================================================\n');
    fprintf(' Duration: %.1f s | Ts: %.4f s | Gait Freq: %.2f Hz\n', ...
        PARAM.simulation.time, PARAM.simulation.Ts, PARAM.gait.frequency);
    fprintf(' Gait Range: %.1f deg to %.1f deg (%.3f to %.3f rad)\n', ...
        PARAM.gait.min_angle, PARAM.gait.max_angle, ...
        PARAM.gait.min_angle_rad, PARAM.gait.max_angle_rad);
    fprintf(' Plant: J = %.2f kg*m^2 | B = %.2f N*m*s/rad | K = %.2f N*m/rad\n', ...
        PARAM.joint.inertia, PARAM.joint.damping, PARAM.joint.stiffness);
    fprintf(' PID Gains: Kp = %.3f, Ki = %.3f, Kd = %.3f\n', ...
        PARAM.pid.Kp, PARAM.pid.Ki, PARAM.pid.Kd);
    fprintf(' SMC Gains: lambda = %.1f, K_sw = %.1f, phi = %.3f\n', ...
        PARAM.smc.lambda, PARAM.smc.K_sw, PARAM.smc.phi);
    fprintf(' Disturbance: %.1f N*m from t = %.1f s to %.1f s\n', ...
        PARAM.disturbance.magnitude, PARAM.disturbance.start_time, PARAM.disturbance.end_time);
    fprintf(' AAN Assistance Range: [%.2f, %.2f] | Error Band: [%.1f, %.1f] deg\n', ...
        PARAM.aan.alpha_min, PARAM.aan.alpha_max, ...
        PARAM.aan.error_low_deg, PARAM.aan.error_high_deg);
    fprintf('========================================================\n\n');
end
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Central Project Parameters
% Part 1 - System Definition
%
% This file contains the common parameters used throughout
% the exoskeleton control-system simulation.

clear;
clc;

%% =========================================================
% 1. SIMULATION PARAMETERS
% ==========================================================

PARAM.simulation.time = 10;          % Simulation duration (s)
PARAM.simulation.fs = 1000;          % Sampling frequency (Hz)
PARAM.simulation.Ts = 1 / PARAM.simulation.fs;


%% =========================================================
% 2. GAIT TRAJECTORY PARAMETERS
% ==========================================================

PARAM.gait.frequency = 0.5;          % Gait frequency (Hz)
PARAM.gait.amplitude = 30;           % Joint-angle amplitude (deg)
PARAM.gait.offset = 30;              % Mean joint angle (deg)

PARAM.gait.min_angle = ...
    PARAM.gait.offset - PARAM.gait.amplitude;

PARAM.gait.max_angle = ...
    PARAM.gait.offset + PARAM.gait.amplitude;


%% =========================================================
% 3. JOINT MODEL PARAMETERS
% ==========================================================
%
% These values will be used later when we construct the
% simplified exoskeleton joint dynamic model.
%
% NOTE:
% These are initial simulation assumptions, not measured
% patient/exoskeleton parameters.

PARAM.joint.inertia = 1.0;            % kg*m^2
PARAM.joint.damping = 0.5;            % N*m*s/rad
PARAM.joint.stiffness = 5.0;          % N*m/rad


%% =========================================================
% 4. DISTURBANCE PARAMETERS
% ==========================================================

PARAM.disturbance.start_time = 5;     % Disturbance start (s)
PARAM.disturbance.duration = 1;        % Duration (s)
PARAM.disturbance.magnitude = 5;       % Initial disturbance level


%% =========================================================
% 5. ASSIST-AS-NEEDED PARAMETERS
% ==========================================================

PARAM.assistance.min = 0.20;           % Minimum assistance
PARAM.assistance.max = 1.00;           % Maximum assistance

PARAM.assistance.error_low = 2;        % Low-error threshold (deg)
PARAM.assistance.error_high = 10;      % High-error threshold (deg)


%% =========================================================
% 6. DISPLAY PROJECT PARAMETERS
% ==========================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf(' LOWER-LIMB EXOSKELETON CONTROL SYSTEM\n');
fprintf(' PROJECT PARAMETERS\n');
fprintf('====================================================\n');

fprintf('\nSimulation Parameters\n');
fprintf('Simulation Time       : %.2f s\n', ...
    PARAM.simulation.time);

fprintf('Sampling Frequency    : %.0f Hz\n', ...
    PARAM.simulation.fs);

fprintf('Sampling Time         : %.4f s\n', ...
    PARAM.simulation.Ts);

fprintf('\nGait Parameters\n');
fprintf('Gait Frequency        : %.2f Hz\n', ...
    PARAM.gait.frequency);

fprintf('Gait Amplitude        : %.2f deg\n', ...
    PARAM.gait.amplitude);

fprintf('Joint Angle Range     : %.2f to %.2f deg\n', ...
    PARAM.gait.min_angle, ...
    PARAM.gait.max_angle);

fprintf('\nJoint Model Parameters\n');
fprintf('Inertia               : %.2f kg*m^2\n', ...
    PARAM.joint.inertia);

fprintf('Damping               : %.2f N*m*s/rad\n', ...
    PARAM.joint.damping);

fprintf('Stiffness             : %.2f N*m/rad\n', ...
    PARAM.joint.stiffness);

fprintf('\n====================================================\n');
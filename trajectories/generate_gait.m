%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 1 - Desired Gait Trajectory
%
% Purpose:
% Generate the reference joint-angle trajectory that the
% future closed-loop controller will attempt to track.

clear;
clc;
close all;

%% Simulation Parameters

simulation_time = 10;      % seconds
sampling_frequency = 1000; % Hz
Ts = 1/sampling_frequency; % sampling time

t = 0:Ts:simulation_time;

%% Gait Parameters

gait_frequency = 0.5;      % Hz
gait_amplitude = 30;       % degrees
joint_angle_offset = 30;   % degrees

%% Desired Joint Angle

theta_desired = joint_angle_offset + ...
    gait_amplitude * ...
    sin(2*pi*gait_frequency*t);

%% Plot Desired Trajectory

figure('Name','Desired Gait Trajectory');

plot(t, theta_desired, 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Joint Angle (degrees)');
title('Desired Lower-Limb Joint Gait Trajectory');

legend('Desired Joint Angle', ...
    'Location','best');

%% Display Parameters

fprintf('\n');
fprintf('=============================================\n');
fprintf(' LOWER-LIMB EXOSKELETON - GAIT GENERATION\n');
fprintf('=============================================\n');

fprintf('Simulation Time     : %.2f s\n', simulation_time);
fprintf('Sampling Frequency  : %.0f Hz\n', sampling_frequency);
fprintf('Sampling Time       : %.4f s\n', Ts);
fprintf('Gait Frequency      : %.2f Hz\n', gait_frequency);
fprintf('Gait Amplitude      : %.2f deg\n', gait_amplitude);
fprintf('Joint Angle Offset  : %.2f deg\n', joint_angle_offset);

fprintf('=============================================\n');
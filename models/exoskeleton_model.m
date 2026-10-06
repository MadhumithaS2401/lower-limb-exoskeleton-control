%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 2 - Mathematical Exoskeleton Model
%
% Rotational joint model:
%
% J*theta_ddot + B*theta_dot + K*theta = tau
%
% Transfer function:
%
%              1
% G(s) = -------------------
%        J*s^2 + B*s + K

clear;
clc;
close all;

%% Load Project Parameters

run('../scripts/project_parameters.m');

%% Extract Joint Parameters

J = PARAM.joint.inertia;
B = PARAM.joint.damping;
K = PARAM.joint.stiffness;

%% Create Laplace Variable

s = tf('s');

%% Exoskeleton Joint Plant

G = 1/(J*s^2 + B*s + K);

%% Display Model

fprintf('\n');
fprintf('=============================================\n');
fprintf(' EXOSKELETON JOINT MATHEMATICAL MODEL\n');
fprintf('=============================================\n');

fprintf('\nJoint Parameters:\n');
fprintf('Inertia    J = %.2f kg*m^2\n', J);
fprintf('Damping    B = %.2f N*m*s/rad\n', B);
fprintf('Stiffness  K = %.2f N*m/rad\n', K);

fprintf('\nTransfer Function:\n');
G

%% Poles of the Plant

plant_poles = pole(G);

fprintf('\nPlant Poles:\n');
disp(plant_poles);

%% Open-Loop Step Response

figure('Name','Exoskeleton Plant Step Response');

step(G);

grid on;

title('Open-Loop Exoskeleton Joint Step Response');

xlabel('Time (s)');
ylabel('Joint Angle (rad)');

%% Basic Plant Information

fprintf('\n=============================================\n');
fprintf(' Plant model created successfully.\n');
fprintf('=============================================\n');
%% State-Space Representation

A = [0 1;
    -K/J -B/J];

B_state = [0;
    1/J];

C = [1 0];

D = 0;

% Create state-space model
G_ss = ss(A, B_state, C, D);

%% Display State-Space Model

fprintf('\n');
fprintf('=============================================\n');
fprintf(' STATE-SPACE EXOSKELETON MODEL\n');
fprintf('=============================================\n');

fprintf('\nA matrix:\n');
disp(A);

fprintf('B matrix:\n');
disp(B_state);

fprintf('C matrix:\n');
disp(C);

fprintf('D matrix:\n');
disp(D);

fprintf('State-space model:\n');
G_ss